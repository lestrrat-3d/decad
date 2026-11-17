package retainedblend

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// tangentFixture is a C3 end: a rim of radius R about the z axis whose wall
// meets G*, the circle of radius rG about (d', 0, 0) with d' = R + rG (external)
// or R − rG (internal), at the tangent point (R, 0, 0).
type tangentFixture struct {
	cylEndFixture
	external bool
}

func newTangentFixture(external bool, spec blendSpec, sigma float64) tangentFixture {
	f := cylEndFixture{R: 7.5, rG: 2, spec: spec, sigma: sigma}
	if external {
		f.kappa, f.side, f.d = 1, 1, f.R+f.rG
	} else {
		f.kappa, f.side, f.d = -1, -1, f.R-f.rG
	}
	return tangentFixture{cylEndFixture: f, external: external}
}

func (f tangentFixture) end() *cylEnd {
	ce := f.cylEndFixture.end()
	ce.tan = &tangentEnd{external: f.external, t: r3.NewVec(f.R, 0, 0), eD: 16 * ulp * ce.sFar}
	ce.tan.eR = ce.tan.eD
	ce.tan.e = ce.tan.eD
	ce.tan.reach = ce.tan.eD * (2 + 2*f.R/f.d)
	return ce
}

func (f tangentFixture) section() (arcRim, endSection, error) {
	a := f.rim()
	a.cyl[0] = f.end()
	e, err := a.endSectionAt(f.spec, 0)
	return a, e, err
}

// bigAcosNear is acos(y) for y within 10⁻⁶ of ±1 or inside (−1, 1), by Newton's
// method from a seed that survives a float y that rounds to ±1.
func bigAcosNear(y *big.Float) *big.Float {
	one := bf(1)
	var seed float64
	if y.Sign() > 0 {
		h, _ := bquo(bsub(one, y), bf(2)).Float64()
		seed = 2 * math.Asin(math.Sqrt(h))
	} else {
		h, _ := bquo(badd(one, y), bf(2)).Float64()
		seed = math.Pi - 2*math.Asin(math.Sqrt(h))
	}
	x := bf(seed)
	for range 8 {
		s, c := bigSinCos(x)
		if s.Sign() == 0 {
			break
		}
		x = badd(x, bquo(bsub(c, y), s))
	}
	return x
}

func (f tangentFixture) psiExact(rho *big.Float) *big.Float {
	d, rG := bf(f.d), bf(f.rG)
	num := bsub(badd(bmul(d, d), bmul(rG, rG)), bmul(rho, rho))
	return bigAcosNear(bquo(num, bmul(bf(2), bmul(d, rG))))
}

func (f tangentFixture) alphaExact(rho *big.Float) *big.Float {
	d, rG := bf(f.d), bf(f.rG)
	num := bsub(badd(bmul(rho, rho), bmul(d, d)), bmul(rG, rG))
	return bigAcosNear(bquo(num, bmul(bf(2), bmul(rho, d))))
}

// TestTangentEndWallRow reads the wall's row, s = 1, of a C3 end on both ends,
// both senses of θ and both tangencies: it sits at the tangent point's azimuth
// with ψ exactly 0 (external) or π (internal), however the profile's float ρ(1)
// rounds, and a row before it keeps the acos reading.
func TestTangentEndWallRow(t *testing.T) {
	t.Parallel()
	spec := blendSpec{fillet: true, r: 0.5}
	for _, external := range []bool{true, false} {
		for _, sigma := range []float64{1, -1} {
			for _, sign := range []float64{1, -1} {
				f := newTangentFixture(external, spec, sigma)
				a, sec, err := f.section()
				require.NoError(t, err)
				a.sign = sign
				a.cyl[1] = f.end()
				a.cyl[1].sigma = -1
				a.cyl[0].sigma = sigma
				ce := a.cyl[0]
				ce.thetaC, a.cyl[1].thetaC = 0.4, 0.4
				p, _ := a.profile(spec)
				for _, ref := range []float64{0.4, 0.4 + 5, 0.4 - 5} {
					th, err := a.rowAngles(p, 1, [2]float64{ref, ref})
					require.NoError(t, err)
					require.Equal(t, 0.4+2*math.Pi*math.Round((ref-0.4)/(2*math.Pi)), th[0], "α = 0 exactly")
				}
				u, _, q := sec.at(1)
				wantPsi := 0.0
				if !external {
					wantPsi = math.Pi
				}
				require.Equal(t, -f.kappa*f.rG*wantPsi, u)
				require.Zero(t, q)
				require.Equal(t, wantPsi, sec.psiAt(1))
				require.Zero(t, sec.alphaAt(1))

				// A row before the wall's takes the acos reading and the side rule.
				rho, _ := p(0.5)
				th, ok := a.endAzimuthsAt(0, 0.5, rho, 0.4)
				require.True(t, ok)
				require.InDelta(t, 0.4+sigma*f.alphaFloat(rho), th, 1e-12)
				require.Greater(t, math.Abs(th-0.4), 1e-3)
			}
		}
	}
}

func (f tangentFixture) alphaFloat(rho float64) float64 {
	return math.Acos((rho*rho + f.d*f.d - f.rG*f.rG) / (2 * rho * f.d))
}

// TestTangentEndMarginCoversRounding measures, in 200-bit arithmetic, the
// rounding the margin of each piece stands for at rows approaching the wall's:
// u = −κ·r_G·ψ and the placed row vertex, each against the exact meeting of the
// row's circle with G* for the same computed ρ, stay within a quarter of the
// margin of the piece that ends at the wall's row, and of the piece between the
// row and its neighbour. The computed ψ differs from the exact one by at most
// the angle allowance of its piece.
func TestTangentEndMarginCoversRounding(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		for _, spec := range []blendSpec{{fillet: true, r: 0.5}, {fillet: true, r: 1.2}} {
			f := newTangentFixture(external, spec, 1)
			a, sec, err := f.section()
			require.NoError(t, err)
			worstU, worstV, worstPsi := 0.0, 0.0, 0.0
			for k := 1; k <= 20; k++ {
				s := 1 - math.Ldexp(1, -k)
				rho, z := sec.p(s)
				rhoB := bf(rho)
				u, _, _ := sec.at(s)
				du := bigDiff(u, bmul(bf(-f.kappa*f.rG), f.psiExact(rhoB)))
				th, ok := a.endAzimuthsAt(0, 0, rho, 0)
				require.True(t, ok, "k = %d", k)
				v := a.atTheta(rho, th, z)
				sinA, cosA := bigSinCos(f.alphaExact(rhoB))
				dx, dy := bigDiff(v.X, bmul(rhoB, cosA)), bigDiff(v.Y, bmul(rhoB, sinA))
				dv := math.Hypot(dx, dy)
				dpsi := bigDiff(sec.cyl.psi(rho), f.psiExact(rhoB))
				worstU, worstV, worstPsi = math.Max(worstU, du), math.Max(worstV, dv), math.Max(worstPsi, dpsi)
				margin := sec.pieceMargin(s, 1)
				require.LessOrEqual(t, du, margin/4, "u, k = %d", k)
				require.LessOrEqual(t, dv, margin/4, "vertex, k = %d", k)
				if k > 1 {
					prev := 1 - math.Ldexp(1, -(k-1))
					require.LessOrEqual(t, du, sec.pieceMargin(prev, s)/4, "u in the piece before, k = %d", k)
					require.LessOrEqual(t, dv, sec.pieceMargin(prev, s)/4, "vertex in the piece before, k = %d", k)
					require.LessOrEqual(t, dpsi, sec.pieceAngle(prev, s)/4, "psi, k = %d", k)
				}
			}
			t.Logf("external %v r %v: worst u %.3g, vertex %.3g, psi %.3g; last piece margin %.3g", external,
				spec.r, worstU, worstV, worstPsi, sec.pieceMargin(0.99, 1))
			// The wall's own row is exact, and the last piece's margin is H of the
			// arguments' errors: far above the rounding it covers, far below τ.
			require.Less(t, sec.pieceMargin(0.99, 1), math.Ldexp(spec.r, -10))
		}
	}
}

// TestTangentEndApexBox certifies the end curve through the wall's row: the
// piece that ends at s = 1 takes the box of the cone [0, π/2], so the rows
// spaced for τ reach the wall's row, and a τ below the last piece's margin does
// not certify.
func TestTangentEndApexBox(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		for _, sigma := range []float64{1, -1} {
			spec := blendSpec{fillet: true, r: 0.5}
			f := newTangentFixture(external, spec, sigma)
			_, sec, err := f.section()
			require.NoError(t, err)
			tau := f.tau()
			require.True(t, sec.certify(0.99, 1, tau), "a narrow last gap certifies")
			require.True(t, sec.certify(0.999, 1, tau))
			lo, hi := sec.tangentCone(0.5, 1, 0.3, 0)
			require.Equal(t, 0.0, lo)
			require.Equal(t, math.Pi/2, hi)
			require.False(t, sec.certify(0.99, 1, 1e-9), "a τ under the margin does not certify")
			rows, err := arcRows(spec, tau, arcProfileShare(tau), []endSection{sec})
			require.NoError(t, err)
			require.Equal(t, 1.0, rows[len(rows)-1])
			require.Zero(t, rows[0])
			require.LessOrEqual(t, len(rows), maxSectionRows)
			// Each gap's end curve stays within τ of its chord, at 64 samples.
			for j := 0; j+1 < len(rows); j++ {
				au, av, _ := sec.at(rows[j])
				bu, bv, _ := sec.at(rows[j+1])
				for k := 1; k < 64; k++ {
					s := rows[j] + (rows[j+1]-rows[j])*float64(k)/64
					pu, pv, _ := sec.at(s)
					dist := math.Abs((bu-au)*(pv-av)-(bv-av)*(pu-au)) / math.Hypot(bu-au, bv-av)
					require.LessOrEqual(t, dist, tau, "gap %d", j)
				}
			}
		}
	}
}

// TestTangentEndSectionRefusals checks the strict range of the rows before the
// wall's, and the tilt the section does not carry.
func TestTangentEndSectionRefusals(t *testing.T) {
	t.Parallel()
	spec := blendSpec{fillet: true, r: 0.5}
	// A fillet so large that the plane's row passes G*'s far side.
	f := newTangentFixture(true, blendSpec{fillet: true, r: 5}, 1)
	_, _, err := f.section()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "does not reach an end face")
	// A tilt that D + e_d cannot cover.
	f = newTangentFixture(true, spec, 1)
	f.eps = 1e-6
	_, _, err = f.section()
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "tilt")
	// The non-tangent end keeps R4's reading: no piece margin, the constant one.
	g := f.cylEndFixture
	g.eps = 0
	g.d = 9
	_, sec, err := g.section()
	require.NoError(t, err)
	require.Nil(t, sec.cyl.tan)
	require.Equal(t, sec.angle, sec.curvAngle(0, 1))
}

func TestHUpCoversAcosContinuity(t *testing.T) {
	t.Parallel()
	for _, x := range []float64{1, 1 - 1e-9, 0.5, 0, -0.5, -1 + 1e-9} {
		for _, h := range []float64{1e-16, 1e-12, 1e-8, 1e-4} {
			xt := math.Min(1, x+h)
			require.LessOrEqual(t, bigDiff(math.Acos(xt), bigAcosNear(bf(x))), hUp(xt-x)+1e-15, "x %v h %v", x, h)
		}
	}
	require.Zero(t, hUp(0))
	require.InDelta(t, math.Pi, hUp(2), 1e-12)
}

// TestMixedTangentEnds refuses an arc with one C3 end and one C2 end by name,
// and admits two of either kind.
func TestMixedTangentEnds(t *testing.T) {
	t.Parallel()
	tan := &cylEnd{tan: &tangentEnd{}}
	cross := &cylEnd{}
	for _, tc := range []struct {
		ends [2]*cylEnd
		ok   bool
	}{
		{[2]*cylEnd{tan, cross}, false},
		{[2]*cylEnd{cross, tan}, false},
		{[2]*cylEnd{tan, tan}, true},
		{[2]*cylEnd{cross, cross}, true},
		{[2]*cylEnd{nil, nil}, true},
	} {
		err := arcRim{cyl: tc.ends}.mixedTangent()
		if tc.ok {
			require.NoError(t, err)
			continue
		}
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an arc with one tangent and one crossing cylinder end is not built")
		require.True(t, isTangentRefusal(err))
	}
}

// tangentCornerFixture is the rim of the fixture with a proven corner at
// (R, 0, 0) and the held vertex there.
func tangentCornerFixture(f tangentFixture, rG float64) (arcRim, tangentCorner, facetedrecord.Payload) {
	a := f.rim()
	a.matOutside = !f.external
	a.scale = 20
	a.center = r3.Vec{}
	g := &topo.Face{
		Surface: surfacegeom.Cylinder{Origin: r3.NewVec(f.d, 0, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(rG)},
		RetainedTag: true, Reversed: true,
	}
	fp := facetedrecord.Payload{Verts: []r3.Vec{r3.NewVec(f.R, 0, 0)}, VertexBound: []float64{0}}
	return a, tangentCorner{g: g, proof: TangentProof{At: r3.NewVec(f.R, 0, 0)}}, fp
}

// TestTangentCylEnd reads the ideal end from the tags: G* at distance d' with
// the tangent point T on the wall's circle, and the reject-only checks.
func TestTangentCylEnd(t *testing.T) {
	t.Parallel()
	for _, external := range []bool{true, false} {
		f := newTangentFixture(external, blendSpec{fillet: true, r: 0.5}, 1)
		// The tag centre is a few ulps off the ideal distance: c* takes d'.
		f.d += 1e-13
		a, corner, fp := tangentCornerFixture(f, f.rG)
		end, ce, err := a.tangentCylEnd(1, 0, corner, fp)
		require.NoError(t, err)
		dPrime := f.R + f.rG
		if !external {
			dPrime = f.R - f.rG
		}
		require.Equal(t, dPrime, ce.d)
		require.InDelta(t, dPrime, ce.c.X, 1e-15)
		require.Equal(t, r3.NewVec(f.R, 0, 0), end.exact)
		require.Equal(t, ce.tan.t, end.exact)
		require.Equal(t, external, ce.tan.external)
		require.Equal(t, -1.0, ce.sigma)
		require.Zero(t, ce.thetaC)
		require.Zero(t, ce.tan.eC)
		require.Greater(t, ce.tan.e, 0.0)
		require.Greater(t, ce.tan.reach, ce.tan.e)
		require.Equal(t, 1, end.v+1)
		_, ce0, err := a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		require.Equal(t, 1.0, ce0.sigma)
	}

	t.Run("tags that disagree with the tangency refuse", func(t *testing.T) {
		t.Parallel()
		f := newTangentFixture(true, blendSpec{fillet: true, r: 0.5}, 1)
		f.d += 1e-6
		a, corner, fp := tangentCornerFixture(f, f.rG)
		_, _, err := a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an arc end's tags disagree with its proven tangency")
		require.True(t, isTangentRefusal(err))
	})
	t.Run("a held vertex off the tangent point refuses", func(t *testing.T) {
		t.Parallel()
		f := newTangentFixture(true, blendSpec{fillet: true, r: 0.5}, 1)
		a, corner, fp := tangentCornerFixture(f, f.rG)
		fp.Verts[0] = r3.NewVec(f.R+1e-3, 0, 0)
		_, _, err := a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorContains(t, err, "an arc end is not where its three tags meet")
		fp.Verts[0] = r3.NewVec(f.R, 0, 0)
		corner.proof.At = r3.NewVec(f.R, 1e-3, 0)
		_, _, err = a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorContains(t, err, "an arc end is not where its three tags meet")
	})
	t.Run("an internal end needs a smaller cylinder", func(t *testing.T) {
		t.Parallel()
		f := newTangentFixture(false, blendSpec{fillet: true, r: 0.5}, 1)
		a, corner, fp := tangentCornerFixture(f, f.R+1)
		_, _, err := a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorContains(t, err, "an internal arc end needs a cylinder smaller than the wall")
	})
	t.Run("a charged end builds with its charge part", func(t *testing.T) {
		t.Parallel()
		f := newTangentFixture(true, blendSpec{fillet: true, r: 0.5}, 1)
		a, corner, fp := tangentCornerFixture(f, f.rG)
		_, ce, err := a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		require.Zero(t, ce.tan.eC)
		corner.proof.Turn = 1e-12
		_, ce, err = a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		require.InDelta(t, f.rG*1e-12, ce.tan.eC, 1e-24)
		corner.proof.Turn = 0
		a.wallDelta = 1e-12
		_, ce, err = a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		require.InDelta(t, 1e-12, ce.tan.eC, 1e-24)
		a.wallDelta = 0
		corner.g.HasAxialDelta, corner.g.AxialDelta = true, 1e-12
		_, ce, err = a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		require.Greater(t, ce.tan.e, 1e-12)
		require.GreaterOrEqual(t, ce.tan.e, ce.tan.eC)
	})
	t.Run("a charge too large for the crossing bound refuses in the section", func(t *testing.T) {
		t.Parallel()
		spec := blendSpec{fillet: true, r: 0.5}
		f := newTangentFixture(true, spec, 1)
		a, corner, fp := tangentCornerFixture(f, f.rG)
		corner.proof.Turn = 1e-3
		_, ce, err := a.tangentCylEnd(0, 0, corner, fp)
		require.NoError(t, err)
		_, err = a.tangentEndSection(spec, ce)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "move its crossing too far")
	})
	t.Run("G keeps its tag tests", func(t *testing.T) {
		t.Parallel()
		f := newTangentFixture(true, blendSpec{fillet: true, r: 0.5}, 1)
		a, corner, fp := tangentCornerFixture(f, f.rG)
		corner.g.TagDeparture = 1
		_, _, err := a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorContains(t, err, "not a retained cylinder")
		corner.g.TagDeparture = 0
		corner.g.Reversed = false
		_, _, err = a.tangentCylEnd(0, 0, corner, fp)
		require.ErrorContains(t, err, "whose material lies inside it")
	})
}

// TestFourFaceCornerNeedsProof reads an arc end that is a corner of four faces:
// without a prover it keeps cornerOf's refusal, and with one it must still be
// four faces in the held mesh.
func TestFourFaceCornerNeedsProof(t *testing.T) {
	t.Parallel()
	all, arc, v, a, b, g, _ := fourFaceCorner(t)
	f := newTangentFixture(true, blendSpec{fillet: true, r: 0.5}, 1)
	rm := f.rim()
	rm.planeFace, rm.wallFace = a, b
	// A held mesh whose vertex 0 lies on four triangles of four faces.
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{v.Position, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)},
		VertexBound: make([]float64, 4),
		Tris:        [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {0, 1, 3}},
		FaceOf:      []int{0, 1, 2, 3},
	}
	index := map[*topo.Face]int{a: 0, b: 1, g: 2}
	_, _, err := rm.readEnd(0, v, fp, index, 0, 1, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "an edge end is not a corner of exactly three faces")
	require.False(t, isTangentRefusal(err))

	// An unproving prover reads no corner and leaves the same refusal.
	_, _, err = rm.readEnd(0, v, fp, index, 0, 1, &tangentRead{tangent: &fakeTangent{}, all: all, edge: arc})
	require.ErrorContains(t, err, "an edge end is not a corner of exactly three faces")
	require.False(t, isTangentRefusal(err))

	// A proving prover whose corner has a face the held body lacks refuses by name.
	_, _, err = rm.readEnd(0, v, fp, index, 0, 1,
		&tangentRead{tangent: &fakeTangent{ok: true}, all: all, edge: arc})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.True(t, isTangentRefusal(err))
}

// endAzimuths is endAzimuthsAt off the wall's row, the reading R4's cylinder-end
// tests take. Those tests read end 0 only.
//
//nolint:unparam // the existing tests name the end they read, as endAzimuthsAt does.
func (a arcRim) endAzimuths(i int, rho, ref float64) (float64, bool) {
	return a.endAzimuthsAt(i, 0, rho, ref)
}
