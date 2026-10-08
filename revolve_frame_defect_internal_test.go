package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/evaluator-design.md §6's frame charge: a revolve
// denotes its plane-coordinate solid carried through L = B·[U V U×V], the
// held frame and placement read as exact rationals, and r3 keeps L
// orthonormal only to rounding. Each expected value is computed here from
// those exact leaves in 512-bit floating point, independently of the build.
// Legs shown to fail by deleting them and watching this test go red, then
// restoring them (on amd64):
//
//   - the volume charge (BoundedStretch by frameCharge's volume): every
//     case's volume went red, the tilted plane under the rotation at 13.4×
//     its bound;
//   - the wall area stretch: every full turn's annulus faces went red, and
//     the tilted plane's total area read 2.0× its bound;
//   - the cap area stretch: both quarter turns' caps went red, each
//     published Exact;
//   - the length stretch: every full turn's latitude circles went red.

// revolveDefectMap is L with the denoted offset, every leaf exact.
type revolveDefectMap struct {
	l      [3][3]*big.Rat
	origin [3]*big.Rat
}

func revolveDefectMapOf(t *testing.T, rp revolvePayload) revolveDefectMap {
	t.Helper()
	l, err := massmoment.PlaneMap(rp.frame, rp.xform)
	require.NoError(t, err)
	b := rp.xform.Basis()
	o, ok := massmoment.ExactVec(rp.frame.Origin())
	require.True(t, ok)
	tr, ok := massmoment.ExactVec(rp.xform.Translation())
	require.True(t, ok)
	var origin [3]*big.Rat
	for i := range 3 {
		origin[i] = new(big.Rat).Set(tr[i])
		for k, col := range []r3.Vec{b.EX, b.EY, b.EZ} {
			c, ok := massmoment.ExactVec(col)
			require.True(t, ok)
			origin[i].Add(origin[i], new(big.Rat).Mul(c[i], o[k]))
		}
	}
	return revolveDefectMap{l: l, origin: origin}
}

// dir maps plane coordinates (u, v, n) through L.
func (m revolveDefectMap) dir(x [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range 3 {
		out[i] = new(big.Rat)
		for k := range 3 {
			out[i].Add(out[i], new(big.Rat).Mul(m.l[i][k], x[k]))
		}
	}
	return out
}

func (m revolveDefectMap) point(x [3]*big.Rat) [3]*big.Rat {
	out := m.dir(x)
	for i := range 3 {
		out[i].Add(out[i], m.origin[i])
	}
	return out
}

func bigVec(v [3]*big.Rat) [3]*big.Float {
	return [3]*big.Float{bigRat(v[0]), bigRat(v[1]), bigRat(v[2])}
}

func bigRat(x *big.Rat) *big.Float { return new(big.Float).SetPrec(512).SetRat(x) }

func bigCrossLen(a, b [3]*big.Float) *big.Float {
	c := func(i, j int) *big.Float {
		x := new(big.Float).SetPrec(512).Mul(a[i], b[j])
		return x.Sub(x, new(big.Float).SetPrec(512).Mul(a[j], b[i]))
	}
	sum := new(big.Float).SetPrec(512)
	for _, x := range []*big.Float{c(1, 2), c(2, 0), c(0, 1)} {
		sum.Add(sum, new(big.Float).SetPrec(512).Mul(x, x))
	}
	return sum.Sqrt(sum)
}

// periodicIntegral is ∫₀^{2π} f(cos φ, sin φ) dφ by the trapezoid rule on 16
// nodes. Every integrand here is the length of a vector linear in (cos φ,
// sin φ) under a map within ~1e-15 of orthonormal, so its Fourier
// coefficients fall by that factor per harmonic and the rule's aliasing error
// is far below the 1e-60 slack requireEnclosesBig allows.
func periodicIntegral(f func(cs, sn *big.Float) *big.Float) *big.Float {
	sq := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(512).Sqrt(x) }
	two := new(big.Float).SetPrec(512).SetInt64(2)
	s2 := sq(two)
	half := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(512).Quo(x, two) }
	cosT := []*big.Float{big.NewFloat(1), half(sq(new(big.Float).SetPrec(512).Add(two, s2))), half(s2), half(sq(new(big.Float).SetPrec(512).Sub(two, s2)))}
	sinT := []*big.Float{big.NewFloat(0), cosT[3], cosT[2], cosT[1]}
	neg := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(512).Neg(x) }
	sum := new(big.Float).SetPrec(512)
	for k := range 16 {
		c, s := cosT[k%4], sinT[k%4]
		switch k / 4 {
		case 1:
			c, s = neg(s), c
		case 2:
			c, s = neg(c), neg(s)
		case 3:
			c, s = s, neg(c)
		}
		sum.Add(sum, f(c, s))
	}
	sum.Mul(sum, new(big.Float).SetPrec(512).Mul(bigPi(), two))
	return sum.Quo(sum, big.NewFloat(16))
}

// exactSinCos is the exact sine and cosine of the held quarter-turn angles a
// sweep stated in degrees ends at.
func exactSinCos(t *testing.T, phi float64) (*big.Rat, *big.Rat) {
	t.Helper()
	switch phi {
	case 0:
		return new(big.Rat), big.NewRat(1, 1)
	case math.Pi / 2:
		return big.NewRat(1, 1), new(big.Rat)
	case -math.Pi / 2:
		return big.NewRat(-1, 1), new(big.Rat)
	}
	t.Fatalf("no exact sine and cosine for %v", phi)
	return nil, nil
}

// TestRevolveReadingsCoverFrameDefect revolves the square ρ ∈ [2, 3],
// z ∈ [0, 1] about the sketch's V axis on a tilted plane, under a rotation,
// and both, a full turn and a quarter turn, and requires every published
// volume, area, length and centroid to enclose the reading of the solid L
// carries the plane-coordinate one to.
func TestRevolveReadingsCoverFrameDefect(t *testing.T) {
	t.Parallel()
	tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	for _, c := range []struct {
		name  string
		frame r3.Frame
		place r3.Transform
		full  bool
	}{
		{"tilted plane", tilted, r3.Identity(), true},
		{"tilted plane, quarter turn", tilted, r3.Identity(), false},
		{"rotated", xy, rot, true},
		{"rotated, quarter turn", xy, rot, false},
		{"tilted plane rotated", tilted, rot, true},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(c.frame)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			loop := [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}
			pts := make([]*sketch.Point, len(loop))
			for i, p := range loop {
				pts[i] = s.CreatePoint(p[0], p[1])
				s.Fix(pts[i])
			}
			for i := range pts {
				s.CreateLine(pts[i], pts[(i+1)%len(pts)])
			}
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			var ext AngularExtent = FullRevolution{}
			if !c.full {
				ext = AngleExtent{A: units.Degrees(90), Dir: Along}
			}
			b, err := New().Revolve(s, s.Profiles()[0], coilAxisV, ext)
			require.NoError(t, err)
			if c.place != r3.Identity() {
				b, err = b.Placed(t.Context(), c.place)
				require.NoError(t, err)
			}
			rp := b.payload.(revolvePayload)
			m := revolveDefectMapOf(t, rp)
			det := massmoment.Determinant(m.l)
			require.NotZero(t, det.Cmp(big.NewRat(1, 1)), "the fixture's map must not be exactly orthonormal")

			// The sketch's V axis is exact: z = ±v and ρ = ∓u, so the square
			// reads ∫|ρ| dA = 5/2, ∫z|ρ| dA / ∫|ρ| dA = ±1/2 and
			// ∫ρ|ρ| dA = ∓19/3.
			require.Zero(t, rp.ax.aU)
			require.Zero(t, rp.ax.aV)
			require.Zero(t, rp.ax.dU)
			require.Equal(t, 1.0, math.Abs(rp.ax.dV))
			dV := new(big.Rat).SetFloat64(rp.ax.dV)
			q := big.NewRat(5, 2)
			rhoRho := new(big.Rat).Mul(big.NewRat(-19, 3), dV)
			zero := new(big.Rat)
			dHat := [3]*big.Rat{zero, dV, zero}
			e0 := [3]*big.Rat{new(big.Rat).Neg(dV), zero, zero}
			e1 := [3]*big.Rat{zero, zero, big.NewRat(1, 1)}
			wl, e0l, e1l := bigVec(m.dir(dHat)), bigVec(m.dir(e0)), bigVec(m.dir(e1))

			theta := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(2))
			if !c.full {
				theta.Quo(theta, big.NewFloat(4))
			}
			absDet := new(big.Rat).Abs(det)
			wantVol := new(big.Float).SetPrec(512).Mul(theta, bigRat(new(big.Rat).Mul(q, absDet)))
			vol, err := b.Volume()
			require.NoError(t, err)
			requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), wantVol, "volume")

			// The centroid of the image is the image of the plane-coordinate
			// centroid d̂·z̄ + (∫ρ|ρ|/(Θ·∫|ρ|))·(e0·(sin φ1 − sin φ0) +
			// e1·(cos φ0 − cos φ1)), with z̄ = v̄ = 1/2 along d̂ = ±V.
			want := bigVec(m.point([3]*big.Rat{zero, big.NewRat(1, 2), zero}))
			if !c.full {
				s0, c0 := exactSinCos(t, rp.phi0)
				s1, c1 := exactSinCos(t, rp.phi1)
				var radial [3]*big.Rat
				for i := range 3 {
					radial[i] = new(big.Rat).Add(new(big.Rat).Mul(e0[i], new(big.Rat).Sub(s1, s0)), new(big.Rat).Mul(e1[i], new(big.Rat).Sub(c0, c1)))
				}
				lr := m.dir(radial)
				for i := range 3 {
					term := new(big.Float).SetPrec(512).Quo(bigRat(new(big.Rat).Mul(lr[i], new(big.Rat).Quo(rhoRho, q))), theta)
					want[i].Add(want[i], term)
				}
			}
			cen, err := b.Centroid()
			require.NoError(t, err)
			for i, got := range []float64{cen.Value.X, cen.Value.Y, cen.Value.Z} {
				requireEnclosesBig(t, got, cen.Bound.Base(), want[i], "centroid")
			}

			if !c.full {
				// Each cap is the unit square in the plane through the axis at
				// its end's angle, scaled by |L·d̂ × L·r̂(φ)|.
				caps := 0
				for _, f := range b.Faces() {
					phi := rp.phi0
					switch f.origins[0].Role {
					case roleCapStart:
					case roleCapEnd:
						phi = rp.phi1
					default:
						continue
					}
					caps++
					sn, cs := exactSinCos(t, phi)
					var r [3]*big.Float
					for i := range 3 {
						r[i] = new(big.Float).SetPrec(512).Add(new(big.Float).SetPrec(512).Mul(bigRat(cs), e0l[i]), new(big.Float).SetPrec(512).Mul(bigRat(sn), e1l[i]))
					}
					fa, err := f.Area()
					require.NoError(t, err)
					requireEnclosesBig(t, fa.Value.Base(), fa.Bound.Base(), bigCrossLen(wl, r), "cap area")
				}
				require.Equal(t, 2, caps)
				return
			}

			// A full turn: each annulus is planar, area 5π·|L·e0 × L·e1|; a
			// cylinder of radius ρ and unit height reads
			// ρ·∫|L·d̂ × L·t̂(φ)| dφ and a latitude circle ρ·∫|L·t̂(φ)| dφ,
			// with t̂ = −sin φ·e0 + cos φ·e1.
			tangent := func(cs, sn *big.Float) [3]*big.Float {
				var out [3]*big.Float
				for i := range 3 {
					out[i] = new(big.Float).SetPrec(512).Sub(new(big.Float).SetPrec(512).Mul(cs, e1l[i]), new(big.Float).SetPrec(512).Mul(sn, e0l[i]))
				}
				return out
			}
			wall := periodicIntegral(func(cs, sn *big.Float) *big.Float { return bigCrossLen(wl, tangent(cs, sn)) })
			rim := periodicIntegral(func(cs, sn *big.Float) *big.Float {
				sum := new(big.Float).SetPrec(512)
				for _, x := range tangent(cs, sn) {
					sum.Add(sum, new(big.Float).SetPrec(512).Mul(x, x))
				}
				return sum.Sqrt(sum)
			})
			annulus := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(5))
			annulus.Mul(annulus, bigCrossLen(e0l, e1l))
			for _, f := range b.Faces() {
				fa, err := f.Area()
				require.NoError(t, err)
				exact := annulus
				if cyl, ok := f.surface.(Cylinder); ok {
					exact = new(big.Float).SetPrec(512).Mul(wall, big.NewFloat(cyl.Radius.Base()))
				}
				requireEnclosesBig(t, fa.Value.Base(), fa.Bound.Base(), exact, "face area")
			}
			total := new(big.Float).SetPrec(512).Mul(annulus, big.NewFloat(2))
			total.Add(total, new(big.Float).SetPrec(512).Mul(wall, big.NewFloat(5)))
			ar, err := b.Area()
			require.NoError(t, err)
			requireEnclosesBig(t, ar.Value.Base(), ar.Bound.Base(), total, "area")
			edges := b.Edges()
			require.Len(t, edges, 4)
			for _, e := range edges {
				circle, ok := e.curve.(Circle3)
				require.True(t, ok)
				l, err := e.Length()
				require.NoError(t, err)
				exact := new(big.Float).SetPrec(512).Mul(rim, big.NewFloat(circle.Radius.Base()))
				requireEnclosesBig(t, l.Value.Base(), l.Bound.Base(), exact, "latitude length")
			}
		})
	}
}
