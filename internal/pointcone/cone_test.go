package pointcone

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointConeTriangleBoundIncludesReciprocalCurvature(t *testing.T) {
	t.Parallel()
	c := xConsts(big.NewRat(1, 5), big.NewRat(1, 1))
	tri := [3]r3.Vec{{X: -0.9, Y: 1.1}, {X: -0.9, Y: 1.1125},
		{X: -0.9, Y: 1.1, Z: 0.0125}}
	_, alphaBound, _, err := triangleBound(tri, c)
	require.NoError(t, err)
	alpha := func(p r3.Vec) float64 {
		return 0.2 / (p.X + math.Hypot(p.Y, p.Z))
	}
	mid := tri[0].Scale(0.5).Add(tri[1].Scale(0.5))
	departure := math.Abs(alpha(mid) - 0.5*(alpha(tri[0])+alpha(tri[1])))
	require.Greater(t, departure, 0.0008)
	require.GreaterOrEqual(t, alphaBound, departure)
}

func TestPointConeSourceMustClearFiniteFarCap(t *testing.T) {
	t.Parallel()
	far := big.NewRat(-2, 1)
	verts := []r3.Vec{{}, {X: -3, Y: 10}}
	require.False(t, AboveFarCap(verts, 0, far, coneaxis.X()))
	verts[1].X = -1
	require.True(t, AboveFarCap(verts, 0, far, coneaxis.X()))
	require.False(t, AboveFarCap(verts, 1, far, coneaxis.X()))
}

// Ordered widens the source fan by the mesh bound: the margin must beat the
// charge for that movement, not merely be positive.
func TestOrderedChargesMeshBound(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 5, Y: 1}, {X: 5, Z: 1}}
	one := big.NewRat(1, 1)
	four, eight := big.NewRat(4, 1), big.NewRat(8, 1)
	require.True(t, Ordered(four, one, eight, one, verts, 0, coneaxis.X()))
	require.False(t, Ordered(eight, one, four, one, verts, 0, coneaxis.X()))
	require.False(t, Ordered(four, one, eight, one, verts, 10, coneaxis.X()))
}

// pointConeTestCones returns apex and slope pairs: the small and large
// unit-slope cones, a non-dyadic apex and slope, and random fixture-shaped
// cones.
func pointConeTestCones(r *rand.Rand, extra ...[2]*big.Rat) [][2]*big.Rat {
	cones := append([][2]*big.Rat(nil), extra...)
	for range 6 {
		apex := 5 + 10*r.Float64()
		far := -1 - 3*r.Float64()
		radius := 0.5 + 5*r.Float64()
		a := proofarith.FloatRat(apex)
		k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
		cones = append(cones, [2]*big.Rat{a, k})
	}
	return cones
}

// triangleBound is the bound of one triangle's own geometry.
func triangleBound(tri [3]r3.Vec, c *Consts) (float64, float64, float64, error) {
	return TriangleBound(CellGeometry(tri, coneaxis.X()), c)
}

// triangleBoundRat forms the same bounds as TriangleBound from big.Rat values.
func triangleBoundRat(tri [3]r3.Vec, apex, slope *big.Rat) (float64, float64, float64, error) {
	xMin, yMin, yMax, zMin, zMax := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	reach, h := 0.0, 0.0
	for i, p := range tri {
		xMin = min(xMin, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	rhoMin := max(minAbs(yMin, yMax), minAbs(zMin, zMax))
	if rhoMin <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap reaches the axis", decaderr.ErrUnsupported)
	}
	gMin := new(big.Rat).Add(proofarith.FloatRat(xMin),
		new(big.Rat).Mul(slope, proofarith.FloatRat(rhoMin)))
	if gMin.Sign() <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap has no positive ray denominator", decaderr.ErrUnsupported)
	}
	// The crossing map is F(p)=p*a/g(p), where g=x+k*rho. Its Hessian
	// includes both the curvature of rho and the reciprocal/product terms.
	// For triangle diameter h, the interpolation departure is at most
	// h²/2 times sup ||H_F||. The scalar alpha bound also covers the
	// piecewise contact decision at g=a.
	a, k := apex, slope
	onePlusK := new(big.Rat).Add(big.NewRat(1, 1), k)
	h2 := new(big.Rat).Mul(proofarith.FloatRat(h), proofarith.FloatRat(h))
	g2 := new(big.Rat).Mul(gMin, gMin)
	g3 := new(big.Rat).Mul(g2, gMin)
	reciprocal := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, onePlusK), new(big.Rat).Mul(onePlusK, h2)), g3)
	radial := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, k), h2),
		new(big.Rat).Mul(big.NewRat(2, 1),
			new(big.Rat).Mul(proofarith.FloatRat(rhoMin), g2)))
	alphaBound := proofbound.AbsSumUpper(proofbound.RatFloatUp(reciprocal), proofbound.RatFloatUp(radial))
	product := new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Mul(a, onePlusK), h2), g2)
	capBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(reach, alphaBound), proofbound.RatFloatUp(product))
	alphaMax := proofbound.RatFloatUp(new(big.Rat).Quo(apex, gMin))
	lipNumer := new(big.Rat).Mul(apex, new(big.Rat).Add(big.NewRat(1, 1), slope))
	lipDenom := new(big.Rat).Mul(gMin, gMin)
	lip := proofbound.AbsSumUpper(alphaMax,
		proofbound.ProductUpper(reach, proofbound.RatFloatUp(new(big.Rat).Quo(lipNumer, lipDenom))))
	return capBound, alphaBound, lip, nil
}

// The Dyadic triangle bound is bit-identical to the big.Rat form on random
// cones and triangles.
func TestPointConeTriangleBoundMatchesRationalForm(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(1, 2))
	cones := pointConeTestCones(r,
		[2]*big.Rat{big.NewRat(1, 5), big.NewRat(1, 1)},
		[2]*big.Rat{big.NewRat(10, 1), big.NewRat(1, 1)})
	for i := range 1500 {
		cone := cones[i%len(cones)]
		scale := math.Ldexp(1, r.IntN(12)-6)
		base := r3.Vec{X: (r.Float64()*2 - 1) * 10 * scale, Y: (r.Float64()*2 - 1) * 10 * scale,
			Z: (r.Float64()*2 - 1) * 10 * scale}
		var tri [3]r3.Vec
		for j := range tri {
			tri[j] = base.Add(r3.Vec{X: r.Float64() * scale, Y: r.Float64() * scale, Z: r.Float64() * scale})
		}
		wantCap, wantAlpha, wantLip, wantErr := triangleBoundRat(tri, cone[0], cone[1])
		gotCap, gotAlpha, gotLip, gotErr := triangleBound(tri, xConsts(cone[0], cone[1]))
		if wantErr != nil {
			require.Error(t, gotErr)
			require.Equal(t, wantErr.Error(), gotErr.Error())
			continue
		}
		require.NoError(t, gotErr)
		require.Equal(t, math.Float64bits(wantCap), math.Float64bits(gotCap), "cap %v", tri)
		require.Equal(t, math.Float64bits(wantAlpha), math.Float64bits(gotAlpha), "alpha %v", tri)
		require.Equal(t, math.Float64bits(wantLip), math.Float64bits(gotLip), "lip %v", tri)
	}
}

// The Dyadic station enclosure is bit-identical to the big.Rat form, including
// zero coordinates and stations behind the apex.
func TestPointConeFactorMatchesRationalForm(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(4, 8))
	cones := pointConeTestCones(r,
		[2]*big.Rat{big.NewRat(10, 1), big.NewRat(1, 1)},
		[2]*big.Rat{big.NewRat(1, 5), big.NewRat(3, 7)})
	for i := range 2000 {
		cone := cones[i%len(cones)]
		scale := math.Ldexp(1, r.IntN(10)-4)
		p := r3.Vec{X: (r.Float64()*2 - 0.5) * 10 * scale, Y: (r.Float64()*2 - 1) * 10 * scale,
			Z: (r.Float64()*2 - 1) * 10 * scale}
		switch i % 9 {
		case 1:
			p.Y = 0
		case 2:
			p.Z = 0
		case 3:
			p.Y, p.Z = 0, 0
		case 4:
			p.X = 0
		case 5:
			p.X = -math.Abs(p.X)
		}
		want, wantErr := FactorRat(p, coneaxis.X(), cone[0], cone[1])
		got, gotErr := FactorDyadic(p, coneaxis.X(), xConsts(cone[0], cone[1]))
		if wantErr != nil {
			require.Error(t, gotErr, "%v", p)
			require.Equal(t, wantErr.Error(), gotErr.Error())
			continue
		}
		require.NoError(t, gotErr, "%v", p)
		require.Equal(t, math.Float64bits(want.Q), math.Float64bits(got.Q), "q %v", p)
		require.Equal(t, math.Float64bits(want.Alpha), math.Float64bits(got.Alpha), "alpha %v", p)
		require.Equal(t, math.Float64bits(want.QError), math.Float64bits(got.QError), "qError %v", p)
		require.Equal(t, math.Float64bits(want.PointError), math.Float64bits(got.PointError), "pointError %v", p)
	}
}

// legacyPointConeTriangleBound is the per-cone geometry pass that
// TriangleBound ran before the geometry was shared between cones.
func legacyPointConeTriangleBound(tri [3]r3.Vec, c *Consts) (float64, float64, error) {
	xMin, yMin, yMax, zMin, zMax := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	reach, h := 0.0, 0.0
	for i, p := range tri {
		xMin = min(xMin, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	geom := CellGeom{xMin: xMin, rhoMin: max(minAbs(yMin, yMax), minAbs(zMin, zMax)),
		reach: reach, h: h}
	bound, _, lip, err := TriangleBound(geom, c)
	return bound, lip, err
}

func legacyPointConeCapBound(far []r3.Vec, caps [][3]int, c *Consts) (float64, float64, error) {
	capBound, lip := 0.0, 0.0
	for _, tri := range caps {
		cellBound, cellLip, err := legacyPointConeTriangleBound(
			[3]r3.Vec{far[tri[0]], far[tri[1]], far[tri[2]]}, c)
		if err != nil {
			return 0, 0, err
		}
		capBound = max(capBound, cellBound)
		lip = max(lip, cellLip)
	}
	return capBound, lip, nil
}

// Sharing one geometry pass between the upper and lower cones gives the same
// bounds, Lipschitz values and refusals as computing it per cone.
func TestPointConeCapGeometryMatchesPerConeBound(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(11, 13))
	upper := xConsts(big.NewRat(1, 5), big.NewRat(3, 7))
	lower := xConsts(big.NewRat(10, 1), big.NewRat(1, 1))
	for i := range 30 {
		scale := math.Ldexp(1, r.IntN(6)-2)
		far := make([]r3.Vec, 40)
		for j := range far {
			// Mostly off the axis with a positive ray denominator; every
			// third cap strays across it, every fourth behind the apex.
			far[j] = r3.Vec{X: (r.Float64()*2 + 0.1) * 4 * scale,
				Y: (r.Float64() + 0.5) * 4 * scale, Z: (r.Float64() + 0.5) * 4 * scale}
			switch {
			case i%3 == 0 && j%7 == 0:
				far[j].Y = -far[j].Y
			case i%4 == 0 && j%9 == 0:
				far[j].X = -8 * scale * (r.Float64() + 1)
			}
		}
		if i%5 == 0 {
			far[r.IntN(len(far))] = r3.Vec{X: scale}
		}
		caps := make([][3]int, 60)
		for j := range caps {
			caps[j] = [3]int{r.IntN(len(far)), r.IntN(len(far)), r.IntN(len(far))}
		}
		geom := CapGeometry(far, caps, coneaxis.X())
		for _, c := range []*Consts{upper, lower} {
			wantBound, wantLip, wantErr := legacyPointConeCapBound(far, caps, c)
			gotBound, gotLip, gotErr := CapBound(geom, c)
			if wantErr != nil {
				require.EqualError(t, gotErr, wantErr.Error())
				continue
			}
			require.NoError(t, gotErr)
			require.Equal(t, math.Float64bits(wantBound), math.Float64bits(gotBound))
			require.Equal(t, math.Float64bits(wantLip), math.Float64bits(gotLip))
		}
	}
}

// The Dyadic midpoint rounding error equals the big.Rat form bit for bit.
func TestPointConeMidpointRoundMatchesRational(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(21, 22))
	coord := func() float64 {
		return (r.Float64()*2 - 1) * math.Ldexp(1, r.IntN(21)-10)
	}
	for i := range 20000 {
		p := r3.Vec{X: coord(), Y: coord(), Z: coord()}
		q := r3.Vec{X: coord(), Y: coord(), Z: coord()}
		switch i % 4 {
		case 1:
			q = p
		case 2:
			q = p.Scale(-1)
		case 3:
			q.Y = p.Y
		}
		held := p.Scale(0.5).Add(q.Scale(0.5))
		half := big.NewRat(1, 2)
		mid := func(a, b float64) *big.Rat {
			return new(big.Rat).Mul(new(big.Rat).Add(proofarith.FloatRat(a), proofarith.FloatRat(b)), half)
		}
		want := proofbound.Radius3D(max(
			proofbound.RatAbsDiff(mid(p.X, q.X), held.X),
			proofbound.RatAbsDiff(mid(p.Y, q.Y), held.Y),
			proofbound.RatAbsDiff(mid(p.Z, q.Z), held.Z)))
		got := MidpointRound(p, q, held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%v %v", p, q)
	}
}

// xConsts is the one constant set of a cone about world X.
func xConsts(apex, slope *big.Rat) *Consts {
	sample, _ := ConstantsOf(coneaxis.X(), apex, slope)
	return sample
}
