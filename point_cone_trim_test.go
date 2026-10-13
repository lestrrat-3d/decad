package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointConeTriangleBoundIncludesReciprocalCurvature(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(1, 5), slope: big.NewRat(1, 1)}
	tri := [3]r3.Vec{{X: -0.9, Y: 1.1}, {X: -0.9, Y: 1.1125},
		{X: -0.9, Y: 1.1, Z: 0.0125}}
	_, alphaBound, _, err := pointConeTriangleBound(tri, cone)
	require.NoError(t, err)
	alpha := func(p r3.Vec) float64 {
		return 0.2 / (p.X + math.Hypot(p.Y, p.Z))
	}
	mid := tri[0].Scale(0.5).Add(tri[1].Scale(0.5))
	departure := math.Abs(alpha(mid) - 0.5*(alpha(tri[0])+alpha(tri[1])))
	require.Greater(t, departure, 0.0008)
	require.GreaterOrEqual(t, alphaBound, departure)
}

func TestPointConeUpperFarPlaneNodeRequiresCapCharge(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(10, 1), slope: big.NewRat(1, 1)}
	far := r3.Vec{X: 5, Y: 4.9}
	sample, err := pointConeFactor(far, cone)
	require.NoError(t, err)
	require.Less(t, sample.q+sample.qError, 1.0)
	_, _, err = sectorUpperCapNode(far, sample, false)
	require.ErrorIs(t, err, ErrUnsupported)
	clipped, round, err := sectorUpperCapNode(far, sample, true)
	require.NoError(t, err)
	require.Equal(t, far, clipped)
	require.GreaterOrEqual(t, round, far.Scale(sample.alpha).Sub(far).Len())
}

func TestPointConeSourceMustClearFiniteFarCap(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{far: big.NewRat(-2, 1)}
	base := facetedPayload{verts: []r3.Vec{{}, {X: -3, Y: 10}}}
	require.False(t, pointConeSourceAboveFarCap(base, cone))
	base.verts[1].X = -1
	require.True(t, pointConeSourceAboveFarCap(base, cone))
	base.meshBound = 1
	require.False(t, pointConeSourceAboveFarCap(base, cone))
}

func TestPointConeOutsideWitnessRejectsUncertainContact(t *testing.T) {
	t.Parallel()
	cone := &pointConeLimit{apex: big.NewRat(13, 1), slope: big.NewRat(1, 1)}
	base := facetedPayload{verts: []r3.Vec{{}, {X: 8, Y: 5.00001}}}
	require.True(t, pointConeHasOutsideWitness(base, cone))
	base.meshBound = 0.001
	require.False(t, pointConeHasOutsideWitness(base, cone))
}

// pointConeTestCones returns a mix of cones: the small and large unit-slope
// cones, a non-dyadic apex and slope, and random fixture-shaped cones.
func pointConeTestCones(r *rand.Rand, extra ...*pointConeLimit) []*pointConeLimit {
	cones := append([]*pointConeLimit(nil), extra...)
	for range 6 {
		apex := 5 + 10*r.Float64()
		far := -1 - 3*r.Float64()
		radius := 0.5 + 5*r.Float64()
		a := proofarith.FloatRat(apex)
		k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
		cones = append(cones, &pointConeLimit{apex: a, far: proofarith.FloatRat(far), slope: k})
	}
	return cones
}

// The Dyadic triangle bound is bit-identical to the big.Rat form on random
// cones and triangles.
func TestPointConeTriangleBoundMatchesRationalForm(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(1, 2))
	cones := pointConeTestCones(r,
		&pointConeLimit{apex: big.NewRat(1, 5), slope: big.NewRat(1, 1)},
		&pointConeLimit{apex: big.NewRat(10, 1), slope: big.NewRat(1, 1)})
	for i := range 1500 {
		cone := cones[i%len(cones)]
		scale := math.Ldexp(1, r.IntN(12)-6)
		base := r3.Vec{X: (r.Float64()*2 - 1) * 10 * scale, Y: (r.Float64()*2 - 1) * 10 * scale,
			Z: (r.Float64()*2 - 1) * 10 * scale}
		var tri [3]r3.Vec
		for j := range tri {
			tri[j] = base.Add(r3.Vec{X: r.Float64() * scale, Y: r.Float64() * scale, Z: r.Float64() * scale})
		}
		wantCap, wantAlpha, wantLip, wantErr := pointConeTriangleBoundRat(tri, cone)
		gotCap, gotAlpha, gotLip, gotErr := pointConeTriangleBound(tri, cone)
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
		&pointConeLimit{apex: big.NewRat(10, 1), slope: big.NewRat(1, 1)},
		&pointConeLimit{apex: big.NewRat(1, 5), slope: big.NewRat(3, 7)})
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
		want, wantErr := pointConeFactorRat(p, cone)
		got, gotErr := pointConeFactor(p, cone)
		if wantErr != nil {
			require.Error(t, gotErr, "%v", p)
			require.Equal(t, wantErr.Error(), gotErr.Error())
			continue
		}
		require.NoError(t, gotErr, "%v", p)
		require.Equal(t, math.Float64bits(want.q), math.Float64bits(got.q), "q %v", p)
		require.Equal(t, math.Float64bits(want.alpha), math.Float64bits(got.alpha), "alpha %v", p)
		require.Equal(t, math.Float64bits(want.qError), math.Float64bits(got.qError), "qError %v", p)
		require.Equal(t, math.Float64bits(want.pointError), math.Float64bits(got.pointError), "pointError %v", p)
	}
}
