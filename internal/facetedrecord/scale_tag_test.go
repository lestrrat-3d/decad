package facetedrecord_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// exactScaledGap is the exact distance bound of the float point got from
// anchor + s*(p - anchor), computed over big.Rat.
func exactScaledGap(anchor r3.Vec, s float64, p, got r3.Vec) float64 {
	rat := func(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }
	a, q, g := [3]float64{anchor.X, anchor.Y, anchor.Z}, [3]float64{p.X, p.Y, p.Z}, [3]float64{got.X, got.Y, got.Z}
	gap := 0.0
	for i := range a {
		exact := new(big.Rat).Sub(rat(q[i]), rat(a[i]))
		exact.Mul(exact, rat(s))
		exact.Add(exact, rat(a[i]))
		gap = max(gap, proofbound.RatAbsDiff(exact, g[i]))
	}
	return proofbound.Radius3D(gap)
}

func TestScaleTagDeparture(t *testing.T) {
	t.Parallel()
	anchor := r3.NewVec(1.3, -2.1, 0.7)
	origin := r3.NewVec(20.3, 0.1, 10.7)
	frame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane := surfacegeom.Plane{Frame: frame}
	cyl := surfacegeom.Cylinder{Origin: origin, Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(5)}

	t.Run("a cylinder charges the held term at twice the origin and radius gaps", func(t *testing.T) {
		t.Parallel()
		const s = 1.1
		got, add, ok := facetedrecord.ScaleTag(cyl, anchor, s)
		require.True(t, ok)
		moved, isCyl := got.(surfacegeom.Cylinder)
		require.True(t, isCyl)
		require.Equal(t, cyl.Axis, moved.Axis, "a uniform scale keeps the axis")
		rho := proofbound.RatAbsDiff(new(big.Rat).Mul(new(big.Rat).SetFloat64(s), big.NewRat(5, 1)), moved.Radius.Base())
		require.Positive(t, rho, "1.1 times 5 is not representable")
		p := exactScaledGap(anchor, s, cyl.Origin, moved.Origin)
		term, ok := facetedrecord.HeldNormalTerm(moved, 2*proofbound.AbsSumUpper(p, rho), 0)
		require.True(t, ok)
		require.GreaterOrEqual(t, add, term)
		require.Positive(t, add)
		require.Less(t, add, 1e-9, "the charge is rounding-sized, far below the trivial 2")
	})

	t.Run("a sphere and a torus keep a stated bound", func(t *testing.T) {
		t.Parallel()
		sphere := surfacegeom.Sphere{Center: origin, Radius: units.Millimeters(3)}
		got, add, ok := facetedrecord.ScaleTag(sphere, anchor, 1.1)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Sphere{}, got)
		require.Positive(t, add)
		torus := surfacegeom.Torus{Center: origin, Axis: r3.NewVec(0, 0, 1),
			Major: units.Millimeters(10), Minor: units.Millimeters(3)}
		got, add, ok = facetedrecord.ScaleTag(torus, anchor, 1.1)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Torus{}, got)
		require.Positive(t, add)
	})

	t.Run("a plane whose image is exact charges nothing", func(t *testing.T) {
		t.Parallel()
		exact, err := r3.NewFrame(r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		got, add, ok := facetedrecord.ScaleTag(surfacegeom.Plane{Frame: exact}, r3.NewVec(1, 2, 3), 2)
		require.True(t, ok)
		require.IsType(t, surfacegeom.Plane{}, got)
		require.Zero(t, add)
	})

	t.Run("a plane whose origin rounds charges at least that rounding", func(t *testing.T) {
		t.Parallel()
		const s = 1.1
		got, add, ok := facetedrecord.ScaleTag(plane, anchor, s)
		require.True(t, ok)
		moved := got.(surfacegeom.Plane)
		p := exactScaledGap(anchor, s, origin, moved.Frame.Origin())
		require.Positive(t, p)
		require.GreaterOrEqual(t, add, p)
		require.Less(t, add, 1e-9)
	})

	t.Run("a cone has no similarity image here", func(t *testing.T) {
		t.Parallel()
		cone := surfacegeom.Cone{Origin: origin, Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(1), HalfAngle: units.Degrees(10)}
		_, _, ok := facetedrecord.ScaleTag(cone, anchor, 2)
		require.False(t, ok)
	})
}
