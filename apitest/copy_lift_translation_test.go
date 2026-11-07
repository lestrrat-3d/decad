package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// liftTestTransform is a lift with an inexact rotation and a translation.
func liftTestTransform(t *testing.T) r3.Transform {
	t.Helper()
	lift, err := r3.RotationAround(r3.Vec{X: 3, Y: -2, Z: 1}, r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.4))
	require.NoError(t, err)
	return lift
}

// exactLiftShift is lift_lin * (to - from), each component rounded to the
// nearest float, computed independently in big.Rat.
func exactLiftShift(lift r3.Transform, from, to r3.Vec) r3.Vec {
	b := lift.Basis()
	cols := [3]r3.Vec{b.EX, b.EY, b.EZ}
	f := [3]float64{from.X, from.Y, from.Z}
	g := [3]float64{to.X, to.Y, to.Z}
	var out [3]float64
	for i := range out {
		sum := new(big.Rat)
		for j, col := range cols {
			c := [3]float64{col.X, col.Y, col.Z}
			d := new(big.Rat).Sub(new(big.Rat).SetFloat64(g[j]), new(big.Rat).SetFloat64(f[j]))
			sum.Add(sum, new(big.Rat).Mul(new(big.Rat).SetFloat64(c[i]), d))
		}
		out[i], _ = sum.Float64()
	}
	return r3.Vec{X: out[0], Y: out[1], Z: out[2]}
}

// TestCopyLiftTranslationFaceted moves a faceted body by a lift-local offset.
// The copy's bounds are the source's bounds shifted by the exact world vector
// rounded to floats, and its volume is the source's, each within the published
// bound.
//
// Leg shown to fail: placing the copy by the lift-local offset read as a world
// vector moves the bounds by a different vector.
func TestCopyLiftTranslationFaceted(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := slab(t, doc, 0, -5, 20, 5, 0, 10, r3.Identity())
	source, err := decad.Union(t.Context(), a, hookRing(t, doc, 12))
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(source))

	lift := liftTestTransform(t)
	from, to := r3.Vec{X: 1, Y: 0, Z: 0}, r3.Vec{X: 21.5, Y: -3, Z: 2}
	copied, err := decad.Copy(t.Context(), source, decad.WithCopyLiftTranslation(lift, from, to))
	require.NoError(t, err)

	shift := exactLiftShift(lift, from, to)
	sb, err := source.Bounds()
	require.NoError(t, err)
	cb, err := copied.Bounds()
	require.NoError(t, err)
	slack := cb.Bound.Base() + sb.Bound.Base()
	require.InDelta(t, sb.Min.X+shift.X, cb.Min.X, slack)
	require.InDelta(t, sb.Min.Y+shift.Y, cb.Min.Y, slack)
	require.InDelta(t, sb.Min.Z+shift.Z, cb.Min.Z, slack)
	require.InDelta(t, sb.Max.X+shift.X, cb.Max.X, slack)
	require.InDelta(t, sb.Max.Y+shift.Y, cb.Max.Y, slack)
	require.InDelta(t, sb.Max.Z+shift.Z, cb.Max.Z, slack)
	require.Greater(t, math.Abs(shift.X)+math.Abs(shift.Y)+math.Abs(shift.Z), 5.0)

	sv, err := source.Volume()
	require.NoError(t, err)
	cv, err := copied.Volume()
	require.NoError(t, err)
	require.InDelta(t, sv.Value.Base(), cv.Value.Base(), sv.Bound.Base()+cv.Bound.Base()+1e-9)
	sc, err := source.Centroid()
	require.NoError(t, err)
	cc, err := copied.Centroid()
	require.NoError(t, err)
	cslack := sc.Bound.Base() + cc.Bound.Base() + 1e-9
	require.InDelta(t, sc.Value.X+shift.X, cc.Value.X, cslack)
	require.InDelta(t, sc.Value.Y+shift.Y, cc.Value.Y, cslack)
	require.InDelta(t, sc.Value.Z+shift.Z, cc.Value.Z, cslack)
	require.Len(t, doc.Bodies(), 2)
}

// TestCopyLiftTranslationErrors holds the option's refusals.
func TestCopyLiftTranslationErrors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 2)
	lift := liftTestTransform(t)
	tests := []struct {
		name string
		lift r3.Transform
		to   r3.Vec
		want error
	}{
		{"invalid lift", r3.Transform{}, r3.Vec{X: 1}, decad.ErrDegenerate},
		{"non-finite to", lift, r3.Vec{X: math.NaN()}, decad.ErrNotFinite},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			_, err := decad.Copy(t.Context(), box, decad.WithCopyLiftTranslation(tc.lift, r3.Vec{}, tc.to))
			require.ErrorIs(t, err, tc.want)
			require.Len(t, doc.Bodies(), 1)
		})
	}
}

// TestCopyLiftTranslationPrism copies a prism, which is not faceted: it is
// placed by the float world translation, exactly as WithCopyPlacement.
func TestCopyLiftTranslationPrism(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 2)
	lift := liftTestTransform(t)
	from, to := r3.Vec{}, r3.Vec{X: 7, Y: 2, Z: -1}
	copied, err := decad.Copy(t.Context(), box, decad.WithCopyLiftTranslation(lift, from, to))
	require.NoError(t, err)
	bb, err := box.Bounds()
	require.NoError(t, err)
	cb, err := copied.Bounds()
	require.NoError(t, err)
	shift := exactLiftShift(lift, from, to)
	require.InDelta(t, bb.Min.X+shift.X, cb.Min.X, 1e-9)
	require.InDelta(t, bb.Min.Y+shift.Y, cb.Min.Y, 1e-9)
	require.InDelta(t, bb.Min.Z+shift.Z, cb.Min.Z, 1e-9)
}
