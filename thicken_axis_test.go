package decad

import (
	"math"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// The public sketch seam rejects this very large translated rectangle before
// Thicken can read it. Exercise the generated-section gate directly instead.
func TestThickenPrismUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 53)
	points := []Point2{{U: base, V: 0}, {U: base + 100, V: 0},
		{U: base + 100, V: 60}, {U: base, V: 60}}
	segments := make([]CurveSegment, len(points))
	for i := range points {
		segments[i] = LineSeg{Start: points[i], End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
	}
	pp := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: segments}},
		z0:      0, z1: 10, surfaceResult: true,
	}
	budget := newWorkBudget(t.Context())
	loops, err := prismCornerLoopsBudget(budget, pp)
	require.NoError(t, err)
	dirs, err := thickenAxisDirections(loops[0], budget)
	require.NoError(t, err)
	generated, err := offsetProfile(budget, pp.profile, +1, 1)
	require.NoError(t, err)
	first, ok := generated.Outer.Segments[0].(LineSeg)
	require.True(t, ok)
	require.Equal(t, base, first.Start.U)
	err = thickenCertifyAxisOffset(loops[0], dirs, generated.Outer, +1, 1, budget)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())

	doc := New()
	result, err := thickenPrism(t.Context(), doc, pp, ThickenNegative, 1, 0)
	require.Nil(t, result)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
	require.Empty(t, doc.Bodies())
}
