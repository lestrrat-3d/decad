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

// A resolved axis whose anchor or direction carries a proven bound reaches
// thickenRadialOf's second leg. No public axis constructor produces one
// alongside an exactly axis-parallel direction, so the gate is exercised
// where it lives.
func TestThickenRadialRefusesBoundedAxis(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		ax   axisFrame
		want string
	}{
		{"bounded anchor", axisFrame{dU: 0, dV: 1, aUBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"bounded direction", axisFrame{dU: 0, dV: 1, dVBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"off a plane axis", axisFrame{dU: 0.6, dV: 0.8},
			"not parallel to a recorded plane axis"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := thickenRadialOf(tc.ax)
			require.ErrorIs(t, err, ErrUnsupported)
			require.True(t, strings.Contains(err.Error(), tc.want), err.Error())
		})
	}
}

// The public sketch seam rejects a walk this far from the origin before
// Thicken can read it, so the ribbon's exact-generation gate is exercised on
// its own record: at u = 2^53 the float spacing is 2, so the offset
// coordinate u + 1 is not representable and the gate must name the rounding
// rather than a contact.
func TestThickenRibbonUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 53)
	chain := ChainRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: base, V: 0}, End: Point2{U: base, V: 40}, TStart: 0, TEnd: 1},
	}}
	budget := newWorkBudget(t.Context())
	_, err := thickenRibbon(t.Context(), chain, ThickenPositive, 1, budget, newFreeformWork(), nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
}

// A walk this arm cannot read exactly refuses before any section is
// assembled, each on its own stated reason.
func TestThickenRibbonWalkClassRefusals(t *testing.T) {
	t.Parallel()
	line := func(a, b Point2) CurveSegment {
		return LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	for _, tc := range []struct {
		name  string
		chain ChainRecord
		want  string
	}{
		{"not axis parallel", ChainRecord{Segments: []CurveSegment{
			line(Point2{U: 0, V: 0}, Point2{U: 10, V: 10}),
		}}, "not axis-parallel"},
		{"reverses on itself", ChainRecord{Segments: []CurveSegment{
			line(Point2{U: 0, V: 0}, Point2{U: 10, V: 0}),
			line(Point2{U: 10, V: 0}, Point2{U: 0, V: 0}),
		}}, "not a right angle"},
		{"arc segment", ChainRecord{Segments: []CurveSegment{
			ArcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 10, V: 0},
				End: Point2{U: 0, V: 10}, TStart: 0, TEnd: 1},
		}}, "line-only axis-parallel segments"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget := newWorkBudget(t.Context())
			_, err := thickenRibbon(t.Context(), tc.chain, ThickenPositive, 1, budget, newFreeformWork(), nil)
			require.ErrorIs(t, err, ErrUnsupported)
			require.True(t, strings.Contains(err.Error(), tc.want), err.Error())
		})
	}
}
