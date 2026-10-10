package decad

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	"github.com/stretchr/testify/require"
)

// The public sketch seam rejects this very large translated rectangle before
// Thicken can read it. Exercise the generated-section gate directly instead.
func TestThickenPrismUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 53)
	points := []Point2{{U: base, V: 0}, {U: base + 100, V: 0},
		{U: base + 100, V: 60}, {U: base, V: 60}}
	segments := make([]curveSegment, len(points))
	for i := range points {
		segments[i] = lineSeg{Start: points[i], End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
	}
	pp := prismPayload{
		profile: profileRecord{Outer: loopRecord{Segments: segments}},
		z0:      0, z1: 10, surfaceResult: true,
	}
	budget := proofbound.NewWorkBudget(t.Context())
	loops, err := prismCornerLoopsBudget(budget, pp)
	require.NoError(t, err)
	dirs, err := thickenaxis.AxisDirections(loops[0].walks, budget)
	require.NoError(t, err)
	generated, err := offsetProfile(budget, pp.profile, +1, 1)
	require.NoError(t, err)
	first, ok := generated.Outer.Segments[0].(lineSeg)
	require.True(t, ok)
	require.Equal(t, base, first.Start.U)
	err = thickenaxis.CertifyAxisOffset(loops[0].walks, dirs, generated.Outer, +1, 1, budget)
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
		{"bounded anchor", axisFrame{DU: 0, DV: 1, AUBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"bounded direction", axisFrame{DU: 0, DV: 1, DVBound: math.Ldexp(1, -40)},
			"not stated exactly in the sketch plane"},
		{"off a plane axis", axisFrame{DU: 0.6, DV: 0.8},
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
	chain := chainRecord{Segments: []curveSegment{
		lineSeg{Start: Point2{U: base, V: 0}, End: Point2{U: base, V: 40}, TStart: 0, TEnd: 1},
	}}
	budget := proofbound.NewWorkBudget(t.Context())
	_, err := thickenaxis.RibbonProfile(t.Context(), chain, 1, 0, 1, budget, freeform.NewFreeformWork(), nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
}

// A walk this arm cannot read exactly refuses before any section is
// assembled, each on its own stated reason.
func TestThickenRibbonWalkClassRefusals(t *testing.T) {
	t.Parallel()
	line := func(a, b Point2) curveSegment {
		return lineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	for _, tc := range []struct {
		name  string
		chain chainRecord
		want  string
	}{
		{"not axis parallel", chainRecord{Segments: []curveSegment{
			line(Point2{U: 0, V: 0}, Point2{U: 10, V: 10}),
		}}, "not axis-parallel"},
		{"reverses on itself", chainRecord{Segments: []curveSegment{
			line(Point2{U: 0, V: 0}, Point2{U: 10, V: 0}),
			line(Point2{U: 10, V: 0}, Point2{U: 0, V: 0}),
		}}, "not a right angle"},
		{"arc segment", chainRecord{Segments: []curveSegment{
			arcSeg{Center: Point2{U: 0, V: 0}, Start: Point2{U: 10, V: 0},
				End: Point2{U: 0, V: 10}, TStart: 0, TEnd: 1},
		}}, "line-only axis-parallel segments"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget := proofbound.NewWorkBudget(t.Context())
			_, err := thickenaxis.RibbonProfile(t.Context(), tc.chain, 1, 0, 1, budget, freeform.NewFreeformWork(), nil)
			require.ErrorIs(t, err, ErrUnsupported)
			require.True(t, strings.Contains(err.Error(), tc.want), err.Error())
		})
	}
}
