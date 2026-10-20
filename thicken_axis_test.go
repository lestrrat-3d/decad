package decad

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	"github.com/stretchr/testify/require"
)

// The public sketch seam rejects this very large translated rectangle before
// Thicken can read it. Exercise the generated-section gate directly instead.
func TestThickenPrismUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 53)
	points := []planePoint{{U: base, V: 0}, {U: base + 100, V: 0},
		{U: base + 100, V: 60}, {U: base, V: 60}}
	segments := make([]curveSegment, len(points))
	for i := range points {
		segments[i] = lineSeg{Start: points[i], End: points[(i+1)%len(points)], TStart: 0, TEnd: 1}
	}
	pp := prismrecord.Payload{
		Profile: profileRecord{Outer: loopRecord{Segments: segments}},
		Z0:      0, Z1: 10, SurfaceResult: true,
	}
	budget := proofbound.NewWorkBudget(t.Context())
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	require.NoError(t, err)
	dirs, err := thickenaxis.AxisDirections(loops[0].Walks, budget)
	require.NoError(t, err)
	generated, err := offset2d.OffsetProfile(budget, pp.Profile, +1, 1, shellTol)
	require.NoError(t, err)
	first, ok := generated.Outer.Segments[0].(lineSeg)
	require.True(t, ok)
	require.Equal(t, base, first.Start.U)
	err = thickenaxis.CertifyAxisOffset(loops[0].Walks, dirs, generated.Outer, +1, 1, budget)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())

	doc := New()
	node, err := thickenbuild.Thicken(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), &topo.Body{Payload: pp}, thickenbuild.Negative, 1, 0)
	result := node
	require.Nil(t, result)
	require.ErrorIs(t, err, ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
	require.Empty(t, doc.Bodies())
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
		lineSeg{Start: planePoint{U: base, V: 0}, End: planePoint{U: base, V: 40}, TStart: 0, TEnd: 1},
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
	line := func(a, b planePoint) curveSegment {
		return lineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	for _, tc := range []struct {
		name  string
		chain chainRecord
		want  string
	}{
		{"not axis parallel", chainRecord{Segments: []curveSegment{
			line(planePoint{U: 0, V: 0}, planePoint{U: 10, V: 10}),
		}}, "not axis-parallel"},
		{"reverses on itself", chainRecord{Segments: []curveSegment{
			line(planePoint{U: 0, V: 0}, planePoint{U: 10, V: 0}),
			line(planePoint{U: 10, V: 0}, planePoint{U: 0, V: 0}),
		}}, "not a right angle"},
		{"arc segment", chainRecord{Segments: []curveSegment{
			arcSeg{Center: planePoint{U: 0, V: 0}, Start: planePoint{U: 10, V: 0},
				End: planePoint{U: 0, V: 10}, TStart: 0, TEnd: 1},
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
