package prismcells

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// The junction between two lines whose ends differ closes through the corner
// of their supports, and the area enclosure charges that triangle.
func TestLoopAreaEnclosureCornerTriangle(t *testing.T) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	// Segment 0 stops 1 short of the corner (10, 0) of its support and segment
	// 1 starts 1 above it: the corner path adds a right triangle of area 1/2.
	open := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		rgLine(p(0, 0), p(9, 0)), rgLine(p(10, 1), p(10, 10)), rgLine(p(10, 10), p(0, 10)), rgLine(p(0, 10), p(0, 0)),
	}}
	budget := proofbound.NewWorkBudget(t.Context())
	enc, ok, err := LoopAreaEnclosure(budget, open, 0)
	require.NoError(t, err)
	require.True(t, ok)
	width, _ := enc.Hi.Sub(enc.Hi, enc.Lo).Float64()

	gap := math.Sqrt2
	reach := math.Hypot(10, 1) + gap
	require.InDelta(t, 2*(gap*reach+0.5), width, 1e-6, `the chord's strip plus the corner triangle, both ways`)
	cornerless, ok := cornerTriangleUpper(rgLine(p(0, 0), p(9, 0)), rgLine(p(0, 5), p(9, 5)))
	require.True(t, ok)
	require.Zero(t, cornerless, `parallel supports have no corner`)
	tri, ok := cornerTriangleUpper(rgLine(p(0, 0), p(9, 0)), rgLine(p(10, 1), p(10, 10)))
	require.True(t, ok)
	require.InDelta(t, 0.5, tri, 1e-12)
}
