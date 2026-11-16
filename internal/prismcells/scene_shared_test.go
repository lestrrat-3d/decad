package prismcells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSceneDeltaSharedEmptyBeforeCharge(t *testing.T) {
	t.Parallel()
	var d prismcells.SceneDelta
	require.Empty(t, d.Shared().Spans)
	require.Empty(t, d.Shared().Edges)
}

func TestCircleRadiusBound(t *testing.T) {
	t.Parallel()
	pt := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	circle := func(r float64) sectionrecord.CurveSegment {
		return sectionrecord.CircleSeg{Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	}
	wide := 7.0000000000000009
	t.Run("a circle of exactly that radius is zero", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, 0.0, prismcells.CircleRadiusBound(circle(7), 7))
	})
	t.Run("an arc whose ends lie exactly on the radius is zero", func(t *testing.T) {
		t.Parallel()
		seg := sectionrecord.ArcSeg{Center: pt(1, 2), Start: pt(8, 2), End: pt(1, 9), TEnd: 1}
		require.Equal(t, 0.0, prismcells.CircleRadiusBound(seg, 7))
	})
	t.Run("a different radius is bounded outward", func(t *testing.T) {
		t.Parallel()
		got := prismcells.CircleRadiusBound(circle(wide), 7)
		require.Greater(t, got, 0.0)
		require.GreaterOrEqual(t, got, wide-7)
		require.Less(t, got, 4*(wide-7))
		require.Equal(t, got, prismcells.CircleRadiusBound(circle(7), wide), "symmetric in its two radii")
	})
	t.Run("an arc takes the larger of its two radii", func(t *testing.T) {
		t.Parallel()
		seg := sectionrecord.ArcSeg{Center: pt(0, 0), Start: pt(7, 0), End: pt(0, wide), TEnd: 1}
		got := prismcells.CircleRadiusBound(seg, 7)
		require.GreaterOrEqual(t, got, wide-7)
		require.Less(t, got, 4*(wide-7))
	})
	t.Run("the bound covers a coarse difference", func(t *testing.T) {
		t.Parallel()
		got := prismcells.CircleRadiusBound(circle(5), 3)
		require.GreaterOrEqual(t, got, 2.0)
		require.Less(t, got, 2.0000001)
	})
	t.Run("a line or a free-form segment is infinite", func(t *testing.T) {
		t.Parallel()
		line := sectionrecord.LineSeg{Start: pt(0, 0), End: pt(1, 0), TEnd: 1}
		require.True(t, math.IsInf(prismcells.CircleRadiusBound(line, 1), 1))
	})
	t.Run("a non-finite value is infinite", func(t *testing.T) {
		t.Parallel()
		require.True(t, math.IsInf(prismcells.CircleRadiusBound(circle(7), math.NaN()), 1))
		require.True(t, math.IsInf(prismcells.CircleRadiusBound(circle(math.Inf(1)), 7), 1))
		seg := sectionrecord.ArcSeg{Start: pt(math.Inf(1), 0), End: pt(0, 1), TEnd: 1}
		require.True(t, math.IsInf(prismcells.CircleRadiusBound(seg, 1), 1))
	})
	t.Run("a radius in other units is infinite", func(t *testing.T) {
		t.Parallel()
		seg := sectionrecord.CircleSeg{Radius: units.Centimeters(7), CCW: true, TEnd: 1}
		require.True(t, math.IsInf(prismcells.CircleRadiusBound(seg, 7), 1))
	})
}

func TestRegionsWithinSegmentCap(t *testing.T) {
	t.Parallel()
	circle := momentinput.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 1}
	profile := momentinput.Profile{
		Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{circle}},
		Holes: []momentinput.LoopRecord{{Segments: []momentinput.CurveSegment{circle}}},
	}
	budget := func(t *testing.T) *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	t.Run("a count at the limit passes", func(t *testing.T) {
		t.Parallel()
		n, ok, err := prismcells.RegionsWithinSegmentCap(budget(t), 512, profile)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, 512, n)
	})
	t.Run("a count over the limit stops", func(t *testing.T) {
		t.Parallel()
		_, ok, err := prismcells.RegionsWithinSegmentCap(budget(t), 511, profile)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("the work cap is the segment cap at its constant", func(t *testing.T) {
		t.Parallel()
		many := make([]momentinput.Profile, 0, 16)
		for range 16 {
			many = append(many, profile)
		}
		n, ok, err := prismcells.RegionsWithinWorkCap(budget(t), many...)
		require.NoError(t, err)
		require.False(t, ok)
		require.Greater(t, n, prismcells.MaxArrangementSegments)
		n, ok, err = prismcells.RegionsWithinSegmentCap(budget(t), 2*prismcells.MaxArrangementSegments, many...)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, 16*512, n)
	})
}
