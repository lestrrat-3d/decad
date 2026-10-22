package loftrecord

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

func fragment(center sectionrecord.Point2, tStart, tEnd float64) sectionrecord.CircleSeg {
	return sectionrecord.CircleSeg{Center: center, Radius: units.Millimeters(2), CCW: tStart < tEnd, TStart: tStart, TEnd: tEnd}
}

func TestCircleFootParameterFollowsTheFragmentsSense(t *testing.T) {
	t.Parallel()
	c := sectionrecord.Point2{U: 1, V: -1}
	at := func(angle float64) sectionrecord.Point2 {
		return sectionrecord.Point2{U: c.U + 2*math.Cos(angle), V: c.V + 2*math.Sin(angle)}
	}
	t.Run("an increasing range", func(t *testing.T) {
		t.Parallel()
		seg := fragment(c, 0.1, 0.4)
		got, ok := circleFootParameter(seg, at(2*math.Pi*0.25))
		require.True(t, ok)
		require.InDelta(t, 0.25, got, 1e-12)
	})
	t.Run("a decreasing range", func(t *testing.T) {
		t.Parallel()
		seg := fragment(c, 0.4, 0.1)
		got, ok := circleFootParameter(seg, at(2*math.Pi*0.25))
		require.True(t, ok)
		require.InDelta(t, 0.25, got, 1e-12)
	})
	t.Run("an angle past atan2's cut", func(t *testing.T) {
		t.Parallel()
		seg := fragment(c, 0.6, 0.95)
		got, ok := circleFootParameter(seg, at(2*math.Pi*0.8))
		require.True(t, ok)
		require.InDelta(t, 0.8, got, 1e-12)
	})
	t.Run("a point outside the fragment", func(t *testing.T) {
		t.Parallel()
		_, ok := circleFootParameter(fragment(c, 0.1, 0.4), at(2*math.Pi*0.6))
		require.False(t, ok)
		_, ok = circleFootParameter(fragment(c, 0.4, 0.1), at(2*math.Pi*0.6))
		require.False(t, ok)
	})
}

func TestLineCirclePairNeedsATrimmedFragment(t *testing.T) {
	t.Parallel()
	line := sectionrecord.LineSeg{TStart: 0, TEnd: 1}
	arc := sectionrecord.ArcSeg{TStart: 0, TEnd: 1}
	trimmed := fragment(sectionrecord.Point2{}, 0.2, 0.7)
	whole := fragment(sectionrecord.Point2{}, 0, 1)
	require.True(t, lineCirclePair(line, trimmed))
	require.True(t, lineCirclePair(trimmed, arc))
	require.False(t, lineCirclePair(line, whole), "a whole circle is a loop of its own")
	require.False(t, lineCirclePair(line, arc))
	require.False(t, lineCirclePair(trimmed, trimmed))
	require.False(t, lineCirclePair(line, sectionrecord.FitSplineSeg{}))
}
