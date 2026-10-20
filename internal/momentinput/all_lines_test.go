package momentinput_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestProfileAllLines(t *testing.T) {
	t.Parallel()

	line := sectionrecord.LineSeg{End: sectionrecord.Point2{U: 1}, TEnd: 1}
	lines := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{line, line}}
	arcs := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{line, sectionrecord.ArcSeg{}}}
	empty := sectionrecord.LoopRecord{}

	require.True(t, momentinput.Profile{Outer: lines}.AllLines())
	require.True(t, momentinput.Profile{Outer: lines, Holes: []sectionrecord.LoopRecord{lines, lines}}.AllLines())
	require.False(t, momentinput.Profile{Outer: arcs}.AllLines())
	require.False(t, momentinput.Profile{Outer: empty}.AllLines())
	require.False(t, momentinput.Profile{Outer: lines, Holes: []sectionrecord.LoopRecord{arcs}}.AllLines())
	require.False(t, momentinput.Profile{Outer: lines, Holes: []sectionrecord.LoopRecord{lines, empty}}.AllLines())
}
