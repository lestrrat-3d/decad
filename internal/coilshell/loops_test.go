package coilshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// TestLoopsKeepsLinesWholePastTheAuditCeiling checks that coilshell.Loops
// chords the radial lines of a profile reaching near its axis (ρ ∈ [0.5, 3])
// at two turns, and keeps every line as one chord at twenty turns, where
// the chorded shell would pass loftmesh.MaxLoftAuditTriangles.
func TestLoopsKeepsLinesWholePastTheAuditCeiling(t *testing.T) {
	pts := []sectionrecord.Point2{{U: 0.5}, {U: 3}, {U: 3, V: 1}, {U: 0.5, V: 1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	axis := coilshell.AxisInput{DV: 1, Side: -1}

	split, err := coilshell.Loops(profile, axis, 1.5, 2, coilshell.StationsPerTurn, coilshell.ArcChordsPerTurn, false)
	require.NoError(t, err)
	require.Greater(t, len(split.Pts), 4)
	require.Equal(t, 1, split.Segments[1].Chords, "the band parallel to the axis keeps one chord")
	n := int64(2 * coilshell.StationsPerTurn)
	require.LessOrEqual(t, 2*n*int64(len(split.Pts))+2*int64(len(split.Pts)-2), int64(loftmesh.MaxLoftAuditTriangles))

	derived, err := coilshell.Loops(profile, axis, 1.5, 2, coilshell.StationsPerTurn, coilshell.ArcChordsPerTurn, true)
	require.NoError(t, err)
	require.Len(t, derived.Pts, 4, "a derived coil asking for whole lines keeps one chord per line")

	whole, err := coilshell.Loops(profile, axis, 1.5, 20, coilshell.StationsPerTurn, coilshell.ArcChordsPerTurn, false)
	require.NoError(t, err)
	require.Len(t, whole.Pts, 4)
	n = int64(20 * coilshell.StationsPerTurn)
	require.Greater(t, 2*n*int64(len(split.Pts)), int64(loftmesh.MaxLoftAuditTriangles),
		"the two-turn chording would pass the ceiling at twenty turns")
}
