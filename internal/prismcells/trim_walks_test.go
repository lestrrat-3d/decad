package prismcells_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func squareLoop(x0, y0, x1, y1 float64) sectionrecord.LoopRecord {
	corners := [4]sectionrecord.Point2{{U: x0, V: y0}, {U: x1, V: y0}, {U: x1, V: y1}, {U: x0, V: y1}}
	segs := make([]sectionrecord.CurveSegment, 4)
	for i := range segs {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestTrimProfileWalks(t *testing.T) {
	profile := momentinput.Profile{
		Outer: squareLoop(0, 0, 4, 4),
		Holes: []sectionrecord.LoopRecord{squareLoop(1, 1, 2, 2)},
	}
	// The counters are pre-set so the result must hold only the spend of this
	// resolution, not the work's running totals.
	work := freeform.NewFreeformWork()
	work.Spent = 7
	work.ReconstructionSpent = 3

	walks, err := prismcells.TrimProfileWalks(profile, work)
	require.NoError(t, err)
	require.True(t, walks.Metered)
	require.True(t, walks.Matches(profile))
	require.Len(t, walks.Outer, 4)
	require.Len(t, walks.Holes, 1)
	require.Len(t, walks.Holes[0], 4)
	require.Equal(t, 1.0, walks.Holes[0][0].StartU)
	require.Zero(t, walks.Spent)
	require.Zero(t, walks.ReconstructionSpent)
}
