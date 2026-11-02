package stackedrecord

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestReplayBlindSourceWalksUsesProducedArcAndRefusesChangedRecord(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	right, left := sectionrecord.Point2{U: 1}, sectionrecord.Point2{U: -1}
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Start: right, End: left, TEnd: 1},
		sectionrecord.LineSeg{Start: left, End: right, TEnd: 1},
	}}
	input := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: 2,
		Profile: momentinput.Profile{Outer: loop}}
	body, err := prismrecord.Build(t.Context(),
		topo.Env{Doc: &topo.Document{}, Alloc: topotest.Alloc{}}, input, freeform.NewFreeformWork())
	require.NoError(t, err)
	source, ok := body.Payload.(prismrecord.Payload)
	require.True(t, ok)
	require.NotNil(t, source.Walks)
	newHole := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Center: sectionrecord.Point2{}, Radius: units.Millimeters(0.1),
			TStart: 1, CCW: false},
	}}
	columns := []WallColumn{{Column: Column{Loop: loop}}, //nolint:modernize // Named fields leave the remaining fields zero.
		{Column: Column{Loop: newHole, LoopIndex: 1}}} //nolint:modernize // Named fields leave the remaining fields zero.
	budget := proofbound.NewReplayTrialBudget()
	walks, ok, err := replayBlindSourceWalks(t.Context(), columns, source, budget)
	require.NoError(t, err)
	require.True(t, ok)
	require.Same(t, source.Walks, walks[0])
	require.Nil(t, walks[1])

	changed := source
	changed.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), source.Profile.Outer.Segments...)
	arc := changed.Profile.Outer.Segments[0].(sectionrecord.ArcSeg)
	arc.Start.V = math.Copysign(0, -1)
	changed.Profile.Outer.Segments[0] = arc
	walks, ok, err = replayBlindSourceWalks(t.Context(), columns, changed, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, walks)
}
