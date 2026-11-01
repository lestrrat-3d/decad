package prismcells

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestWholeArcIdentity(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-10, -10, 15, 20)
	s.Fix(rect.A)
	slot, err := s.CreateSlot(-5, -6, -5, -4, 1.1)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var host *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			host = p
		}
	}
	require.NotNil(t, host)
	outer, holes, err := sketchrecord.RecordProfileLoops(host)
	require.NoError(t, err)
	target := momentinput.Profile{Outer: outer, Holes: holes}
	group := []momentinput.Profile{
		{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 3, V: 0}, Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}},
		{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 3, V: 5}, Radius: units.Millimeters(1), CCW: true, TEnd: 1}}}},
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	place := prismplacement.Operand{Frame: frame, Xform: r3.Identity()}
	meter := proofbound.NewReplayTrialBudget()
	scene, ok, err := BuildGroupScene(t.Context(), meter.WorkBudget(t.Context()), []momentinput.Profile{target}, group, place, place, 0, 0, "cut")
	require.NoError(t, err)
	require.True(t, ok)
	match, ok, err := GroupCutMatch(meter.WorkBudget(t.Context()), scene.Profiles, scene.Tags, len(target.Holes), len(group))
	require.NoError(t, err)
	require.True(t, ok)
	bound, ok, err := replayWholeArcIdentity(t.Context(), target, match, scene.Tags, meter)
	require.NoError(t, err)
	require.True(t, ok)
	require.GreaterOrEqual(t, bound, 0.0)

	baseline := proofbound.NewReplayTrialBudget()
	_, identity, err := replayWholeArcIdentity(t.Context(), target, match, scene.Tags, baseline)
	require.NoError(t, err)
	require.True(t, identity)
	work, records := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		w, r, b uint64
		pass    bool
	}{{work, records, bytes, true}, {work - 1, records, bytes, false}, {work, records - 1, bytes, false}, {work, records, bytes - 1, false}} {
		meter := &arcChargeMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
		_, accepted, e := replayWholeArcIdentity(t.Context(), target, match, scene.Tags, meter)
		if tc.pass {
			require.NoError(t, e)
			require.True(t, accepted)
		} else {
			require.ErrorIs(t, e, errOriginLimit)
			require.False(t, accepted)
		}
	}
	changed := target
	changed.Holes = append([]sectionrecord.LoopRecord(nil), target.Holes...)
	changed.Holes[0].Segments = append([]sectionrecord.CurveSegment(nil), target.Holes[0].Segments...)
	for i, segment := range changed.Holes[0].Segments {
		if arc, yes := segment.(sectionrecord.ArcSeg); yes {
			arc.Start, arc.End = arc.End, arc.Start
			changed.Holes[0].Segments[i] = arc
			break
		}
	}
	_, ok, err = replayWholeArcIdentity(t.Context(), changed, match, scene.Tags, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	var entity sketch.Entity
	var origin Origin
	for e, o := range scene.Tags {
		if !o.IsB {
			if _, yes := e.(*sketch.Arc); yes {
				entity, origin = e, o
				break
			}
		}
	}
	require.NotNil(t, entity)
	corrupt := origin
	corrupt.Segment = 999999
	scene.Tags[entity] = corrupt
	_, ok, err = replayWholeArcIdentity(t.Context(), target, match, scene.Tags, proofbound.NewReplayTrialBudget())
	scene.Tags[entity] = origin
	require.NoError(t, err)
	require.False(t, ok)
}
