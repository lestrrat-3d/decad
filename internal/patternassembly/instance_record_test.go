package patternassembly_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/patternassembly"
	"github.com/lestrrat-3d/decad/internal/patternrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func pt(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }

func line(a, b sectionrecord.Point2) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
}

// instance builds instance 1 of a 17-fold turn of payload about the Z axis.
func instance(t *testing.T, p prismrecord.Payload) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	p.Frame, p.Xform, p.Z0, p.Z1 = frame, r3.Identity(), 0, 1
	rp, err := patternrecord.ResolveCircular(r3.Vec{}, r3.NewVec(0, 0, 1), 17)
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	body, fallback, err := patternassembly.FrameKeepingInstance(t.Context(), env, rp, p, 1)
	require.NoError(t, err)
	require.False(t, fallback)
	got, ok := body.Payload.(prismrecord.Payload)
	require.True(t, ok)
	return got
}

func tooth() momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt(30, 0), pt(34, 1)), line(pt(34, 1), pt(34, 3)), line(pt(34, 3), pt(30, 4)), line(pt(30, 4), pt(30, 0)),
	}}}
}

func TestInstanceRecordOfATooth(t *testing.T) {
	got := instance(t, prismrecord.Payload{Profile: tooth()})
	require.NotZero(t, got.SectionDelta)
	cr, ok, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, ok)
	for l := range cr.Segment {
		for j := range cr.Segment[l] {
			require.Equal(t, got.SectionDelta, cr.Segment[l][j])
			require.Equal(t, got.SectionDelta, cr.Junction[l][j])
		}
	}
	require.Equal(t, got.SectionDelta, cr.HMax(), "an all-line instance keeps its charge")

	t.Run("a displaced source gets none", func(t *testing.T) {
		got := instance(t, prismrecord.Payload{Profile: tooth(), SectionDelta: 1e-13})
		require.Nil(t, got.Carriers)
	})
	t.Run("two narrowed lines at distinct read ends get none", func(t *testing.T) {
		p := tooth()
		narrowed := p.Outer.Segments[0].(sectionrecord.LineSeg)
		narrowed.TEnd = 0.9
		p.Outer.Segments[0] = narrowed
		got := instance(t, prismrecord.Payload{Profile: p})
		require.NotZero(t, got.SectionDelta)
		require.Nil(t, got.Carriers)
	})
}

func TestInstanceRecordOfAnArcSource(t *testing.T) {
	const r = 20.0
	end := pt(r*math.Cos(1), r*math.Sin(1))
	p := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: pt(5, 0), Start: pt(5+r, 0), End: pt(5+end.U, end.V), TStart: 0, TEnd: 1},
		line(pt(5+end.U, end.V), pt(5+r, 0)),
	}}}
	got := instance(t, prismrecord.Payload{Profile: p})
	cr, ok, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, ok)
	require.Greater(t, cr.Junction[0][0], cr.Segment[0][0], "the arc's end junction takes delta_End")
	require.Equal(t, cr.Segment[0][0], cr.Junction[0][1])
	require.GreaterOrEqual(t, got.SectionDelta, cr.HMax(), "CR4: the displacement covers every h_loop")
	require.GreaterOrEqual(t, got.SectionDelta, cr.Max())
}

// TestInstanceRecordOfAClockwiseArc places delta_End at the arc's End field,
// which a clockwise arc (TStart 1) walks first: the junction before it.
func TestInstanceRecordOfAClockwiseArc(t *testing.T) {
	const r = 20.0
	end := pt(r*math.Cos(1), r*math.Sin(1))
	// A clockwise loop is a hole: the arc is walked End to Start.
	hole := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: pt(5, 0), Start: pt(5+r, 0), End: pt(5+end.U, end.V), TStart: 1, TEnd: 0},
		line(pt(5+r, 0), pt(5+end.U, end.V)),
	}}
	p := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line(pt(-60, -60), pt(60, -60)), line(pt(60, -60), pt(60, 60)), line(pt(60, 60), pt(-60, 60)), line(pt(-60, 60), pt(-60, -60)),
	}}, Holes: []sectionrecord.LoopRecord{hole}}
	got := instance(t, prismrecord.Payload{Profile: p})
	cr, ok, err := got.CarrierRecord(proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.True(t, ok)
	// Segment 0 is the arc; its End is its walk start, the junction before it (1).
	require.Greater(t, cr.Junction[1][1], cr.Junction[1][0])
}
