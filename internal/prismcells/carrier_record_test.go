package prismcells_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func cutPayload(t *testing.T, p momentinput.Profile) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return prismrecord.Payload{Profile: p, Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 1}
}

func rect(u0, v0, u1, v1 float64) momentinput.Profile {
	l := func(a, b momentinput.Point2) momentinput.CurveSegment {
		return momentinput.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		l(capPt(u0, v0), capPt(u1, v0)), l(capPt(u1, v0), capPt(u1, v1)),
		l(capPt(u1, v1), capPt(u0, v1)), l(capPt(u0, v1), capPt(u0, v0)),
	}}}
}

// TestPairUnionRecord checks §5.2: a disc united with a tooth records zero
// carriers, a junction entry at each cut, and a SectionDelta raised to the
// trimmed circle's Lemma H bound.
func TestPairUnionRecord(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	disc := cutPayload(t, momentinput.Profile{Outer: momentinput.LoopRecord{
		Segments: []momentinput.CurveSegment{capCircle(0, 0, 10, true)}}})
	tooth := cutPayload(t, rect(8, -1, 14, 1))
	re, err := prismcells.NewReexpression(disc.PlacementOperand(), tooth.PlacementOperand())
	require.NoError(t, err)
	got, ok, err := prismcells.ResolveAndBuildUnion(t.Context(), budget, disc, tooth, re)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, got.Carriers)
	cr, valid, err := got.CarrierRecord(budget)
	require.NoError(t, err)
	require.True(t, valid, "the record is valid against its own payload (CR4 included)")
	for l := range cr.Segment {
		for _, eps := range cr.Segment[l] {
			require.Zero(t, eps, "every carrier is an operand's own record")
		}
	}
	require.GreaterOrEqual(t, got.SectionDelta, cr.HMax())
	require.Greater(t, cr.HMax(), 0.0)
	// The trimmed circle's h is twice its larger junction entry.
	maxJunction := 0.0
	for _, d := range cr.Junction[0] {
		maxJunction = math.Max(maxJunction, d)
	}
	require.Greater(t, maxJunction, 0.0, "a cut junction carries its cut allowance")
	require.InDelta(t, 2*maxJunction, cr.HMax(), 1e-12*maxJunction+1e-300)

	t.Run("a displaced operand keeps no record", func(t *testing.T) {
		moved := tooth
		moved.SectionDelta = 1e-13
		got, ok, err := prismcells.ResolveAndBuildUnion(t.Context(), budget, disc, moved, re)
		require.NoError(t, err)
		require.True(t, ok, "the union still builds")
		require.Nil(t, got.Carriers)
	})
}

// TestCleanNestingCutRecord checks §5.4: a clean-nesting Cut of a displaced
// recorded target maps every entry through the entity tags, gives the tool's
// loop zero entries, and keeps no record when the tool sits within the floor
// of the displaced loop.
func TestCleanNestingCutRecord(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	const delta = 1e-9
	target := cutPayload(t, rect(0, 0, 20, 20))
	cr := prismrecord.UniformCarrierRecord(target.Profile, delta)
	// Distinct entries show the tag mapping: segment j carries 1e-9·(j+1) with its junction.
	for j := range cr.Segment[0] {
		cr.Segment[0][j] = delta * float64(j+1) / 4
		cr.Junction[0][j] = delta * float64(j+1) / 4
	}
	target.SectionDelta, target.Carriers = delta, &cr
	re, err := prismcells.NewReexpression(target.PlacementOperand(), target.PlacementOperand())
	require.NoError(t, err)

	tool := cutPayload(t, momentinput.Profile{Outer: momentinput.LoopRecord{
		Segments: []momentinput.CurveSegment{capCircle(10, 10, 3, true)}}})
	got, ok, err := prismcells.ResolveAndBuildCut(t.Context(), budget, target, tool, re)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, got.Carriers)
	rec, valid, err := got.CarrierRecord(budget)
	require.NoError(t, err)
	require.True(t, valid)
	require.Len(t, rec.Segment, 2)
	require.Zero(t, rec.Segment[1][0], "the tool's loop carries no displacement")
	require.Zero(t, rec.Junction[1][0])
	seen := map[float64]bool{}
	for j := range rec.Segment[0] {
		seen[rec.Segment[0][j]] = true
	}
	for j := range 4 {
		require.True(t, seen[delta*float64(j+1)/4], "target segment %d's entry maps to the result", j)
	}

	t.Run("a tool within the floor of the displaced loop is a miss with no record", func(t *testing.T) {
		near := cutPayload(t, momentinput.Profile{Outer: momentinput.LoopRecord{
			Segments: []momentinput.CurveSegment{capCircle(10, 10, 10-1e-12, true)}}})
		got, ok, err := prismcells.ResolveAndBuildCut(t.Context(), budget, target, near, re)
		require.NoError(t, err)
		require.False(t, ok, "the audit refuses the near tool")
		require.Nil(t, got.Carriers)
	})
}

// TestCleanNestingCutRecordNarrowedLines is §5.4 for a target whose lines are
// narrowed (a trimmed slot's flats, TStart and TEnd inside 0 and 1) beside
// whole arcs. Each junction there pairs a line's exact end with an arc's end
// read verbatim in both records, so its path distance is the line end's alone
// and the record is kept, with each target entry mapped through.
func TestCleanNestingCutRecordNarrowedLines(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	const delta = 1e-9
	flat := func(a, b momentinput.Point2) momentinput.CurveSegment {
		return momentinput.LineSeg{Start: a, End: b, TStart: 0.25, TEnd: 0.75}
	}
	slot := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.ArcSeg{Center: capPt(0, 0), Start: capPt(0, 5), End: capPt(0, -5), TStart: 0, TEnd: 1},
		flat(capPt(-10, -5), capPt(30, -5)),
		momentinput.ArcSeg{Center: capPt(20, 0), Start: capPt(20, -5), End: capPt(20, 5), TStart: 0, TEnd: 1},
		flat(capPt(30, 5), capPt(-10, 5)),
	}}}
	target := cutPayload(t, slot)
	cr := prismrecord.UniformCarrierRecord(slot, delta)
	for j := range cr.Segment[0] {
		cr.Segment[0][j] = delta * float64(j+1) / 4
		cr.Junction[0][j] = delta * float64(j+1) / 4
	}
	cr.SectionDelta = 1e-7 // above every loop's Lemma H bound (CR4)
	target.SectionDelta, target.Carriers = 1e-7, &cr
	re, err := prismcells.NewReexpression(target.PlacementOperand(), target.PlacementOperand())
	require.NoError(t, err)
	tool := cutPayload(t, momentinput.Profile{Outer: momentinput.LoopRecord{
		Segments: []momentinput.CurveSegment{capCircle(10, 0, 2, true)}}})
	got, ok, err := prismcells.ResolveAndBuildCut(t.Context(), budget, target, tool, re)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, got.Carriers, "a narrowed line beside an arc keeps the record")
	rec, valid, err := got.CarrierRecord(budget)
	require.NoError(t, err)
	require.True(t, valid)
	require.Len(t, rec.Segment, 2)
	require.Len(t, rec.Segment[0], 4)
	for j, seg := range got.Profile.Outer.Segments {
		switch seg.(type) {
		case momentinput.ArcSeg: // a whole arc keeps its target entry (arcs carry delta/4 and 3*delta/4)
			require.True(t, math.Abs(rec.Segment[0][j]-delta/4) < 1e-22 || math.Abs(rec.Segment[0][j]-3*delta/4) < 1e-22,
				"arc %d's carrier entry %g", j, rec.Segment[0][j])
		default: // a narrowed flat: its end junctions' entry (2 and 4 times delta/4) plus the walk allowance
			require.GreaterOrEqual(t, rec.Segment[0][j], delta/2, "flat %d's carrier entry", j)
			require.InDelta(t, delta, rec.Segment[0][j], delta/2, "flat %d's carrier entry", j)
		}
		require.Positive(t, rec.Junction[0][j], "junction %d", j)
		require.Less(t, rec.Junction[0][j], 2*delta, "junction %d adds a path distance of zero", j)
	}
}
