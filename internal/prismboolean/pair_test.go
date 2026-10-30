package prismboolean_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// square is the counter-clockwise loop of the axis-aligned square [u, u+side] × [v, v+side].
func square(u, v, side float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: u, V: v}, {U: u + side, V: v}, {U: u + side, V: v + side}, {U: u, V: v + side}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		segs[i] = sectionrecord.LineSeg{Start: p, End: pts[(i+1)%len(pts)], TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// prism sweeps profile along +Z over [0, height] in the world XY frame.
func prism(t *testing.T, profile momentinput.Profile, height float64) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return prismrecord.Payload{Profile: profile, Frame: frame, Xform: r3.Identity(), Z1: height}
}

// requireArea checks the absolute enclosed area of loop against want within its bound.
func requireArea(t *testing.T, want float64, loop sectionrecord.LoopRecord) {
	t.Helper()
	area, err := momentinput.LoopEnclosedArea(t.Context(), loop)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(math.Abs(area.Value)-want), area.Bound)
}

// TestTryHoledIntersect checks the one-hole Intersect route Try runs ahead of
// prismcells.TryAnalytic. The shared Intersect admission is hole-free, so it
// misses this pair, and Try must answer it with the ring itself, in either
// operand order.
func TestTryHoledIntersect(t *testing.T) {
	t.Parallel()
	hole, err := offset2d.ReverseLoopRecordContext(t.Context(), square(4, 4, 2))
	require.NoError(t, err)
	plate := prism(t, momentinput.Profile{Outer: square(0, 0, 10)}, 4)
	ring := prism(t, momentinput.Profile{Outer: square(2, 2, 6), Holes: []sectionrecord.LoopRecord{hole}}, 4)

	_, ok, err := prismcells.TryAnalytic(t.Context(), meshbool.OpIntersect, plate, ring)
	require.NoError(t, err)
	require.False(t, ok, "the shared Intersect admission is hole-free")

	for name, pair := range map[string][2]prismrecord.Payload{"plate_first": {plate, ring}, "ring_first": {ring, plate}} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			got, ok, err := prismboolean.Try(t.Context(), meshbool.OpIntersect, pair[0], pair[1])
			require.NoError(t, err)
			require.True(t, ok)
			require.Equal(t, 0.0, got.Z0)
			require.Equal(t, 4.0, got.Z1)
			require.Len(t, got.Profile.Holes, 1)
			requireArea(t, 36, got.Profile.Outer)
			requireArea(t, 4, got.Profile.Holes[0])
		})
	}
}

// TestTryStackedUnion checks A1: a plate and a taller boss over the same base
// level union into two slabs, the plate's section below its top and the
// boss's above. Equal intervals are the prism Union row and miss silently.
func TestTryStackedUnion(t *testing.T) {
	t.Parallel()
	plate := prism(t, momentinput.Profile{Outer: square(0, 0, 10)}, 4)
	boss := prism(t, momentinput.Profile{Outer: square(2, 2, 6)}, 6)

	payload, ok, err := prismboolean.TryStackedUnion(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	stack, isStack := payload.(stackedrecord.Payload)
	require.True(t, isStack, "a plate and a boss with one region per slab stay a stacked record")
	require.Len(t, stack.Slabs, 2)
	require.Equal(t, [2]float64{0, 4}, [2]float64{stack.Slabs[0].Z0, stack.Slabs[0].Z1})
	require.Equal(t, [2]float64{4, 6}, [2]float64{stack.Slabs[1].Z0, stack.Slabs[1].Z1})
	require.Len(t, stack.Slabs[0].Regions, 1)
	require.Len(t, stack.Slabs[1].Regions, 1)
	requireArea(t, 100, stack.Slabs[0].Regions[0].Outer)
	requireArea(t, 36, stack.Slabs[1].Regions[0].Outer)

	_, ok, err = prismboolean.TryStackedUnion(t.Context(), plate, prism(t, boss.Profile, 4))
	require.NoError(t, err)
	require.False(t, ok, "equal intervals are not a stack")
}

// rect is the counter-clockwise loop of the axis-aligned rectangle
// [u0, u1] × [v0, v1].
func rect(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		segs[i] = sectionrecord.LineSeg{Start: p, End: pts[(i+1)%len(pts)], TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// TestTryStackedUnionLBracketFaceDelta checks that an axis-aligned A1 BRep
// charges the slab-merge cut only to its kept stack. The upright's wall meets
// the base's side at (8, 0), a cut at t = 2/15 that falls back to the cut
// allowance of a 60 mm carrier. No face coordinate reads that parameter, so
// every face is exact and a modify op may take it for the face it denotes.
func TestTryStackedUnionLBracketFaceDelta(t *testing.T) {
	t.Parallel()
	base := prism(t, momentinput.Profile{Outer: rect(0, 0, 60, 40)}, 8)
	upright := prism(t, momentinput.Profile{Outer: rect(0, 0, 8, 40)}, 50)

	payload, ok, err := prismboolean.TryStackedUnion(t.Context(), base, upright)
	require.NoError(t, err)
	require.True(t, ok)
	bp, isBrep := payload.(breprecord.Payload)
	require.True(t, isBrep, "the L bracket's crossing slab is a brep record")
	require.Zero(t, bp.SectionDelta(), "no face coordinate reads a merge's cut parameter")
	require.NotNil(t, bp.Stack)
	cut := proofbound.CutDisplacementAllow(60)
	require.Positive(t, cut)
	require.GreaterOrEqual(t, bp.Stack.Delta, cut, "the kept slab fragments still carry the merge's cut charge")
	require.InEpsilon(t, cut, bp.Stack.Delta, 1e-9, "the stack adds no charge beyond the cut and the vertex allowance")
}

// roundBoss is the full circle of radius r about (u, v), counter-clockwise.
func roundBoss(u, v, r float64) sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Center: sectionrecord.Point2{U: u, V: v}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1,
	}}}
}

// TestTryStackedUnionRoundBossCrossingFaceDelta checks the invariant that lets
// an axis-aligned A1 BRep charge its faces only the vertex allowance: where a
// slab merge's cut line end becomes a line-circle vertex, the vertex carries
// that cut's charge itself. A round boss r = 4 about (20, 10) crosses the
// plate's x = 20 side at the cut parameters 0.3 and 0.7 of a 20 mm carrier,
// which no float holds exactly.
func TestTryStackedUnionRoundBossCrossingFaceDelta(t *testing.T) {
	t.Parallel()
	plate := prism(t, momentinput.Profile{Outer: rect(0, 0, 20, 20)}, 4)
	boss := prism(t, momentinput.Profile{Outer: roundBoss(20, 10, 4)}, 6)

	payload, ok, err := prismboolean.TryStackedUnion(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, ok)
	bp, isBrep := payload.(breprecord.Payload)
	require.True(t, isBrep, "a hole-free line-circle slab is a brep record")
	require.GreaterOrEqual(t, bp.SectionDelta(), proofbound.CutDisplacementAllow(20),
		"the line-circle vertex carries its cut end's displacement")
}
