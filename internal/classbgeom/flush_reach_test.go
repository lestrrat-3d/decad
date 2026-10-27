package classbgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// flushPlate is the brep face view of a 40×20 plate on XY from z = 0 to 10,
// its top level displaced by topDelta.
func flushPlate(t *testing.T, topDelta float64) breprecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 40, V: 0}, {U: 40, V: 20}, {U: 0, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	bp, err := breprecord.OfPrism(prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 10, Z1Delta: topDelta,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}})
	require.NoError(t, err)
	return bp
}

// flushBoss is a Ø6 boss about (20, 10) from z = 10 to 25, its base cap in
// the plate's top plane.
func flushBoss(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 20, V: 10}, Radius: units.Millimeters(3),
		CCW: true, TStart: 0, TEnd: 1}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 10, Z1: 25,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}}
}

// TestFlushReachNeedsUndisplacedPlane builds the boss standing on the brep
// plate, and misses when the plate's top level carries a displacement or the
// caller charges one beyond the records: either can open a gap the shared
// float level does not show.
func TestFlushReachNeedsUndisplacedPlane(t *testing.T) {
	t.Parallel()
	out, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpUnion, flushPlate(t, 0), flushBoss(t), r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok)
	bp, isBrep := out.(breprecord.Payload)
	require.True(t, isBrep)
	require.Len(t, bp.Faces, 8)

	_, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpUnion, flushPlate(t, 1e-15), flushBoss(t), r3.Identity(), 0)
	require.NoError(t, err)
	require.False(t, ok, `the plate's top level is displaced`)

	_, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpUnion, flushPlate(t, 0), flushBoss(t), r3.Identity(), 1e-9)
	require.NoError(t, err)
	require.False(t, ok, `the caller charges a displacement beyond the records`)
}
