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
	return flushBossSpan(t, 10, 25, 0)
}

// flushBossSpan is the Ø6 boss about (20, 10) from z = z0 to z1, its top
// level displaced by topDelta.
func flushBossSpan(t *testing.T, z0, z1, topDelta float64) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 20, V: 10}, Radius: units.Millimeters(3),
		CCW: true, TStart: 0, TEnd: 1}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: z0, Z1: z1, Z1Delta: topDelta,
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

// TestFlushReachThroughBothCaps runs the boss from the brep plate's bottom
// z = 0 to its top z = 10, both caps flush. Cut drills the through hole: the
// plate's six faces, two of them holed, and the hole's wall. Intersect is the
// boss over [0, 10]. A Union of the boss inside is the plate itself and
// misses, and a displaced plate top or boss top misses.
func TestFlushReachThroughBothCaps(t *testing.T) {
	t.Parallel()
	out, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpCut, flushPlate(t, 0), flushBossSpan(t, 0, 10, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok)
	bp, isBrep := out.(breprecord.Payload)
	require.True(t, isBrep)
	require.Len(t, bp.Faces, 7)

	out, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpIntersect, flushPlate(t, 0), flushBossSpan(t, 0, 10, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok)
	prism, isPrism := out.(prismrecord.Payload)
	require.True(t, isPrism)
	require.Equal(t, [2]float64{0, 10}, [2]float64{prism.Z0, prism.Z1})

	_, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpUnion, flushPlate(t, 0), flushBossSpan(t, 0, 10, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.False(t, ok, `a Union of a tool inside is the target itself`)

	_, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpCut, flushPlate(t, 1e-15), flushBossSpan(t, 0, 10, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.False(t, ok, `the plate's top level is displaced`)

	_, ok, err = classbgeom.OfPayloads(t.Context(), meshbool.OpCut, flushPlate(t, 0), flushBossSpan(t, 0, 10, 1e-15), r3.Identity(), 0)
	require.NoError(t, err)
	require.False(t, ok, `the boss's top level is displaced`)
}

// squareFace is the planar face [u0, u1] × [0, 20] of the reference XY plane
// at level z, its outward normal +z when up.
func squareFace(u0, u1, z float64, up bool) classbgeom.FaceRecord {
	corners := []sectionrecord.Point2{{U: u0, V: 0}, {U: u1, V: 0}, {U: u1, V: 20}, {U: u0, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	return classbgeom.FaceRecord{
		Region: &momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}},
		Z0:     z, Z1: z, Outward: up, Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1},
	}
}

// TestFlushReachTwoFaceSides reads the side of a tool from z = 0 to 10 whose
// tube meets two faces, one at each cap level. Faces whose outward normals
// point into the tool put it outside, faces pointing away put it inside, and
// two faces that disagree, or two faces at one cap's level, miss.
func TestFlushReachTwoFaceSides(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 10, V: 10}, Radius: units.Millimeters(3),
		CCW: true, TStart: 0, TEnd: 1}
	tool := classbgeom.ToolRecord{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Z0:      0, Z1: 10, Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1},
	}
	for _, tc := range []struct {
		name  string
		faces []classbgeom.FaceRecord
		flush [2]bool
		ok    bool
		side  classbgeom.FlushSide
	}{
		{name: "bridging", faces: []classbgeom.FaceRecord{squareFace(0, 20, 0, true), squareFace(0, 20, 10, false)},
			flush: [2]bool{true, true}, ok: true, side: classbgeom.FlushOutside},
		{name: "through", faces: []classbgeom.FaceRecord{squareFace(0, 20, 0, false), squareFace(0, 20, 10, true)},
			flush: [2]bool{true, true}, ok: true, side: classbgeom.FlushInside},
		{name: "sides_disagree", faces: []classbgeom.FaceRecord{squareFace(0, 20, 0, false), squareFace(0, 20, 10, false)},
			flush: [2]bool{true, true}},
		{name: "one_cap_twice", faces: []classbgeom.FaceRecord{squareFace(0, 10, 10, true), squareFace(10, 20, 10, true)},
			flush: [2]bool{false, true}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			slabs, side, ok, err := classbgeom.FlushReach(t.Context(), tc.faces, tool, tc.flush)
			require.NoError(t, err)
			require.Equal(t, tc.ok, ok)
			if !tc.ok {
				return
			}
			require.Len(t, slabs, 2)
			require.Equal(t, tc.side, side)
		})
	}
}
