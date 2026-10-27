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

// walkTool is a Ø6 tool about (10, 10) from z = z0 to z1, its top level
// displaced by topDelta.
func walkTool(z0, z1, topDelta float64) classbgeom.ToolRecord {
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 10, V: 10}, Radius: units.Millimeters(3),
		CCW: true, TStart: 0, TEnd: 1}
	return classbgeom.ToolRecord{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}},
		Z0:      z0, Z1: z1, Z1Delta: topDelta, Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1},
	}
}

// TestWalkReachReadsLayers walks a tool from z = 0 to 20 through faces at
// z = 0, 5, 12 and 20 whose outward normals alternate, and reads each layer
// of the tube. The walk misses when a face meeting the tube runs along the
// sweep, when two faces share a level, when
// consecutive faces point the same way, when a face level or the tool's top
// carries a displacement, and when the tool's interval runs backwards.
func TestWalkReachReadsLayers(t *testing.T) {
	t.Parallel()
	faces := func(f ...classbgeom.FaceRecord) []classbgeom.FaceRecord { return f }
	layered := faces(squareFace(0, 20, 12, true), squareFace(0, 20, 0, true), squareFace(0, 20, 5, false),
		squareFace(0, 20, 20, false))
	w, ok, err := classbgeom.WalkReach(t.Context(), layered, walkTool(0, 20, 0))
	require.NoError(t, err)
	require.True(t, ok)
	levels := make([]float64, len(w.Slabs))
	for i, s := range w.Slabs {
		levels[i] = s.GLevel
	}
	require.Equal(t, []float64{0, 5, 12, 20}, levels)
	require.Equal(t, []bool{true, false, true, false, true}, w.Inside)
	require.True(t, w.LowFlush())
	require.True(t, w.HighFlush())

	// A wall at x = 8 runs along the sweep through the tube's box.
	along := classbgeom.FaceRecord{
		Wall: sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 8, V: 0}, End: sectionrecord.Point2{U: 8, V: 20},
			TStart: 0, TEnd: 1},
		Z0: 0, Z1: 20, Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1},
	}
	displaced := squareFace(0, 20, 12, true)
	displaced.Z0Delta, displaced.Z1Delta = 1e-15, 1e-15
	for _, tc := range []struct {
		name  string
		faces []classbgeom.FaceRecord
		tool  classbgeom.ToolRecord
	}{
		{name: "face_along_sweep", faces: faces(squareFace(0, 20, 5, false), squareFace(0, 20, 12, true), along),
			tool: walkTool(0, 20, 0)},
		{name: "one_level_twice", faces: faces(squareFace(0, 10, 5, true), squareFace(10, 20, 5, false)),
			tool: walkTool(0, 20, 0)},
		{name: "same_sense", faces: faces(squareFace(0, 20, 5, true), squareFace(0, 20, 12, true)),
			tool: walkTool(0, 20, 0)},
		{name: "displaced_face", faces: faces(squareFace(0, 20, 5, false), displaced), tool: walkTool(0, 20, 0)},
		{name: "displaced_tool", faces: faces(squareFace(0, 20, 5, false), squareFace(0, 20, 12, true)),
			tool: walkTool(0, 20, 1e-15)},
		{name: "backwards", faces: faces(squareFace(0, 20, 5, false), squareFace(0, 20, 12, true)),
			tool: walkTool(20, 0, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok, err := classbgeom.WalkReach(t.Context(), tc.faces, tc.tool)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
}

// TestWalkBossThroughBrepPlate runs a Ø6 boss from the brep plate's bottom
// z = 0, flush, through its top z = 10 to z = 15. The flush reach misses it,
// because the top meets the tube strictly inside its interval, and the walk
// builds it. Union keeps the bottom whole, holes the top and adds the boss's
// wall and cap over [10, 15]: 8 faces. Cut holes both caps with the hole's
// wall between: 7 faces. Intersect is the boss over [0, 10]. A displaced
// plate top misses.
func TestWalkBossThroughBrepPlate(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		op    meshbool.OperationKind
		faces int
	}{
		{op: meshbool.OpUnion, faces: 8},
		{op: meshbool.OpCut, faces: 7},
	} {
		out, ok, err := classbgeom.OfPayloads(t.Context(), tc.op, flushPlate(t, 0), flushBossSpan(t, 0, 15, 0), r3.Identity(), 0)
		require.NoError(t, err)
		require.True(t, ok)
		bp, isBrep := out.(breprecord.Payload)
		require.True(t, isBrep)
		require.Len(t, bp.Faces, tc.faces)

		_, ok, err = classbgeom.OfPayloads(t.Context(), tc.op, flushPlate(t, 1e-15), flushBossSpan(t, 0, 15, 0), r3.Identity(), 0)
		require.NoError(t, err)
		require.False(t, ok, `the plate's top level is displaced`)
	}
	out, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpIntersect, flushPlate(t, 0), flushBossSpan(t, 0, 15, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.True(t, ok)
	prism, isPrism := out.(prismrecord.Payload)
	require.True(t, isPrism)
	require.Equal(t, [2]float64{0, 10}, [2]float64{prism.Z0, prism.Z1})
}

// TestWalkNeedsSectionInsideEachFace adds two faces at z = 4 and 6 to the
// brep plate whose regions, 2 mm squares about the boss's axis, do not hold
// its Ø6 section. Their senses alternate with the plate's caps, so the walk
// reads the layers, but the perpendicular-face scene finds no clean nesting
// and the pair misses.
func TestWalkNeedsSectionInsideEachFace(t *testing.T) {
	t.Parallel()
	plate := flushPlate(t, 0)
	var top breprecord.Face
	for _, f := range plate.Faces {
		if f.Region != nil && f.Z0 == 10 {
			top = f
		}
	}
	require.NotNil(t, top.Region)
	corners := []sectionrecord.Point2{{U: 19, V: 9}, {U: 21, V: 9}, {U: 21, V: 11}, {U: 19, V: 11}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	small := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	for _, level := range []struct {
		z  float64
		up bool
	}{{4, true}, {6, false}} {
		f := top
		region := small
		f.Region, f.Z0, f.Z1, f.Outward = &region, level.z, level.z, level.up
		plate.Faces = append(plate.Faces, f)
	}
	_, ok, err := classbgeom.OfPayloads(t.Context(), meshbool.OpCut, plate, flushBossSpan(t, -5, 15, 0), r3.Identity(), 0)
	require.NoError(t, err)
	require.False(t, ok)
}
