package breprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// boxRecord is a 10x20x30 box stated as a brep record: one swept face per
// side of its counter-clockwise section, then the bottom and top caps.
func boxRecord(t *testing.T) breprecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 20}, {U: 0, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1})
	}
	region := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
	bp := breprecord.Payload{Xform: r3.Identity()}
	for _, seg := range segs {
		bp.Faces = append(bp.Faces, breprecord.Face{Frame: frame, Wall: seg, Z0: 0, Z1: 30})
	}
	bp.Faces = append(bp.Faces,
		breprecord.Face{Frame: frame, Region: &region, Z0: 0, Z1: 0},
		breprecord.Face{Frame: frame, Region: &region, Outward: true, Z0: 30, Z1: 30})
	bp.AssignRoles()
	return bp
}

func TestBuildBoxRecord(t *testing.T) {
	t.Parallel()
	bp := boxRecord(t)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	body, err := breprecord.Build(t.Context(), env, bp)
	require.NoError(t, err)
	require.Same(t, env.Doc, body.Doc)
	require.True(t, body.Solid)
	faces := topobuild.Faces(body)
	require.Len(t, faces, 6)
	require.Equal(t, "wall(0)", faces[0].Origins[0].Role)
	require.Equal(t, "face(5)", faces[5].Origins[0].Role)
	for _, e := range topobuild.Edges(body) {
		require.Len(t, e.Faces, 2)
	}
	require.Len(t, topobuild.Vertices(body), 8)
	require.Equal(t, 6000.0, body.Volume.Value.Base())
	require.Equal(t, 2200.0, body.Area.Value.Base())
	require.Equal(t, r3.NewVec(5, 10, 15), body.Centroid.Value)
	require.Equal(t, r3.NewVec(0, 0, 0), body.Bounds.Min)
	require.Equal(t, r3.NewVec(10, 20, 30), body.Bounds.Max)
	require.Equal(t, bp, body.Payload)
}

func TestPlacedMovesTheRecord(t *testing.T) {
	t.Parallel()
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	placed, err := boxRecord(t).Placed(t.Context(), env, shift)
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(100, 0, 0), placed.Bounds.Min)
	require.Equal(t, r3.NewVec(110, 20, 30), placed.Bounds.Max)
	require.Equal(t, r3.NewVec(105, 10, 15), placed.Centroid.Value)
	require.Equal(t, shift, placed.Payload.Transform())
}
