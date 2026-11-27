package loftrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDirectSectionsBindBothNativeLoops(t *testing.T) {
	b := new(topo.Body)
	profile := func() momentinput.Profile {
		return momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{momentinput.LineSeg{End: momentinput.Point2{U: 1}, TEnd: 1}}}}
	}
	p := Payload{Profile0: profile(), Profile1: profile(), AuthenticatedReconstruction: true, Xform: r3.Identity(), Alignment: []int{0}}
	p.SourceIdentity = &SourceIdentity{direct: sealDirectSections(b, p)}
	require.True(t, p.MatchesDirectSections(b))
	require.False(t, p.MatchesDirectSections(new(topo.Body)))
	p.Alignment[0] = 1
	require.False(t, p.MatchesDirectSections(b))
	p.Alignment[0] = 0
	p.Profile1.Outer.Segments[0] = momentinput.LineSeg{End: momentinput.Point2{U: 2}, TEnd: 1}
	require.False(t, p.MatchesDirectSections(b))
}

func TestDirectFaceSourceCell(t *testing.T) {
	body := new(topo.Body)
	body.Origin.Producer = 7
	profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{End: momentinput.Point2{U: 1}, TEnd: 1},
		momentinput.LineSeg{Start: momentinput.Point2{U: 1}, End: momentinput.Point2{U: 2}, TEnd: 1},
	}}}
	p := Payload{Profile0: profile, Profile1: profile, AuthenticatedReconstruction: true, Xform: r3.Identity()}
	p.SourceIdentity = &SourceIdentity{Origin: 7, direct: sealDirectSections(body, p)}
	require.NoError(t, sealDirectCellSegments(p.SourceIdentity.direct, [][]int{{0, 1}}, 2, proofbound.NewWorkBudget(t.Context())))
	caps := [2]*topo.Face{{Body: body}, {Body: body}}
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 1, 2}, {1, 0, 3}, {0, 2, 3}, {1, 3, 2}, {0, 2, 1}, {1, 2, 3}}
	walls := make([]*topo.Face, 4)
	for i, tri := range tris[:len(walls)] {
		points := [3]*topo.Vertex{{Position: verts[tri[0]]}, {Position: verts[tri[1]]}, {Position: verts[tri[2]]}}
		uses := make([]topo.CoEdge, 3)
		for j := range uses {
			uses[j] = topo.CoEdge{Edge: &topo.Edge{Start: points[j], End: points[(j+1)%3]}, Forward: true}
		}
		walls[i] = &topo.Face{Body: body, Loops: []*topo.Loop{{CoEdges: uses}}}
	}
	cells := [][2]int{{0, 0}, {0, 0}, {0, 1}, {0, 1}}
	sides := []uint8{0, 1, 0, 1}
	err := sealDirectFaces(p.SourceIdentity.direct, body, caps, walls, cells, sides, verts, tris, 1, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	segment, ok := p.DirectWallFaceSourceSegment(body, walls[2], 1, 0)
	require.True(t, ok)
	require.Equal(t, 1, segment)
	_, ok = p.DirectWallFaceSourceSegment(body, walls[3], 1, 0)
	require.False(t, ok)
	_, ok = p.DirectWallFaceSourceSegment(body, &topo.Face{Body: body}, 1, 0)
	require.False(t, ok)
	require.True(t, p.DirectCapFaceMatches(body, caps[0], 0))
	require.False(t, p.DirectCapFaceMatches(body, caps[1], 0))
	start, end, ok := p.DirectCapFacetRange(body, 0)
	require.True(t, ok)
	require.Equal(t, 4, start)
	require.Equal(t, 5, end)
	start, end, ok = p.DirectCapFacetRange(body, 1)
	require.True(t, ok)
	require.Equal(t, 5, start)
	require.Equal(t, 6, end)
	_, _, ok = p.DirectCapFacetRange(new(topo.Body), 0)
	require.False(t, ok)
	sources := append(append([]*topo.Face(nil), walls...), caps[:]...)
	mesh := &topo.Mesh{Vertices: verts, Triangles: tris, Source: sources, BoundaryOK: true}
	prepared := [3]r3.Vec{verts[0], verts[2], verts[3]}
	segment, ok = p.DirectWallFacetSourceSegment(body, mesh, 2, 1, 0, prepared)
	require.True(t, ok)
	require.Equal(t, 1, segment)
	wrongPrepared := prepared
	wrongPrepared[1].X = 1
	_, ok = p.DirectWallFacetSourceSegment(body, mesh, 2, 1, 0, wrongPrepared)
	require.False(t, ok)
	wrongMesh := *mesh
	wrongMesh.Source = append([]*topo.Face(nil), mesh.Source...)
	wrongMesh.Source[2] = walls[3]
	_, ok = p.DirectWallFacetSourceSegment(body, &wrongMesh, 2, 1, 0, prepared)
	require.False(t, ok)
	wrongMesh = *mesh
	wrongMesh.Vertices = append([]r3.Vec(nil), mesh.Vertices...)
	wrongMesh.Vertices[2].Y += 1e-9
	_, ok = p.DirectWallFacetSourceSegment(body, &wrongMesh, 2, 1, 0, prepared)
	require.False(t, ok)
	wrongMesh = *mesh
	wrongMesh.BoundaryOK = false
	_, ok = p.DirectWallFacetSourceSegment(body, &wrongMesh, 2, 1, 0, prepared)
	require.False(t, ok)
	capPrepared := [3]r3.Vec{verts[0], verts[2], verts[1]}
	require.True(t, p.DirectCapFacetMatches(body, mesh, 4, 0, capPrepared))
	require.False(t, p.DirectCapFacetMatches(body, mesh, 4, 1, capPrepared))
	capPrepared[0].Z = 1e-9
	require.False(t, p.DirectCapFacetMatches(body, mesh, 4, 0, capPrepared))
	endPrepared := [3]r3.Vec{verts[1], verts[2], verts[3]}
	require.True(t, p.DirectCapFacetMatches(body, mesh, 5, 1, endPrepared))
	wrongCapMesh := *mesh
	wrongCapMesh.Source = append([]*topo.Face(nil), mesh.Source...)
	wrongCapMesh.Source[5] = caps[0]
	require.False(t, p.DirectCapFacetMatches(body, &wrongCapMesh, 5, 1, endPrepared))
	wrongCapMesh = *mesh
	wrongCapMesh.BoundaryOK = false
	require.False(t, p.DirectCapFacetMatches(body, &wrongCapMesh, 5, 1, endPrepared))
	walls[2] = &topo.Face{Body: body}
	_, ok = p.DirectWallFaceSourceSegment(body, walls[2], 1, 0)
	require.False(t, ok, "the private face list must not follow a changed input slice")
	_, ok = p.DirectWallFaceSourceSegment(new(topo.Body), walls[0], 0, 0)
	require.False(t, ok)
	fresh := func() *directSections {
		s := sealDirectSections(body, p)
		require.NoError(t, sealDirectCellSegments(s, [][]int{{0, 1}}, 2, proofbound.NewWorkBudget(t.Context())))
		return s
	}
	duplicate := append([]*topo.Face(nil), walls...)
	duplicate[1] = duplicate[0]
	s := fresh()
	err = sealDirectFaces(s, body, caps, duplicate, cells, sides, verts, tris, 1, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.Nil(t, s.wallFaces)
	wrongCells := append([][2]int(nil), cells...)
	wrongCells[2] = [2]int{0, 0}
	s = fresh()
	err = sealDirectFaces(s, body, caps, walls, wrongCells, sides, verts, tris, 1, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.Nil(t, s.wallFaces)
	s = fresh()
	budget := &proofbound.WorkBudget{StepFn: func() error { return proofbound.ErrWallWorkBudget }, ErrFn: func() error { return nil }}
	err = sealDirectFaces(s, body, caps, walls, cells, sides, verts, tris, 1, budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.Nil(t, s.wallFaces)
}

func TestDirectCellSourceSegment(t *testing.T) {
	body := new(topo.Body)
	body.Origin.Producer = 7
	profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{End: momentinput.Point2{U: 1}, TEnd: 1},
		momentinput.LineSeg{Start: momentinput.Point2{U: 1}, End: momentinput.Point2{U: 2}, TEnd: 1},
	}}}
	p := Payload{Profile0: profile, Profile1: profile, AuthenticatedReconstruction: true, Xform: r3.Identity()}
	p.SourceIdentity = &SourceIdentity{Origin: 7, direct: sealDirectSections(body, p)}
	p.Segment = [][]int{{0, 1, 1}}
	require.NoError(t, sealDirectCellSegments(p.SourceIdentity.direct, p.Segment, len(p.Profile0.Outer.Segments), proofbound.NewWorkBudget(t.Context())))
	segment, ok := p.DirectCellSourceSegment(body, 0, 1)
	require.True(t, ok)
	require.Equal(t, 1, segment)
	p.Segment[0][1] = 0
	segment, ok = p.DirectCellSourceSegment(body, 0, 1)
	require.True(t, ok)
	require.Equal(t, 1, segment, "the builder's private copy must not change with the payload")
	_, ok = p.DirectCellSourceSegment(body, 0, 3)
	require.False(t, ok)
	_, ok = p.DirectCellSourceSegment(body, 1, 0)
	require.False(t, ok)
	_, ok = p.DirectCellSourceSegment(new(topo.Body), 0, 1)
	require.False(t, ok)
	modified := p
	modified.ConstructionDelta = 1e-9
	_, ok = modified.DirectCellSourceSegment(body, 0, 1)
	require.False(t, ok)
	unsealed := sealDirectSections(body, p)
	require.NoError(t, sealDirectCellSegments(unsealed, [][]int{{0, 2}}, len(p.Profile0.Outer.Segments), proofbound.NewWorkBudget(t.Context())))
	p.SourceIdentity = &SourceIdentity{Origin: 7, direct: unsealed}
	_, ok = p.DirectCellSourceSegment(body, 0, 0)
	require.False(t, ok)
	tooLarge := sealDirectSections(body, p)
	require.NoError(t, sealDirectCellSegments(tooLarge, [][]int{make([]int, 8193)}, len(p.Profile0.Outer.Segments), proofbound.NewWorkBudget(t.Context())))
	p.SourceIdentity = &SourceIdentity{Origin: 7, direct: tooLarge}
	_, ok = p.DirectCellSourceSegment(body, 0, 0)
	require.False(t, ok)
	limited := sealDirectSections(body, p)
	budget := &proofbound.WorkBudget{StepFn: func() error { return proofbound.ErrWallWorkBudget }, ErrFn: func() error { return nil }}
	err := sealDirectCellSegments(limited, [][]int{{0, 1}}, len(p.Profile0.Outer.Segments), budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.Nil(t, limited.cellSegments)
}
