package booleaneval

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type directLoftEdgeFixture struct {
	body                *topo.Body
	loft                *topo.Mesh
	loftBool            *meshbool.BoolMesh
	loftFacet, loftEdge int
	target              *topo.Mesh
	targetBool          *meshbool.BoolMesh
}

func certifierFor(t *testing.T, f directLoftEdgeFixture) *directLoftEdgeCertifier {
	t.Helper()
	cert, ok := newDirectLoftEdgeCertifier(f.target, f.targetBool, f.body, f.loft, f.loftBool).(*directLoftEdgeCertifier)
	require.True(t, ok)
	return cert
}

// capSide returns the start-cap facet beside the fixture's wall edge.
func capSide(t *testing.T, f directLoftEdgeFixture) (facet, edge int) {
	t.Helper()
	facet, ok := f.loftBool.TwinFacet(f.loftFacet, f.loftEdge)
	require.True(t, ok)
	for e := range 3 {
		if twin, ok := f.loftBool.TwinFacet(facet, e); ok && twin == f.loftFacet {
			return facet, e
		}
	}
	require.Fail(t, "cap edge")
	return 0, 0
}

func makeDirectLoftEdgeFixture(t *testing.T) directLoftEdgeFixture {
	return makeDirectLoftEdgeFixtureOffset(t, 0)
}

func makeDirectLoftEdgeFixtureOffset(t *testing.T, offset float64) directLoftEdgeFixture {
	t.Helper()
	points := []momentinput.Point2{{}, {U: 2}, {U: 2, V: 2}, {V: 2}}
	var segments []momentinput.CurveSegment
	for i := range points {
		segments = append(segments, momentinput.LineSeg{Start: points[i], End: points[(i+1)%len(points)], TEnd: 1})
	}
	profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: segments}}
	p0 := sectionrecord.PlaneRecord{Origin: r3.Vec{X: offset}, U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}
	p1 := p0
	p1.Origin.Z = 3
	f0, err := r3.NewFrame(p0.Origin, p0.U, p0.V)
	require.NoError(t, err)
	f1, err := r3.NewFrame(p1.Origin, p1.U, p1.V)
	require.NoError(t, err)
	input := loftrecord.Payload{Profile0: profile, Profile1: profile, Frame0: f0, Frame1: f1, Plane0: p0, Plane1: p1,
		Xform: r3.Identity(), RecordArea: [2]float64{4, 4}, AuthenticatedReconstruction: true}
	env := topo.Env{Doc: new(topo.Document), Alloc: topotest.Alloc{}, Ref: 1}
	body, err := loftrecord.Build(t.Context(), env, input, proofbound.NewWorkBudget(t.Context()),
		freeform.NewFreeformWork(), freeform.NewFreeformWork())
	require.NoError(t, err)
	lp := body.Payload.(loftrecord.Payload)
	mesh, err := loftrecord.Tessellate(t.Context(), body, lp)
	require.NoError(t, err)
	// The root tessellator sets this after the native builder's boundary audit.
	mesh.BoundaryOK = true
	bm, err := meshbool.PrepContext(t.Context(), mesh.Vertices, mesh.Triangles, make([]int, len(mesh.Triangles)))
	require.NoError(t, err)
	facet, edge := -1, -1
	for i := range lp.Cell {
		for k := range 3 {
			twin, ok := bm.TwinFacet(i, k)
			if ok && lp.DirectCapFaceMatches(body, mesh.Source[twin], 0) {
				facet, edge = i, k
				break
			}
		}
		if facet >= 0 {
			break
		}
	}
	require.GreaterOrEqual(t, facet, 0)
	tri := bm.Tris[facet]
	a, b := bm.Verts[tri[edge]], bm.Verts[tri[(edge+1)%3]]
	a1, b1 := a, b
	a1.Z, b1.Z = 1, -1
	verts := []r3.Vec{a, b, a1, b1}
	tris := [][3]int{{0, 1, 2}, {1, 0, 3}}
	fa, fb := &topo.Face{}, &topo.Face{}
	shared := &topo.Edge{RetainedCurve: true, Curve: surfacegeom.Line3{}, Faces: []*topo.Face{fa, fb},
		Start: &topo.Vertex{Position: a, Bound: units.Millimeters(0)}, End: &topo.Vertex{Position: b, Bound: units.Millimeters(0)}}
	fa.Loops = []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: shared, Forward: true}}}}
	fb.Loops = []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: shared, Forward: false}}}}
	target := &topo.Mesh{Vertices: verts, Triangles: tris, Source: []*topo.Face{fa, fb}, BoundaryOK: true,
		FaceBound: map[*topo.Face]float64{fa: 0, fb: 0}}
	targetBool, err := meshbool.PrepContext(t.Context(), verts, tris, []int{0, 1})
	require.NoError(t, err)
	return directLoftEdgeFixture{body: body, loft: mesh, loftBool: bm, loftFacet: facet, loftEdge: edge,
		target: target, targetBool: targetBool}
}
