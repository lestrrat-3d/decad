package patchmesh

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Profile chords a PatchProfile's recorded planar region once, triangulates
// it with holes, and attributes every facet to its one live face. Its trim
// and coordinate bounds cover both directions between the analytic patch
// and the held facets. A sheet has no occupied volume to prove. ctx and body
// must be non-nil.
func Profile(ctx context.Context, body *topo.Body, pp patchchain.Payload, chord float64, verify Level) (*topo.Mesh, error) {
	faces := topobuild.Faces(body)
	if body.Kind != topo.BodySheet || len(faces) != 1 {
		return nil, fmt.Errorf(`%w: a recorded patch must have one sheet face`, decaderr.ErrDegenerate)
	}
	face := faces[0]
	work := freeform.NewFreeformWork()
	pw := pp.Walks
	if pw.Reusable(pp.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		pw = nil
	}
	loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	var points []sectionrecord.Point2
	var sampleBounds []proofbound.WalkEndBound
	var loopIndices [][]int
	var loopSag []float64
	trim, capSlack := 0.0, 0.0
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		cl, err := tessellation.ChordLoop(ctx, loop, chord, 0, work, pw, li,
			func(_ survey2d.SideWalk) (*topo.Face, error) { return face, nil }, stationbound.ChordStationBound)
		if err != nil {
			return nil, err
		}
		indices := make([]int, len(cl.Samples))
		for i := range cl.Samples {
			indices[i] = len(points) + i
		}
		loopIndices = append(loopIndices, indices)
		points = append(points, cl.Samples...)
		sampleBounds = append(sampleBounds, cl.BoundOf...)
		loopSag = append(loopSag, cl.MaxSag)
		trim = math.Max(trim, cl.MaxSag)
		capSlack = proofbound.AbsSumUpper(capSlack, cl.CapSlack)
	}
	if err := tessellation.RequireLoopClearance(ctx, points, loopIndices, loopSag); err != nil {
		return nil, err
	}
	triangles, err := triangulation.Triangulate(ctx, points, loopIndices)
	if err != nil {
		return nil, err
	}
	view := pp.Prism()
	mesh := &topo.Mesh{
		Vertices:  make([]r3.Vec, len(points)),
		Triangles: triangles,
		Source:    make([]*topo.Face, len(triangles)),
	}
	store := make([]float64, len(points))
	budget := proofbound.NewWorkBudget(ctx)
	for i, point := range points {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		held := view.Point(point.U, point.V, 0)
		mesh.Vertices[i] = held
		store[i] = proofbound.AbsSumUpper(
			proofbound.WalkEndBoundAllow(sampleBounds[i]),
			view.PointRound(point.U, point.V, 0, held),
		)
	}
	mesh.CoordBound, err = tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	for i := range mesh.Source {
		mesh.Source[i] = face
	}
	if view.Reflected() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTriangles, err := revolvemesh.RequireRevolveFacetAreas(
		auditBudget, mesh.Vertices, mesh.Triangles, mesh.CoordBound)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := revolvemesh.RevolveContactAudit(
			auditBudget, auditTriangles, mesh.Triangles, mesh.CoordBound); err != nil {
			return nil, err
		}
	}
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source,
		map[*topo.Face]float64{face: trim}, nil, store, 0)
	if err != nil {
		return nil, err
	}
	for f, bound := range bounds {
		topobuild.SetFaceBound(mesh, f, bound)
	}
	if verify >= VerifyAll {
		mesh.AreaSlack = proofbound.AbsSumUpper(capSlack,
			tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, store))
		if proofbound.IsNonFinite(mesh.AreaSlack) {
			return nil, fmt.Errorf(`%w: a patch mesh states no finite area allowance`, decaderr.ErrUnsupported)
		}
	}
	return mesh, nil
}
