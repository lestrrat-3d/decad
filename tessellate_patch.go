package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// tessellatePatch chords a Document.Patch's recorded planar region once,
// triangulates it with holes, and attributes every facet to its one live face.
// Its trim and coordinate bounds cover both directions between the analytic
// patch and the held facets. A sheet has no occupied volume to prove.
func tessellatePatch(ctx context.Context, b *Body, pp patchPayload, chord float64, verify Verification) (*Mesh, error) {
	faces := b.Faces()
	if b.Kind() != BodySheet || len(faces) != 1 {
		return nil, fmt.Errorf(`%w: a recorded patch must have one sheet face`, ErrDegenerate)
	}
	face := faces[0]
	work := freeform.NewFreeformWork()
	pw := pp.walks
	if pw.Reusable(pp.profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		pw = nil
	}
	loops := append([]loopRecord{pp.profile.Outer}, pp.profile.Holes...)
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
			func(_ survey2d.SideWalk) (*Face, error) { return face, nil }, stationbound.ChordStationBound)
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
	if err := requireRecordLoopClearance(ctx, points, loopIndices, loopSag); err != nil {
		return nil, err
	}
	triangles, err := triangulation.Triangulate(ctx, points, loopIndices)
	if err != nil {
		return nil, err
	}
	view := pp.prism()
	mesh := &Mesh{
		vertices:  make([]r3.Vec, len(points)),
		triangles: triangles,
		source:    make([]*Face, len(triangles)),
	}
	store := make([]float64, len(points))
	budget := proofbound.NewWorkBudget(ctx)
	for i, point := range points {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		held := view.point(point.U, point.V, 0)
		mesh.vertices[i] = held
		store[i] = proofbound.AbsSumUpper(
			proofbound.WalkEndBoundAllow(sampleBounds[i]),
			exactPrismPointRound(view, point.U, point.V, 0, held),
		)
	}
	mesh.coordBound, err = tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	for i := range mesh.source {
		mesh.source[i] = face
	}
	if view.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTriangles, err := revolvemesh.RequireRevolveFacetAreas(
		auditBudget, mesh.vertices, mesh.triangles, mesh.coordBound)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := revolvemesh.RevolveContactAudit(
			auditBudget, auditTriangles, mesh.triangles, mesh.coordBound); err != nil {
			return nil, err
		}
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	if err := composeFaceBounds(mesh, map[*Face]float64{face: trim}, nil, store, 0); err != nil {
		return nil, err
	}
	if verify >= VerifyAll {
		mesh.areaSlack = proofbound.AbsSumUpper(capSlack,
			tessellation.StoreAreaAllow(mesh.vertices, mesh.triangles, store))
		if proofbound.IsNonFinite(mesh.areaSlack) {
			return nil, fmt.Errorf(`%w: a patch mesh states no finite area allowance`, ErrUnsupported)
		}
	}
	return mesh, nil
}
