package prismtess

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// circleGeometryReplay builds private geometry only. The caller must authenticate
// its source records and faces; the result carries no boundary or volume proof.
func circleGeometryReplay(ctx context.Context, p prismrecord.Payload, samples tessellation.ChordSamples[*topo.Face],
	faces [3]*topo.Face, lift float64, budget *proofbound.ReplayBudget) (*topo.Mesh, []float64, error) {
	if budget == nil {
		return nil, nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, nil, err
	}
	if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 || proofbound.IsNonFinite(p.Z0) || proofbound.IsNonFinite(p.Z1) || p.Z0 >= p.Z1 || lift < 1 || proofbound.IsNonFinite(lift) || faces[0] == nil || faces[1] == nil || faces[2] == nil || faces[0] == faces[1] || faces[0] == faces[2] || faces[1] == faces[2] {
		return nil, nil, decaderr.ErrUnsupported
	}
	loop := tessellation.PrismLoop[*topo.Face]{Samples: samples.Samples, FaceOf: samples.FaceOf, SagOf: samples.SagOf, BoundOf: samples.BoundOf, MaxSag: samples.MaxSag, WallSlack: samples.WallSlack, CapSlack: samples.CapSlack, SegmentArea: samples.SegmentArea, PerimeterUpper: samples.PerimeterUpper, Walks: samples.Walks}
	t, err := tessellation.CircleWallsReplay(ctx, loop, 0, budget)
	if err != nil {
		return nil, nil, err
	}
	if len(t.ReplayFaceBounds) != 1 || t.ReplayFaceBounds[0].Face != faces[0] {
		return nil, nil, decaderr.ErrUnsupported
	}
	if err = circleCapsReplay(ctx, &t, faces[1], faces[2], budget); err != nil {
		return nil, nil, err
	}
	n := len(t.Points)
	if err = budget.ReserveRecords(ctx, uint64(4*n+1)); err != nil {
		return nil, nil, err
	}
	if err = budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Mesh{}))); err != nil {
		return nil, nil, err
	}
	if err = budget.ReserveArray(ctx, uint64(2*n), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
		return nil, nil, err
	}
	if err = budget.ReserveArray(ctx, uint64(2*n), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, nil, err
	}
	mesh := &topo.Mesh{Vertices: make([]r3.Vec, 2*n), Triangles: t.Triangles, Source: t.Sources}
	store := make([]float64, 2*n)
	pointCache, err := newPointRoundReplayCache(ctx, p, budget)
	if err != nil {
		return nil, nil, err
	}
	for i, pt := range t.Points {
		if err = budget.SpendWork(ctx, 128); err != nil {
			return nil, nil, err
		}
		plane := proofbound.ProductUpper(lift, proofbound.WalkEndBoundAllow(t.Bounds[i]))
		for j, z := range [...]float64{p.Z0, p.Z1} {
			held := p.Point(pt.U, pt.V, z)
			gap, e := pointCache.bound(ctx, p, pt.U, pt.V, z, held, budget)
			if e != nil {
				return nil, nil, e
			}
			value := proofbound.AbsSumUpper(plane, gap)
			if proofbound.IsNonFinite(value) {
				return nil, nil, decaderr.ErrUnsupported
			}
			mesh.Vertices[2*i+j], store[2*i+j] = held, value
			mesh.CoordBound = math.Max(mesh.CoordBound, value)
		}
	}
	if err = budget.SpendWork(ctx, uint64(8*len(mesh.Triangles)+16)); err != nil {
		return nil, nil, err
	}
	if p.Reflected() {
		for i := range mesh.Triangles {
			if i%128 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, nil, err
				}
			}
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}

	if err = tessellation.RequireClosedMeshReplay(ctx, mesh.Triangles, budget); err != nil {
		return nil, nil, err
	}
	if err = tessellation.RequireVertexLinksReplay(ctx, len(mesh.Vertices), mesh.Triangles, budget); err != nil {
		return nil, nil, err
	}
	if err = budget.SpendWork(ctx, 32); err != nil {
		return nil, nil, err
	}
	for i := range t.ReplayFaceBounds {
		t.ReplayFaceBounds[i].Trim = proofbound.ProductUpper(lift, t.ReplayFaceBounds[i].Trim)
	}
	if err = verifiedFaceBoundsReplay(ctx, mesh, t.ReplayFaceBounds, store, 0, budget); err != nil {
		return nil, nil, err
	}
	mesh.VertexBound, err = topobuild.VertexBoundsReplay(ctx, mesh, budget)
	if err != nil {
		return nil, nil, err
	}

	if _, err = revolvemesh.AuditMeshReplay(ctx, mesh.Vertices, mesh.Triangles, budget); err != nil {
		return nil, nil, err
	}
	return mesh, store, nil
}
