package facetedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// RestateReplay restates a held faceted mesh under the caller's replay budget.
// It does not perform the caller's shell-winding or VerifyAll gates.
func RestateReplay(ctx context.Context, body *topo.Body, fp Payload, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	return restateReplay(ctx, body, fp, chord, budget, false)
}

// RestateReplayTable builds a replay mesh with a fixed face-bound table.
func RestateReplayTable(ctx context.Context, body *topo.Body, fp Payload, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	return restateReplay(ctx, body, fp, chord, budget, true)
}

func restateReplay(ctx context.Context, body *topo.Body, fp Payload, chord float64,
	budget *proofbound.ReplayBudget, table bool) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if budget == nil || body == nil {
		return nil, errBrakeReplayBoundedRestatementRequiresABodyAndBudget
	}
	if chord < fp.MeshBound {
		return nil, &boundError{requested: chord, held: fp.MeshBound}
	}
	if len(fp.VertexBound) != len(fp.Verts) || len(fp.FaceOf) != len(fp.Tris) {
		return nil, errBrakeReplayAFacetedPayloadSPerVertexOrPerFacetRecordDoesNotMatchItsMesh
	}

	faceCount := uint64(0)
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if lump == nil {
			return nil, errBrakeReplayNilLumpInFacetedRestatement
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if shell == nil || uint64(len(shell.Faces)) > ^uint64(0)-faceCount {
				return nil, errBrakeReplayInvalidShellInFacetedRestatement
			}
			faceCount += uint64(len(shell.Faces))
		}
	}
	// Reserve exact-capacity slices before allocating them. The face-bound map
	// is charged one record before each new source face is inserted.
	if err := budget.ReserveRecords(ctx, faceCount); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, faceCount); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, faceCount, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return nil, err
	}
	faces := make([]*topo.Face, 0, int(faceCount))
	for _, lump := range body.Lumps {
		for _, shell := range lump.Shells {
			faces = append(faces, shell.Faces...)
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(fp.Tris))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(fp.Tris)), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return nil, err
	}
	sources := make([]*topo.Face, len(fp.Tris))
	for i, face := range fp.FaceOf {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if face < 0 || face >= len(faces) {
			return nil, errBrakeReplayAFacetMapsToNoFace
		}
		sources[i] = faces[face]
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(fp.VertexBound))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(len(fp.VertexBound))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(fp.VertexBound)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Mesh{}))); err != nil {
		return nil, err
	}
	m := &topo.Mesh{Vertices: fp.Verts, Triangles: fp.Tris, AreaSlack: fp.AreaSlack,
		VolSymDiff: fp.VolSymDiff, SymDiffOK: true, Source: sources,
		VertexBound: append([]float64(nil), fp.VertexBound...)}
	if table {
		var err error
		m.ReplayFaceBounds, err = topo.NewFaceBoundTable(ctx, int(faceCount), budget)
		if err != nil {
			return nil, err
		}
	}
	for i, tri := range m.Triangles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if tri[0] < 0 || tri[0] >= len(m.VertexBound) || tri[1] < 0 || tri[1] >= len(m.VertexBound) ||
			tri[2] < 0 || tri[2] >= len(m.VertexBound) {
			return nil, errBrakeReplayInvalidFacetVertexInFacetedRestatement
		}
		face := sources[i]
		delta := max(m.VertexBound[tri[0]], m.VertexBound[tri[1]], m.VertexBound[tri[2]])
		if table {
			prev, found, err := m.ReplayFaceBounds.Lookup(ctx, face, budget)
			if err != nil {
				return nil, err
			}
			if !found || delta > prev {
				if err := m.ReplayFaceBounds.Set(ctx, face, delta, budget); err != nil {
					return nil, err
				}
			}
			m.Bound = max(m.Bound, delta)
			continue
		}
		if m.FaceBound == nil {
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return nil, err
			}
		}
		if _, exists := m.FaceBound[face]; !exists {
			if err := budget.ReserveRecords(ctx, 1); err != nil {
				return nil, err
			}
		}
		topobuild.SetFaceBound(m, face, delta)
	}
	if m.Bound != fp.MeshBound {
		return nil, errBrakeReplayAFacetedPayloadSMeshBoundIsNotTheLargestOfItsFacetBounds
	}
	return m, nil
}
