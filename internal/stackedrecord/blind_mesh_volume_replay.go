package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindMeshVolumeWithSource adds occupied-volume terms to the sealed
// and topology-checked blind-group mesh under the same consumer meter.
func ReplayBlindMeshVolumeWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindMeshAssembly, bool, error) {
	out, ok, err := ReplayBlindMeshTopologyWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := tessellation.ReplayRequireEmbeddedBoundary(ctx, out.Mesh.Vertices, out.Mesh.Triangles, budget); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	columns, rows, _, ok, err := ReplayBlindTessStructure(ctx, body, budget)
	if err != nil || !ok {
		return ReplayBlindMeshAssembly{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok || len(columns) != len(out.Rings) || len(rows) != len(sp.Slabs) {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	if err := budget.ReserveArray(ctx, uint64(len(columns)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(sp.Slabs)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(sp.Slabs)), uint64(unsafe.Sizeof([]int{}))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	columnHeights := make([]float64, len(columns))
	slabHeights := make([]float64, len(sp.Slabs))
	slabColumns := make([][]int, len(sp.Slabs))
	for i, column := range columns {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		if column.Start < 0 || column.End < column.Start || column.End >= len(sp.Slabs) {
			return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
		}
		columnHeights[i] = sp.Slabs[column.End].Z1 - sp.Slabs[column.Start].Z0
	}
	for k, row := range rows {
		if err := budget.ReserveArray(ctx, uint64(len(row)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		slabHeights[k] = sp.Slabs[k].Z1 - sp.Slabs[k].Z0
		slabColumns[k] = make([]int, len(row))
		for i, entry := range row {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			slabColumns[k][i] = entry.Column
		}
	}
	var count uint64
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			count += uint64(len(shell.Faces))
		}
	}
	if count != uint64(len(out.Mesh.FaceBound)) {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	if err := budget.ReserveArray(ctx, count, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	faces := make([]*topo.Face, 0, count)
	for _, lump := range body.Lumps {
		for _, shell := range lump.Shells {
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return ReplayBlindMeshAssembly{}, false, err
				}
				faces = append(faces, face)
			}
		}
	}
	proof, err := tessellation.ReplayProveStackedConservative(ctx,
		tessellation.StackedProofInput[*topo.Face]{
			Rings: out.Rings, ColumnHeights: columnHeights,
			SlabHeights: slabHeights, SlabColumns: slabColumns,
			SectionDelta: sp.SectionDelta, Faces: faces,
			Source: out.Mesh.Source, FaceAxial: out.FaceAxial,
			IsPlanar: func(face *topo.Face) bool {
				_, planar := face.Surface.(surfacegeom.Plane)
				return planar
			},
			Vertices: out.Mesh.Vertices, Triangles: out.Mesh.Triangles,
			Store: out.Store, StoreMax: out.StoreMax,
			AreaSlack: out.Mesh.AreaSlack,
		}, budget)
	if err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	volume, err := tessellation.ReplaySymDiff(ctx, proof.Terms, budget)
	if err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	out.Mesh.AreaSlack = proof.AreaSlack
	out.Mesh.VolSymDiff = volume
	out.Mesh.BoundaryOK = true
	out.Mesh.SymDiffOK = true
	return out, true, nil
}
