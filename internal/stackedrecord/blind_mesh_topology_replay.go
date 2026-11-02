package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// ReplayBlindMeshTopologyWithSource carries the private consumer meter
// through closedness, every source-face displacement, and shell winding.
// Occupied-volume and embedded-boundary proofs remain separate.
func ReplayBlindMeshTopologyWithSource(ctx context.Context, body, host *topo.Body,
	requestedChord float64, budget *proofbound.ReplayBudget) (ReplayBlindMeshAssembly, bool, error) {
	out, ok, err := ReplayBlindMeshAssemblyWithSource(ctx, body, host, requestedChord, budget)
	if err != nil || !ok {
		return ReplayBlindMeshAssembly{}, false, err
	}
	return replayBlindMeshTopology(ctx, body, out, budget)
}

func replayBlindMeshTopology(ctx context.Context, body *topo.Body, out ReplayBlindMeshAssembly,
	budget *proofbound.ReplayBudget) (ReplayBlindMeshAssembly, bool, error) {
	if err := tessellation.ReplayRequireClosedMesh(ctx, out.Mesh.Triangles, budget); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := tessellation.ReplayRequireVertexLinks(ctx, len(out.Mesh.Vertices), out.Mesh.Triangles, budget); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	sp, ok := body.Payload.(Payload)
	if !ok {
		return ReplayBlindMeshAssembly{}, false, nil
	}
	var faceCount uint64
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return ReplayBlindMeshAssembly{}, false, err
			}
			faceCount += uint64(len(shell.Faces))
		}
	}
	if faceCount == 0 || faceCount > 4096 {
		return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
	}
	if err := budget.ReserveArray(ctx, faceCount, uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, faceCount, uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveArray(ctx, faceCount, uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	faces := make([]*topo.Face, 0, faceCount)
	trim := make([]float64, 0, faceCount)
	axial := make([]float64, 0, faceCount)
	for _, lump := range body.Lumps {
		for _, shell := range lump.Shells {
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 16); err != nil {
					return ReplayBlindMeshAssembly{}, false, err
				}
				t, present := out.FaceTrim[face]
				if !present {
					return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
				}
				a, present := out.FaceAxial[face]
				if !present {
					return ReplayBlindMeshAssembly{}, false, decaderr.ErrUnsupported
				}
				faces = append(faces, face)
				trim = append(trim, t)
				axial = append(axial, a)
			}
		}
	}
	bounds, err := tessellation.ReplayFaceBounds(ctx, out.Mesh.Triangles, out.Mesh.Source,
		faces, trim, axial, out.Store, sp.SectionDelta, budget)
	if err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	if err := budget.ReserveBundle(ctx, faceCount*32+128, faceCount,
		faceCount*256+512); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	out.Mesh.FaceBound = make(map[*topo.Face]float64, faceCount)
	for i, face := range faces {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return ReplayBlindMeshAssembly{}, false, err
		}
		out.Mesh.FaceBound[face] = bounds[i]
		if bounds[i] > out.Mesh.Bound {
			out.Mesh.Bound = bounds[i]
		}
	}
	if err := topobuild.RequireShellWindingReplay(ctx, out.Mesh, body.Lumps, budget); err != nil {
		return ReplayBlindMeshAssembly{}, false, err
	}
	return out, true, nil
}
