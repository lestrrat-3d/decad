package topobuild

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RequireShellWindingReplay charges the closed-shell face map and per-facet
// indices before the same exact shell orientation check as RequireShellWinding.
func RequireShellWindingReplay(ctx context.Context, mesh *topo.Mesh,
	lumps []*topo.Lump, budget *proofbound.ReplayBudget) error {
	if budget == nil || mesh == nil {
		return decaderr.ErrUnsupported
	}
	var shells, faces uint64
	for _, lump := range lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if lump == nil {
			return decaderr.ErrUnsupported
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if shell == nil {
				return decaderr.ErrUnsupported
			}
			if shell.Open {
				continue
			}
			shells++
			faces += uint64(len(shell.Faces))
		}
	}
	if shells > proofbound.ReplayTrialRecords || faces > proofbound.ReplayTrialRecords {
		return decaderr.ErrUnsupported
	}
	if err := budget.ReserveBundle(ctx, shells+faces, shells+faces, 256+faces*256); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, shells, uint64(unsafe.Sizeof(false))); err != nil {
		return err
	}
	shellOf := make(map[*topo.Face]int, int(faces))
	void := make([]bool, 0, int(shells))
	for _, lump := range lumps {
		for _, shell := range lump.Shells {
			if shell.Open {
				continue
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return err
				}
				if face == nil {
					return decaderr.ErrUnsupported
				}
				shellOf[face] = len(void)
			}
			void = append(void, shell.Void)
		}
	}
	if len(void) < 2 || len(mesh.Source) != len(mesh.Triangles) {
		return nil
	}
	if err := budget.ReserveRecords(ctx, uint64(len(mesh.Triangles))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, uint64(len(mesh.Triangles)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return err
	}
	index := make([]int, len(mesh.Triangles))
	for i, face := range mesh.Source {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		index[i] = -1
		if s, ok := shellOf[face]; ok {
			index[i] = s
		}
	}
	return tessellation.ReplayRequireShellWinding(ctx, mesh.Vertices, mesh.Triangles, index, void, budget)
}
