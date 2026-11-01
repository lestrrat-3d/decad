package topobuild

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RequireSingleShellWindingReplay admits only the single-closed-shell case of
// RequireShellWinding. The source mesh's own winding proof remains required.
func RequireSingleShellWindingReplay(ctx context.Context, m *topo.Mesh, lumps []*topo.Lump,
	budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if budget == nil || m == nil {
		return fmt.Errorf("%w: bounded winding requires a mesh and budget", decaderr.ErrBooleanFailed)
	}
	if len(m.Source) != len(m.Triangles) {
		return fmt.Errorf("%w: bounded winding requires facet sources", decaderr.ErrUnsupported)
	}
	closed := 0
	for _, lump := range lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if lump == nil {
			return fmt.Errorf("%w: nil lump in bounded winding", decaderr.ErrBooleanFailed)
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if shell == nil {
				return fmt.Errorf("%w: nil shell in bounded winding", decaderr.ErrBooleanFailed)
			}
			if !shell.Open {
				closed++
				if closed > 1 {
					return fmt.Errorf("%w: bounded winding admits one closed shell", decaderr.ErrUnsupported)
				}
			}
		}
	}
	if closed != 1 {
		return fmt.Errorf("%w: bounded winding requires one closed shell", decaderr.ErrUnsupported)
	}
	return nil
}
