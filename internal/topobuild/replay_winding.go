package topobuild

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errReplayWinding1 = fmt.Errorf("%w: bounded winding requires a mesh and budget", decaderr.ErrBooleanFailed)
	errReplayWinding2 = fmt.Errorf("%w: bounded winding requires facet sources", decaderr.ErrUnsupported)
	errReplayWinding3 = fmt.Errorf("%w: nil lump in bounded winding", decaderr.ErrBooleanFailed)
	errReplayWinding4 = fmt.Errorf("%w: nil shell in bounded winding", decaderr.ErrBooleanFailed)
	errReplayWinding5 = fmt.Errorf("%w: bounded winding admits one closed shell", decaderr.ErrUnsupported)
	errReplayWinding6 = fmt.Errorf("%w: bounded winding requires one closed shell", decaderr.ErrUnsupported)
)

// RequireSingleShellWindingReplay admits only the single-closed-shell case of
// RequireShellWinding. The source mesh's own winding proof remains required.
func RequireSingleShellWindingReplay(ctx context.Context, m *topo.Mesh, lumps []*topo.Lump,
	budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if budget == nil || m == nil {
		return errReplayWinding1
	}
	if len(m.Source) != len(m.Triangles) {
		return errReplayWinding2
	}
	closed := 0
	for _, lump := range lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if lump == nil {
			return errReplayWinding3
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if shell == nil {
				return errReplayWinding4
			}
			if !shell.Open {
				closed++
				if closed > 1 {
					return errReplayWinding5
				}
			}
		}
	}
	if closed != 1 {
		return errReplayWinding6
	}
	return nil
}
