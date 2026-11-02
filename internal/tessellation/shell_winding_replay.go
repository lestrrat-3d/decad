package tessellation

import (
	"context"
	"errors"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayRequireShellWinding checks the same closed-shell orientation as
// RequireShellWinding, charging its grouping, edge audit and exact signed sum.
func ReplayRequireShellWinding(ctx context.Context, vertices []r3.Vec, triangles [][3]int,
	shellOf []int, void []bool, budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if len(shellOf) != len(triangles) {
		return fmt.Errorf("%w: the shell winding audit holds %d shell indices for %d facets",
			decaderr.ErrDegenerate, len(shellOf), len(triangles))
	}
	if err := budget.ReserveRecords(ctx, uint64(len(void))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, uint64(len(void)), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return err
	}
	counts := make([]int, len(void))
	for _, s := range shellOf {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if s < 0 {
			continue
		}
		if s >= len(void) {
			return decaderr.ErrDegenerate
		}
		counts[s]++
	}
	if err := budget.ReserveArray(ctx, uint64(len(void)), uint64(unsafe.Sizeof([][3]int{}))); err != nil {
		return err
	}
	byShell := make([][][3]int, len(void))
	for i, count := range counts {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if err := budget.ReserveRecords(ctx, uint64(count)); err != nil {
			return err
		}
		if err := budget.ReserveArray(ctx, uint64(count), uint64(unsafe.Sizeof([3]int{}))); err != nil {
			return err
		}
		byShell[i] = make([][3]int, 0, count)
	}
	for i, s := range shellOf {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if s >= 0 {
			byShell[s] = append(byShell[s], triangles[i])
		}
	}
	for s, tris := range byShell {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if len(tris) == 0 {
			continue
		}
		if err := ReplayRequireClosedMesh(ctx, tris, budget); err != nil {
			var audit *AuditError
			if errors.As(err, &audit) && audit.Sentinel == Degenerate {
				continue
			}
			return err
		}
		sign, err := ReplayOrientationSign(ctx, vertices, tris, budget)
		if err != nil {
			return err
		}
		if void[s] && sign >= 0 || !void[s] && sign <= 0 {
			if err := budget.ReserveBundle(ctx, 64, 1, 512); err != nil {
				return err
			}
			if void[s] {
				return &ShellWindingError{Msg: fmt.Sprintf(
					"the facets of void shell %d do not wind into the cavity they bound", s)}
			}
			return &ShellWindingError{Msg: fmt.Sprintf(
				"the facets of outer shell %d do not wind outward; the body reads a cavity or an inside-out shell as material", s)}
		}
	}
	return ctx.Err()
}
