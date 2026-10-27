package tessellation

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RequireShellWinding is agent-docs/tessellation-design.md §4's per-shell
// orientation audit: the facets of every outer shell must enclose a positive
// signed volume and the facets of every void shell a negative one. shellOf
// names, parallel to triangles, the shell each facet belongs to (-1 for a
// facet of no closed shell), and void says which shells are cavities.
//
// The audit only falsifies. A shell whose facets do not close on their own
// (every directed edge met once and its reverse once, among that shell's
// facets alone) decides nothing, since its signed sum then depends on the
// anchor, and it is skipped. A closed shell's sum is the exact dyadic
// OrientationSign over its own facets, so no rounding decides the sign. A
// shell wound against its kind is a ShellWindingError: the body's lump
// reading disagrees with its own facets.
func RequireShellWinding(ctx context.Context, vertices []r3.Vec, triangles [][3]int, shellOf []int, void []bool) error {
	if len(shellOf) != len(triangles) {
		return fmt.Errorf(`%w: the shell winding audit holds %d shell indices for %d facets`, decaderr.ErrDegenerate, len(shellOf), len(triangles))
	}
	byShell := make([][][3]int, len(void))
	budget := proofbound.NewWorkBudget(ctx)
	for i, s := range shellOf {
		if err := budget.Step(); err != nil {
			return err
		}
		if s < 0 {
			continue
		}
		if s >= len(void) {
			return fmt.Errorf(`%w: facet %d names shell %d of %d`, decaderr.ErrDegenerate, i, s, len(void))
		}
		byShell[s] = append(byShell[s], triangles[i])
	}
	for s, tris := range byShell {
		if err := ctx.Err(); err != nil {
			return err
		}
		if len(tris) == 0 || RequireClosedMesh(tris) != nil {
			continue
		}
		sign := OrientationSign(vertices, tris, r3.Vec{})
		switch {
		case void[s] && sign >= 0:
			return &ShellWindingError{Msg: fmt.Sprintf(`the facets of void shell %d do not wind into the cavity they bound`, s)}
		case !void[s] && sign <= 0:
			return &ShellWindingError{Msg: fmt.Sprintf(`the facets of outer shell %d do not wind outward; the body reads a cavity or an inside-out shell as material`, s)}
		}
	}
	return nil
}

// ShellWindingError is RequireShellWinding's refusal: a closed shell whose
// facets wind against its kind. It is decaderr.ErrUnsupported, and Verify
// reads it as proof that the body's lump reading is invalid.
type ShellWindingError struct {
	Msg string
}

func (e *ShellWindingError) Error() string {
	return fmt.Sprintf(`%s: %s`, decaderr.ErrUnsupported, e.Msg)
}

// Unwrap is decaderr.ErrUnsupported.
func (e *ShellWindingError) Unwrap() error { return decaderr.ErrUnsupported }
