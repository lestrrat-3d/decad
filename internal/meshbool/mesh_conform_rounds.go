package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// conformSession carries a ConformScan and its edge memo from one conforming
// round to the next over one immutable vertex set.
//
// A round's result for an edge is a function of the edge and the scan alone,
// and the scan is a function of the vertices alone (never of the triangles or
// of xbeta). A split only inserts existing vertices into edges, so the
// vertices, their rounded points, the point tree and the grid are the same in
// every round. Reusing the scan therefore changes nothing a round computes, and
// the memo's entries stay valid: a triangle unchanged since the previous round
// has the same three edges, which return the same step counts and hit lists,
// so it yields the same hits as before. Only edges that first appear in a
// later round (the new triangles' edges) walk the tree.
//
// A round still charges every step the unreused pass charged: the scan's
// two per-vertex loops, one step per triangle, and each edge's recorded walk
// steps, so a refusal falls at the same step.
type conformSession struct {
	scan  *ConformScan
	memo  *conformEdgeMemo
	verts int
}

// once is conformOnceBudget for the session's vertices: the first call builds
// the scan, later calls reuse it. Every call must pass the same verts and
// approx.
func (c *conformSession) once(ctx context.Context, budget *proofbound.WorkBudget, verts []proof.Xpt, approx []r3.Vec,
	tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) (bool, error) {
	if err := budget.Err(); err != nil {
		return false, err
	}
	if c.scan == nil {
		scan, err := newConformScanApprox(budget, verts, approx)
		if err != nil {
			return false, err
		}
		c.scan, c.verts = scan, len(verts)
		c.memo = newConformEdgeMemo(len(*tris))
	} else {
		if len(verts) != c.verts {
			return false, fmt.Errorf(`%w: the conforming pass changed the vertex set`, decaderr.ErrBooleanFailed)
		}
		// The scan build steps once per vertex before the extent check and
		// once per vertex after it; the extent was checked in the first round.
		for range 2 {
			if _, err := budget.StepN(uint64(len(verts))); err != nil {
				return false, err
			}
		}
	}
	return conformSplitPass(ctx, budget, c.scan, c.memo, verts, tris, src, bounds, xbeta)
}

// conformRounds runs conforming rounds until one splits nothing, each round
// charging the budget newBudget returns. It refuses after the seventh round.
func conformRounds(ctx context.Context, newBudget func() *proofbound.WorkBudget, verts []proof.Xpt, approx []r3.Vec,
	tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) error {
	var session conformSession
	for round := 0; ; round++ {
		if round > 6 {
			return fmt.Errorf(`%w: the conforming pass did not converge`, decaderr.ErrBooleanFailed)
		}
		if err := ctx.Err(); err != nil {
			return err
		}
		split, err := session.once(ctx, newBudget(), verts, approx, tris, src, bounds, xbeta)
		if err != nil {
			return err
		}
		if !split {
			return nil
		}
	}
}
