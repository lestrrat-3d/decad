package meshbool

import (
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// conformRoundsPerRound is the multi-round conforming loop as it stood before
// the scan and edge memo were carried between rounds: every round builds its
// own scan and memo through conformOnceBudget. It is the reference
// conformRounds is held to. It returns the number of rounds that ran.
func conformRoundsPerRound(ctx context.Context, newBudget func() *proofbound.WorkBudget, verts []proof.Xpt,
	approx []r3.Vec, tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) (int, error) {
	for round := 0; ; round++ {
		if round > 6 {
			return round, fmt.Errorf(`%w: the conforming pass did not converge`, decaderr.ErrBooleanFailed)
		}
		if err := ctx.Err(); err != nil {
			return round, err
		}
		split, err := conformOnceBudget(ctx, newBudget(), verts, approx, tris, src, bounds, xbeta)
		if err != nil {
			return round + 1, err
		}
		if !split {
			return round + 1, nil
		}
	}
}

// conformRoundsRun is one multi-round run with one step-counting budget shared
// by every round, refusing once limit steps were charged (0 never refuses).
type conformRoundsRun struct {
	err    error
	steps  uint64
	rounds int
	mesh   approxMesh
}

func runConformRounds(t *testing.T, base approxMesh, limit uint64, reference bool) conformRoundsRun {
	t.Helper()
	m := base.clone()
	var steps uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if limit != 0 && steps >= limit {
				return errConformStep
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	newBudget := func() *proofbound.WorkBudget { return budget }
	run := conformRoundsRun{}
	if reference {
		run.rounds, run.err = conformRoundsPerRound(t.Context(), newBudget, m.verts, base.rounded(),
			&m.tris, &m.src, &m.bounds, m.xbeta)
	} else {
		run.err = conformRounds(t.Context(), newBudget, m.verts, base.rounded(), &m.tris, &m.src, &m.bounds, m.xbeta)
	}
	run.steps, run.mesh = steps, m
	return run
}

func newTestBudget(t *testing.T) *proofbound.WorkBudget {
	t.Helper()
	return proofbound.NewWorkBudget(t.Context())
}
