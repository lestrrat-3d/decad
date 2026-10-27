package loftmesh

import (
	"errors"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftRowObserver observes serial pairs on the caller, or one joined chunk
// after its row counts and budget steps have been validated. ObserveRows
// must join any workers before returning; workers must not charge the budget.
type loftRowObserver interface {
	Observe(i, j int)
	ObserveRows(*proofbound.WorkBudget, []int, []loftRowResult, int) error
}

type loftPairObserver func(int, int)

func (f loftPairObserver) Observe(i, j int) { f(i, j) }

func (f loftPairObserver) ObserveRows(budget *proofbound.WorkBudget, members []int, rows []loftRowResult, _ int) error {
	for k, i := range members {
		for pos, j := range rows[k].row {
			if pos%256 == 0 {
				if err := budget.Err(); err != nil {
					return err
				}
			}
			f(i, int(j))
		}
	}
	return nil
}

// loftFacetSelector belongs to one preflight. Its geometry and initial flat
// mask are read-only. Workers only set atomic uncertainty flags, which the
// caller reads after every worker has joined. No per-pair result is retained.
type loftFacetSelector struct {
	verts     []r3.Vec
	tris      [][3]int
	flat      []bool
	uncertain []atomic.Bool
	shortcuts LoftAuditShortcuts
}

func (s *loftFacetSelector) Observe(i, j int) {
	if s.flat[i] && s.flat[j] {
		return
	}
	if loftFloatPairProof(s.verts, s.tris, i, j, s.shortcuts) == loftFloatUnknown {
		s.uncertain[i].Store(true)
		s.uncertain[j].Store(true)
	}
}

func (s *loftFacetSelector) ObserveRows(budget *proofbound.WorkBudget, members []int,
	rows []loftRowResult, workers int,
) error {
	var stopped atomic.Bool
	poll := func() error {
		err := budget.Err()
		if err != nil {
			stopped.Store(true)
		}
		return err
	}
	_, err := orderedwork.Each(len(members), workers, 64, poll, func(k int) error {
		for pos, j := range rows[k].row {
			if pos%64 == 0 && stopped.Load() {
				return nil
			}
			s.Observe(members[k], int(j))
		}
		return nil
	})
	if errors.Is(err, proofbound.ErrWallWorkBudget) {
		// These rows have already replayed their counted steps. Finish their
		// observations on the caller without charging again, then let the next
		// counted row or trailing poll report the budget's original error.
		// Repeated stores are idempotent, including after a partial worker pass.
		for k, i := range members {
			for _, j := range rows[k].row {
				s.Observe(i, int(j))
			}
		}
		return nil
	}
	return err
}
