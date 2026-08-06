package decad

import (
	"context"
	"errors"
)

var errWallWorkBudget = errors.New("wall survey work budget exhausted")

// newWallWorkBudget adapts the fixed shell-admission ceiling to the shared
// workBudget interface used by the streaming wall kernel.
func newWallWorkBudget(limit uint64) *workBudget {
	remaining := limit
	return &workBudget{
		stepFn: func() error {
			if remaining == 0 {
				return errWallWorkBudget
			}
			remaining--
			return nil
		},
		errFn: func() error {
			if remaining == 0 {
				return errWallWorkBudget
			}
			return nil
		},
	}
}

// newWallWorkBudgetWithOperation shares the operation cancellation budget with
// the fixed wall-survey ceiling.
func newWallWorkBudgetWithOperation(limit uint64, operation *workBudget) *workBudget {
	wall := newWallWorkBudget(limit)
	return &workBudget{
		stepFn: func() error {
			if operation != nil {
				if err := operation.step(); err != nil {
					return err
				}
			}
			return wall.step()
		},
		errFn: func() error {
			if operation != nil {
				if err := operation.err(); err != nil {
					return err
				}
			}
			return wall.err()
		},
	}
}

func wallCheckedAdd(a, b uint64) (uint64, bool) {
	if ^uint64(0)-a < b {
		return 0, false
	}
	return a + b, true
}

func wallCheckedMul(a, b uint64) (uint64, bool) {
	if a != 0 && b > ^uint64(0)/a {
		return 0, false
	}
	return a * b, true
}

func wallChoose2(n uint64) (uint64, bool) {
	if n < 2 {
		return 0, true
	}
	a, b := n, n-1
	if a%2 == 0 {
		a /= 2
	} else {
		b /= 2
	}
	return wallCheckedMul(a, b)
}

func wallChoose3(n uint64) (uint64, bool) {
	if n < 3 {
		return 0, true
	}
	factors := [3]uint64{n, n - 1, n - 2}
	for i := range factors {
		if factors[i]%2 == 0 {
			factors[i] /= 2
			break
		}
	}
	for i := range factors {
		if factors[i]%3 == 0 {
			factors[i] /= 3
			break
		}
	}
	v, ok := wallCheckedMul(factors[0], factors[1])
	if !ok {
		return 0, false
	}
	v, ok = wallCheckedMul(v, factors[2])
	if !ok {
		return 0, false
	}
	return v, true
}

// wallCandidateWork counts candidate-family visits before validation.
func wallCandidateWork(elementCount, vertexCount int, wedge bool) (uint64, bool) {
	if elementCount < 0 || vertexCount < 0 {
		return 0, false
	}
	e, v := uint64(elementCount), uint64(vertexCount)
	ee, ok := wallChoose2(e)
	if !ok {
		return 0, false
	}
	ev, ok := wallCheckedMul(e, v)
	if !ok {
		return 0, false
	}
	vv, ok := wallChoose2(v)
	if !ok {
		return 0, false
	}
	q, ok := wallCheckedAdd(e, v)
	if !ok {
		return 0, false
	}
	wedgeWork := uint64(0)
	if wedge {
		q, ok = wallCheckedAdd(q, 1)
		if !ok {
			return 0, false
		}
		wedgeWork, ok = wallCheckedAdd(e, v)
		if !ok {
			return 0, false
		}
	}
	triples, ok := wallChoose3(q)
	if !ok {
		return 0, false
	}
	total := e
	for _, n := range []uint64{ee, ev, vv, wedgeWork, triples} {
		total, ok = wallCheckedAdd(total, n)
		if !ok {
			return 0, false
		}
	}
	return total, true
}

// workPollInterval is how many candidate operations may pass between context
// polls (docs/interference-design.md §7.2).
const workPollInterval = 256

// maxFacetPairTestsPerCall is the fixed ceiling on exact triangle-pair
// predicate invocations for one call (docs/tessellation-design.md §3): the
// tessellation boolean pre-pass and the loft crossing audit
// (docs/loft-design.md §6) both charge every pair test against this one
// constant rather than minting a second ceiling for the identical quantity.
const maxFacetPairTestsPerCall = 8_000_000

// workBudget shares one bounded cancellation counter across every nested loop
// of one read-only or pre-commit audit phase. Leaf exact predicates stay
// context-free (docs/interference-design.md §7.2); their callers step this
// counter, which polls the context at least once per workPollInterval candidate
// operations. Phase boundaries call err instead, which polls unconditionally.
//
// The counter is shared rather than per-loop on purpose: a nest of loops that
// each counted to workPollInterval alone would let the innermost scan run the
// interval's worth of work for every step of the outermost one.
//
// workBudget holds closures rather than a context.Context field, so no
// long-lived geometry state stores a context.
type workBudget struct {
	stepFn func() error
	errFn  func() error
}

func newWorkBudget(ctx context.Context) *workBudget {
	work := 0
	return &workBudget{
		stepFn: func() error {
			work++
			if work%workPollInterval == 0 {
				return ctx.Err()
			}
			return nil
		},
		errFn: ctx.Err,
	}
}

// step counts one candidate operation and returns ctx.Err() on the polling
// interval.
func (b *workBudget) step() error { return b.stepFn() }

// err polls the context unconditionally — the phase-boundary check.
func (b *workBudget) err() error { return b.errFn() }
