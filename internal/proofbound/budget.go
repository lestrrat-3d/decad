package proofbound

import (
	"context"
	"errors"
)

var ErrWallWorkBudget = errors.New("wall survey work budget exhausted")

// NewWallWorkBudget adapts the fixed shell-admission ceiling to the shared
// WorkBudget interface used by the streaming wall kernel.
func NewWallWorkBudget(limit uint64) *WorkBudget {
	remaining := limit
	return &WorkBudget{
		StepFn: func() error {
			if remaining == 0 {
				return ErrWallWorkBudget
			}
			remaining--
			return nil
		},
		ErrFn: func() error {
			if remaining == 0 {
				return ErrWallWorkBudget
			}
			return nil
		},
	}
}

// NewWallWorkBudgetWithOperation shares the operation cancellation budget with
// the fixed wall-survey ceiling.
func NewWallWorkBudgetWithOperation(limit uint64, operation *WorkBudget) *WorkBudget {
	wall := NewWallWorkBudget(limit)
	return &WorkBudget{
		StepFn: func() error {
			if operation != nil {
				if err := operation.Step(); err != nil {
					return err
				}
			}
			return wall.Step()
		},
		ErrFn: func() error {
			if operation != nil {
				if err := operation.Err(); err != nil {
					return err
				}
			}
			return wall.Err()
		},
	}
}

func WallCheckedAdd(a, b uint64) (uint64, bool) {
	if ^uint64(0)-a < b {
		return 0, false
	}
	return a + b, true
}

func WallCheckedMul(a, b uint64) (uint64, bool) {
	if a != 0 && b > ^uint64(0)/a {
		return 0, false
	}
	return a * b, true
}

func WallChoose2(n uint64) (uint64, bool) {
	if n < 2 {
		return 0, true
	}
	a, b := n, n-1
	if a%2 == 0 {
		a /= 2
	} else {
		b /= 2
	}
	return WallCheckedMul(a, b)
}

func WallChoose3(n uint64) (uint64, bool) {
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
	v, ok := WallCheckedMul(factors[0], factors[1])
	if !ok {
		return 0, false
	}
	v, ok = WallCheckedMul(v, factors[2])
	if !ok {
		return 0, false
	}
	return v, true
}

// WallCandidateWork counts candidate-family visits before validation.
func WallCandidateWork(elementCount, vertexCount int, wedge bool) (uint64, bool) {
	if elementCount < 0 || vertexCount < 0 {
		return 0, false
	}
	e, v := uint64(elementCount), uint64(vertexCount)
	ee, ok := WallChoose2(e)
	if !ok {
		return 0, false
	}
	ev, ok := WallCheckedMul(e, v)
	if !ok {
		return 0, false
	}
	vv, ok := WallChoose2(v)
	if !ok {
		return 0, false
	}
	q, ok := WallCheckedAdd(e, v)
	if !ok {
		return 0, false
	}
	wedgeWork := uint64(0)
	if wedge {
		q, ok = WallCheckedAdd(q, 1)
		if !ok {
			return 0, false
		}
		wedgeWork, ok = WallCheckedAdd(e, v)
		if !ok {
			return 0, false
		}
	}
	triples, ok := WallChoose3(q)
	if !ok {
		return 0, false
	}
	total := e
	for _, n := range []uint64{ee, ev, vv, wedgeWork, triples} {
		total, ok = WallCheckedAdd(total, n)
		if !ok {
			return 0, false
		}
	}
	return total, true
}

// WorkPollInterval is how many candidate operations may pass between context
// polls (docs/interference-design.md §7.2).
const WorkPollInterval = 256

// MaxFacetPairTestsPerCall is the fixed ceiling on exact triangle-pair
// predicate invocations for one call (docs/tessellation-design.md §3): the
// tessellation boolean pre-pass and the loft crossing audit
// (docs/loft-design.md §6) both charge every pair test against this one
// constant rather than minting a second ceiling for the identical quantity.
// The crossing audit compares it, before testing any pair, against the pairs
// its sweep scans and the candidate pairs it will test: the pairs whose
// bounding boxes overlap, less every pair a cap proof decides (S8).
const MaxFacetPairTestsPerCall = 8_000_000

// WorkBudget shares one bounded cancellation counter across every nested loop
// of one read-only or pre-commit audit phase. Leaf exact predicates stay
// context-free (docs/interference-design.md §7.2); their callers step this
// counter, which polls the context at least once per WorkPollInterval candidate
// operations. Phase boundaries call err instead, which polls unconditionally.
//
// The counter is shared rather than per-loop on purpose: a nest of loops that
// each counted to WorkPollInterval alone would let the innermost scan run the
// interval's worth of work for every step of the outermost one.
//
// WorkBudget holds closures rather than a context.Context field, so no
// long-lived geometry state stores a context.
type WorkBudget struct {
	StepFn func() error
	ErrFn  func() error
}

func NewWorkBudget(ctx context.Context) *WorkBudget {
	work := 0
	return &WorkBudget{
		StepFn: func() error {
			work++
			if work%WorkPollInterval == 0 {
				return ctx.Err()
			}
			return nil
		},
		ErrFn: ctx.Err,
	}
}

// step counts one candidate operation and returns ctx.Err() on the polling
// interval.
func (b *WorkBudget) Step() error { return b.StepFn() }

// err polls the context unconditionally — the phase-boundary check.
func (b *WorkBudget) Err() error { return b.ErrFn() }
