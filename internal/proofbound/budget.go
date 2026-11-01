package proofbound

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

var ErrWallWorkBudget = errors.New("wall survey work budget exhausted")

// NewWallWorkBudget adapts the fixed shell-admission ceiling to the shared
// WorkBudget interface used by the streaming wall kernel.
func NewWallWorkBudget(limit uint64) *WorkBudget {
	remaining := limit
	return &WorkBudget{
		StepFn: func() error {
			if remaining == 0 {
				if limit == ^uint64(0) {
					return ErrWallWorkBudget
				}
				return WithWorkLimit(ErrWallWorkBudget, "proof_steps", limit+1, limit)
			}
			remaining--
			return nil
		},
		ErrFn: func() error {
			if remaining == 0 {
				return WithWorkLimit(ErrWallWorkBudget, "proof_steps", limit, limit)
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
	triples, ok := wallChoose3(q)
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
// polls (agent-docs/interference-design.md §7.2).
const WorkPollInterval = 256

// MaxFacetPairTestsPerCall is the fixed ceiling on facet-pair audit work for one
// call (agent-docs/tessellation-design.md §3). Revolve charges candidate-sweep
// visits and pairs reserved for exact predicates against it. The tessellation boolean
// pre-pass and loft crossing audit charge their exact pair tests against this
// same ceiling rather than minting another limit for pair work.
const MaxFacetPairTestsPerCall = 8_000_000

// WorkBudget shares one bounded cancellation counter across every nested loop
// of one read-only or pre-commit audit phase. Leaf exact predicates stay
// context-free (agent-docs/interference-design.md §7.2); their callers step this
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
	Replay         bool
	ReplayID       *ReplayBudget
	StepFn         func() error
	ErrFn          func() error
	ReserveFn      func(uint64) error
	ReserveBytesFn func(uint64) error
	poll           *workPoll // NewPollWorkBudget's counter; used only while StepFn is nil
}

// workPoll is NewWorkBudget's step counter as a value: a step increments
// work and polls ctx when work reaches a multiple of WorkPollInterval.
type workPoll struct {
	err  func() error // the context's Err
	work uint64
}

func (p *workPoll) step() error {
	p.work++
	if p.work%WorkPollInterval == 0 {
		return p.err()
	}
	return nil
}

// stepN is n calls of step, stopping at the first error. It advances work to
// each multiple of WorkPollInterval in one move and polls ctx there. A failing
// poll counts its step in work, as step does, and is not a successful step.
func (p *workPoll) stepN(n uint64) (uint64, error) {
	var done uint64
	for done < n {
		next := WorkPollInterval - p.work%WorkPollInterval
		if n-done < next {
			p.work += n - done
			return n, nil
		}
		p.work += next
		done += next
		if err := p.err(); err != nil {
			return done - 1, err
		}
	}
	return n, nil
}

// NewPollWorkBudget is NewWorkBudget with StepFn nil: Step and StepN count
// on the budget's own counter and poll ctx when it reaches a multiple of
// WorkPollInterval, exactly as NewWorkBudget's StepFn does. Code that reads
// StepFn of this budget gets nil.
func NewPollWorkBudget(ctx context.Context) *WorkBudget {
	return &WorkBudget{ErrFn: ctx.Err, poll: &workPoll{err: ctx.Err}}
}

// StepN is n calls of Step stopping at the first error; it returns how many
// succeeded and that error. A budget made by NewPollWorkBudget skips the
// per-step calls; every other budget loops Step.
func (b *WorkBudget) StepN(n uint64) (uint64, error) {
	if b.StepFn == nil && b.poll != nil {
		return b.poll.stepN(n)
	}
	for s := range n {
		if err := b.Step(); err != nil {
			return s, err
		}
	}
	return n, nil
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
func (b *WorkBudget) Step() error {
	if b.StepFn == nil && b.poll != nil {
		return b.poll.step()
	}
	return b.StepFn()
}

// Reserve charges records before allocation when this work budget has a record limit.
func (b *WorkBudget) Reserve(n uint64) error {
	if b.ReserveFn == nil {
		return nil
	}
	return b.ReserveFn(n)
}

// ReserveBytes charges storage before allocating when a byte meter is present.
func (b *WorkBudget) ReserveBytes(n uint64) error {
	if b.ReserveBytesFn == nil {
		return nil
	}
	return b.ReserveBytesFn(n)
}

// ReserveBytesMul checks a backing-array byte count before reserving it.
func (b *WorkBudget) ReserveBytesMul(n, size uint64) error {
	if size != 0 && n > ^uint64(0)/size {
		return fmt.Errorf("%w: byte reservation overflow", decaderr.ErrUnsupported)
	}
	bytes := n * size
	if bytes > (^uint64(0)-128)/2 {
		return fmt.Errorf("%w: byte reservation envelope overflow", decaderr.ErrUnsupported)
	}
	return b.ReserveBytes(2*bytes + 128)
}

// ReserveMul checks a record-count product before reserving it.
func (b *WorkBudget) ReserveMul(n, scale uint64) error {
	if scale != 0 && n > ^uint64(0)/scale {
		return fmt.Errorf("%w: record reservation overflow", decaderr.ErrUnsupported)
	}
	return b.Reserve(n * scale)
}

// err polls the context unconditionally — the phase-boundary check.
func (b *WorkBudget) Err() error { return b.ErrFn() }
