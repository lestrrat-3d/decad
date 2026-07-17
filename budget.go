package decad

import "context"

// workPollInterval is how many candidate operations may pass between context
// polls (docs/interference-design.md §7.2).
const workPollInterval = 256

// workBudget shares one bounded cancellation counter across every nested loop
// of one read-only phase. Leaf exact predicates stay context-free
// (docs/interference-design.md §7.2); their callers step this counter, which
// polls the context at least once per workPollInterval candidate operations.
// Phase boundaries call err instead, which polls unconditionally.
//
// The counter is shared rather than per-loop on purpose: a nest of loops that
// each counted to workPollInterval alone would let the innermost scan run the
// interval's worth of work for every step of the outermost one.
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
