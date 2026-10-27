package proofbound_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// countingContext counts Err calls and fails from call failFrom on; failFrom
// 0 never fails.
type countingContext struct {
	calls    int
	failFrom int
}

func (c *countingContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *countingContext) Done() <-chan struct{}       { return nil }
func (c *countingContext) Value(any) any               { return nil }

func (c *countingContext) Err() error {
	c.calls++
	if c.failFrom != 0 && c.calls >= c.failFrom {
		return context.Canceled
	}
	return nil
}

// TestStepNMatchesStepLoop compares StepN on a poll budget with n Step calls
// on a second one: same count, same error, same Err calls, same next Step.
func TestStepNMatchesStepLoop(t *testing.T) {
	for _, n := range []uint64{0, 1, 255, 256, 257, 1000, 100_000} {
		for offset := range 301 {
			if n > 257 && offset%53 != 0 {
				continue
			}
			for _, failFrom := range []int{1, 2, 5, 0} {
				want := &countingContext{failFrom: failFrom}
				got := &countingContext{failFrom: failFrom}
				wantB, gotB := proofbound.NewPollWorkBudget(want), proofbound.NewPollWorkBudget(got)
				advance(t, wantB, offset)
				advance(t, gotB, offset)
				// Align the failure point: offset steps may already have polled.
				want.calls, got.calls = 0, 0

				var wantDone uint64
				var wantErr error
				for wantDone < n {
					if wantErr = wantB.Step(); wantErr != nil {
						break
					}
					wantDone++
				}
				gotDone, gotErr := gotB.StepN(n)
				require.Equal(t, wantDone, gotDone, "n %d offset %d fail %d", n, offset, failFrom)
				require.Equal(t, wantErr, gotErr, "n %d offset %d fail %d", n, offset, failFrom)
				require.Equal(t, want.calls, got.calls, "n %d offset %d fail %d", n, offset, failFrom)
				for range 300 {
					require.Equal(t, wantB.Step(), gotB.Step(), "n %d offset %d fail %d", n, offset, failFrom)
				}
				require.Equal(t, want.calls, got.calls, "n %d offset %d fail %d", n, offset, failFrom)
			}
		}
	}
}

// advance steps b offset times, ignoring errors.
func advance(t *testing.T, b *proofbound.WorkBudget, offset int) {
	t.Helper()
	for range offset {
		_ = b.Step()
	}
}

// TestPollWorkBudgetMatchesNewWorkBudget checks the poll budget against the
// closure budget it replaces.
func TestPollWorkBudgetMatchesNewWorkBudget(t *testing.T) {
	for _, failFrom := range []int{1, 3, 0} {
		want := &countingContext{failFrom: failFrom}
		got := &countingContext{failFrom: failFrom}
		wantB, gotB := proofbound.NewWorkBudget(want), proofbound.NewPollWorkBudget(got)
		for range 5000 {
			require.Equal(t, wantB.Step(), gotB.Step())
		}
		require.Equal(t, want.calls, got.calls)
		require.Equal(t, wantB.Err(), gotB.Err())
		require.Equal(t, want.calls, got.calls)
	}
	require.Nil(t, proofbound.NewPollWorkBudget(t.Context()).StepFn)
}

// budgetOp is one call a budget's user makes: run Step calls, one StepN(n)
// call, or one Err call.
type budgetOp struct {
	kind budgetOpKind
	n    uint64
}

type budgetOpKind int

const (
	opStep budgetOpKind = iota
	opStepN
	opErr
)

var budgetOpNames = [...]string{opStep: "Step run", opStepN: "StepN", opErr: "Err"}

// boreAuditOps is a call sequence shaped like the sector bore audits'
// (internal/revolveshell sector_bore_cut.go and sector_bore_band.go), scaled
// down: runs of thousands of single steps and StepN replays that each cross
// hundreds of polls, with phase-boundary Err calls between them. The lengths
// are not multiples of WorkPollInterval, so each replay starts and ends
// between polls, and the short StepN calls end on, before and after a poll.
var boreAuditOps = []budgetOp{
	{kind: opErr}, {kind: opStep, n: 10_003}, {kind: opStepN, n: 149_224}, {kind: opErr},
	{kind: opStep, n: 5_595}, {kind: opStepN, n: 70_001}, {kind: opStepN, n: 0},
	{kind: opStepN, n: 1}, {kind: opStepN, n: 255}, {kind: opStepN, n: 256}, {kind: opErr},
	{kind: opStep, n: 333}, {kind: opErr},
}

// budgetResult is what one budgetOp returned: successful steps, the error,
// and the context's Err calls so far.
type budgetResult struct {
	done  uint64
	err   error
	calls int
}

// runBudgetOps applies ops to b and records each op's result. A step run
// stops at its first error, as a caller loop does; later ops still run.
func runBudgetOps(b *proofbound.WorkBudget, ctx *countingContext, ops []budgetOp) []budgetResult {
	out := make([]budgetResult, 0, len(ops))
	for _, op := range ops {
		var r budgetResult
		switch op.kind {
		case opStep:
			for r.done < op.n {
				if r.err = b.Step(); r.err != nil {
					break
				}
				r.done++
			}
		case opStepN:
			r.done, r.err = b.StepN(op.n)
		case opErr:
			r.err = b.Err()
		}
		r.calls = ctx.calls
		out = append(out, r)
	}
	return out
}

// TestPollWorkBudgetMatchesNewWorkBudgetOnBoreAuditOps runs the bore audits'
// call shape through NewWorkBudget, the reference, and NewPollWorkBudget with
// a context that starts failing at its k-th Err call. Every op must return
// the same successful step count and error and leave the same number of Err
// calls, so a cancel or deadline stops a caller at the same step with the same
// error. k covers the first calls, the calls on and around each op's
// boundary, the middle of each long op, and a context that never fails.
func TestPollWorkBudgetMatchesNewWorkBudgetOnBoreAuditOps(t *testing.T) {
	live := &countingContext{}
	reference := runBudgetOps(proofbound.NewWorkBudget(live), live, boreAuditOps)
	failFrom := []int{0, 1, 2, 3}
	prev := 0
	for _, r := range reference {
		for _, k := range []int{(prev + r.calls) / 2, r.calls - 1, r.calls, r.calls + 1, r.calls + 2} {
			if k > 0 {
				failFrom = append(failFrom, k)
			}
		}
		prev = r.calls
	}
	require.Greater(t, prev, 900, "the sequence must poll through every long op")
	for _, k := range failFrom {
		wantCtx, gotCtx := &countingContext{failFrom: k}, &countingContext{failFrom: k}
		want := runBudgetOps(proofbound.NewWorkBudget(wantCtx), wantCtx, boreAuditOps)
		got := runBudgetOps(proofbound.NewPollWorkBudget(gotCtx), gotCtx, boreAuditOps)
		for i := range want {
			require.Equal(t, want[i], got[i], "fail from Err call %d, op %d (%s %d)",
				k, i, budgetOpNames[boreAuditOps[i].kind], boreAuditOps[i].n)
		}
	}
}

// TestStepNLoopsStepFnBudgets checks that budgets with a StepFn get the plain
// Step loop, at limits n-1, n and n+1.
func TestStepNLoopsStepFnBudgets(t *testing.T) {
	for _, n := range []uint64{0, 1, 255, 256, 257, 1000} {
		for _, limit := range []uint64{max(n, 1) - 1, n, n + 1} {
			var steps uint64
			wall := proofbound.NewWallWorkBudget(limit)
			literal := &proofbound.WorkBudget{
				StepFn: func() error {
					if steps == limit {
						return proofbound.ErrWallWorkBudget
					}
					steps++
					return nil
				},
				ErrFn: func() error { return nil },
			}
			wantDone := min(n, limit)
			wantErr := error(nil)
			if n > limit {
				wantErr = proofbound.ErrWallWorkBudget
			}
			for name, b := range map[string]*proofbound.WorkBudget{"wall": wall, "literal": literal} {
				done, err := b.StepN(n)
				require.Equal(t, wantDone, done, "%s n %d limit %d", name, n, limit)
				require.True(t, errors.Is(err, wantErr) && (err == nil) == (wantErr == nil),
					"%s n %d limit %d: %v", name, n, limit, err)
			}
			require.Equal(t, wantDone, steps)
		}
	}
}
