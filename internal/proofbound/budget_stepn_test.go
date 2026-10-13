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
