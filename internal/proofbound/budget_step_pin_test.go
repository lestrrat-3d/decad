package proofbound_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// TestNewWorkBudgetStepPollsAtMultiples pins NewWorkBudget's Step: the counter
// charges first, then polls ctx exactly when the count reaches a multiple of
// WorkPollInterval. A cancellation made after cancelAfter steps is first seen
// at the step number wantFirst, and a failing poll is not sticky: the steps
// after it succeed until the next multiple, where the poll fails again.
func TestNewWorkBudgetStepPollsAtMultiples(t *testing.T) {
	require.Equal(t, 256, proofbound.WorkPollInterval)
	tests := []struct {
		name        string
		cancelAfter int // steps taken before the cancel; -1 cancels before the first step
		wantFirst   int // 1-based step that first returns the error; 0 for none
	}{
		{"before first step", -1, 256},
		{"after 1 step", 1, 256},
		{"after 255 steps", 255, 256},
		{"after 256 steps", 256, 512},
		{"after 300 steps", 300, 512},
		{"after 511 steps", 511, 512},
		{"after 512 steps", 512, 768},
		{"never", 1 << 30, 0},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(t.Context())
			defer cancel()
			if tt.cancelAfter < 0 {
				cancel()
			}
			b := proofbound.NewWorkBudget(ctx)
			var errSteps []int
			for step := 1; step <= 1100; step++ {
				err := b.Step()
				if err != nil {
					require.Equal(t, context.Canceled, err)
					errSteps = append(errSteps, step)
				}
				if step == tt.cancelAfter {
					cancel()
				}
			}
			if tt.wantFirst == 0 {
				require.Empty(t, errSteps)
				return
			}
			require.Equal(t, tt.wantFirst, errSteps[0])
			// Not sticky: only multiples of the interval fail.
			for _, s := range errSteps {
				require.Zero(t, s%proofbound.WorkPollInterval, "step %d", s)
			}
			require.Len(t, errSteps, (1100/proofbound.WorkPollInterval)-(tt.wantFirst/proofbound.WorkPollInterval)+1)
		})
	}
}

// TestNewWorkBudgetStepCountsPolls pins how many times Step reads the context.
func TestNewWorkBudgetStepCountsPolls(t *testing.T) {
	for _, tt := range []struct{ steps, wantCalls int }{
		{0, 0}, {255, 0}, {256, 1}, {511, 1}, {512, 2}, {1000, 3},
	} {
		c := &countingContext{}
		b := proofbound.NewWorkBudget(c)
		for range tt.steps {
			require.NoError(t, b.Step())
		}
		require.Equal(t, tt.wantCalls, c.calls, "steps %d", tt.steps)
		require.NoError(t, b.Err())
		require.Equal(t, tt.wantCalls+1, c.calls, "Err polls unconditionally")
	}
}

// TestNewWorkBudgetStepNPinned pins StepN on a context that is already
// cancelled: the result is (steps that succeeded, the error), the failing poll
// counts toward the work total, and later steps continue from there.
func TestNewWorkBudgetStepNPinned(t *testing.T) {
	tests := []struct {
		name     string
		prior    int    // Step calls made (on a live context) before StepN
		n        uint64 // StepN argument
		wantDone uint64
		wantErr  bool
		wantNext int // 1-based Step call after StepN that first errors; 0 for none within 600
	}{
		{"zero", 0, 0, 0, false, 256},
		{"one", 0, 1, 1, false, 255},
		{"below interval", 0, 255, 255, false, 1},
		{"exactly interval", 0, 256, 255, true, 256},
		{"above interval", 0, 257, 255, true, 256},
		{"many intervals", 0, 600, 255, true, 256},
		{"mid interval ends before poll", 100, 155, 155, false, 1},
		{"mid interval crosses poll", 100, 200, 155, true, 256},
		{"on boundary", 256, 1, 1, false, 255},
		{"just before boundary", 255, 1, 0, true, 256},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(t.Context())
			b := proofbound.NewWorkBudget(ctx)
			for range tt.prior {
				require.NoError(t, b.Step())
			}
			cancel()
			done, err := b.StepN(tt.n)
			require.Equal(t, tt.wantDone, done)
			if tt.wantErr {
				require.Equal(t, context.Canceled, err)
			} else {
				require.NoError(t, err)
			}
			first := 0
			for i := 1; i <= 600; i++ {
				if b.Step() != nil {
					first = i
					break
				}
			}
			require.Equal(t, tt.wantNext, first)
		})
	}
}
