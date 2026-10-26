package orderedwork_test

import (
	"context"
	"errors"
	"fmt"
	"runtime"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/stretchr/testify/require"
)

type proofsKey struct{}

// waitGoroutines reads the goroutine count in a plain loop until it is back
// to want or one second passes.
func waitGoroutines(want int) int {
	deadline := time.Now().Add(time.Second)
	for runtime.NumGoroutine() > want && time.Now().Before(deadline) {
		time.Sleep(5 * time.Millisecond)
	}
	return runtime.NumGoroutine()
}

func TestProofsReturnsTheSerialFirstFailure(t *testing.T) {
	const n = 6
	for _, workers := range []int{1, 4} {
		for mask := range 1 << n {
			errs := make([]error, n)
			lowest := -1
			for k := range n {
				if mask&(1<<k) != 0 {
					errs[k] = fmt.Errorf("proof %d failed", k)
					if lowest < 0 {
						lowest = k
					}
				}
			}
			var ran [n]atomic.Bool
			err := orderedwork.Proofs(t.Context(), n, workers, func(_ context.Context, k int) error {
				time.Sleep(time.Duration(n-k) * time.Millisecond)
				ran[k].Store(true)
				return errs[k]
			})
			if lowest < 0 {
				require.NoError(t, err, "workers %d mask %b", workers, mask)
				lowest = n
			} else {
				require.Same(t, errs[lowest], err, "workers %d mask %b", workers, mask)
			}
			for k := range lowest {
				require.True(t, ran[k].Load(), "workers %d mask %b proof %d", workers, mask, k)
			}
		}
	}
}

func TestProofsBothFailingReturnsTheFirst(t *testing.T) {
	first, second := errors.New("first"), errors.New("second")
	for range 20 {
		err := orderedwork.Proofs(t.Context(), 2, 2, func(ctx context.Context, k int) error {
			if k == 0 {
				time.Sleep(10 * time.Millisecond)
				return first
			}
			return second
		})
		require.Same(t, first, err)
	}
}

func TestProofsCancelsLaterProofs(t *testing.T) {
	prior := runtime.GOMAXPROCS(2)
	defer runtime.GOMAXPROCS(prior)
	boom := errors.New("proof 0 failed")
	var saw atomic.Bool
	started := make(chan struct{})
	start := time.Now()
	err := orderedwork.Proofs(t.Context(), 2, 2, func(ctx context.Context, k int) error {
		if k == 0 {
			<-started
			return boom
		}
		close(started)
		<-ctx.Done()
		saw.Store(true)
		return ctx.Err()
	})
	require.Same(t, boom, err)
	require.True(t, saw.Load())
	require.Less(t, time.Since(start), time.Second)
}

func TestProofsReturnsCallerCancellation(t *testing.T) {
	block := func(ctx context.Context, _ int) error {
		<-ctx.Done()
		return errors.New("private context cancelled")
	}
	t.Run("canceled", func(t *testing.T) {
		before := runtime.NumGoroutine()
		ctx, cancel := context.WithCancel(t.Context())
		time.AfterFunc(20*time.Millisecond, cancel)
		start := time.Now()
		err := orderedwork.Proofs(ctx, 2, 2, block)
		require.ErrorIs(t, err, context.Canceled)
		require.Less(t, time.Since(start), time.Second)
		require.LessOrEqual(t, waitGoroutines(before), before)
	})
	t.Run("deadline", func(t *testing.T) {
		before := runtime.NumGoroutine()
		ctx, cancel := context.WithTimeout(t.Context(), 20*time.Millisecond)
		defer cancel()
		start := time.Now()
		err := orderedwork.Proofs(ctx, 2, 2, block)
		require.ErrorIs(t, err, context.DeadlineExceeded)
		require.Less(t, time.Since(start), time.Second)
		require.LessOrEqual(t, waitGoroutines(before), before)
	})
	t.Run("already canceled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		var ran atomic.Bool
		err := orderedwork.Proofs(ctx, 2, 2, func(context.Context, int) error {
			ran.Store(true)
			return nil
		})
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ran.Load())
	})
}

func TestProofsNeverHandsOutTheCallerContext(t *testing.T) {
	caller := context.WithValue(t.Context(), proofsKey{}, "kept")
	var handed [2]context.Context
	err := orderedwork.Proofs(caller, 2, 2, func(ctx context.Context, k int) error {
		handed[k] = ctx //nolint:fatcontext // records what each proof was handed
		return nil
	})
	require.NoError(t, err)
	for k, ctx := range handed {
		require.NotEqual(t, caller, ctx, "proof %d", k)
		require.Equal(t, "kept", ctx.Value(proofsKey{}), "proof %d", k)
	}
}

func TestProofsRaceBetweenCallerCancelAndProofSuccess(t *testing.T) {
	// A proof that finishes before the caller's context fails sees no
	// cancellation, and a failing caller context never reaches the proof as
	// the caller's own error value.
	caller, cancel := context.WithCancel(t.Context())
	defer cancel()
	err := orderedwork.Proofs(caller, 2, 2, func(ctx context.Context, _ int) error {
		cancel()
		return ctx.Err()
	})
	require.NoError(t, err)
}

func TestProofsOneWorkerRunsInOrderWithTheCallerContext(t *testing.T) {
	caller := context.WithValue(t.Context(), proofsKey{}, "kept")
	var order []int
	err := orderedwork.Proofs(caller, 3, 1, func(ctx context.Context, k int) error {
		require.Equal(t, caller, ctx)
		order = append(order, k)
		return nil
	})
	require.NoError(t, err)
	require.Equal(t, []int{0, 1, 2}, order)
}
