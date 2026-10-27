package orderedwork_test

import (
	"context"
	"errors"
	"runtime"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/stretchr/testify/require"
)

func TestRunWithWorkerKeepsScratchLocalAndResultsOrdered(t *testing.T) {
	prior := runtime.GOMAXPROCS(4)
	defer runtime.GOMAXPROCS(prior)
	for _, workers := range []int{1, 2, 4} {
		ctx, cancel := context.WithTimeout(t.Context(), 3*time.Second)
		jobs := make([]int, 64)
		for i := range jobs {
			jobs[i] = i
		}
		var created atomic.Int32
		ready := make(chan struct{})
		got, err := orderedwork.RunWithWorker(ctx, ctx, jobs, workers,
			func() func(context.Context, int) (int, error) {
				var scratch [8]int
				if created.Add(1) == int32(workers) {
					close(ready)
				}
				return func(ctx context.Context, job int) (int, error) {
					select {
					case <-ready:
					case <-ctx.Done():
						return 0, ctx.Err()
					}
					for i := range scratch {
						scratch[i] = job
					}
					runtime.Gosched()
					sum := 0
					for _, v := range scratch {
						sum += v
					}
					return sum, nil
				}
			})
		cancel()
		require.NoError(t, err)
		require.Equal(t, int32(workers), created.Load())
		require.Len(t, got, len(jobs))
		for i, v := range got {
			require.Equal(t, 8*i, v, "workers %d job %d", workers, i)
		}
	}
	called := false
	got, err := orderedwork.RunWithWorker(t.Context(), t.Context(), []int(nil), 4,
		func() func(context.Context, int) (int, error) {
			called = true
			return func(context.Context, int) (int, error) { return 0, nil }
		})
	require.NoError(t, err)
	require.Empty(t, got)
	require.False(t, called, "empty input creates no worker")
}

func TestRunWithWorkerReturnsEarlierFailureAndJoins(t *testing.T) {
	prior := runtime.GOMAXPROCS(4)
	defer runtime.GOMAXPROCS(prior)
	ctx, cancel := context.WithTimeout(t.Context(), 3*time.Second)
	defer cancel()
	first, later := errors.New("job 1 failed"), errors.New("job 5 failed")
	laterFailed := make(chan struct{})
	var active atomic.Int32
	_, err := orderedwork.RunWithWorker(ctx, ctx, []int{0, 1, 2, 3, 4, 5, 6, 7}, 4,
		func() func(context.Context, int) (int, error) {
			return func(ctx context.Context, job int) (int, error) {
				active.Add(1)
				defer active.Add(-1)
				switch job {
				case 1:
					select {
					case <-laterFailed:
						return 0, first
					case <-ctx.Done():
						return 0, ctx.Err()
					}
				case 5:
					close(laterFailed)
					return 0, later
				default:
					return job, nil
				}
			}
		})
	require.ErrorIs(t, err, first)
	require.Zero(t, active.Load(), "all running jobs finished before return")
}
