package orderedwork_test

import (
	"context"
	"errors"
	"runtime"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/stretchr/testify/require"
)

func TestWorkerReservationsFollowAvailableCPUs(t *testing.T) {
	prior := runtime.GOMAXPROCS(4)
	defer runtime.GOMAXPROCS(prior)
	workers, held := orderedwork.ReserveWorkers(4)
	defer func() { orderedwork.ReleaseWorkers(held) }()
	require.Equal(t, 4, workers)
	require.Equal(t, 3, held)

	// A smaller CPU allowance cannot revoke workers that are still running.
	// Further calls must still progress without taking an extra worker.
	runtime.GOMAXPROCS(2)
	workers, reserved := orderedwork.ReserveWorkers(4)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 1, workers)
	require.Zero(t, reserved)
	orderedwork.ReleaseWorkers(held)
	held = 0

	workers, reserved = orderedwork.ReserveWorkers(4)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 2, workers)
	require.Equal(t, 1, reserved)

	runtime.GOMAXPROCS(1)
	workers, reserved = orderedwork.ReserveWorkers(4)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 1, workers)
	require.Zero(t, reserved)
}

func TestWorkerReservationsShareExtrasWithoutWaiting(t *testing.T) {
	prior := runtime.GOMAXPROCS(4)
	defer runtime.GOMAXPROCS(prior)
	ctx, cancel := context.WithTimeout(t.Context(), 3*time.Second)
	defer cancel()
	const calls = 16
	grants := make(chan int, calls)
	release := make(chan struct{})
	var group sync.WaitGroup
	group.Add(calls)
	for range calls {
		go func() {
			defer group.Done()
			workers, reserved := orderedwork.ReserveWorkers(4)
			defer orderedwork.ReleaseWorkers(reserved)
			grants <- workers
			select {
			case <-release:
			case <-ctx.Done():
			}
		}()
	}
	// Always join, including when a broken reservation blocks until timeout.
	defer func() { cancel(); group.Wait() }()
	extras := 0
	for range calls {
		select {
		case workers := <-grants:
			require.GreaterOrEqual(t, workers, 1)
			extras += workers - 1
		case <-ctx.Done():
			t.Fatal("a reservation waited for another call to release its workers")
		}
	}
	require.Equal(t, 3, extras)
	close(release)
	group.Wait()
	workers, reserved := orderedwork.ReserveWorkers(4)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 4, workers, "every reservation was released after the join")
}

type budgetContextKey struct{}

func TestNestedRunAndEachKeepPollingAndParallelContext(t *testing.T) {
	prior := runtime.GOMAXPROCS(2)
	defer runtime.GOMAXPROCS(prior)
	ctx, cancel := context.WithTimeout(t.Context(), 3*time.Second)
	defer cancel()
	parallelCtx := context.WithValue(ctx, budgetContextKey{}, true)
	stop := errors.New("poll stopped the nested loop")
	release := make(chan struct{})
	var active, factories, calls atomic.Int32
	var sawParallel atomic.Bool
	polls := 0
	_, err := orderedwork.Run(ctx, parallelCtx, []int{0, 1}, 2,
		func(_ context.Context, job int) (int, error) {
			if job != 0 {
				return job, nil
			}
			_, err := orderedwork.Each(4, 4, 1, func() error {
				if calls.Load() != 1 || factories.Load() != 1 {
					return ctx.Err()
				}
				polls++
				close(release)
				return stop
			}, func(int) error {
				calls.Add(1)
				active.Add(1)
				defer active.Add(-1)
				_, err := orderedwork.RunWithWorker(ctx, parallelCtx, []int{0, 1}, 4,
					func() func(context.Context, int) (int, error) {
						factories.Add(1)
						return func(ctx context.Context, k int) (int, error) {
							sawParallel.Store(ctx.Value(budgetContextKey{}) == true)
							return k, nil
						}
					})
				if err != nil {
					return err
				}
				select {
				case <-release:
					return nil
				case <-ctx.Done():
					return ctx.Err()
				}
			})
			return 0, err
		})
	require.ErrorIs(t, err, stop)
	require.Equal(t, 1, polls)
	require.Equal(t, int32(1), calls.Load())
	require.Equal(t, int32(1), factories.Load())
	require.True(t, sawParallel.Load(), "a granted single worker still uses parallelCtx")
	require.Zero(t, active.Load(), "the nested worker joined before return")
	workers, reserved := orderedwork.ReserveWorkers(2)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 2, workers, "all outer and nested reservations were released")
	for _, serial := range []struct{ workers, jobs int }{{1, 2}, {4, 1}} {
		_, err = orderedwork.Run(ctx, parallelCtx, make([]int, serial.jobs), serial.workers,
			func(workCtx context.Context, k int) (int, error) {
				if workCtx.Value(budgetContextKey{}) != nil {
					return 0, errors.New("serial work received parallelCtx")
				}
				return k, nil
			})
		require.NoError(t, err)
	}
}
