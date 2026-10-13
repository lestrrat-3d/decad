package orderedwork_test

import (
	"errors"
	"fmt"
	"math/rand/v2"
	"runtime"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/stretchr/testify/require"
)

func TestEachVisitsEveryIndexOnce(t *testing.T) {
	for _, n := range []int{0, 1, 7, 1000, 4097} {
		for _, grain := range []int{0, 1, 3, 256} {
			for _, workers := range []int{1, 2, 4, 8} {
				visits := make([]atomic.Int32, n)
				first, err := orderedwork.Each(n, workers, grain, nil, func(i int) error {
					visits[i].Add(1)
					return nil
				})
				require.NoError(t, err)
				require.Equal(t, n, first)
				for i := range visits {
					require.Equal(t, int32(1), visits[i].Load(), "n %d grain %d workers %d index %d", n, grain, workers, i)
				}
			}
		}
	}
}

func TestEachReturnsTheSerialFirstFailure(t *testing.T) {
	r := rand.New(rand.NewPCG(5, 6))
	for trial := range 300 {
		n := 1 + r.IntN(3000)
		grain := 1 + r.IntN(300)
		workers := 1 + r.IntN(6)
		failing := make([]bool, n)
		want := n
		for range r.IntN(4) {
			i := r.IntN(n)
			failing[i] = true
			want = min(want, i)
		}
		visits := make([]atomic.Int32, n)
		first, err := orderedwork.Each(n, workers, grain, nil, func(i int) error {
			visits[i].Add(1)
			if failing[i] {
				return fmt.Errorf("item %d", i)
			}
			return nil
		})
		require.Equal(t, want, first, "trial %d", trial)
		if want == n {
			require.NoError(t, err)
		} else {
			require.EqualError(t, err, fmt.Sprintf("item %d", want), "trial %d", trial)
		}
		for i := range min(want+1, n) {
			require.Equal(t, int32(1), visits[i].Load(), "trial %d index %d", trial, i)
		}
	}
}

func TestEachStopsOnPollFailure(t *testing.T) {
	stop := errors.New("stop")
	before := runtime.NumGoroutine()
	var visits atomic.Int32
	polls := 0
	first, err := orderedwork.Each(400, 2, 4, func() error {
		polls++
		return stop
	}, func(int) error {
		visits.Add(1)
		time.Sleep(time.Millisecond)
		return nil
	})
	require.Equal(t, -1, first)
	require.ErrorIs(t, err, stop)
	require.Equal(t, 1, polls)
	after := visits.Load()
	require.Less(t, after, int32(400))
	time.Sleep(20 * time.Millisecond)
	require.Equal(t, after, visits.Load(), "a block started after Each returned")
	// require.Eventually runs its condition on a goroutine of its own, so
	// the count is read here.
	deadline := time.Now().Add(time.Second)
	for runtime.NumGoroutine() > before && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	require.LessOrEqual(t, runtime.NumGoroutine(), before)
}

func TestEachInlineStartsNoGoroutine(t *testing.T) {
	before := runtime.NumGoroutine()
	first, err := orderedwork.Each(10, 4, 16, func() error {
		t.Fatal("inline Each polled")
		return nil
	}, func(int) error {
		require.Equal(t, before, runtime.NumGoroutine())
		return nil
	})
	require.NoError(t, err)
	require.Equal(t, 10, first)
}
