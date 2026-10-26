package orderedwork

import (
	"runtime"
	"sync"
	"sync/atomic"
	"time"
)

// MaxEachWorkers caps the goroutines Each starts. The CI runner has four
// CPUs, and each race shard runs two test binaries on them at once.
const MaxEachWorkers = 4

// EachWorkers requests a worker count for a loop over n items: 1 below minItems,
// otherwise min(GOMAXPROCS, MaxEachWorkers).
func EachWorkers(n, minItems int) int {
	if n < minItems {
		return 1
	}
	return max(1, min(runtime.GOMAXPROCS(0), MaxEachWorkers))
}

// eachPollInterval is how often Each runs poll while its workers run.
const eachPollInterval = 50 * time.Millisecond

// Each runs fn(i) for every i in [0, n), handing out blocks of grain
// consecutive indices in ascending order to up to workers goroutines.
// ReserveWorkers shares extra workers across concurrent calls. A call with
// no extra worker available keeps one worker and the same caller-side polling
// while its work runs.
//
// fn must write only state owned by index i, and read only state that nothing
// writes during the call. fn must not call poll, a WorkBudget, or any method
// that fills a cache shared between indices.
//
// It returns the lowest index whose fn failed and that error, or n and nil.
// No block starts at or past the lowest failure recorded so far, and every
// index below the returned one has run, so the result is the one a serial loop
// over [0, n) returns, whatever the scheduling.
//
// poll, when non-nil, runs only on the calling goroutine, every
// eachPollInterval while the workers run. When it fails, no new block starts,
// Each waits for the running blocks and returns -1 and poll's error.
//
// With workers < 2, or a single block, Each starts no goroutine: it runs fn in
// index order on the calling goroutine and stops at the first failure, without
// calling poll. Every goroutine Each starts has finished when it returns.
// Each does not recover panics.
func Each(n, workers, grain int, poll func() error, fn func(i int) error) (int, error) {
	if grain < 1 {
		grain = 1
	}
	blocks := (n + grain - 1) / grain
	if workers < 2 || blocks < 2 {
		for i := range n {
			if err := fn(i); err != nil {
				return i, err
			}
		}
		return n, nil
	}
	workers = min(workers, blocks)
	workers, reserved := ReserveWorkers(workers)
	if reserved != 0 {
		defer ReleaseWorkers(reserved)
	}
	errs := make([]error, workers)
	errAt := make([]int, workers)
	var next, best atomic.Int64
	best.Store(int64(n))
	var stopped atomic.Bool
	var running atomic.Int64
	running.Store(int64(workers))
	done := make(chan struct{})
	var group sync.WaitGroup
	group.Add(workers)
	for w := range workers {
		errAt[w] = n
		go func() {
			defer group.Done()
			defer func() {
				if running.Add(-1) == 0 {
					close(done)
				}
			}()
			for !stopped.Load() {
				b := next.Add(1) - 1
				lo := b * int64(grain)
				if b >= int64(blocks) || lo >= best.Load() {
					return
				}
				hi := min(int(lo)+grain, n)
				for i := int(lo); i < hi; i++ {
					err := fn(i)
					if err == nil {
						continue
					}
					if i < errAt[w] {
						errs[w], errAt[w] = err, i
					}
					for {
						cur := best.Load()
						if int64(i) >= cur || best.CompareAndSwap(cur, int64(i)) {
							break
						}
					}
					break
				}
			}
		}()
	}
	if poll != nil {
		ticker := time.NewTicker(eachPollInterval)
		defer ticker.Stop()
		for waiting := true; waiting; {
			select {
			case <-done:
				waiting = false
			case <-ticker.C:
				if err := poll(); err != nil {
					stopped.Store(true)
					group.Wait()
					return -1, err
				}
			}
		}
	}
	group.Wait()
	first, firstErr := n, error(nil)
	for w, err := range errs {
		if err != nil && errAt[w] < first {
			first, firstErr = errAt[w], err
		}
	}
	return first, firstErr
}
