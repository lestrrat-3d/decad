package orderedwork

import (
	"runtime"
	"sync/atomic"
)

var extraWorkers atomic.Int64

// ReserveWorkers shares extra workers across calls without waiting. Each
// call keeps one worker, so nested work can always make progress. The caller
// releases the returned reservation only after all its workers have joined.
// The returned worker count is at least one, even when workers is smaller.
func ReserveWorkers(workers int) (int, int) {
	limit := max(0, runtime.GOMAXPROCS(0)-1)
	for {
		used := extraWorkers.Load()
		extra := min(workers-1, max(0, limit-int(used)))
		if extra <= 0 {
			return 1, 0
		}
		if extraWorkers.CompareAndSwap(used, used+int64(extra)) {
			return 1 + extra, extra
		}
	}
}

// ReleaseWorkers returns a reservation after its workers have joined.
// Each reservation must be released exactly once.
func ReleaseWorkers(reserved int) { extraWorkers.Add(-int64(reserved)) }
