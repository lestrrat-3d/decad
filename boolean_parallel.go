package decad

import (
	"context"
	"errors"
	"runtime"
	"sync"
)

var errContactBatchStop = errors.New("decad: contact batch stopped")

type contactWorkerContextKey struct{}

// withContactWorkers is an internal test/benchmark hook. Keeping the worker
// count in the call context lets concurrent boolean calls choose different
// caps without mutating package state.
func withContactWorkers(ctx context.Context, workers int) context.Context {
	if workers < 1 {
		workers = 1
	}
	return context.WithValue(ctx, contactWorkerContextKey{}, workers)
}

func contactWorkers(ctx context.Context) int {
	if workers, ok := ctx.Value(contactWorkerContextKey{}).(int); ok && workers > 0 {
		return workers
	}
	return defaultContactWorkers()
}

// contactPair names one facet pair in the order the producer discovered it.
// The pair order is part of the boolean evaluator's determinism contract: the
// batch executor may complete work out of order, but it never changes the
// order in which the caller observes contacts or errors.
type contactPair struct {
	i int
	j int
}

type contactBatchResult struct {
	contact triContact
	err     error
}

// contactBatchRunner is injectable so internal tests can force arbitrary
// completion order without changing the production executor. The returned
// slice MUST use the input order; the production runner writes by slot while
// workers finish in any order.
type contactBatchRunner func(context.Context, *boolMesh, *boolMesh, []contactPair, int) ([]contactBatchResult, error)

// contactBatchExecutor collects one ordered batch of candidate pairs. Memo
// reads happen while the producer adds candidates, and memo writes happen in
// flush after all worker results are complete. Workers therefore only read
// immutable meshes and write their own result slot.
type contactBatchExecutor struct {
	ctx     context.Context //nolint:containedctx // one call-scoped cancellation source for this short-lived executor.
	ma      *boolMesh
	mb      *boolMesh
	memo    *contactMemo
	workers int
	limit   int
	run     contactBatchRunner
	consume func(contactPair, triContact) error
	batch   []contactBatchEntry
}

type contactBatchEntry struct {
	pair    contactPair
	contact triContact
	miss    bool
}

func newContactBatchExecutor(ctx context.Context, ma, mb *boolMesh, memo *contactMemo, workers int, consume func(contactPair, triContact) error) *contactBatchExecutor {
	if workers < 1 {
		workers = 1
	}
	return &contactBatchExecutor{
		ctx:     ctx,
		ma:      ma,
		mb:      mb,
		memo:    memo,
		workers: workers,
		limit:   contactBatchSize,
		run:     runContactBatch,
		consume: consume,
		batch:   make([]contactBatchEntry, 0, contactBatchSize),
	}
}

// contactBatchSize bounds both queued work and result storage. A single batch
// is in flight, so an ordered error can cause at most this many extra
// classifications after its position has been reached by a worker.
const contactBatchSize = 256

// add records one candidate in discovery order. A memo hit remains in the
// ordered batch so a cached result cannot leapfrog an earlier uncached pair.
func (e *contactBatchExecutor) add(i, j int) error {
	if err := e.ctx.Err(); err != nil {
		return err
	}
	pair := contactPair{i: i, j: j}
	entry := contactBatchEntry{pair: pair}
	if contact, ok := e.memo.lookup(i, j); ok {
		entry.contact = contact
	} else {
		entry.miss = true
	}
	e.batch = append(e.batch, entry)
	if len(e.batch) < e.limit {
		return nil
	}
	return e.flush()
}

func (e *contactBatchExecutor) flush() error {
	if len(e.batch) == 0 {
		return nil
	}
	misses := make([]contactPair, 0, len(e.batch))
	for _, entry := range e.batch {
		if entry.miss {
			misses = append(misses, entry.pair)
		}
	}
	results, err := e.run(e.ctx, e.ma, e.mb, misses, e.workers)
	if err != nil {
		e.batch = e.batch[:0]
		return err
	}
	miss := 0
	for _, entry := range e.batch {
		if err := e.ctx.Err(); err != nil {
			e.batch = e.batch[:0]
			return err
		}
		contact := entry.contact
		if entry.miss {
			result := results[miss]
			miss++
			if result.err != nil {
				e.batch = e.batch[:0]
				return result.err
			}
			contact = result.contact
			e.memo.store(entry.pair.i, entry.pair.j, contact)
		}
		if e.consume != nil {
			if err := e.consume(entry.pair, contact); err != nil {
				e.batch = e.batch[:0]
				return err
			}
		}
	}
	e.batch = e.batch[:0]
	return e.ctx.Err()
}

func (e *contactBatchExecutor) done() error {
	if err := e.flush(); err != nil {
		return err
	}
	return e.ctx.Err()
}

// defaultContactWorkers uses the process's measured parallelism and caps it
// at the largest worker count used by the evaluator's benchmark matrix. A
// single-process caller keeps the old serial path, while a large process does
// not oversubscribe the boolean operation with an unbounded private pool.
func defaultContactWorkers() int {
	workers := runtime.GOMAXPROCS(0)
	if workers < 1 {
		return 1
	}
	if workers > 12 {
		return 12
	}
	return workers
}

// runContactBatch executes one bounded batch. The producer never submits a
// second batch until this function returns, and each worker writes only its
// indexed result slot. The caller merges those slots in input order.
func runContactBatch(ctx context.Context, ma, mb *boolMesh, pairs []contactPair, workers int) ([]contactBatchResult, error) {
	results := make([]contactBatchResult, len(pairs))
	if len(pairs) == 0 {
		return results, ctx.Err()
	}
	if workers > len(pairs) {
		workers = len(pairs)
	}
	if workers < 1 {
		workers = 1
	}
	type job struct {
		slot int
		pair contactPair
	}
	jobs := make(chan job, workers)
	var wg sync.WaitGroup
	wg.Add(workers)
	for range workers {
		go func() {
			defer wg.Done()
			for {
				select {
				case <-ctx.Done():
					return
				case next, ok := <-jobs:
					if !ok {
						return
					}
					if err := ctx.Err(); err != nil {
						return
					}
					contact, err := triTriClassify(
						triCorners(ma, next.pair.i),
						triCorners(mb, next.pair.j),
						xtriCorners(ma, next.pair.i),
						xtriCorners(mb, next.pair.j),
						ma.norms[next.pair.i], mb.norms[next.pair.j],
					)
					results[next.slot] = contactBatchResult{contact: contact, err: err}
				}
			}
		}()
	}
	for slot, pair := range pairs {
		select {
		case <-ctx.Done():
			close(jobs)
			wg.Wait()
			return nil, ctx.Err()
		case jobs <- job{slot: slot, pair: pair}:
		}
	}
	close(jobs)
	wg.Wait()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return results, nil
}
