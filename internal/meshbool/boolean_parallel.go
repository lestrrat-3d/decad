package meshbool

import (
	"context"
	"errors"
	"runtime"
	"sync"
)

var ErrContactBatchStop = errors.New("decad: contact batch stopped")

type ContactWorkerContextKey struct{}

// WithContactWorkers sets how many workers each contact batch under ctx
// runs on. Keeping the worker count in the call context lets concurrent
// boolean calls choose different caps without mutating package state: tests
// and benchmarks pick a cap, and Verify's parallel pair pool runs every
// boolean inside its pairs on one worker. The count never changes a result.
func WithContactWorkers(ctx context.Context, workers int) context.Context {
	if workers < 1 {
		workers = 1
	}
	return context.WithValue(ctx, ContactWorkerContextKey{}, workers)
}

func ContactWorkers(ctx context.Context) int {
	if workers, ok := ctx.Value(ContactWorkerContextKey{}).(int); ok && workers > 0 {
		return workers
	}
	return DefaultContactWorkers()
}

// ContactPair names one facet pair in the order the producer discovered it.
// The pair order is part of the boolean evaluator's determinism contract: the
// batch executor may complete work out of order, but it never changes the
// order in which the caller observes contacts or errors.
type ContactPair struct {
	I int
	J int
}

type ContactBatchResult struct {
	Contact TriContact
	Err     error
}

// ContactBatchRunner is injectable so internal tests can force arbitrary
// completion order without changing the production executor. The runner must
// fill every result slot in input order; the production runner writes by slot
// while workers finish in any order.
type ContactBatchRunner func(context.Context, *BoolMesh, *BoolMesh, []ContactPair, []ContactBatchResult, int) error

// ContactBatchExecutor collects one ordered batch of candidate pairs. Memo
// reads happen while the producer adds candidates, and memo writes happen in
// flush after all worker results are complete. The production runner prepares
// cached float normals before starting workers. Workers therefore only read
// immutable mesh data and write their own result slot.
type ContactBatchExecutor struct {
	Ctx     context.Context //nolint:containedctx // one call-scoped cancellation source for this short-lived executor.
	Ma      *BoolMesh
	Mb      *BoolMesh
	Memo    *ContactMemo
	Workers int
	Limit   int
	Run     ContactBatchRunner
	Consume func(ContactPair, TriContact) error
	Batch   []ContactBatchEntry
	Misses  []ContactPair
	Results []ContactBatchResult
}

type ContactBatchEntry struct {
	Pair    ContactPair
	Contact TriContact
	Miss    bool
}

func NewContactBatchExecutor(ctx context.Context, ma, mb *BoolMesh, memo *ContactMemo, workers int, consume func(ContactPair, TriContact) error) *ContactBatchExecutor {
	if workers < 1 {
		workers = 1
	}
	return &ContactBatchExecutor{
		Ctx:     ctx,
		Ma:      ma,
		Mb:      mb,
		Memo:    memo,
		Workers: workers,
		Limit:   ContactBatchSize,
		Run:     RunContactBatch,
		Consume: consume,
	}
}

// ContactBatchSize bounds both queued work and result storage. A single batch
// is in flight, so an ordered error can cause at most this many extra
// classifications after its position has been reached by a worker.
const ContactBatchSize = 256

// add records one candidate in discovery order. A memo hit remains in the
// ordered batch so a cached result cannot leapfrog an earlier uncached pair.
func (e *ContactBatchExecutor) Add(i, j int) error {
	if err := e.Ctx.Err(); err != nil {
		return err
	}
	pair := ContactPair{I: i, J: j}
	entry := ContactBatchEntry{Pair: pair}
	if contact, ok := e.Memo.Lookup(i, j); ok {
		entry.Contact = contact
	} else {
		entry.Miss = true
	}
	if e.Batch == nil {
		e.Batch = make([]ContactBatchEntry, 0, ContactBatchSize)
	}
	e.Batch = append(e.Batch, entry)
	if len(e.Batch) < e.Limit {
		return nil
	}
	return e.Flush()
}

func (e *ContactBatchExecutor) Flush() error {
	if len(e.Batch) == 0 {
		return nil
	}
	e.Misses = e.Misses[:0]
	for _, entry := range e.Batch {
		if entry.Miss {
			if e.Misses == nil {
				e.Misses = make([]ContactPair, 0, ContactBatchSize)
			}
			e.Misses = append(e.Misses, entry.Pair)
		}
	}
	if cap(e.Results) < len(e.Misses) {
		e.Results = make([]ContactBatchResult, len(e.Misses), ContactBatchSize)
	} else {
		e.Results = e.Results[:len(e.Misses)]
	}
	if err := e.Run(e.Ctx, e.Ma, e.Mb, e.Misses, e.Results, e.Workers); err != nil {
		e.Batch = e.Batch[:0]
		return err
	}
	miss := 0
	for _, entry := range e.Batch {
		if err := e.Ctx.Err(); err != nil {
			e.Batch = e.Batch[:0]
			return err
		}
		contact := entry.Contact
		if entry.Miss {
			result := e.Results[miss]
			miss++
			if result.Err != nil {
				e.Batch = e.Batch[:0]
				return result.Err
			}
			contact = result.Contact
			e.Memo.Store(entry.Pair.I, entry.Pair.J, contact)
		}
		if e.Consume != nil {
			if err := e.Consume(entry.Pair, contact); err != nil {
				e.Batch = e.Batch[:0]
				return err
			}
		}
	}
	e.Batch = e.Batch[:0]
	return e.Ctx.Err()
}

func (e *ContactBatchExecutor) Done() error {
	if err := e.Flush(); err != nil {
		return err
	}
	return e.Ctx.Err()
}

// DefaultContactWorkers uses the process's measured parallelism and caps it
// at the largest worker count used by the evaluator's benchmark matrix. A
// single-process caller keeps the old serial path, while a large process does
// not oversubscribe the boolean operation with an unbounded private pool.
func DefaultContactWorkers() int {
	workers := runtime.GOMAXPROCS(0)
	if workers < 1 {
		return 1
	}
	if workers > 12 {
		return 12
	}
	return workers
}

// RunContactBatch executes one bounded batch. The producer never submits a
// second batch until this function returns, and each worker writes only its
// indexed result slot. The caller merges those slots in input order.
func RunContactBatch(ctx context.Context, ma, mb *BoolMesh, pairs []ContactPair, results []ContactBatchResult, workers int) error {
	if len(pairs) == 0 {
		return ctx.Err()
	}
	for i, pair := range pairs {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		ma.PrepareFloatNormal(pair.I)
		mb.PrepareFloatNormal(pair.J)
	}
	if workers > len(pairs) {
		workers = len(pairs)
	}
	if workers <= 1 {
		return runContactBatchSerial(ctx, ma, mb, pairs, results)
	}
	type job struct {
		slot int
		pair ContactPair
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
					results[next.slot] = classifyContactPair(ma, mb, next.pair)
				}
			}
		}()
	}
	for slot, pair := range pairs {
		select {
		case <-ctx.Done():
			close(jobs)
			wg.Wait()
			return ctx.Err()
		case jobs <- job{slot: slot, pair: pair}:
		}
	}
	close(jobs)
	wg.Wait()
	if err := ctx.Err(); err != nil {
		return err
	}
	return nil
}

// runContactBatchSerial is RunContactBatch on the calling goroutine alone,
// filling the same slots with the same classifications.
func runContactBatchSerial(ctx context.Context, ma, mb *BoolMesh, pairs []ContactPair, results []ContactBatchResult) error {
	for slot, pair := range pairs {
		if slot%64 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		results[slot] = classifyContactPair(ma, mb, pair)
	}
	return ctx.Err()
}

// classifyContactPair classifies one prepared facet pair.
func classifyContactPair(ma, mb *BoolMesh, pair ContactPair) ContactBatchResult {
	contact, err := TriTriClassifyPrepared(
		TriCorners(ma, pair.I),
		TriCorners(mb, pair.J),
		XtriCorners(ma, pair.I),
		XtriCorners(mb, pair.J),
		ma.Norms[pair.I], mb.Norms[pair.J],
		ma.Fnorms[pair.I], mb.Fnorms[pair.J],
	)
	return ContactBatchResult{Contact: contact, Err: err}
}
