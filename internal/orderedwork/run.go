// Package orderedwork runs independent jobs concurrently and returns their results in input order.
package orderedwork

import (
	"context"
	"sync"
	"sync/atomic"
)

type workerContextKey struct{}

// WithWorkers overrides the worker count for one call through its context.
func WithWorkers(ctx context.Context, workers int) context.Context {
	if workers < 1 {
		workers = 1
	}
	return context.WithValue(ctx, workerContextKey{}, workers)
}

// Workers reads an override or returns the caller's default worker count.
func Workers(ctx context.Context, fallback int) int {
	if workers, ok := ctx.Value(workerContextKey{}).(int); ok && workers > 0 {
		return workers
	}
	return fallback
}

// Run returns results in job order. A failure returns the error at the lowest
// failing index, after all preceding jobs finish. The caller supplies a
// parallel context for work that needs different nested concurrency settings;
// the single-worker path uses ctx itself. Both contexts must share cancellation.
func Run[Job, Result any](ctx, parallelCtx context.Context, jobs []Job, workers int,
	work func(context.Context, Job) (Result, error)) ([]Result, error) {
	out := make([]Result, len(jobs))
	if workers > len(jobs) {
		workers = len(jobs)
	}
	if workers <= 1 {
		for i, job := range jobs {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			res, err := work(ctx, job)
			if err != nil {
				return nil, err
			}
			out[i] = res
		}
		return out, nil
	}

	errs := make([]error, len(jobs))
	var next atomic.Int64
	var firstErr atomic.Int64
	firstErr.Store(int64(len(jobs)))
	fail := func(i int, err error) {
		errs[i] = err
		for {
			cur := firstErr.Load()
			if int64(i) >= cur || firstErr.CompareAndSwap(cur, int64(i)) {
				return
			}
		}
	}
	var wg sync.WaitGroup
	wg.Add(workers)
	for range workers {
		go func() {
			defer wg.Done()
			for {
				i := int(next.Add(1) - 1)
				if i >= len(jobs) || int64(i) > firstErr.Load() {
					return
				}
				if err := parallelCtx.Err(); err != nil {
					fail(i, err)
					return
				}
				res, err := work(parallelCtx, jobs[i])
				if err != nil {
					fail(i, err)
					continue
				}
				out[i] = res
			}
		}()
	}
	wg.Wait()
	if i := firstErr.Load(); i < int64(len(jobs)) {
		return nil, errs[i]
	}
	return out, nil
}
