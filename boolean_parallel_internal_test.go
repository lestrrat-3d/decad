package decad

import (
	"context"
	"errors"
	"slices"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestContactBatchMergesOutOfOrderCompletionsInInputOrder(t *testing.T) {
	ctx := t.Context()
	ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
	memo := meshbool.NewContactMemo(ma, mb)
	var got []meshbool.ContactPair
	e := meshbool.NewContactBatchExecutor(ctx, ma, mb, memo, 4, func(pair meshbool.ContactPair, _ meshbool.TriContact) error {
		got = append(got, pair)
		return nil
	})
	e.Limit = 4
	e.Run = func(_ context.Context, _ *meshbool.BoolMesh, _ *meshbool.BoolMesh, pairs []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
		for i := range slices.Backward(pairs) {
			// Completion order is reverse input order. Results retain their
			// indexed slots, as production workers do.
			results[i] = meshbool.ContactBatchResult{Contact: meshbool.TriContact{Kind: meshbool.ContactPoint, P0: proof.Xpt{}}}
		}
		return nil
	}
	for i := range 4 {
		require.NoError(t, e.Add(i, i+1))
	}
	require.NoError(t, e.Done())
	require.Equal(t, []meshbool.ContactPair{{I: 0, J: 1}, {I: 1, J: 2}, {I: 2, J: 3}, {I: 3, J: 4}}, got)
}

func TestContactBatchReturnsEarliestOrderedError(t *testing.T) {
	ctx := t.Context()
	ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
	memo := meshbool.NewContactMemo(ma, mb)
	first := errors.New("first ordered error")
	later := errors.New("later ordered error")
	e := meshbool.NewContactBatchExecutor(ctx, ma, mb, memo, 8, nil)
	e.Limit = 3
	e.Run = func(_ context.Context, _ *meshbool.BoolMesh, _ *meshbool.BoolMesh, _ []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
		results[2] = meshbool.ContactBatchResult{Err: later}
		results[1] = meshbool.ContactBatchResult{Err: first}
		return nil
	}
	require.NoError(t, e.Add(0, 0))
	require.NoError(t, e.Add(1, 1))
	require.ErrorIs(t, e.Add(2, 2), first)
}

func TestContactBatchSchedulesOnlyMemoMisses(t *testing.T) {
	ctx := t.Context()
	ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
	memo := meshbool.NewContactMemo(ma, mb)
	cached := meshbool.TriContact{Kind: meshbool.ContactSegment}
	memo.Store(1, 1, cached)
	var scheduled []meshbool.ContactPair
	e := meshbool.NewContactBatchExecutor(ctx, ma, mb, memo, 2, nil)
	e.Limit = 3
	e.Run = func(_ context.Context, _ *meshbool.BoolMesh, _ *meshbool.BoolMesh, pairs []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
		scheduled = append(scheduled, pairs...)
		for i := range results {
			results[i].Contact = meshbool.TriContact{Kind: meshbool.ContactPoint}
		}
		return nil
	}
	require.NoError(t, e.Add(0, 0))
	require.NoError(t, e.Add(1, 1))
	require.NoError(t, e.Add(2, 2))
	require.NoError(t, e.Done())
	require.Equal(t, []meshbool.ContactPair{{I: 0, J: 0}, {I: 2, J: 2}}, scheduled)
	got, ok := memo.Lookup(1, 1)
	require.True(t, ok)
	require.Equal(t, cached, got)
}

func TestContactBatchStopsOnCancellationDuringMerge(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
	memo := meshbool.NewContactMemo(ma, mb)
	merged := 0
	e := meshbool.NewContactBatchExecutor(ctx, ma, mb, memo, 2, func(meshbool.ContactPair, meshbool.TriContact) error {
		merged++
		cancel()
		return nil
	})
	e.Limit = 2
	e.Run = func(_ context.Context, _ *meshbool.BoolMesh, _ *meshbool.BoolMesh, _ []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
		for i := range results {
			results[i].Contact = meshbool.TriContact{Kind: meshbool.ContactPoint}
		}
		return nil
	}
	require.NoError(t, e.Add(0, 0))
	require.ErrorIs(t, e.Add(1, 1), context.Canceled)
	require.Equal(t, 1, merged)
}

func TestContactBatchAllocatesBuffersOnDemand(t *testing.T) {
	t.Run("empty", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		e := meshbool.NewContactBatchExecutor(ctx, &meshbool.BoolMesh{}, &meshbool.BoolMesh{}, nil, 1, nil)
		require.NoError(t, e.Done())
		require.Nil(t, e.Batch)
		require.Nil(t, e.Misses)
		require.Nil(t, e.Results)
		cancel()
		require.ErrorIs(t, e.Done(), context.Canceled)
		require.Nil(t, e.Batch)
		require.Nil(t, e.Misses)
		require.Nil(t, e.Results)
	})

	t.Run("partial and repeated flushes", func(t *testing.T) {
		ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
		var got []meshbool.ContactPair
		e := meshbool.NewContactBatchExecutor(t.Context(), ma, mb, meshbool.NewContactMemo(ma, mb), 1,
			func(pair meshbool.ContactPair, _ meshbool.TriContact) error {
				got = append(got, pair)
				return nil
			})
		e.Limit = 2
		e.Run = func(_ context.Context, _, _ *meshbool.BoolMesh, pairs []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
			for i := range pairs {
				results[i].Contact = meshbool.TriContact{Kind: meshbool.ContactPoint}
			}
			return nil
		}
		require.NoError(t, e.Add(0, 0))
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Batch))
		require.Nil(t, e.Misses)
		require.Nil(t, e.Results)
		require.NoError(t, e.Add(1, 1))
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Misses))
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Results))
		batchBuf := &e.Batch[:1][0]
		missBuf := &e.Misses[:1][0]
		resultBuf := &e.Results[:1][0]
		require.NoError(t, e.Add(2, 2))
		require.NoError(t, e.Done())
		require.Same(t, batchBuf, &e.Batch[:1][0])
		require.Same(t, missBuf, &e.Misses[:1][0])
		require.Same(t, resultBuf, &e.Results[:1][0])
		require.Equal(t, []meshbool.ContactPair{{I: 0, J: 0}, {I: 1, J: 1}, {I: 2, J: 2}}, got)
	})

	t.Run("full batch", func(t *testing.T) {
		ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
		var got []meshbool.ContactPair
		e := meshbool.NewContactBatchExecutor(t.Context(), ma, mb, meshbool.NewContactMemo(ma, mb), 1,
			func(pair meshbool.ContactPair, _ meshbool.TriContact) error {
				got = append(got, pair)
				return nil
			})
		e.Run = func(_ context.Context, _, _ *meshbool.BoolMesh, pairs []meshbool.ContactPair, _ []meshbool.ContactBatchResult, _ int) error {
			require.Len(t, pairs, meshbool.ContactBatchSize)
			return nil
		}
		for i := range meshbool.ContactBatchSize {
			require.NoError(t, e.Add(i, i))
		}
		require.Len(t, got, meshbool.ContactBatchSize)
		require.Equal(t, meshbool.ContactPair{I: 0, J: 0}, got[0])
		require.Equal(t, meshbool.ContactPair{I: meshbool.ContactBatchSize - 1, J: meshbool.ContactBatchSize - 1}, got[len(got)-1])
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Batch))
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Misses))
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Results))
	})

	t.Run("memo hits", func(t *testing.T) {
		ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
		memo := meshbool.NewContactMemo(ma, mb)
		memo.Store(0, 0, meshbool.TriContact{Kind: meshbool.ContactPoint})
		e := meshbool.NewContactBatchExecutor(t.Context(), ma, mb, memo, 1, nil)
		e.Run = func(_ context.Context, _, _ *meshbool.BoolMesh, pairs []meshbool.ContactPair, results []meshbool.ContactBatchResult, _ int) error {
			require.Empty(t, pairs)
			require.Empty(t, results)
			return nil
		}
		require.NoError(t, e.Add(0, 0))
		require.NoError(t, e.Done())
		require.Equal(t, meshbool.ContactBatchSize, cap(e.Batch))
		require.Nil(t, e.Misses)
		require.Nil(t, e.Results)
	})
}

func TestMeshBooleanWorkerCountsProduceIdenticalResults(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 20, 20, 8)
	b := internalDiscBody(t, doc, 6, 20)
	tr, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: -6})
	require.NoError(t, err)
	placed, err := b.Placed(t.Context(), tr)
	require.NoError(t, err)

	serial, err := evaluateBoolean(meshbool.WithContactWorkers(t.Context(), 1), meshbool.OpUnion, a, placed)
	require.NoError(t, err)
	for _, workers := range []int{2, 4, 8, 12} {
		parallel, err := evaluateBoolean(meshbool.WithContactWorkers(t.Context(), workers), meshbool.OpUnion, a, placed)
		require.NoError(t, err)
		require.Equal(t, serial.payload, parallel.payload, "worker count %d changed held geometry", workers)
		require.Equal(t, serial.volume, parallel.volume, "worker count %d changed volume", workers)
	}
}

// BenchmarkContactBatchEmpty measures the executor path used when a face pair
// contributes no candidate facets after the box check.
func BenchmarkContactBatchEmpty(b *testing.B) {
	ctx := b.Context()
	ma, mb := &meshbool.BoolMesh{}, &meshbool.BoolMesh{}
	memo := meshbool.NewContactMemo(ma, mb)
	b.ReportAllocs()
	for b.Loop() {
		e := meshbool.NewContactBatchExecutor(ctx, ma, mb, memo, 1, nil)
		if err := e.Done(); err != nil {
			b.Fatal(err)
		}
	}
}

// BenchmarkMeshBooleanWorkers keeps operand tessellations cached so each
// sub-benchmark compares the same read-only boolean pipeline at one worker cap.
func BenchmarkMeshBooleanWorkers(b *testing.B) {
	for _, workers := range []int{1, 2, 4, 8, 12} {
		b.Run(strconv.Itoa(workers), func(b *testing.B) {
			doc := New()
			a := internalDiscBody(b, doc, 10, 20)
			tool := internalDiscBody(b, doc, 7, 22)
			tr, err := r3.Translation(r3.Vec{X: 10, Z: -11})
			if err != nil {
				b.Fatal(err)
			}
			placed, err := tool.Placed(b.Context(), tr)
			if err != nil {
				b.Fatal(err)
			}
			ctx := meshbool.WithContactWorkers(b.Context(), workers)
			if _, err := evaluateBoolean(ctx, meshbool.OpUnion, a, placed); err != nil {
				b.Fatal(err)
			}
			b.ResetTimer()
			for b.Loop() {
				if _, err := evaluateBoolean(ctx, meshbool.OpUnion, a, placed); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
