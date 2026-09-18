package decad

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestContactBatchMergesOutOfOrderCompletionsInInputOrder(t *testing.T) {
	ctx := t.Context()
	ma, mb := &boolMesh{}, &boolMesh{}
	memo := newContactMemo(ma, mb)
	var got []contactPair
	e := newContactBatchExecutor(ctx, ma, mb, memo, 4, func(pair contactPair, _ triContact) error {
		got = append(got, pair)
		return nil
	})
	e.limit = 4
	e.run = func(_ context.Context, _ *boolMesh, _ *boolMesh, pairs []contactPair, _ int) ([]contactBatchResult, error) {
		results := make([]contactBatchResult, len(pairs))
		for i := len(pairs) - 1; i >= 0; i-- {
			// Completion order is reverse input order. Results retain their
			// indexed slots, as production workers do.
			results[i] = contactBatchResult{contact: triContact{kind: contactPoint, p0: xpt{}}}
		}
		return results, nil
	}
	for i := range 4 {
		require.NoError(t, e.add(i, i+1))
	}
	require.NoError(t, e.done())
	require.Equal(t, []contactPair{{0, 1}, {1, 2}, {2, 3}, {3, 4}}, got)
}

func TestContactBatchReturnsEarliestOrderedError(t *testing.T) {
	ctx := t.Context()
	ma, mb := &boolMesh{}, &boolMesh{}
	memo := newContactMemo(ma, mb)
	first := errors.New("first ordered error")
	later := errors.New("later ordered error")
	e := newContactBatchExecutor(ctx, ma, mb, memo, 8, nil)
	e.limit = 3
	e.run = func(_ context.Context, _ *boolMesh, _ *boolMesh, pairs []contactPair, _ int) ([]contactBatchResult, error) {
		results := make([]contactBatchResult, len(pairs))
		results[2] = contactBatchResult{err: later}
		results[1] = contactBatchResult{err: first}
		return results, nil
	}
	require.NoError(t, e.add(0, 0))
	require.NoError(t, e.add(1, 1))
	require.ErrorIs(t, e.add(2, 2), first)
}

func TestContactBatchSchedulesOnlyMemoMisses(t *testing.T) {
	ctx := t.Context()
	ma, mb := &boolMesh{}, &boolMesh{}
	memo := newContactMemo(ma, mb)
	cached := triContact{kind: contactSegment}
	memo.store(1, 1, cached)
	var scheduled []contactPair
	e := newContactBatchExecutor(ctx, ma, mb, memo, 2, nil)
	e.limit = 3
	e.run = func(_ context.Context, _ *boolMesh, _ *boolMesh, pairs []contactPair, _ int) ([]contactBatchResult, error) {
		scheduled = append(scheduled, pairs...)
		results := make([]contactBatchResult, len(pairs))
		for i := range results {
			results[i].contact = triContact{kind: contactPoint}
		}
		return results, nil
	}
	require.NoError(t, e.add(0, 0))
	require.NoError(t, e.add(1, 1))
	require.NoError(t, e.add(2, 2))
	require.NoError(t, e.done())
	require.Equal(t, []contactPair{{0, 0}, {2, 2}}, scheduled)
	got, ok := memo.lookup(1, 1)
	require.True(t, ok)
	require.Equal(t, cached, got)
}

func TestContactBatchStopsOnCancellationDuringMerge(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	ma, mb := &boolMesh{}, &boolMesh{}
	memo := newContactMemo(ma, mb)
	merged := 0
	e := newContactBatchExecutor(ctx, ma, mb, memo, 2, func(contactPair, triContact) error {
		merged++
		cancel()
		return nil
	})
	e.limit = 2
	e.run = func(_ context.Context, _ *boolMesh, _ *boolMesh, pairs []contactPair, _ int) ([]contactBatchResult, error) {
		results := make([]contactBatchResult, len(pairs))
		for i := range results {
			results[i].contact = triContact{kind: contactPoint}
		}
		return results, nil
	}
	require.NoError(t, e.add(0, 0))
	require.ErrorIs(t, e.add(1, 1), context.Canceled)
	require.Equal(t, 1, merged)
}

func TestMeshBooleanWorkerCountsProduceIdenticalResults(t *testing.T) {
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 20, 20, 8)
	b := internalDiscBody(t, doc, 6, 20)
	tr, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: -6})
	require.NoError(t, err)
	placed, err := b.Placed(tr)
	require.NoError(t, err)

	serial, err := evaluateBoolean(withContactWorkers(t.Context(), 1), OpUnion, a, placed)
	require.NoError(t, err)
	for _, workers := range []int{2, 4, 8, 12} {
		parallel, err := evaluateBoolean(withContactWorkers(t.Context(), workers), OpUnion, a, placed)
		require.NoError(t, err)
		require.Equal(t, serial.payload, parallel.payload, "worker count %d changed held geometry", workers)
		require.Equal(t, serial.volume, parallel.volume, "worker count %d changed volume", workers)
	}
}
