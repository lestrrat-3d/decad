package meshbool

import (
	"context"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestContactBatchStopsOnCancellationDuringMerge(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	ma, mb := &BoolMesh{}, &BoolMesh{}
	memo := NewContactMemo(ma, mb)
	merged := 0
	e := NewContactBatchExecutor(ctx, ma, mb, memo, 2, func(ContactPair, TriContact) error {
		merged++
		cancel()
		return nil
	})
	e.Limit = 2
	e.Run = func(_ context.Context, _ *BoolMesh, _ *BoolMesh, _ []ContactPair, results []ContactBatchResult, _ int) error {
		for i := range results {
			results[i].Contact = TriContact{Kind: ContactPoint}
		}
		return nil
	}
	require.NoError(t, e.Add(0, 0))
	require.ErrorIs(t, e.Add(1, 1), context.Canceled)
	require.Equal(t, 1, merged)
}

// BenchmarkContactBatchEmpty measures the executor path used when a face pair
// contributes no candidate facets after the box check.
func BenchmarkContactBatchEmpty(b *testing.B) {
	ctx := b.Context()
	ma, mb := &BoolMesh{}, &BoolMesh{}
	memo := NewContactMemo(ma, mb)
	b.ReportAllocs()
	for b.Loop() {
		e := NewContactBatchExecutor(ctx, ma, mb, memo, 1, nil)
		if err := e.Done(); err != nil {
			b.Fatal(err)
		}
	}
}
