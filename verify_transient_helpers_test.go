package decad

import (
	"sync"
	"testing"

	"github.com/stretchr/testify/require"
)

// overlappingBoxDocument returns a document of n mutually overlapping prisms,
// so a Verify call measures n*(n-1)/2 interfering pairs on its workers.
func overlappingBoxDocument(t *testing.T, n int) *Document {
	t.Helper()
	doc := New()
	for i := range n {
		off := float64(i) * 0.5
		internalBoxBody(t, doc, off, off, off+4, off+4, 4)
	}
	return doc
}

// runConcurrently runs fn on n goroutines at once and fails t on any error.
func runConcurrently(t *testing.T, n int, fn func() error) {
	t.Helper()
	var wg sync.WaitGroup
	errs := make([]error, n)
	for i := range n {
		wg.Go(func() {
			errs[i] = fn()
		})
	}
	wg.Wait()
	for _, err := range errs {
		require.NoError(t, err)
	}
}
