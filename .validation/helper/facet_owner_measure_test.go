package meshbool_test

import (
	"context"
	"encoding/json"
	"os"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

func validationTriangles(n int) [][3]int {
	out := make([][3]int, n)
	for i := range out {
		v := 4 * (i / 2)
		if i%2 == 0 {
			out[i] = [3]int{v, v + 1, v + 2}
		} else {
			out[i] = [3]int{v + 1, v, v + 3}
		}
	}
	return out
}
func validationCheckAdjacency(t testing.TB, got [][]int, n int) {
	t.Helper()
	if len(got) != n {
		t.Fatalf("adjacency length %d want %d", len(got), n)
	}
	for i, row := range got {
		partner := i ^ 1
		if partner >= n {
			if row != nil {
				t.Fatalf("unpaired row %d is nonnil", i)
			}
			continue
		}
		if len(row) != 1 || row[0] != partner {
			t.Fatalf("paired row %d is incorrect", i)
		}
	}
}
func BenchmarkValidationOwnerHelper(b *testing.B) {
	mode := os.Getenv("VALIDATION_MODE")
	small, large, over := validationTriangles(1), validationTriangles(8192), validationTriangles(8193)
	var fixed [][3]int
	if strings.HasPrefix(mode, "size_") {
		n, err := strconv.Atoi(strings.TrimPrefix(mode, "size_"))
		require.NoError(b, err)
		if n < 0 || n > 65536 {
			b.Fatal("helper size outside reviewed bound")
		}
		fixed = validationTriangles(n)
	} else if mode != "alternating" && mode != "overcap" {
		b.Fatal("unknown helper mode")
	}
	i := 0
	for b.Loop() {
		tris := over
		if strings.HasPrefix(mode, "size_") {
			tris = fixed
		}
		if mode == "alternating" {
			tris = small
			if i%2 != 0 {
				tris = large
			}
		}
		got, err := meshbool.FacetAdjacencyContext(b.Context(), tris)
		b.StopTimer()
		require.NoError(b, err)
		validationCheckAdjacency(b, got, len(tris))
		i++
		b.StartTimer()
	}
}
func TestValidationOwnerHelperHeap(t *testing.T) {
	workers, err := strconv.Atoi(os.Getenv("VALIDATION_WORKERS"))
	require.NoError(t, err)
	if workers != 1 && workers != 2 && workers != 4 && workers != 8 {
		t.Fatal("invalid workers")
	}
	tris := validationTriangles(8192)
	held := make([][][]int, workers)
	errs := make([]error, workers)
	read := func(phase string) {
		var m runtime.MemStats
		runtime.ReadMemStats(&m)
		data, err := json.Marshal(struct {
			Phase                                                                string
			HeapAlloc, HeapInuse, HeapObjects, TotalAlloc, Mallocs, PauseTotalNs uint64
			NumGC                                                                uint32
			GCCPUFraction                                                        float64
		}{phase, m.HeapAlloc, m.HeapInuse, m.HeapObjects, m.TotalAlloc, m.Mallocs, m.PauseTotalNs, m.NumGC, m.GCCPUFraction})
		require.NoError(t, err)
		t.Logf("HEAP %s", data)
	}
	runtime.GC()
	runtime.GC()
	read("before")
	var wg sync.WaitGroup
	start := make(chan struct{})
	ctx := t.Context()
	for i := range workers {
		wg.Go(func() { <-start; held[i], errs[i] = meshbool.FacetAdjacencyContext(ctx, tris) })
	}
	close(start)
	wg.Wait()
	for _, err := range errs {
		require.NoError(t, err)
	}
	read("after_call")
	runtime.GC()
	read("one_gc")
	runtime.GC()
	read("two_gc")
	runtime.KeepAlive(held)
	for _, got := range held {
		validationCheckAdjacency(t, got, len(tris))
	}
	held = nil
	tris = nil
	runtime.GC()
	runtime.GC()
	read("released")
}

type validationCancel struct{ calls, stop int }

func (*validationCancel) Deadline() (time.Time, bool) { return time.Time{}, false }
func (*validationCancel) Done() <-chan struct{}       { return nil }
func (*validationCancel) Value(any) any               { return nil }
func (c *validationCancel) Err() error {
	c.calls++
	if c.calls >= c.stop {
		return context.Canceled
	}
	return nil
}
func TestValidationOwnerCancellationTiming(t *testing.T) {
	tris := validationTriangles(8192)
	for _, stop := range []int{1, 2, 17, 34, 35, 36, 50, 67} {
		samples := make([]int64, 100)
		for i := range samples {
			ctx := &validationCancel{stop: stop}
			start := time.Now()
			got, err := meshbool.FacetAdjacencyContext(ctx, tris)
			samples[i] = time.Since(start).Nanoseconds()
			require.ErrorIs(t, err, context.Canceled)
			if got != nil {
				t.Fatal("cancellation returned adjacency")
			}
			if ctx.calls != stop {
				t.Fatalf("poll count %d want %d", ctx.calls, stop)
			}
		}
		sort.Slice(samples, func(i, j int) bool { return samples[i] < samples[j] })
		t.Logf("CANCEL stop=%d p50_ns=%d p95_ns=%d max_ns=%d", stop, samples[49], samples[94], samples[99])
	}
	// Distinct copy-free integer map reset cost, scoped as a reset proxy, not pool benefit.
	reset := make(map[[2]int]int, 24576)
	samples := make([]int64, 100)
	for i := range samples {
		for j := range 24576 {
			reset[[2]int{j, j + 1}] = j
		}
		start := time.Now()
		clear(reset)
		samples[i] = time.Since(start).Nanoseconds()
	}
	sort.Slice(samples, func(i, j int) bool { return samples[i] < samples[j] })
	t.Logf("RESET_PROXY p50_ns=%d p95_ns=%d max_ns=%d", samples[49], samples[94], samples[99])
	runtime.KeepAlive(reset)
}
