package meshbool

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"runtime"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

type validationConformCancel struct {
	context.Context
	cancel         context.CancelFunc
	polls, trigger int
	started        time.Time
}

func (c *validationConformCancel) Err() error {
	err := c.Context.Err()
	c.polls++
	if c.trigger != 0 && c.polls == c.trigger {
		c.started = time.Now()
		c.cancel()
	}
	return err
}

func TestValidationConformCancellationLatency(t *testing.T) {
	mode := os.Getenv("VALIDATION_CASE")
	prefix := 0
	switch mode {
	case "prefix64":
		prefix = 64
	case "prefix512":
		prefix = 512
	case "prefix8192", "nosplit":
		prefix = 8192
	default:
		t.Fatal("unreviewed latency case")
	}
	values := make([]int64, 64)
	for sample := range values {
		// Fixture construction and context setup are outside the measured interval.
		verts := []proof.Xpt{}
		for _, v := range []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 1}, {X: 4}, {X: 5}, {X: 4, Y: 1}} {
			verts = append(verts, proof.XptOf(v))
		}
		n := prefix
		if mode != "nosplit" {
			n++
		}
		tris := make([][3]int, n)
		src := make([]int, n)
		bounds := make([]float64, n)
		beta := make([]float64, len(verts))
		for i := range tris {
			tris[i] = [3]int{4, 5, 6}
			src[i] = i
			bounds[i] = 0.5
		}
		if mode != "nosplit" {
			tris[prefix] = [3]int{0, 1, 2}
		}
		triFirst, srcFirst, boundFirst := &tris[0], &src[0], &bounds[0]
		ctx, cancel := context.WithCancel(t.Context())
		probe := &validationConformCancel{Context: ctx, cancel: cancel}
		hookCalls, hookPolls := 0, 0
		if mode == "nosplit" {
			probe.trigger = 2
		} else {
			validationConformSplitHook = func() { hookCalls++; hookPolls = probe.polls; probe.started = time.Now(); cancel() }
		}
		split, err := ConformOnce(probe, verts, &tris, &src, &bounds, beta)
		elapsed := time.Since(probe.started).Nanoseconds()
		validationConformSplitHook = nil
		cancel()
		if split || !errors.Is(err, context.Canceled) || probe.started.IsZero() || elapsed <= 0 {
			t.Fatalf("sample=%d canceled=%t split=%t elapsed=%d", sample, errors.Is(err, context.Canceled), split, elapsed)
		}
		if mode == "nosplit" {
			if hookCalls != 0 || probe.polls != 3 || beta[3] != 0 {
				t.Fatalf("no-split sample=%d hooks=%d polls=%d beta=%g", sample, hookCalls, probe.polls, beta[3])
			}
		} else if hookCalls != 1 || probe.polls != hookPolls+1 || beta[3] != 0.5 {
			t.Fatalf("split sample=%d hooks=%d polls=%d hookpolls=%d beta=%g", sample, hookCalls, probe.polls, hookPolls, beta[3])
		}
		if len(tris) != n || len(src) != n || len(bounds) != n || &tris[0] != triFirst || &src[0] != srcFirst || &bounds[0] != boundFirst {
			t.Fatal("cancellation changed output headers")
		}
		for i := range tris {
			want := [3]int{4, 5, 6}
			if mode != "nosplit" && i == prefix {
				want = [3]int{0, 1, 2}
			}
			for k := range 3 {
				if tris[i][k] != want[k] {
					t.Fatalf("triangle=%d corner=%d", i, k)
				}
			}
			if src[i] != i || bounds[i] != 0.5 {
				t.Fatalf("parallel data index=%d", i)
			}
		}
		values[sample] = elapsed
	}
	sorted := slices.Clone(values)
	slices.Sort(sorted)
	t.Logf("LATENCY count=64 p50_ns=%d p95_ns=%d max_ns=%d samples_ns=%v", sorted[31], sorted[60], sorted[63], values)
}

func TestValidationConformRetainedHeap(t *testing.T) {
	mode := os.Getenv("VALIDATION_CASE")
	const n = 8192
	capacity := n
	if mode == "spare" {
		capacity = 4 * n
	} else if mode != "exact" {
		t.Fatal("unreviewed heap case")
	}
	verts := []proof.Xpt{}
	for _, v := range []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 1}, {X: 4}, {X: 5}, {X: 4, Y: 1}} {
		verts = append(verts, proof.XptOf(v))
	}
	tris := make([][3]int, n, capacity)
	src := make([]int, n, capacity)
	bounds := make([]float64, n, capacity)
	beta := make([]float64, len(verts))
	for i := range tris {
		tris[i] = [3]int{4, 5, 6}
		src[i] = i
		bounds[i] = 0.5
	}
	read := func(phase string) {
		var m runtime.MemStats
		runtime.ReadMemStats(&m)
		data, err := json.Marshal(struct {
			Phase                                                                string
			HeapAlloc, HeapInuse, HeapObjects, TotalAlloc, Mallocs, PauseTotalNs uint64
			NumGC                                                                uint32
		}{phase, m.HeapAlloc, m.HeapInuse, m.HeapObjects, m.TotalAlloc, m.Mallocs, m.PauseTotalNs, m.NumGC})
		if err != nil {
			t.Fatal("heap JSON")
		}
		t.Logf("HEAP %s", data)
	}
	runtime.GC()
	runtime.GC()
	read("before")
	split, err := ConformOnce(t.Context(), verts, &tris, &src, &bounds, beta)
	if err != nil || split {
		t.Fatalf("heap fixture split=%t error=%t", split, err != nil)
	}
	read("after_call")
	runtime.GC()
	read("one_gc")
	runtime.GC()
	read("two_gc")
	runtime.KeepAlive(tris)
	runtime.KeepAlive(src)
	runtime.KeepAlive(bounds)
	if len(tris) != n || len(src) != n || len(bounds) != n {
		t.Fatal("heap fixture output length")
	}
	for i := range tris {
		if tris[i] != [3]int{4, 5, 6} || src[i] != i || bounds[i] != 0.5 {
			t.Fatalf("heap output index=%d", i)
		}
	}
	for i, v := range beta {
		if v != 0 {
			t.Fatalf("heap beta index=%d", i)
		}
	}
	t.Logf("CAPACITY input=%d tris=%d src=%d bounds=%d", capacity, cap(tris), cap(src), cap(bounds))
	tris = nil
	src = nil
	bounds = nil
	verts = nil
	beta = nil
	runtime.GC()
	runtime.GC()
	read("released")
}
