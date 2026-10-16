package meshbool

import (
	"context"
	"errors"
	"os"
	"runtime"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func validationIncidenceFixture(t testing.TB) HeldRounding {
	t.Helper()
	var h HeldRounding
	switch os.Getenv("VALIDATION_CASE") {
	case "empty":
	case "unused":
		h.Verts = make([]r3.Vec, 1024)
	case "sparse_unused":
		h.Verts = make([]r3.Vec, 8192)
		h.Tris = make([][3]int, 256)
		for i := range h.Tris {
			h.Tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
		}
	case "sparse":
		h.Verts = make([]r3.Vec, 3072)
		h.Tris = make([][3]int, 1024)
		for i := range h.Tris {
			h.Tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
		}
	case "fan":
		h.Verts = make([]r3.Vec, 1026)
		h.Tris = make([][3]int, 1024)
		for i := range h.Tris {
			h.Tris[i] = [3]int{0, i + 1, (i+1)%1024 + 1}
		}
	case "closed":
		h.Verts = make([]r3.Vec, 1024)
		for i := 0; i < 1024; i += 4 {
			h.Tris = append(h.Tris, [3]int{i, i + 2, i + 1}, [3]int{i, i + 1, i + 3},
				[3]int{i, i + 3, i + 2}, [3]int{i + 1, i + 2, i + 3})
		}
	default:
		t.Fatal("unknown constructor case")
	}
	// Unique coordinates exercise the constructor's existing taken map too.
	// Geometry is not queried by this constructor-only benchmark.
	for i := range h.Verts {
		h.Verts[i] = r3.Vec{X: float64(i)}
	}
	return h
}

func BenchmarkValidationIncidenceConstructor(b *testing.B) {
	b.StopTimer()
	h := validationIncidenceFixture(b)
	want := make([][]int, len(h.Verts))
	for f, tri := range h.Tris {
		for _, v := range tri {
			want[v] = append(want[v], f)
		}
	}
	budget := proofbound.NewWorkBudget(b.Context())
	for b.Loop() {
		e := newHeldEmbedding(h, budget, true)
		b.StopTimer()
		if len(e.inc) != len(want) {
			b.Fatal("incidence outer length")
		}
		for v, row := range want {
			if (row == nil) != (e.inc[v] == nil) || len(row) != len(e.inc[v]) {
				b.Fatalf("incidence shape vertex=%d", v)
			}
			for k, f := range row {
				if e.inc[v][k] != f {
					b.Fatalf("incidence content vertex=%d index=%d", v, k)
				}
			}
		}
		runtime.KeepAlive(e)
		b.StartTimer()
	}
}

// This context reads the successful initial poll, then cancels immediately.
// The elapsed interval conservatively includes all subsequent construction and
// the first 256 query steps; it needs no production instrumentation or scheduler.
type validationIncidenceCancelContext struct {
	context.Context
	cancel  context.CancelFunc
	started time.Time
	polls   int
}

func (c *validationIncidenceCancelContext) Err() error {
	err := c.Context.Err()
	c.polls++
	if c.polls == 1 {
		c.started = time.Now()
		c.cancel()
	}
	return err
}

func TestValidationIncidenceCancellationLatency(t *testing.T) {
	h := validationIncidenceFixture(t)
	h.Moved = make([]bool, len(h.Verts))
	if len(h.Tris) < proofbound.WorkPollInterval {
		t.Fatal("latency fixture must reach next poll")
	}
	values := make([]int64, 64)
	for i := range values {
		ctx, cancel := context.WithCancel(t.Context())
		probe := &validationIncidenceCancelContext{Context: ctx, cancel: cancel}
		n, err := EnforceHeldEmbedding(probe, h)
		elapsed := time.Since(probe.started).Nanoseconds()
		cancel()
		if !errors.Is(err, context.Canceled) || n != 0 || probe.polls != 2 || elapsed <= 0 {
			t.Fatalf("cancellation sample=%d moved=%d polls=%d elapsed=%d", i, n, probe.polls, elapsed)
		}
		values[i] = elapsed
	}
	slices.Sort(values)
	// Nearest-rank quantiles: ceil(64*.50)-1 and ceil(64*.95)-1.
	t.Logf("LATENCY count=64 p50_ns=%d p95_ns=%d max_ns=%d", values[31], values[60], values[63])
}
