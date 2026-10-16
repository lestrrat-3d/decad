package meshbool

import (
	"os"
	"runtime"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func BenchmarkValidationProjection(b *testing.B) {
	b.StopTimer()
	mode := os.Getenv("VALIDATION_CASE")
	if mode != "constructor" && mode != "cold" && mode != "hot" {
		b.Fatal("unknown projection mode")
	}
	h := HeldRounding{Verts: make([]r3.Vec, 3072), Tris: make([][3]int, 1024)}
	for i := range h.Tris {
		v := 3 * i
		x := float64(i/2) * 4
		z := float64(i % 2)
		h.Verts[v], h.Verts[v+1], h.Verts[v+2] = r3.Vec{X: x, Z: z}, r3.Vec{X: x + 1, Z: z + 0.25}, r3.Vec{X: x, Y: 1, Z: z + 0.5}
		h.Tris[i] = [3]int{v, v + 1, v + 2}
	}
	budget := proofbound.NewWorkBudget(b.Context())
	e := newHeldEmbedding(h, budget, true)
	pairs := [][2]int{{0, 1}, {0, 2}, {1, 2}, {0, 0}}
	want := []bool{true, true, true, false}
	for k, pair := range pairs {
		if e.projectionsSeparated(pair[0], pair[1]) != want[k] {
			b.Fatalf("fixture pair %d", k)
		}
	}
	b.StartTimer()
	for b.Loop() {
		if mode == "constructor" {
			built := newHeldEmbedding(h, budget, true)
			b.StopTimer()
			if len(built.projected) != len(h.Tris) || len(built.inc) != len(h.Verts) {
				b.Fatal("constructor shape")
			}
			runtime.KeepAlive(built)
			b.StartTimer()
			continue
		}
		var got [4]bool
		for k, pair := range pairs {
			if mode == "cold" {
				e.projectedOK[pair[0]], e.projectedOK[pair[1]] = false, false
			}
			got[k] = e.projectionsSeparated(pair[0], pair[1])
		}
		b.StopTimer()
		for k, value := range got {
			if value != want[k] {
				b.Fatalf("pair result %d", k)
			}
		}
		runtime.KeepAlive(e)
		b.StartTimer()
	}
}
