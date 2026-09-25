package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// BenchmarkSweepCompositeBounds builds a solid through two arcs and one line.
// The solved section and path are shared across iterations.
func BenchmarkSweepCompositeBounds(b *testing.B) {
	world := sketch.NewWorld()
	section, err := world.CreateSketch(world.XY())
	if err != nil {
		b.Fatal(err)
	}
	rect := section.CreateRectangle(-1, -1, 1, 1)
	section.Fix(rect.A)
	if _, err := section.Solve(b.Context()); err != nil {
		b.Fatal(err)
	}
	profile := section.Profiles()[0]
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		decad.LineTo{End: r3.NewVec(15, 0, 5)},
		decad.ArcThrough{Through: r3.NewVec(18, 1, 5), End: r3.NewVec(20, 5, 5)},
	)
	if err != nil {
		b.Fatal(err)
	}
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Sweep(b.Context(), section, profile, path)
		if err != nil {
			b.Fatal(err)
		}
		if got := len(body.Faces()); got != 14 {
			b.Fatalf("unexpected sweep face count: %d", got)
		}
	}
}
