package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// WithMitredJoins lets a sweep follow a path that has sharp corners. Each
// corner is cut at half the angle between the two straight spans, like a
// picture frame, so the sections meet without a gap. Use it for angled tubes
// and supports made of straight segments.
func Example_sweep_mitred_joins() {
	// The cross-section is a hexagon of radius 1.5 mm centered on the origin.
	// CreatePolygon makes the six corners; they are fixed so the shape stays put.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	hexagon, err := s.CreatePolygon(0, 0, 6, 1.5)
	if err != nil {
		fmt.Printf("failed to create polygon: %s\n", err)
		return
	}
	s.Fix(hexagon.Center)
	for _, vertex := range hexagon.Vertices {
		s.Fix(vertex)
	}
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The path rises 6 mm, then leans over at 45 degrees. The corner between
	// the two spans is where the mitre goes.
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 6)},
		decad.LineTo{End: r3.NewVec(4, 0, 10)},
	)
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}

	doc := decad.New()
	branch, err := decad.Sweep(context.Background(), doc, s, s.Profiles()[0], path, decad.WithMitredJoins())
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	volume, err := branch.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\n", len(branch.Faces()))
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 14
	// volume: 68.1 mm^3
}
