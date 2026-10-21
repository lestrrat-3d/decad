package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer bevels edges with a flat cut instead of a round one. Use it when a
// part needs a simple, easy-to-machine corner. Edges are chosen with a query,
// here the four edges parallel to Z.
func Example_chamfer() {
	// Make a box to bevel: sketch a rectangle and extrude it (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	box, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Cut each vertical edge back 10 mm on both sides. The box gains one
	// flat face per edge.
	beveled, err := decad.Chamfer(context.Background(), box,
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(10))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\n", len(beveled.Faces()))
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 10
	// volume: 116000 mm^3
}
