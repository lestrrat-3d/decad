package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Copy makes a new body from an existing one and leaves the original in place.
// Use it when a part appears several times, such as the pegs of a bolt
// pattern: model the part once, then place each instance with
// WithCopyPlacement. Transform, by contrast, moves a body and retires it.
func Example_copy_with_placement() {
	// The master part is a small peg that we will copy several times. Model it
	// once here.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	master, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Place three copies, 40 mm apart along X, by giving each Copy a translation.
	// This builds a row of identical parts without modelling each one again.
	for i := 1; i <= 3; i++ {
		shift, err := r3.Translation(r3.NewVec(float64(40*i), 0, 0))
		if err != nil {
			fmt.Printf("failed to build translation: %s\n", err)
			return
		}
		peg, err := decad.Copy(context.Background(), master, decad.WithCopyPlacement(shift))
		if err != nil {
			fmt.Printf("failed to copy: %s\n", err)
			return
		}
		bounds, err := peg.Bounds()
		if err != nil {
			fmt.Printf("failed to measure bounds: %s\n", err)
			return
		}
		fmt.Printf("copy %d starts at x = %v\n", i, bounds.Min.X)
	}

	// The master is still live, next to its three copies.
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// copy 1 starts at x = 40
	// copy 2 starts at x = 80
	// copy 3 starts at x = 120
	// live bodies: 4
}
