package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Transform moves a body with a rigid motion (a translation or rotation). Use
// it to place a part where it belongs in an assembly. The moved body is a new
// body, and the original is retired: it can still be measured, but no
// further operation accepts it. Use Copy to keep the original.
func Example_transform() {
	// Build a 100 x 60 x 10 mm plate to move.
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
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Move the plate 200 mm along X and 25 mm up.
	motion, err := r3.Translation(r3.NewVec(200, 0, 25))
	if err != nil {
		fmt.Printf("failed to build motion: %s\n", err)
		return
	}
	moved, err := decad.Transform(context.Background(), plate, motion)
	if err != nil {
		fmt.Printf("failed to transform: %s\n", err)
		return
	}

	bounds, err := moved.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// bounds: {200 0 25} .. {300 60 35}
	// live bodies: 1
}
