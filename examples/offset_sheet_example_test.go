package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Offset makes a new sheet that runs parallel to an existing one at a fixed
// distance. Use it to model a surface that sits a set distance from another,
// such as the outer skin of a part or the path a tool center follows. The
// source sheet is kept.
func Example_offset_sheet() {
	// Offset works on sheets, so make one: extrude a rectangle with
	// WithSurfaceResult to get four walls and no caps.
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
	sheet, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Grow the sheet outward by 3 mm. Each corner becomes a rounded wall so
	// the new sheet stays 3 mm from the old one everywhere.
	grown, err := decad.Offset(context.Background(), sheet, units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to offset: %s\n", err)
		return
	}
	bounds, err := grown.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// bounds: {-3 -3 0} .. {103 63 10}
	// live bodies: 2
}
