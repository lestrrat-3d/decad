package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// SectionOutline slices a body with a plane and returns the outlines of the
// cut, so you can measure a wall or a web at that height. It reads the body's
// mesh, so it works on bodies whose exact section the library does not build,
// and it states how far its points can lie from the true surface. Loops that
// run counter-clockwise seen from the plane's normal are outer boundaries, and
// clockwise loops are holes.
func Example_section_outline() {
	// Make a 20 x 20 x 10 mm plate (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Cut at mid-height with a plane whose normal points up. The frame's axes
	// span the plane, and its normal is their cross product.
	plane, err := r3.NewFrame(r3.Vec{X: 10, Y: 10, Z: 5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		fmt.Printf("failed to build the plane: %s\n", err)
		return
	}
	outline, err := plate.SectionOutline(context.Background(), plane, units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	fmt.Printf("loops: %d\n", len(outline.Loops))
	fmt.Printf("area: %.0f mm^2\n", outline.PolygonArea().Base())
	// Output:
	// loops: 1
	// area: 400 mm^2
}
