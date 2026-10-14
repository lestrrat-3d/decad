package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A copy can be placed by a rotation instead of a transform. WithCopyRotation
// takes the center, the axis and the angle. A faceted copy made this way
// denotes an exactly orthogonal rational rotation near the angle, so its
// bounds are charged against that exact rotation. The copy gives up the
// analytic surface tags of a transform-placed copy; a prism copy, as here,
// is placed exactly as WithCopyPlacement of the same rotation.
func Example_decad_copy_rotation() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	peg := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(peg.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	master, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude peg: %s\n", err)
		return
	}

	// A quarter turn about the Z axis through the origin carries the peg
	// from the first quadrant into the second.
	turned, err := decad.Copy(context.Background(), master,
		decad.WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90)))
	if err != nil {
		fmt.Printf("failed to rotate copy: %s\n", err)
		return
	}
	c, err := turned.Centroid()
	if err != nil {
		fmt.Printf("failed to measure copy: %s\n", err)
		return
	}
	fmt.Printf("copy centroid: (%.3f, %.3f, %.3f)\n", c.Value.X, c.Value.Y, c.Value.Z)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// copy centroid: (-5.000, 5.000, 2.000)
	// live bodies: 2
}
