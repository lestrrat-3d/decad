package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Coil turns a profile around an axis while sliding it along that axis, which
// makes a helix. Use it for springs, screw threads and wire windings. The
// profile is the cross-section of the wire, drawn beside the axis.
func Example_coil() {
	// The wire's cross-section is a 1 x 1 mm square, its nearest side 2 mm
	// from the axis. Two corners are fixed so the square cannot move or turn.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	wire := s.CreateRectangle(2, 0, 3, 1)
	s.Fix(wire.A)
	s.Fix(wire.C)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own V (vertical) axis. The coil advances along it.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

	// Wind two turns, advancing 1.5 mm per turn (the pitch). The length is
	// stated in turns rather than millimeters.
	doc := decad.New()
	spring, err := decad.Coil(context.Background(), doc, s, s.Profiles()[0], axis,
		units.Millimeters(1.5), units.Scalar(2))
	if err != nil {
		fmt.Printf("failed to coil: %s\n", err)
		return
	}
	volume, err := spring.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The wire section (1 mm^2), swept two turns at an average radius of
	// 2.5 mm, holds 2π x 2 x 2.5 = 10π mm^3.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 31.4 mm^3
}
