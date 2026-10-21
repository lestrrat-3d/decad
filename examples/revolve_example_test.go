package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Revolve spins a sketch profile about an axis to make a solid of revolution.
// Use it for round parts such as shafts, bushings, knobs and bottles: draw
// half of the side view, then spin it.
func Example_revolve() {
	// Draw the half side view of a cylinder: a rectangle 10 mm long and 8 mm
	// tall. Its bottom edge lies on the X axis, which will be the spin axis.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is given in sketch coordinates: from (0, 0) to (1, 0), which
	// is the sketch's own U (horizontal) axis.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

	// Spin the profile one full turn. The edge on the axis sweeps nothing, so
	// the result is a solid cylinder with no hole through it.
	doc := decad.New()
	cylinder, err := decad.Revolve(context.Background(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}
	volume, err := cylinder.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The volume of a cylinder is π x 8² x 10 = 640π mm^3.
	fmt.Printf("faces: %d\n", len(cylinder.Faces()))
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 3
	// volume: 2010.6 mm^3
}
