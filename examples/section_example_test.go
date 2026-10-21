package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Section slices a body with a plane and returns the cut outline as a sketch
// with profiles. Use it to inspect the inside of a part, to check wall
// thickness at one height, or to get an outline you can sketch on or extrude.
func Example_section() {
	// Make a body to slice: sketch a rectangle and extrude it into a block
	// (see Example_extrude for these steps).
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
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Slice halfway up the block, using an offset plane at z = 2.5 mm.
	plane, err := w.CreateOffsetPlane(w.XY(), 2.5)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	frame, err := plane.Frame()
	if err != nil {
		fmt.Printf("failed to get frame: %s\n", err)
		return
	}
	cut, profiles, err := block.Section(context.Background(), frame)
	if err != nil {
		fmt.Printf("failed to section: %s\n", err)
		return
	}

	// The slice is a 10 x 10 mm square. MeasureProfile reads its area.
	measured, err := decad.MeasureProfile(cut, profiles[0])
	if err != nil {
		fmt.Printf("failed to measure profile: %s\n", err)
		return
	}
	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	fmt.Printf("profiles: %d, area: %s\n", len(profiles), area.Value)
	// Output:
	// profiles: 1, area: 100 mm^2
}
