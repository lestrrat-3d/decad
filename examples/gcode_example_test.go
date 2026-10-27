package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// GCodePerimeters writes the toolpath that prints a body's outline layer by layer, the
// first stage of a slicer. Here a 20 x 10 x 6 mm block is printed in 0.5 mm layers,
// and the file holds one layer comment for each.
func Example_gcode_perimeters() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to start a sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	var out bytes.Buffer
	err = export.GCodePerimeters(context.Background(), &out, block, units.Millimeters(0.1), export.GCodeSpec{
		LayerHeight:      units.Millimeters(0.5),
		LineWidth:        units.Millimeters(0.4),
		FilamentDiameter: units.Millimeters(1.75),
		FeedRate:         units.MillimetersPerSecond(30),
	})
	if err != nil {
		fmt.Printf("failed to write G-code: %s\n", err)
		return
	}
	fmt.Printf("layers: %d\n", strings.Count(out.String(), "; layer "))
	// Output:
	// layers: 12
}

// GCodeSpec.InfillSpacing adds rectilinear infill inside the perimeters: lines a stated distance apart, along X on
// one layer and along Y on the next. Here the same block is printed with a line every 5 mm, and the file holds
// the infill's extruding lines as well as the perimeter's.
func Example_gcode_infill() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to start a sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	spec := export.GCodeSpec{
		LayerHeight:      units.Millimeters(0.5),
		LineWidth:        units.Millimeters(0.4),
		FilamentDiameter: units.Millimeters(1.75),
		FeedRate:         units.MillimetersPerSecond(30),
	}
	var perimeters, filled bytes.Buffer
	if err := export.GCodePerimeters(context.Background(), &perimeters, block, units.Millimeters(0.1), spec); err != nil {
		fmt.Printf("failed to write G-code: %s\n", err)
		return
	}
	spec.InfillSpacing = units.Millimeters(5)
	if err := export.GCodePerimeters(context.Background(), &filled, block, units.Millimeters(0.1), spec); err != nil {
		fmt.Printf("failed to write G-code: %s\n", err)
		return
	}
	// Two layers: two lines along X on the first and four along Y on the second.
	fmt.Printf("infill lines: %d\n", strings.Count(filled.String(), "G1 ")-strings.Count(perimeters.String(), "G1 "))
	// Output:
	// infill lines: 6
}
