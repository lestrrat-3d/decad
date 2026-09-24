package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Trim cuts a sheet where it meets another body whose sweep shares its own
// generator, and keeps the pieces on the stated side. A 100x60 mm rectangle
// surface-extruded 10 mm builds a four-wall tube; a solid extruded from a
// narrower square, spanning the tube axially, crosses its bottom and top
// walls at two points each. KeepOutside keeps the tube's own material
// outside that tool, splitting it into the two ribbons that survive on
// either side of the cut.
func Example_decad_body_trim() {
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
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude the sheet: %s\n", err)
		return
	}

	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.XY())
	if err != nil {
		fmt.Printf("failed to create the tool sketch: %s\n", err)
		return
	}
	toolRect := ts.CreateRectangle(40, -10, 60, 70)
	ts.Fix(toolRect.A)
	if _, err := ts.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the tool sketch: %s\n", err)
		return
	}
	tool, err := doc.Extrude(ts, ts.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude the tool: %s\n", err)
		return
	}

	trimmed, err := sheet.Trim(context.Background(), tool, decad.KeepOutside)
	if err != nil {
		fmt.Printf("failed to trim: %s\n", err)
		return
	}

	area, err := trimmed.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	surface, err := area.Value.In(units.SquareMillimeter)
	if err != nil {
		fmt.Printf("failed to convert area: %s\n", err)
		return
	}

	fmt.Printf("sheet: %v, lumps: %d\n", trimmed.Kind() == decad.BodySheet, len(trimmed.Lumps()))
	fmt.Printf("area: %.1f mm² (%s)\n", surface, area.Exactness)
	// Output:
	// sheet: true, lumps: 2
	// area: 2800.0 mm² (Approximate)
}
