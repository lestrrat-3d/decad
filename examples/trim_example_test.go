package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Trim cuts a sheet where another body crosses it and keeps the pieces on one
// side. Use it to cut an opening in a surface, or to trim a sheet back to the
// outline of a mating part.
func Example_trim() {
	// The sheet to trim is a four-wall tube: a rectangle extruded with
	// WithSurfaceResult (see Example_surface_result).
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
	tube, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude tube: %s\n", err)
		return
	}

	// The tool is a solid bar, narrower than the tube, that runs across it
	// from front to back. It sticks out past the tube on both sides and
	// above and below, so it passes completely through the tube's walls.
	toolSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create tool sketch: %s\n", err)
		return
	}
	bar := toolSketch.CreateRectangle(40, -10, 60, 70)
	toolSketch.Fix(bar.A)
	if _, err := toolSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve tool sketch: %s\n", err)
		return
	}
	tool, err := decad.Extrude(context.Background(), doc, toolSketch, toolSketch.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}

	// KeepOutside keeps the parts of the tube that are outside the bar. The
	// bar cuts the tube's front and back walls in two, leaving two pieces.
	trimmed, err := decad.Trim(context.Background(), tube, tool, decad.KeepOutside)
	if err != nil {
		fmt.Printf("failed to trim: %s\n", err)
		return
	}
	fmt.Printf("still a sheet: %v, pieces: %d\n", trimmed.Kind() == decad.BodySheet, len(trimmed.Lumps()))
	// Output:
	// still a sheet: true, pieces: 2
}
