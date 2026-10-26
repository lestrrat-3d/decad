package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// PathPatternCopies repeats a body along a path, such as bolts along a curved
// flange or ribs along a rail. The copies sit at equal distances measured along
// the path's length and keep the body's orientation. The body itself counts as
// the first instance, taken to sit at the path's start.
func Example_path_pattern() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 4, 4)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	peg, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// An L-shaped path: 20 mm along X, then 20 mm along Y. Three copies go at
	// every third of the 40 mm length, 13.3 mm apart along the path.
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.Vec{X: 20}}, decad.LineTo{End: r3.Vec{X: 20, Y: 20}})
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}
	copies, err := decad.PathPatternCopies(context.Background(), peg, path, 4)
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	for _, c := range copies {
		box, err := c.Bounds()
		if err != nil {
			fmt.Printf("failed to read bounds: %s\n", err)
			return
		}
		fmt.Printf("x %.1f..%.1f, y %.1f..%.1f\n", box.Min.X, box.Max.X, box.Min.Y, box.Max.Y)
	}
	// Output:
	// x 13.3..17.3, y 0.0..4.0
	// x 20.0..24.0, y 6.7..10.7
	// x 20.0..24.0, y 20.0..24.0
}

// WithPathFollow turns each copy to follow the path, as a car follows a road.
// The body sits at the path's start with its own +X along the path, and each
// copy is turned by the bend of the path up to its point.
func Example_path_pattern_follow() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 4, 4)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	peg, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The same L-shaped path. Once it turns to run along +Y, a copy is turned a
	// quarter turn, so its 4 x 4 footprint lies to the left of its point.
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.Vec{X: 20}}, decad.LineTo{End: r3.Vec{X: 20, Y: 20}})
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}
	copies, err := decad.PathPatternCopies(context.Background(), peg, path, 4, decad.WithPathFollow())
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	for _, c := range copies {
		box, err := c.Bounds()
		if err != nil {
			fmt.Printf("failed to read bounds: %s\n", err)
			return
		}
		fmt.Printf("x %.1f..%.1f, y %.1f..%.1f\n", box.Min.X, box.Max.X, box.Min.Y, box.Max.Y)
	}
	// Output:
	// x 13.3..17.3, y 0.0..4.0
	// x 16.0..20.0, y 6.7..10.7
	// x 16.0..20.0, y 20.0..24.0
}
