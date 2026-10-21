package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LocatePoint says whether a point is inside a body, outside it, or exactly on
// its surface. Use it to test whether a part, a probe or a drill path ends up
// in solid material.
func Example_locate_point() {
	// Make a body to test against: sketch a rectangle and extrude it into a
	// cube (see Example_extrude for these steps).
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
	cube, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Test one point in each situation: in the middle of the cube, 2 mm
	// outside its +X face, and exactly on its -X face.
	points := map[string]r3.Vec{
		"center":    r3.NewVec(5, 5, 5),
		"beyond +X": r3.NewVec(12, 5, 5),
		"on -X":     r3.NewVec(0, 5, 5),
	}
	for _, name := range []string{"center", "beyond +X", "on -X"} {
		location, err := cube.LocatePoint(context.Background(), points[name], units.Millimeters(0.1))
		if err != nil {
			fmt.Printf("failed to locate %s: %s\n", name, err)
			return
		}
		fmt.Printf("%s: %v\n", name, location)
	}
	// Output:
	// center: Inside
	// beyond +X: Outside
	// on -X: OnBoundary
}
