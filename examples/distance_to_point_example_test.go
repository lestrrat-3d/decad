package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// DistanceToPoint measures how far a point is from a body's surface. Use it to
// check clearances, for example whether a bolt head or a hand reaches a part.
// The answer comes with a bound, which says how far off it could be.
func Example_distance_to_point() {
	// Make a body to measure from: sketch a rectangle and extrude it into a
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

	// Ask for the distance from a point 2 mm outside the cube's +X face. The
	// last argument is the tolerance: how precise the answer must be.
	distance, err := cube.DistanceToPoint(context.Background(), r3.NewVec(12, 5, 5), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to measure distance: %s\n", err)
		return
	}
	fmt.Printf("distance: %.1f mm (%s)\n", distance.Value.Base(), distance.Exactness)
	// Output:
	// distance: 2.0 mm (Exact)
}
