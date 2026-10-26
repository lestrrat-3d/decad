package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Hole drills radially into a round face too: a cross hole through a shaft for
// a pin, or a port out of a bore. Name the cylindrical face and a point on its
// surface, and the hole runs along the radius through that point.
func Example_hole_cylinder() {
	// Make a 10 mm shaft, 20 mm long (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	shaft, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// A 3 mm hole all the way across, starting at the shaft's side at x = 5,
	// halfway along its length.
	drilled, err := decad.Hole(context.Background(), shaft, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.Vec{X: 5, Y: 0, Z: 10},
		Diameter: units.Millimeters(3),
		Depth:    decad.ThroughHole{},
	})
	if err != nil {
		fmt.Printf("failed to drill: %s\n", err)
		return
	}
	// The pin hole's axis lies along X through the shaft's middle: its centre
	// is empty, and a point 3 mm to the side is still shaft.
	centreLocation, err := drilled.LocatePoint(context.Background(), r3.Vec{X: 0, Y: 0, Z: 10}, units.Millimeters(0.01))
	if err != nil {
		fmt.Printf("failed to locate: %s\n", err)
		return
	}
	sideLocation, err := drilled.LocatePoint(context.Background(), r3.Vec{X: 0, Y: 3, Z: 10}, units.Millimeters(0.01))
	if err != nil {
		fmt.Printf("failed to locate: %s\n", err)
		return
	}
	fmt.Printf("hole centre: %s\n", centreLocation)
	fmt.Printf("beside it: %s\n", sideLocation)
	// Output:
	// hole centre: Outside
	// beside it: Inside
}
