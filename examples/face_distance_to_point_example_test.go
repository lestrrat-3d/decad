package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A single face can answer the same question as a whole body. Use
// Face.DistanceToPoint when only one surface matters, such as the mounting
// face of a part, and the rest of the body should not count.
func Example_face_distance_to_point() {
	// Make a body that has a top face to measure to: sketch a rectangle and
	// extrude it into a cube (see Example_extrude for these steps).
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

	// Pick the top face, the one that faces +Z. Exactly(1) fails the call if
	// the query does not find exactly one face.
	tops, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(1).SelectFaces(cube)
	if err != nil {
		fmt.Printf("failed to select the top face: %s\n", err)
		return
	}

	// The cube's center is 5 mm below its top face, so that is the distance.
	distance, err := tops[0].DistanceToPoint(context.Background(), r3.NewVec(5, 5, 5), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to measure distance: %s\n", err)
		return
	}
	fmt.Printf("distance: %.1f mm (%s)\n", distance.Value.Base(), distance.Exactness)
	// Output:
	// distance: 5.0 mm (Exact)
}
