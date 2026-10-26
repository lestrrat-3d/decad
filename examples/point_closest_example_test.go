package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ClosestPoint finds the place on a body that lies nearest a point you give.
// Use it to see where a probe, a neighbouring part or a fastener would touch
// the body first, not only how far away it is. A point inside the body is its
// own closest point.
func Example_point_closest() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// A probe 7 mm beyond the +X wall and above the plate's mid-height sees the
	// wall's nearest point straight across from it.
	nearest, err := plate.ClosestPoint(context.Background(), r3.Vec{X: 47, Y: 12, Z: 4}, units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to find the closest point: %s\n", err)
		return
	}
	fmt.Printf("closest point: (%.0f, %.0f, %.0f)\n", nearest.Point.X, nearest.Point.Y, nearest.Point.Z)
	fmt.Printf("distance: %.0f mm\n", nearest.Distance.Value.Base())
	// Output:
	// closest point: (40, 12, 4)
	// distance: 7 mm
}
