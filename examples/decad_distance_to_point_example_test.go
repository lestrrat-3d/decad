package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_distance_to_point() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	box, err := decad.New().Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude box: %s\n", err)
		return
	}
	distance, err := box.DistanceToPoint(context.Background(), r3.NewVec(12, 5, 5), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to measure distance: %s\n", err)
		return
	}
	fmt.Printf("%.1f mm, %s, bound %.1f mm\n",
		distance.Value.Base(), distance.Exactness, distance.Bound.Base())
	// Output:
	// 2.0 mm, Exact, bound 0.0 mm
}
