package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// LoftFromPoint lofts from a single point to a profile, making a pointed solid
// such as a pyramid or cone. Use it for tips, spikes and any shape that
// closes down to a point.
func Example_loft_from_point() {
	// The base of the pyramid: a 4 x 4 mm square on a plane 6 mm above the
	// XY plane.
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 6)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	square := s.CreateRectangle(-2, -2, 2, 2)
	s.Fix(square.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The tip is the origin, 6 mm below the base.
	pyramid, err := decad.LoftFromPoint(context.Background(), decad.New(),
		r3.NewVec(0, 0, 0), s, s.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}
	volume, err := pyramid.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// A pyramid holds a third of base x height: 16 x 6 / 3 = 32 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 32 mm^3
}
