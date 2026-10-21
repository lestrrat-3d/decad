package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithSweepTwist rotates the profile as it travels along the path, so the
// result spirals. Use it for twisted bars, drill-bit flutes and decorative
// columns. The angle is the total turn from the start of the path to its end.
func Example_sweep_twist() {
	// The cross-section is a 2 x 2 mm square centered on the origin.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	square := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(square.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// A straight 5 mm path, along which the square turns 30 degrees.
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}
	bar, err := decad.Sweep(context.Background(), decad.New(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(30)))
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	volume, err := bar.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// Twisting does not change the volume: it is 2 x 2 x 5 = 20 mm^3.
	fmt.Printf("volume: %s\n", volume.Value)
	// Output:
	// volume: 20 mm^3
}
