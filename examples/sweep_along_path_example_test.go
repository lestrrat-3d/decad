package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Sweep moves a profile along a 3D path to make a solid, such as a pipe, duct
// or rail. Use it when the part follows a curve that Extrude (straight) and
// Revolve (about an axis) cannot describe. The profile is sketched at the
// start of the path, square to it.
func Example_sweep_along_path() {
	// The cross-section of the duct: a 2 x 2 mm square centered on the origin
	// in the XY plane. The path starts at the origin and leaves along +Z.
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

	// The path rises while bending toward +X (an arc), runs straight along X
	// (a line), then bends sideways toward +Y (an arc). An ArcThrough span
	// names one point on the arc and the point where it ends.
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		decad.LineTo{End: r3.NewVec(15, 0, 5)},
		decad.ArcThrough{Through: r3.NewVec(18, 1, 5), End: r3.NewVec(20, 5, 5)},
	)
	if err != nil {
		fmt.Printf("failed to build path: %s\n", err)
		return
	}

	doc := decad.New()
	duct, err := decad.Sweep(context.Background(), doc, s, s.Profiles()[0], path)
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	volume, err := duct.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	bounds, err := duct.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	// Output:
	// volume: 102.8 mm^3
	// bounds: {-1 -1 0} .. {21 5 6}
}
