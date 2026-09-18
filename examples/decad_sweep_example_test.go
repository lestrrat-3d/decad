package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Sweep moves a solved planar profile along an immutable spatial Path. This
// example uses one straight span along the profile plane's positive normal;
// the path start may lie outside the profile itself.
func Example_decad_sweep() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	path, err := decad.NewPath(
		r3.NewVec(1000, 1000, 0),
		decad.LineTo{End: r3.NewVec(1000, 1000, 10)},
	)
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Sweep(s, s.Profiles()[0], path)
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	bounds, err := body.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}

	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("volume: %s (%s)\n", volume.Value, volume.Exactness)
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// solid: true, faces: 6
	// volume: 60000 mm^3 (Exact)
	// bounds: {0 0 0} .. {100 60 10}
	// live bodies: 1
}
