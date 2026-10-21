package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer can bevel all twelve edges of a box in one call, instead of just a
// single edge or corner at a time. Use it to break every sharp edge of a block
// at once. The twelve cuts happen together, so the three bevels at each corner
// meet at one point instead of overlapping. This works on a box whose sides
// line up with the X, Y and Z axes, with a setback that is exact in floating
// point, such as 1 or 0.5 mm.
func Example_chamfer_all_edges() {
	// Make a box to bevel: sketch a rectangle and extrude it (see
	// Example_extrude for these steps).
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
	box, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Select every convex edge, and ask for exactly twelve so a mistake in the
	// query fails here. Cut each one back 1 mm on both sides.
	beveled, err := decad.Chamfer(context.Background(), box,
		decad.Edges(decad.Convex()).Exactly(12), units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The box held 12000 mm^3. The twelve cuts remove 154 mm^3 together.
	fmt.Printf("faces: %d\n", len(beveled.Faces()))
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 18
	// volume: 11846 mm^3
}
