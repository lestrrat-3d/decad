package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer takes all twelve edges of an axis-aligned box in one call. The twelve
// cuts are simultaneous, so the three bevels at each corner meet at one point.
// The volume is the box minus the twelve wedges and plus what they share at the
// eight corners: 40*30*10 - 2*1^2*(40+30+10) + 6*1^3.
func Example_decad_boxAllEdgeChamfer() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	doc := decad.New()
	box, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude box: %s\n", err)
		return
	}
	part, err := decad.Chamfer(ctx, box, decad.Edges(decad.Convex()).Exactly(12), units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to chamfer box: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure part: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f mm^3 (%s)\n", volume.Value.Base(), volume.Exactness)
	fmt.Printf("faces: %d\n", len(part.Faces()))
	// Output:
	// volume: 11846.0 mm^3 (Exact)
	// faces: 18
}
