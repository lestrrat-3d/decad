package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// FilletChord rounds a right-angled edge from the length of its chord, the
// straight line between the two places where the rounded arc leaves the walls.
// Use it when a drawing dimensions a round that way instead of by radius.
func Example_fillet_chord() {
	// Make a plate with a sharp vertical corner to bevel (see Example_extrude
	// for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 20)
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

	// The edge is the vertical one at the corner (40, 20). A 4 mm chord on a
	// right angle is an arc of radius 4/sqrt(2).
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(1)
	rounded, err := decad.FilletChord(context.Background(), plate, edge, units.Millimeters(4))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	volume, err := rounded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate is 8000 mm^3. The round removes (1 - pi/4) * 8 mm^2, 10 mm tall.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 7983 mm^3
}
