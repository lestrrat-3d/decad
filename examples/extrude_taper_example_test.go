package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithTaper adds a draft angle to an extrude: every wall leans inward as it
// leaves the sketch plane. Molded and cast parts need this so they can be
// released from the mold. The far end of the body is smaller than the sketch.
func Example_extrude_taper() {
	// Sketch the outline to be drafted. It is a 20 x 20 mm square centered on the
	// origin, so the walls lean in evenly on every side.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Extrude 10 mm with a 5 degree taper. For comparison, the same extrude
	// without WithTaper would hold 20 x 20 x 10 = 4000 mm^3.
	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithTaper(units.Degrees(5)))
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The result is smaller than 4000 mm^3 because the walls lean in.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 3660.3 mm^3
}
