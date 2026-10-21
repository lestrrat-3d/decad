package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithMatchingFaces tells Cut or Intersect that faces of the two bodies are
// meant to match where they meet. A cap drawn to touch a cone, or two faces
// meant to coincide, can miss by a few last-place steps once the numbers are
// rounded; with the option decad takes that miss as the match you intended. A
// real gap is still refused. Where the bodies' faces do not meet, as here, the
// option changes nothing.
func Example_intersect_matching_faces() {
	w := sketch.NewWorld()
	doc := decad.New()
	box := func(x0, y0, x1, y1, height float64) (*decad.Body, error) {
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	a, err := box(0, 0, 20, 20, 10)
	if err != nil {
		fmt.Printf("failed to build box A: %s\n", err)
		return
	}
	b, err := box(10, 10, 30, 30, 10)
	if err != nil {
		fmt.Printf("failed to build box B: %s\n", err)
		return
	}
	common, err := decad.Intersect(context.Background(), a, b, decad.WithMatchingFaces())
	if err != nil {
		fmt.Printf("failed to intersect: %s\n", err)
		return
	}
	volume, err := common.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1000 mm^3
}
