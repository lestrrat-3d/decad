package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LoftSections lofts through three or more profiles, in order. The skin passes
// through every section, so a middle section can make the body bulge or pinch.
// Use it for organic shapes such as vases and bottle bodies, where two
// profiles cannot describe the shape.
func Example_loft_multiple_sections() {
	// Three square sections, 5 mm apart in height. The middle one is wider
	// than the other two, which makes the body bulge in the middle.
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i, half := range []float64{10, 15, 8} {
		plane := w.XY()
		if i > 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i)*5)
			if err != nil {
				fmt.Printf("failed to create plane: %s\n", err)
				return
			}
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		square := s.CreateRectangle(-half, -half, half, half)
		s.Fix(square.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		sections[i] = decad.LoftSection{Sketch: s, Profile: s.Profiles()[0]}
	}

	// The result is one solid. Its volume carries a bound because the
	// bulging skin is curved.
	body, err := decad.LoftSections(context.Background(), decad.New(), sections[:]...)
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	bound, err := volume.Bound.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert bound: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3, within %.0f mm^3\n", volume.Value.Base(), bound)
	// Output:
	// volume: 6885 mm^3, within 16 mm^3
}
