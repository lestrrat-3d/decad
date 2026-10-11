package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A middle section sets a quadratic bulge through three matched profiles.
func Example_decad_loft_sections() {
	ctx := context.Background()
	w := sketch.NewWorld()
	var sections [3]decad.LoftSection
	for i, half := range []float64{10, 15, 8} {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), float64(i)*5)
			if err != nil {
				fmt.Printf("failed to create loft plane: %s\n", err)
				return
			}
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			fmt.Printf("failed to create loft sketch: %s\n", err)
			return
		}
		r := s.CreateRectangle(-half, -half, half, half)
		s.Fix(r.A)
		if _, err := s.Solve(ctx); err != nil {
			fmt.Printf("failed to solve loft sketch: %s\n", err)
			return
		}
		sections[i] = decad.LoftSection{Sketch: s, Profile: s.Profiles()[0]}
	}
	body, err := decad.New().LoftSections(ctx, sections[:]...)
	if err != nil {
		fmt.Printf("failed to loft sections: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure loft volume: %s\n", err)
		return
	}
	mesh, err := body.Tessellate(ctx, units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Printf("failed to tessellate loft: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f ± %.1f mm³\n", volume.Value.Base(), volume.Bound.Base())
	fmt.Printf("verified mesh: %t, faces: %d\n", mesh.VolumeVerified(), len(body.Faces()))
	// Output:
	// volume: 6885.1 ± 16.2 mm³
	// verified mesh: true, faces: 6
}
