package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// MeasureProfile reads the area and centroid of a 2D profile before you extrude
// it. Use it to check a drawing, for example that a cross-section has the
// intended area, without building a solid first.
func Example_measure_profile() {
	// Draw a 100 x 60 mm plate with a round hole 20 mm across, off to one
	// side. The off-center hole moves the centroid away from the plate's
	// middle, which makes the centroid worth reading.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Sketch finds a profile for the plate and one for the hole. Choose the
	// plate, which is the one that has a hole in it.
	var plate *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			plate = p
			break
		}
	}
	if plate == nil {
		fmt.Println("no plate profile found")
		return
	}

	measured, err := decad.MeasureProfile(s, plate)
	if err != nil {
		fmt.Printf("failed to measure profile: %s\n", err)
		return
	}
	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	centroid, err := measured.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	fmt.Printf("area: %.1f mm^2\n", area.Value.Base())
	fmt.Printf("centroid x: %.2f mm\n", centroid.Value.X)
	// Output:
	// area: 5685.8 mm^2
	// centroid x: 48.89 mm
}
