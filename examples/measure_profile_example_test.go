package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// MeasureProfile gives bounded 2D readings from a current sketch profile.
// It rejects boundaries that decad cannot record exactly.
func Example_decad_measureProfile() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate with a Ø20 hole.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(50, 30), 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The plate region: four lines outside, the circle bounding its hole.
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
			break
		}
	}

	measured, err := decad.MeasureProfile(s, prof)
	if err != nil {
		fmt.Printf("failed to measure profile: %s\n", err)
		return
	}
	area, err := measured.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	frame, err := s.Plane().Frame()
	if err != nil {
		fmt.Printf("failed to read plane: %s\n", err)
		return
	}

	fmt.Printf("outer segments: %d\n", len(prof.Outer))
	fmt.Printf("holes: %d\n", len(prof.Holes))
	fmt.Printf("area: %s\n", area.Exactness)
	fmt.Printf("plane normal: %v\n", frame.U().Cross(frame.V()))
	// Output:
	// outer segments: 4
	// holes: 1
	// area: Approximate
	// plane normal: {0 0 1}
}
