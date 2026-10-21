package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Loft also works with profiles that contain arcs. Each segment of the first
// profile is joined to the matching segment of the second, so a line must
// pair with a line and an arc with an arc. Use it for curved transitions, such
// as the wedge here, which has one rounded side.
func Example_loft_arc_sections() {
	// The wedge section is two straight edges and a 5 mm radius quarter arc
	// between them. Both profiles need the same shape, so draw it twice:
	// first on the XY plane, then on a plane 10 mm above.
	w := sketch.NewWorld()
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	var sketches [2]*sketch.Sketch
	for i, plane := range []*sketch.Plane{w.XY(), topPlane} {
		s, err := w.CreateSketch(plane)
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		origin := s.CreatePoint(0, 0)
		s.Fix(origin)
		onX := s.CreatePoint(5, 0)
		onY := s.CreatePoint(0, 5)
		s.CreateLine(origin, onX)
		s.CreateLine(onY, origin)
		s.CreateArc(origin, onX, onY)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		sketches[i] = s
	}

	doc := decad.New()
	wedge, err := decad.Loft(context.Background(), doc,
		sketches[0], sketches[0].Profiles()[0], sketches[1], sketches[1].Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}
	volume, err := wedge.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// A quarter circle of radius 5, 10 mm tall: (π x 5² / 4) x 10 = 196.3 mm^3.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 196.3 mm^3
}
