package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shelling a plate that has holes, with both flat faces removed, leaves only
// the walls: one band along the outer edge and one lining each hole. They are
// separate pieces, so the body has several lumps. Use this to see what Shell
// does to a part with internal cutouts.
func Example_shell_plate_with_holes() {
	// A 100 x 60 mm plate with a rectangular slot and a round bore.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(20, 20, 30, 40)
	s.CreateCircle(s.CreatePoint(70, 30), 8)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	// The profile we want is the plate region, the one with two holes.
	var plate *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			plate = p
		}
	}
	doc := decad.New()
	block, err := decad.Extrude(context.Background(), doc, s, plate,
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Remove both flat faces (those normal to Z) and keep 3 mm walls.
	bands, err := decad.Shell(context.Background(), block,
		decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	fmt.Printf("separate pieces: %d\n", len(bands.Lumps()))
	// Output:
	// separate pieces: 3
}
