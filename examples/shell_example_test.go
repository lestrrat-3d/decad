package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shell hollows a body, leaving walls of a chosen thickness. The faces you
// name are removed to make the opening. Use it to turn a solid block into a
// box, cup or housing without modelling the cavity by hand.
func Example_shell() {
	// A solid 100 x 60 x 20 mm block.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Remove the top face (the one facing +Z) and leave 5 mm walls. The
	// result is an open box.
	top := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(0, 0, 1)))
	box, err := decad.Shell(context.Background(), block, top, units.Millimeters(5))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}
	volume, err := box.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The cavity is 90 x 50 x 15 mm, so the walls hold 120000 - 67500 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 52500 mm^3
}
