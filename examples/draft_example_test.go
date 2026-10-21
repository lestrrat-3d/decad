package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Draft tilts the walls of a body that already exists. Molded and cast parts
// need drafted walls so they can be pulled out of the mold. A neutral face
// tells Draft which end stays the same size; the walls lean inward away from
// it. If you are drawing the part from scratch, WithTaper on Extrude does the
// same in one step.
func Example_draft() {
	// Make the body to draft: sketch a 20 x 20 mm square and extrude it into
	// a block 10 mm tall (see Example_extrude for these steps).
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
	doc := decad.New()
	block, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Lean every side wall by 5 degrees. The neutral face is the bottom, the
	// face on the sketch plane, so the bottom keeps its 20 x 20 mm size and
	// the top shrinks. (To draft only some walls, narrow the first query,
	// for example with decad.Facing.)
	drafted, err := decad.Draft(context.Background(), block, decad.Faces(decad.Walls(block)),
		decad.NeutralFace{Body: block, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(block)))},
		units.Degrees(5))
	if err != nil {
		fmt.Printf("failed to draft: %s\n", err)
		return
	}
	volume, err := drafted.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The undrafted block holds 4000 mm^3. Leaning the walls removes some.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 3660.3 mm^3
}
