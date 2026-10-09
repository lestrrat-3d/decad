package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Draft tilts the walls of a prism that already exists. The neutral face names
// the cap whose section stays put, and Walls selects the faces to lean: a
// positive angle narrows the body away from the neutral cap, so a molded part
// releases from the mold opening on that side. The result is the body a tapered
// extrude of the same sketch builds, and the receiver is retired.
func Example_decad_draft() {
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
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Lean every wall 5 degrees about the cap on the sketch plane.
	drafted, err := box.Draft(context.Background(),
		decad.Faces(decad.Walls(box)),
		decad.NeutralFace{Body: box, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))},
		units.Degrees(5))
	if err != nil {
		fmt.Printf("failed to draft: %s\n", err)
		return
	}

	vol, err := drafted.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The drafted far cap is the square offset inward by h·tan 5 degrees, so the
	// volume falls below the 4000 mm^3 of the straight box.
	fmt.Printf("faces: %d\n", len(drafted.Faces()))
	fmt.Printf("volume: %.3f mm^3 (%s)\n", vol.Value.Base(), vol.Exactness)
	fmt.Printf("bodies in the document: %d\n", len(doc.Bodies()))
	// Output:
	// faces: 6
	// volume: 3660.251 mm^3 (Approximate)
	// bodies in the document: 1
}
