package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Draft can lean some walls and keep the rest vertical. Here only the box's
// +X wall is drafted, so the far cap shrinks along X alone: the section at
// height z is 20 × (20 − z·tan 5°), and the kept walls stay perpendicular to
// the caps. A subset that would move only one of two walls meeting tangentially
// at an arc is refused, since their corner would no longer be a line or arc.
func Example_decad_draft_subset() {
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

	// Lean the one wall facing +X by 5 degrees about the cap on the sketch
	// plane.
	drafted, err := box.Draft(context.Background(),
		decad.Faces(decad.Walls(box), decad.Facing(r3.NewVec(1, 0, 0))),
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
	bounds, err := drafted.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	// The volume is h·a(a − d/2) with d = h·tan 5 degrees, and the box keeps
	// its full extent along Y.
	fmt.Printf("faces: %d\n", len(drafted.Faces()))
	fmt.Printf("volume: %.3f mm^3 (%s)\n", vol.Value.Base(), vol.Exactness)
	fmt.Printf("y extent: %.1f mm\n", bounds.Max.Y-bounds.Min.Y)
	// Output:
	// faces: 6
	// volume: 3912.511 mm^3 (Approximate)
	// y extent: 20.0 mm
}
