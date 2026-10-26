package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// TextProfiles turns a line of text into sketch profiles, so Emboss can raise or
// engrave lettering on a face. The letters are set in a TrueType font (Go Regular
// when you give none) and drawn as polygons. Draw them on the plane of the face
// they mark, and ask Emboss for the same face.
func Example_text() {
	// A 60 x 30 x 10 mm plate (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 60, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// "HI" 12 mm to the em, its baseline starting at (8, 10) on the top face, which
	// lies on the plane z = 10.
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	lettering, profiles, err := decad.TextProfiles(context.Background(), w, top, decad.TextSpec{
		Text:   "HI",
		Height: units.Millimeters(12),
		At:     decad.Point2{U: 8, V: 10},
	})
	if err != nil {
		fmt.Printf("failed to set the text: %s\n", err)
		return
	}
	fmt.Printf("regions: %d\n", len(profiles))

	// Raise the letters 2 mm out of the top face.
	labelled, err := decad.Emboss(context.Background(), plate, lettering, profiles,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))), decad.EmbossRaised, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to emboss: %s\n", err)
		return
	}
	volume, err := labelled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("the letters add material: %t\n", volume.Value.Base() > 60*30*10)
	// Output:
	// regions: 2
	// the letters add material: true
}
