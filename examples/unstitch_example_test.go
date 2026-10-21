package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Unstitch takes a solid apart into one single-face sheet per face. Use it when
// you want to work on the faces separately, for example to offset or thicken
// one of them, or to rebuild a part with some faces replaced. It is the
// opposite of Stitch.
func Example_unstitch() {
	// Make the solid to take apart: sketch a rectangle and extrude it into a
	// box (see Example_extrude for these steps).
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
	box, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Each of the box's six faces becomes its own sheet body in the document.
	sheets, err := decad.Unstitch(context.Background(), box)
	if err != nil {
		fmt.Printf("failed to unstitch: %s\n", err)
		return
	}
	fmt.Printf("sheets: %d\n", len(sheets))
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// sheets: 6
	// live bodies: 6
}
