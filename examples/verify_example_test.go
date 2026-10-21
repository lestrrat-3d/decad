package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Verify checks the whole model and reports whether it can be trusted. Run it
// before you hand a design on, or use it as the pass/fail gate in an
// automated workflow: Passed returns one boolean. Verify never modifies the
// model. Anything it cannot decide reads as "not sound" rather than passing
// silently.
func Example_verify() {
	// Two plates that do not touch, 500 mm apart. They sit in one document
	// so Verify checks them against each other as well as on their own. Each
	// is drawn in its own sketch (see Example_extrude for these steps).
	w := sketch.NewWorld()
	doc := decad.New()
	var plateSketches [2]*sketch.Sketch
	for i, x0 := range []float64{0, 500} {
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(x0, 0, x0+100, 60)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		if _, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		plateSketches[i] = s
	}

	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("separate plates: %s, passed: %v\n", report.Status, report.Passed())

	// Add a third plate on top of the first one. The two now occupy the same
	// space, so the model is no longer sound.
	if _, err := decad.Extrude(context.Background(), doc, plateSketches[0], plateSketches[0].Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	report, err = doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("overlapping plates: %s, passed: %v\n", report.Status, report.Passed())
	// Output:
	// separate plates: Sound, passed: true
	// overlapping plates: Interfering, passed: false
}
