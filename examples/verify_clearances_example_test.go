package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithClearances makes Verify measure the gap between every pair of bodies that
// do not touch. Use it to check a design rule such as "parts must be at least
// 2 mm apart". Verify reports the gaps; your program compares them with the
// rule.
func Example_verify_clearances() {
	// Two 100 x 60 x 10 mm plates, 400 mm apart in X. Each has its own sketch
	// (see Example_extrude for these steps).
	w := sketch.NewWorld()
	doc := decad.New()
	for _, x0 := range []float64{0, 500} {
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
	}

	report, err := doc.Verify(context.Background(), decad.WithClearances())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	for _, c := range report.Clearances {
		fmt.Printf("gap: %.0f mm\n", c.Gap.Value.Base())
	}
	// Output:
	// gap: 400 mm
}
