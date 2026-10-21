package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// When Verify says a model is not sound, Report.Diagnostics lists every reason.
// Each reason has a Code your program can switch on, plus the measured value
// and the limit it broke. Use WithMinWallThickness to state the thinnest wall
// your tool or process can make; walls thinner than that are reported.
func Example_verify_diagnostics() {
	// A plate 0.5 mm thick, thinner than the 1 mm limit we are about to set
	// (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	if _, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(0.5), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Verify with a 1 mm minimum wall.
	report, err := doc.Verify(context.Background(), decad.WithMinWallThickness(units.Millimeters(1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("status: %s, passed: %v\n", report.Status, report.Passed())

	// Read the reasons. Code is the stable key to branch on. Observed is what
	// was measured and Required is the limit it broke.
	for _, d := range report.Diagnostics {
		if d.Code == decad.DiagWallTooThin && d.Required != nil {
			fmt.Printf("%s: wall %s, required %s\n", d.Code, d.Observed.Value, *d.Required)
		}
	}
	// Output:
	// status: Violating, passed: false
	// wall_too_thin: wall 0.5 mm, required 1 mm
}
