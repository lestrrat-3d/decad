package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Report.Diagnostics itemizes every reason a report is not Sound as a stable,
// branchable record — a Code an agent switches on, the Status it contributes,
// and the Observed reading against its Required threshold. The slice is empty
// EXACTLY when the report is Sound, so a caller reads WHY, not just THAT, a body
// failed. Here a 0.5 mm plate is checked against a 1 mm tool.
func Example_decad_verify_diagnostics() {
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
	if _, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(0.5), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	report, err := doc.Verify(context.Background(), decad.WithMinWallThickness(units.Millimeters(1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	fmt.Printf("report status: %s\n", report.Status)
	fmt.Printf("trustworthy: %t\n", report.Trustworthy())
	for _, d := range report.Diagnostics {
		if d.Code != decad.DiagWallTooThin {
			continue
		}
		// The Code is the stable branch key; the reading and threshold say by
		// how much. ReadingWall rides the scalar Observed field.
		fmt.Printf("diagnostic: %s (%s)\n", d.Code, d.Status)
		fmt.Printf("wall %s < tool %s\n", d.Observed.Value, *d.Required)
	}
	// Output:
	// report status: Violating
	// trustworthy: false
	// diagnostic: wall_too_thin (Violating)
	// wall 0.5 mm < tool 1 mm
}
