package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Sometimes a reported problem is expected, such as a deliberately thin shim.
// Verify's report is never edited, so to accept one problem you write the
// rule yourself: skip the diagnostics you expect, matched by both Code and
// body, and treat everything else as a failure. Matching the body means a
// second, unexpected thin part is still caught.
func Example_verify_waive_diagnostic() {
	// Two thin plates, 1000 mm apart so they do not interfere. The first is
	// the shim we expect to be thin. Each has its own sketch (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	doc := decad.New()
	var shim *decad.Body
	for i, x0 := range []float64{0, 1000} {
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		rect := s.CreateRectangle(x0, 0, x0+10, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(0.5), Dir: decad.Along})
		if err != nil {
			fmt.Printf("failed to extrude: %s\n", err)
			return
		}
		if i == 0 {
			shim = plate
		}
	}

	report, err := doc.Verify(context.Background(), decad.WithMinWallThickness(units.Millimeters(1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	// Skip the wall-too-thin reason for the shim only. Anything left over is
	// a real problem.
	unexpected := 0
	for _, d := range report.Diagnostics {
		if d.Code == decad.DiagWallTooThin && d.Body == shim {
			continue
		}
		unexpected++
	}
	fmt.Printf("reasons reported: %d, unexpected: %d\n", len(report.Diagnostics), unexpected)
	// Output:
	// reasons reported: 2, unexpected: 1
}
