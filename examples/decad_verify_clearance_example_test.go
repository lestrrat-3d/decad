package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithClearances turns the proven pair partition into measured gaps: every
// pair of proven-disjoint bodies gets a Clearance row whose Gap the analytic
// kernel proves — here the facing side faces of two plates, an Exact 400 mm.
// The gap is a measurement, not a verdict: the caller compares it against
// their own clearance spec, and the report is Trustworthy because every
// answer, the gap included, is proven to the asked figures.
func Example_decad_verify_clearances() {
	buildPlate := func(doc *decad.Document) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(0, 0, 100, 60)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	}

	doc := decad.New()
	first, err := buildPlate(doc)
	if err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	shift, err := r3.Translation(r3.NewVec(500, 0, 0))
	if err != nil {
		fmt.Printf("failed to build motion: %s\n", err)
		return
	}
	if _, err := first.Placed(shift); err != nil {
		fmt.Printf("failed to place: %s\n", err)
		return
	}
	if _, err := buildPlate(doc); err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}

	report, err := doc.Verify(context.Background(), decad.WithClearances())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("status: %s, trustworthy: %v\n", report.Status, report.Trustworthy())
	for _, c := range report.Clearances {
		fmt.Printf("gap: %s (%s)\n", c.Gap.Value, c.Gap.Exactness)
	}
	// Output:
	// status: Sound, trustworthy: true
	// gap: 400 mm (Exact)
}
