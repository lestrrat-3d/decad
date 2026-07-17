package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Verify is the product: one non-mutating call over the live model, a
// verdict per body and for the document, and one bit — Trustworthy() — an
// agent gates on. A question the evaluator cannot decide reads Suspect,
// never a silent pass: here two far-apart plates are provenly disjoint and
// the report is Sound, while two overlapping plates are a pair this
// evaluator can prove neither overlapping nor disjoint.
func Example_decad_verify() {
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

	// Two plates, 500 mm apart: both proven solids, the pair proven
	// disjoint by box separation.
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("apart: %s, trustworthy: %v\n", report.Status, report.Trustworthy())
	body := report.Bodies[0]
	fmt.Printf("body: %s, solid: %v, watertight: %v, manifold: %v\n",
		body.Status, body.Solid, body.Watertight, body.Manifold)
	fmt.Printf("volume: %s (%s)\n", body.Volume.Value, body.Volume.Exactness)

	// A third plate coincident with the second: the boxes overlap, and this
	// evaluator can prove the pair neither overlapping nor disjoint — the
	// undecided pair reads Suspect, never a fabricated verdict.
	if _, err := buildPlate(doc); err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	report, err = doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("overlapping: %s, trustworthy: %v\n", report.Status, report.Trustworthy())
	// Output:
	// apart: Sound, trustworthy: true
	// body: Sound, solid: true, watertight: true, manifold: true
	// volume: 60000 mm^3 (Exact)
	// overlapping: Interfering, trustworthy: false
}
