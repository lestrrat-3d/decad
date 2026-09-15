package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Explicit survey results (proposal §12): a requested wall or concave-radius
// survey answers with a primary outcome plus its reading — ScalarMeasured
// with an interval, or ScalarAbsent when the feature provably does not
// exist — and an unrequested survey answers ScalarNotRequested rather than
// leaving a nil the caller must interpret.
func Example_decad_verify_results() {
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
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
	if _, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Requested: the plate's 10 mm slab is its wall, and the all-convex
	// block has no concave feature at all — a proven absence, not a missing
	// answer.
	requested, err := doc.Verify(context.Background(),
		decad.WithMinWallThickness(units.Millimeters(1)), decad.WithConcaveRadius())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	br := requested.Bodies[0]
	fmt.Printf("wall: %s, minimum %s\n", br.Wall.Outcome, br.Wall.Minimum.Value)
	fmt.Printf("concave radius: %s\n", br.ConcaveRadius.Outcome)

	// The identical body, verified with neither option asked: both outcomes
	// read NotRequested, distinct from every proven answer above.
	unasked, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	ubr := unasked.Bodies[0]
	fmt.Printf("unrequested wall: %s, concave radius: %s\n", ubr.Wall.Outcome, ubr.ConcaveRadius.Outcome)
	// Output:
	// wall: measured, minimum 10 mm
	// concave radius: absent
	// unrequested wall: not_requested, concave radius: not_requested
}
