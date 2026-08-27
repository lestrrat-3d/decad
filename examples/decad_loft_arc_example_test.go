package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Loft admits a same-kind CIRCULAR pairing as of a10-plan.md's arc design:
// two corresponding segments must be the same recorded type — both a
// LineSeg, both an ArcSeg, or both a CircleSeg — and an arc paired against a
// circle is a mix like any other. This wedge pairs two straight radial edges
// plus one 90-degree radius-5 arc on the bottom plane with the identical
// shape on a plane offset 10 mm along the normal, ruling a quarter-cylinder
// wedge whose closed-form volume is (pi*r^2/4)*h = 196.349540849... mm^3.
// The wall is a chorded polyhedron, not the true curved surface, so every
// published measurement carries a proven positive Bound rather than reading
// Exact — the same honesty a straight-walled loft's own Bound reads zero for.
func Example_decad_loft_arc() {
	w := sketch.NewWorld()
	s0, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create the bottom sketch: %s\n", err)
		return
	}
	origin0 := s0.CreatePoint(0, 0)
	s0.Fix(origin0)
	px0 := s0.CreatePoint(5, 0)
	py0 := s0.CreatePoint(0, 5)
	s0.CreateLine(origin0, px0)
	s0.CreateLine(py0, origin0)
	s0.CreateArc(origin0, px0, py0)
	if _, err := s0.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the bottom sketch: %s\n", err)
		return
	}

	top, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create the top plane: %s\n", err)
		return
	}
	s1, err := w.CreateSketch(top)
	if err != nil {
		fmt.Printf("failed to create the top sketch: %s\n", err)
		return
	}
	origin1 := s1.CreatePoint(0, 0)
	s1.Fix(origin1)
	px1 := s1.CreatePoint(5, 0)
	py1 := s1.CreatePoint(0, 5)
	s1.CreateLine(origin1, px1)
	s1.CreateLine(py1, origin1)
	s1.CreateArc(origin1, px1, py1)
	if _, err := s1.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the top sketch: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}

	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	// Values are rounded before printing: the chorded evaluator's own
	// arithmetic differs by a low-order bit between an FMA-contracting host
	// and one that is not, and a deterministic Output block must survive
	// that (never assert on an un-rounded float).
	wantVolume := math.Pi * 25 / 4 * 10
	fmt.Printf("volume: %.2f mm^3 (%s)\n", vol.Value.Base(), vol.Exactness)
	fmt.Printf("volume within closed form: %v\n", math.Abs(vol.Value.Base()-wantVolume) <= vol.Bound.Base())
	fmt.Printf("verify: %s\n", report.Status)
	// Output:
	// volume: 196.33 mm^3 (Approximate)
	// volume within closed form: true
	// verify: Sound
}
