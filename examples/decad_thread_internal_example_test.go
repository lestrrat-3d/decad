package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// An internal thread cuts the groove into a bore. The block is an Extrude of
// the annulus 5 ≤ ρ ≤ 10 drawn on the XZ plane, so it runs 20 mm along −Y
// and its bore lies on the Y axis. The groove, drawn beside the Y axis on
// the XY plane, has its root 0.9 mm into the wall at ρ = 5.9 and its mouth
// opening 0.3 mm into the bore; it starts at y = −17 and coils 8 turns at a
// 1.5 mm pitch, both caps inside the block.
//
// The cut removes the screw sweep of the groove's part outside ρ = 5, Θ·Q
// with Θ = 16π, from π·(10² − 5²)·20. The published volume interval
// encloses that closed form and sits inside Verify's default tolerance; the
// area bound does not, so the report reads Suspect with the solid proven
// valid.
func Example_decad_thread_internal() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	s.CreateCircle(center, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var annulus *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			annulus = p
		}
	}
	if annulus == nil {
		fmt.Println("no annular profile")
		return
	}

	doc := decad.New()
	block, err := doc.Extrude(s, annulus, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	gs, gp, err := threadGroove(5.9, 4.7, -17)
	if err != nil {
		fmt.Printf("failed to draw the groove: %s\n", err)
		return
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	tool, err := doc.Coil(context.Background(), gs, gp, axis, units.Millimeters(1.5), units.Scalar(8))
	if err != nil {
		fmt.Printf("failed to coil: %s\n", err)
		return
	}
	threaded, err := decad.Cut(context.Background(), block, tool)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}

	vol, err := threaded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	exact := math.Pi * ((100-25)*20 - 16*clippedGrooveMoment(5.9, 4.7, 5))
	value, bound := vol.Value.Base(), vol.Bound.Base()
	fmt.Printf("closed form: %.2f mm³\n", exact)
	fmt.Printf("volume: %.1f ± %.1f mm³, encloses the closed form: %v\n", value, bound, math.Abs(value-exact) <= bound)
	fmt.Printf("lumps: %d, validity: %s, verify: %s\n", len(threaded.Lumps()), report.Bodies[0].Validity.Outcome, report.Status)
	// Output:
	// closed form: 4587.80 mm³
	// volume: 4587.3 ± 2.2 mm³, encloses the closed form: true
	// lumps: 1, validity: valid, verify: Suspect
}
