package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shell with WithNoOpenings and a nil selector hollows a straight prism into a
// sealed box: every face stays, a wall of the given thickness lines all six,
// and the cavity is a second, void shell inside the first. The 100 x 60 x 20
// box with a 5 mm wall keeps a 90 x 50 x 10 cavity, so its volume is
// 120000 - 45000 mm^3.
func Example_decad_closed_shell() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
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
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	sealed, err := box.Shell(context.Background(), nil, units.Millimeters(5), decad.WithNoOpenings())
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	vol, err := sealed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	fmt.Printf("lumps: %d\n", len(sealed.Lumps()))
	for i, shell := range sealed.Shells() {
		fmt.Printf("shell %d: %d faces, void: %v\n", i, len(shell.Faces()), shell.IsVoid())
	}
	fmt.Printf("volume: %.3f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("trustworthy: %v\n", report.Passed())
	// Output:
	// lumps: 1
	// shell 0: 6 faces, void: false
	// shell 1: 6 faces, void: true
	// volume: 75000.000 mm^3 (Exact)
	// trustworthy: true
}
