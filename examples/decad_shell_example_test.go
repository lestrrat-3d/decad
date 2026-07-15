package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Shell hollows a straight prism through the faces a selector names. Removing
// both caps of a hole-free box gives a TUBE — a plain prism over the annular
// section, so every measurement stays Exact and the body is a first-class prism
// downstream. The construction is the recorded section's own exact inward
// offset (P ⊖ t): the wall is uniform, and the volume is (A_P − A_Q)·h.
func Example_decad_shell() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 20 mm — a box with two caps.
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

	// Remove both caps (planar, normal to the z sweep) and line the rest with a
	// 5 mm wall — a rectangular tube.
	tube, err := box.Shell(decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(5))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	vol, err := tube.Volume()
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

	fmt.Printf("solid: %v, faces: %d\n", tube.IsSolid(), len(tube.Faces()))
	fmt.Printf("volume: %.3f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("trustworthy: %v\n", report.Trustworthy())
	fmt.Printf("recipe steps: %d\n", len(doc.Recipe().Steps))
	// Output:
	// solid: true, faces: 10
	// volume: 30000.000 mm^3 (Exact)
	// trustworthy: true
	// recipe steps: 2
}
