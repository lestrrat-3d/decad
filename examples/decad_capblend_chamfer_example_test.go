package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer also bevels a COMPLETE prism cap loop, not just lateral edges
// (docs/modify-reach-design.md §8.3): the selected loop's boundary offsets
// into the material at the cap, the original loop holds its shape one
// setback further in, and the band between them is a ruled patch per wall —
// a Plane for a straight one. The rewrite is exact and bounded, never a
// tessellated guess. The call resolves the edge query and single setback
// distance, so repeated construction is deterministic.
func Example_decad_capblend_chamfer() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 20 mm.
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

	// Select every edge of the end cap's rim loop — a complete loop, never a
	// lateral edge.
	loop := decad.Edges(decad.CreatedBy(decad.CapEnd(box)))
	body, err := box.Chamfer(context.Background(), loop, units.Millimeters(5))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}

	patches := 0
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				patches++
			}
		}
	}

	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("cap-blend patches: %d\n", patches)
	fmt.Printf("volume: %.4f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// solid: true, faces: 10
	// cap-blend patches: 4
	// volume: 116166.6667 mm^3 (Approximate)
	// live bodies: 1
}
