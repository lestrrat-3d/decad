package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A single straight cap edge can be chamfered without selecting the whole
// cap loop. A box takes route E and keeps an exact volume.
func Example_decad_single_cap_edge_chamfer() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	edge := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(0, 0, 10))).Exactly(1)
	body, err := box.Chamfer(context.Background(), edge, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3 (%s)\n", mm3, volume.Exactness)
	// Output: volume: 7920 mm^3 (Exact)
}
