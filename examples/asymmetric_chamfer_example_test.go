package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithAsymmetricChamfer bevels an edge by two different distances, one on each
// of the two faces that meet there. Use it when the bevel must be longer on
// one side, for example to clear a mating part. A reference face says which
// side takes the first distance.
func Example_asymmetric_chamfer() {
	// A 40 x 20 mm plate, extruded 10 mm.
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
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The edge is the vertical one at the corner (40, 20). The reference
	// face is the wall facing +X, so that wall takes the first distance (3 mm)
	// and the wall beside it takes the option's distance (5 mm).
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 20, 0))).Exactly(1)
	wall := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
	beveled, err := decad.Chamfer(context.Background(), plate, edge, units.Millimeters(3),
		decad.WithAsymmetricChamfer(wall, units.Millimeters(5)))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate is 8000 mm^3. The bevel removes a 3 x 5 triangle, 10 mm tall.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 7925 mm^3
}
