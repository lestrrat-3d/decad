package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A boolean union can take different chamfer setbacks across its two faces
// beside an independent straight edge. The reference face takes the first
// distance; the other face takes the option's distance.
func Example_decad_asymmetric_brep_chamfer() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	block := func(width, z float64) (*decad.Body, error) {
		plane, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(0, 0, width, 20)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return doc.Extrude(s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	}
	plate, err := block(40, 0)
	if err != nil {
		fmt.Printf("failed to make plate: %s\n", err)
		return
	}
	boss, err := block(10, 10)
	if err != nil {
		fmt.Printf("failed to make boss: %s\n", err)
		return
	}
	part, err := decad.Union(ctx, plate, boss)
	if err != nil {
		fmt.Printf("failed to join bodies: %s\n", err)
		return
	}

	front := decad.Faces(decad.Facing(r3.NewVec(0, -1, 0)))
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	beveled, err := part.Chamfer(ctx, edge, units.Millimeters(2),
		decad.WithAsymmetricChamfer(front, units.Millimeters(3)))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %g mm^3 (%s)\n", mm3, volume.Exactness)
	// Output:
	// volume: 9970 mm^3 (Exact)
}
