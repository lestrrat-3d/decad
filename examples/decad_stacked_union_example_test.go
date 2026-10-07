package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A Union of a block standing on a plate keeps the part analytic: the result
// is a stacked prism with the plate's top as one face around the block.
func Example_decad_stacked_union() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	build := func(x0, y0, x1, y1, z, height float64) (*decad.Body, error) {
		plane, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		//nolint:contextcheck // Extrude has no context parameter.
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	plate, err := build(0, 0, 20, 20, 0, 5)
	if err != nil {
		fmt.Println(err)
		return
	}
	// The block is drawn on the plane of the plate's top and stands 10 mm.
	block, err := build(5, 5, 15, 15, 5, 10)
	if err != nil {
		fmt.Println(err)
		return
	}
	part, err := decad.Union(ctx, plate, block)
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	value, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("volume: %.0f mm^3 (%s), faces: %d\n", value, volume.Exactness, len(part.Faces()))
	// Output: volume: 3000 mm^3 (Exact), faces: 11
}
