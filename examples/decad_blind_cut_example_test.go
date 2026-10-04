package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A blind Cut keeps a plate analytic. Later same-plane through cuts keep its
// section in both slabs, so holes can be added after the pocket.
func Example_decad_blind_cut() {
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
	plate, err := build(0, 0, 10, 10, 0, 10)
	if err != nil {
		fmt.Println(err)
		return
	}
	pocketTool, err := build(3, 3, 7, 7, 6, 4)
	if err != nil {
		fmt.Println(err)
		return
	}
	part, err := decad.Cut(ctx, plate, pocketTool)
	if err != nil {
		fmt.Println(err)
		return
	}
	for _, bounds := range [][4]float64{{1, 1, 2, 2}, {8, 8, 9, 9}} {
		tool, err := build(bounds[0], bounds[1], bounds[2], bounds[3], 0, 10)
		if err != nil {
			fmt.Println(err)
			return
		}
		part, err = decad.Cut(ctx, part, tool)
		if err != nil {
			fmt.Println(err)
			return
		}
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
	// Output: volume: 916 mm^3 (Exact), faces: 19
}
