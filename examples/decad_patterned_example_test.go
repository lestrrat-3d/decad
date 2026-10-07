package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Patterned returns one body holding every instance. Disjoint instances of a
// prism become one prism group, which cuts a row of holes in a single
// analytic step: the plate keeps exact walls and stays filletable.
func Example_decad_patterned() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	extrude := func(draw func(*sketch.Sketch)) (*decad.Body, error) {
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		draw(s)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		//nolint:contextcheck // Extrude has no context parameter.
		return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	}
	plate, err := extrude(func(s *sketch.Sketch) {
		rect := s.CreateRectangle(0, -20, 60, 20)
		s.Fix(rect.A)
	})
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	drill, err := extrude(func(s *sketch.Sketch) {
		c := s.CreatePoint(5, 0)
		s.Fix(c)
		s.CreateCircle(c, 2)
	})
	if err != nil {
		fmt.Printf("failed to build the drill: %s\n", err)
		return
	}

	// Six drills at 10 mm along +x, as one body.
	row, err := drill.Patterned(ctx, decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 6})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}
	part, err := decad.Cut(ctx, plate, row)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert: %s\n", err)
		return
	}
	fmt.Printf("drill lumps: %d\n", len(row.Lumps()))
	fmt.Printf("plate faces: %d, volume: %.3f mm^3\n", len(part.Faces()), mm3)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// drill lumps: 6
	// plate faces: 12, volume: 11623.009 mm^3
	// live bodies: 1
}
