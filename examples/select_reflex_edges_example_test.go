package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Reflex selects the edges whose material angle is over a half turn: the root
// of a boss on a Union result, or an inside corner. Concave cannot pick them,
// because it reads the walked boundary, under which a boss root is convex.
// Edge.Wedge reports the same reading for one edge.
func Example_selectReflexEdges() {
	ctx := context.Background()
	doc := decad.New()
	// box sketches the rectangle x0..x1, y0..y1 at height z0 and extrudes it
	// up by h (see Example_extrude for these steps).
	box := func(x0, y0, x1, y1, z0, h float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), z0)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		s.Fix(rect.C)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	}
	// An 80 x 60 x 10 mm plate with a 20 x 20 mm boss, 15 mm tall.
	plate, err := box(0, 0, 80, 60, 0, 10)
	if err != nil {
		fmt.Printf("failed to build the plate: %s\n", err)
		return
	}
	boss, err := box(30, 20, 50, 40, 10, 15)
	if err != nil {
		fmt.Printf("failed to build the boss: %s\n", err)
		return
	}
	part, err := decad.Union(ctx, plate, boss)
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}

	roots, err := decad.Edges(decad.Reflex()).SelectEdges(part)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("reflex edges: %d\n", len(roots))
	fmt.Printf("first root: wedge %s, IsConvex %v, z = %.0f\n",
		roots[0].Wedge(), roots[0].IsConvex(), roots[0].Start().Position().Value.Z)

	// Round the boss root with a 2 mm fillet. The fillet adds material.
	rounded, err := decad.Fillet(ctx, part, decad.Edges(decad.Reflex()).Exactly(4), units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	volume, err := rounded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.2f mm^3\n", volume.Value.Base())
	// Output:
	// reflex edges: 4
	// first root: wedge reflex, IsConvex true, z = 10
	// volume: 54071.08 mm^3
}
