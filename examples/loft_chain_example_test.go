package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// LoftChain lofts between two open chains instead of closed profiles. The
// result is a sheet that has no volume. Use it for surfaces such as a curved
// panel or a ribbon between two guide curves. The second chain must lie on a
// plane above the first, in the direction of the first plane's normal.
func Example_loft_chain() {
	// Draw the same open L-shaped path twice: on the XY plane, and on a plane
	// 10 mm above it. Each path is a chain, because it is not closed.
	w := sketch.NewWorld()
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	var sketches [2]*sketch.Sketch
	for i, plane := range []*sketch.Plane{w.XY(), topPlane} {
		s, err := w.CreateSketch(plane)
		if err != nil {
			fmt.Printf("failed to create sketch: %s\n", err)
			return
		}
		a := s.CreatePoint(0, 0)
		b := s.CreatePoint(20, 0)
		c := s.CreatePoint(20, 15)
		s.Fix(a)
		s.Fix(b)
		s.Fix(c)
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		if _, err := s.Solve(context.Background()); err != nil {
			fmt.Printf("failed to solve: %s\n", err)
			return
		}
		sketches[i] = s
	}

	// Segment 1 of the first chain pairs with segment 1 of the second, and
	// so on, so the paths should have matching segments.
	doc := decad.New()
	ribbon, err := decad.LoftChain(context.Background(), doc,
		sketches[0], sketches[0].Chains()[0], sketches[1], sketches[1].Chains()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}
	area, err := ribbon.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	// Two flat walls, 20 x 10 mm and 15 x 10 mm, give 350 mm^2.
	fmt.Printf("sheet: %v, area: %.0f mm^2\n", ribbon.Kind() == decad.BodySheet, area.Value.Base())
	// Output:
	// sheet: true, area: 350 mm^2
}
