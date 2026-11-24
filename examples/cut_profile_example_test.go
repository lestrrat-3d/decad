package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// CutExtrude cuts several pockets into a plate from one sketch in one step,
// without building a cutter body first.
func Example_cutExtrude() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	plate := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(plate.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Two 4 x 4 mm pockets, 3 mm deep, drawn in one sketch on the same plane.
	c, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := c.CreateRectangle(6, 6, 10, 10)
	c.Fix(a.A)
	b := c.CreateRectangle(30, 30, 34, 34)
	c.Fix(b.A)
	if _, err := c.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	out, err := decad.CutExtrude(context.Background(), body, c, c.Profiles(),
		decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	vol, err := out.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	mm3, _ := vol.Value.In(units.CubicMillimeter)
	fmt.Printf("%.0f mm3\n", mm3)
	// Output: 7904 mm3
}
