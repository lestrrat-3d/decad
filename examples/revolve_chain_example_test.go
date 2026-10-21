package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Revolve also accepts an open chain of sketch edges instead of a closed
// profile. The result is a surface of revolution: a sheet with no volume. Use
// it for thin shells such as a funnel, a lampshade or a bell, which you may
// thicken into a solid later.
func Example_revolve_chain() {
	// Draw an open two-segment path clear of the axis. It is the wall's
	// side view: straight up 5 mm, then slanting outward.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := s.CreatePoint(10, 0)
	b := s.CreatePoint(10, 5)
	c := s.CreatePoint(15, 8)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Spin it about the sketch's V (vertical) axis. A chain goes where a
	// profile would go.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	doc := decad.New()
	bell, err := decad.Revolve(context.Background(), doc, s, s.Chains()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// One face per path segment, and no caps: a cylinder wall and a cone wall.
	fmt.Printf("sheet: %v, faces: %d\n", bell.Kind() == decad.BodySheet, len(bell.Faces()))
	// Output:
	// sheet: true, faces: 2
}
