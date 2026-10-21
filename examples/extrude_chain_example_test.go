package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Extrude also accepts an open chain of sketch edges instead of a closed
// profile. The result is a sheet body: one wall per edge, with no caps. Use it
// for thin walls and surfaces, which can later be thickened into a solid.
func Example_extrude_chain() {
	// Draw a line-arc-line path that is NOT closed. Because it is open, the
	// sketch has a chain instead of a profile, and a chain is what Extrude needs
	// to build a wall instead of a solid.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(15, 5)
	d := s.CreatePoint(25, 5)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateArc(s.CreatePoint(10, 5), b, c)
	s.CreateLine(c, d)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Pass the chain where a profile would normally go.
	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, s.Chains()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	fmt.Printf("sheet: %v, solid: %v, walls: %d\n",
		body.Kind() == decad.BodySheet, body.IsSolid(), len(body.Faces()))
	// Output:
	// sheet: true, solid: false, walls: 3
}
