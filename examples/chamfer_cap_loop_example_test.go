package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer can bevel the whole outline of an end face at once. Use it to break
// the sharp rim of a plate or lid. The edges are selected by where they came
// from ("the edges created by the end cap") rather than by position.
func Example_chamfer_cap_loop() {
	// Make a plate whose top rim we will bevel: sketch a rectangle and extrude it
	// (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	box, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Select the four edges of the top face. The end cap is the face the extrude
	// created at its far end, so "edges created by the end cap" finds them.
	// Beveling them 5 mm removes the sharp rim.
	rim := decad.Edges(decad.CreatedBy(decad.CapEnd(box)))
	beveled, err := decad.Chamfer(context.Background(), box, rim, units.Millimeters(5))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\n", len(beveled.Faces()))
	// Output:
	// faces: 10
}
