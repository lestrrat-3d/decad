package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A meridian ending on the axis revolves into a cone sheet with one free rim.
func Example_decad_revolve_chain_pole() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	pole := s.CreatePoint(0, 0)
	s.Fix(pole)
	s.CreateLine(pole, s.CreatePoint(4, 3))
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.New().RevolveChain(s, s.Chains()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve chain: %s\n", err)
		return
	}
	free, err := decad.Edges(decad.Free()).Exactly(1).SelectEdges(body)
	if err != nil {
		fmt.Printf("failed to select free rim: %s\n", err)
		return
	}
	area, err := body.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	mm2, err := area.Value.In(units.SquareMillimeter)
	if err != nil {
		fmt.Printf("failed to convert area: %s\n", err)
		return
	}
	fmt.Printf("sheet: %t, faces: %d, free rims: %d, area: %.2f mm2\n",
		body.Kind() == decad.BodySheet, len(body.Faces()), len(free), mm2)
	// Output:
	// sheet: true, faces: 1, free rims: 1, area: 47.12 mm2
}
