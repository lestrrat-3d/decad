package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_extend() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	line := s.CreateLine(a, s.CreatePoint(100, 0))
	c := s.CreatePoint(40, -10)
	s.Fix(c)
	s.CreateLine(c, s.CreatePoint(40, 10))
	if _, err = s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	var left *sketch.Chain
	for _, chain := range s.Chains() {
		if len(chain.Edges) == 1 && chain.Edges[0].Entity == line && chain.Edges[0].TStart == 0 {
			left = chain
			break
		}
	}
	if left == nil {
		fmt.Println("failed to find left fragment")
		return
	}
	doc := decad.New()
	ribbon, err := doc.ExtrudeChain(s, left, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude chain: %s\n", err)
		return
	}
	toolSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create tool sketch: %s\n", err)
		return
	}
	rect := toolSketch.CreateRectangle(70, -10, 90, 10)
	toolSketch.Fix(rect.A)
	if _, err = toolSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve tool: %s\n", err)
		return
	}
	tool, err := doc.Extrude(toolSketch, toolSketch.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}
	query := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	extended, err := ribbon.Extend(ctx, query, tool)
	if err != nil {
		fmt.Printf("failed to extend ribbon: %s\n", err)
		return
	}
	area, err := extended.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	bounds, err := extended.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("area: %.0f mm²; max x: %.0f mm\n", area.Value.Base(), bounds.Max.X)
	// Output:
	// area: 700 mm²; max x: 70 mm
}
