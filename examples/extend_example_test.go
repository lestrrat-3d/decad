package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Extend grows a sheet's free edge until it reaches another body. Use it to
// make a surface run all the way to a mating part, so that no gap is left
// between them.
func Example_extend() {
	ctx := context.Background()

	// Draw a 100 mm line crossed by a short line at x = 40. Sketch splits the
	// long line at the crossing. The left fragment (x = 0 to 40) is the wall
	// we want to extend.
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
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
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
		fmt.Println("failed to find the left fragment")
		return
	}

	// Extrude that fragment 10 mm into a wall: a sheet whose right edge, at
	// x = 40, is free.
	doc := decad.New()
	wall, err := decad.Extrude(ctx, doc, s, left, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude wall: %s\n", err)
		return
	}

	// The body to extend to is a solid block further along X, from x = 70
	// to 90. It is taller than the wall in both directions.
	toolSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create tool sketch: %s\n", err)
		return
	}
	block := toolSketch.CreateRectangle(70, -10, 90, 10)
	toolSketch.Fix(block.A)
	if _, err := toolSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve tool sketch: %s\n", err)
		return
	}
	tool, err := decad.Extrude(ctx, doc, toolSketch, toolSketch.Profiles()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}

	// Name the edge to extend: the free, vertical edge at (40, 0). Extend
	// grows the wall until it meets the block.
	edge := decad.Edges(decad.Free(), decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(40, 0, 0))).Exactly(1)
	extended, err := decad.Extend(ctx, wall, edge, tool)
	if err != nil {
		fmt.Printf("failed to extend: %s\n", err)
		return
	}
	bounds, err := extended.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("wall now reaches x = %v\n", bounds.Max.X)
	// Output:
	// wall now reaches x = 70
}
