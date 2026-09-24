package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Split returns one solid body for each cell a sheet separates from a solid.
func Example_decad_split() {
	w := sketch.NewWorld()
	blockSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create block sketch: %s\n", err)
		return
	}
	rect := blockSketch.CreateRectangle(0, 0, 100, 60)
	blockSketch.Fix(rect.A)
	if _, err := blockSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve block sketch: %s\n", err)
		return
	}
	doc := decad.New()
	block, err := doc.Extrude(blockSketch, blockSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude block: %s\n", err)
		return
	}

	toolSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create tool sketch: %s\n", err)
		return
	}
	start := toolSketch.CreatePoint(-10, 30)
	toolSketch.Fix(start)
	toolSketch.CreateLine(start, toolSketch.CreatePoint(110, 30))
	if _, err := toolSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve tool sketch: %s\n", err)
		return
	}
	tool, err := doc.ExtrudeChain(toolSketch, toolSketch.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}
	pieces, err := doc.Split(context.Background(), block, tool)
	if err != nil {
		fmt.Printf("failed to split block: %s\n", err)
		return
	}
	fmt.Printf("solid pieces: %d\n", len(pieces))
	// Output: solid pieces: 2
}
