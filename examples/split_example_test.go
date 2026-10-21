package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Split divides a solid along a sheet (a zero-thickness surface) and returns
// one solid body per piece. Use it to cut a part in two, for example to
// print it in halves or to work on each side separately.
func Example_split() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The block is 100 x 60 x 10 mm.
	blockSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := blockSketch.CreateRectangle(0, 0, 100, 60)
	blockSketch.Fix(rect.A)
	if _, err := blockSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), doc, blockSketch, blockSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude block: %s\n", err)
		return
	}

	// The cutting sheet comes from extruding a single line (an open chain)
	// at y = 30, so it stands upright across the middle of the block. It
	// extends 15 mm up and 5 mm down, which carries it past both block faces.
	lineSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	start := lineSketch.CreatePoint(-10, 30)
	lineSketch.Fix(start)
	lineSketch.CreateLine(start, lineSketch.CreatePoint(110, 30))
	if _, err := lineSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	sheet, err := decad.Extrude(context.Background(), doc, lineSketch, lineSketch.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(15)},
		Two: decad.DistanceSide{D: units.Millimeters(5)},
	})
	if err != nil {
		fmt.Printf("failed to extrude sheet: %s\n", err)
		return
	}

	pieces, err := decad.Split(context.Background(), block, sheet)
	if err != nil {
		fmt.Printf("failed to split: %s\n", err)
		return
	}
	for i, piece := range pieces {
		volume, err := piece.Volume()
		if err != nil {
			fmt.Printf("failed to measure volume: %s\n", err)
			return
		}
		fmt.Printf("piece %d: %.0f mm^3\n", i, volume.Value.Base())
	}
	// Output:
	// piece 0: 30000 mm^3
	// piece 1: 30000 mm^3
}
