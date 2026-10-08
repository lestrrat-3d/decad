package examples_test

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Split cuts a solid revolve with a sheet spun about the same axis: here a
// cylinder of radius 4 and height 10 by a disc at height 6, which leaves
// cylinders of 96π and 64π mm³.
func Example_decad_splitRevolve() {
	// Both bodies spin about the sketch plane's own v axis.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	w := sketch.NewWorld()
	doc := decad.New()

	cylSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create cylinder sketch: %s\n", err)
		return
	}
	rect := cylSketch.CreateRectangle(0, 0, 4, 10)
	cylSketch.Fix(rect.A)
	if _, err := cylSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve cylinder sketch: %s\n", err)
		return
	}
	cylinder, err := doc.Revolve(cylSketch, cylSketch.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve cylinder: %s\n", err)
		return
	}

	// The disc is one line from the axis outward, spun as an open meridian.
	discSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create disc sketch: %s\n", err)
		return
	}
	start := discSketch.CreatePoint(0, 6)
	discSketch.Fix(start)
	end := discSketch.CreatePoint(8, 6)
	discSketch.Fix(end)
	discSketch.CreateLine(start, end)
	if _, err := discSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve disc sketch: %s\n", err)
		return
	}
	disc, err := doc.RevolveChain(discSketch, discSketch.Chains()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve disc: %s\n", err)
		return
	}

	pieces, err := doc.Split(context.Background(), cylinder, disc)
	if err != nil {
		fmt.Printf("failed to split cylinder: %s\n", err)
		return
	}
	var volumes []float64
	for _, piece := range pieces {
		v, err := piece.Volume()
		if err != nil {
			fmt.Printf("failed to read piece volume: %s\n", err)
			return
		}
		volumes = append(volumes, v.Value.Base()/math.Pi)
	}
	slices.Sort(volumes)
	fmt.Printf("solid pieces: %d\n", len(pieces))
	for _, v := range volumes {
		fmt.Printf("volume: %.6fπ mm³\n", v)
	}
	// Output:
	// solid pieces: 2
	// volume: 64.000000π mm³
	// volume: 96.000000π mm³
}
