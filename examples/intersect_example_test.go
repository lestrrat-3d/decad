package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Intersect keeps only the space two bodies share. Use it to trim one body to
// the envelope of another, or to find where two parts overlap. The inputs are
// consumed and the result is a new body.
func Example_intersect() {
	w := sketch.NewWorld()
	doc := decad.New()

	// Box A is 20 x 20 mm and 10 mm tall.
	aSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	aRect := aSketch.CreateRectangle(0, 0, 20, 20)
	aSketch.Fix(aRect.A)
	if _, err := aSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	a, err := decad.Extrude(context.Background(), doc, aSketch, aSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude box A: %s\n", err)
		return
	}

	// Box B is shifted 10 mm in X and Y, and taller, starting 5 mm below A.
	plane, err := w.CreateOffsetPlane(w.XY(), -5)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	bSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	bRect := bSketch.CreateRectangle(10, 10, 30, 30)
	bSketch.Fix(bRect.A)
	if _, err := bSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	b, err := decad.Extrude(context.Background(), doc, bSketch, bSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude box B: %s\n", err)
		return
	}

	// The shared space is 10 x 10 mm across and 10 mm tall.
	common, err := decad.Intersect(context.Background(), a, b)
	if err != nil {
		fmt.Printf("failed to intersect: %s\n", err)
		return
	}
	volume, err := common.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1000 mm^3
}
