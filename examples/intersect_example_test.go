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

	// Box A is the first body to compare.
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

	// Box B is the second. It overlaps A by half its width in X and Y, and it is
	// taller, starting 5 mm below A, so the shared space is a clean box.
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

	// What remains is the shared space: 10 mm in X, 10 mm in Y and 10 mm tall.
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
