package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Union joins bodies into one. Use it to build a part from simpler pieces,
// such as a boss standing on a plate. Union takes any number of bodies after
// the first, so a whole stack can be joined in one call. The inputs are
// consumed and the result is a new body.
func Example_union() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The first piece is a plate to build on.
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	plateRect := plateSketch.CreateRectangle(0, 0, 20, 20)
	plateSketch.Fix(plateRect.A)
	if _, err := plateSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), doc, plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}

	// The second piece is a block that stands on the plate. Sketching it on a
	// plane 5 mm up puts its bottom on the plate's top face, so the two pieces
	// touch and can be joined.
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	blockSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	blockRect := blockSketch.CreateRectangle(5, 5, 15, 15)
	blockSketch.Fix(blockRect.A)
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

	part, err := decad.Union(context.Background(), plate, block)
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// 2000 mm^3 for the plate plus 1000 mm^3 for the block.
	fmt.Printf("volume: %.0f mm^3, bodies: %d\n", volume.Value.Base(), len(doc.Bodies()))
	// Output:
	// volume: 3000 mm^3, bodies: 1
}
