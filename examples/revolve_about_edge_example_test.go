package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// EdgeAxis uses an edge of an existing body as the axis of a Revolve. Use it
// when the axis is part of the model, so that the revolved feature follows the
// edge if the model changes, instead of repeating its coordinates.
func Example_revolve_about_edge() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The gusset is the existing part. We will use one of its edges as a
	// rotation axis, so extrude a triangle that has one edge along X.
	gussetSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	a := gussetSketch.CreatePoint(0, 0)
	gussetSketch.Fix(a)
	b := gussetSketch.CreatePoint(100, 0)
	c := gussetSketch.CreatePoint(0, 60)
	gussetSketch.CreateLine(a, b)
	gussetSketch.CreateLine(b, c)
	gussetSketch.CreateLine(c, a)
	if _, err := gussetSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	gusset, err := decad.Extrude(context.Background(), doc, gussetSketch, gussetSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The ring's cross-section is a square 5 mm away from that edge. Revolving
	// it around the edge makes a ring.
	ringSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	square := ringSketch.CreateRectangle(0, 5, 10, 15)
	ringSketch.Fix(square.A)
	if _, err := ringSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Name the axis by a query: the one edge created by the gusset's start
	// face that is parallel to X.
	axis := decad.EdgeAxis{
		Body: gusset,
		Edge: decad.Edges(
			decad.CreatedBy(decad.CapStart(gusset)),
			decad.ParallelTo(r3.NewVec(1, 0, 0)),
		).Exactly(1),
	}
	ring, err := decad.Revolve(context.Background(), doc, ringSketch, ringSketch.Profiles()[0],
		axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}
	volume, err := ring.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The ring's volume is 2π x 10 x 100 = 2000π mm^3.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 6283.2 mm^3
}
