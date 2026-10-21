package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Patterned works like PatternCopies but returns all the instances as one
// body. That is what you want when the pattern is a tool: a single Cut then
// removes every instance at once. Here it drills a row of holes.
func Example_pattern_as_one_body() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The plate is 60 x 40 x 5 mm.
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := plateSketch.CreateRectangle(0, -20, 60, 20)
	plateSketch.Fix(rect.A)
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

	// The drill is one round pin, 4 mm across, as long as the plate is thick.
	drillSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := drillSketch.CreatePoint(5, 0)
	drillSketch.Fix(center)
	drillSketch.CreateCircle(center, 2)
	if _, err := drillSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	drill, err := decad.Extrude(context.Background(), doc, drillSketch, drillSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude drill: %s\n", err)
		return
	}

	// Six pins, 10 mm apart along X, as a single tool body.
	pins, err := decad.Patterned(context.Background(), drill,
		decad.LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(10), Count: 6})
	if err != nil {
		fmt.Printf("failed to pattern: %s\n", err)
		return
	}

	// One Cut drills all six holes.
	drilled, err := decad.Cut(context.Background(), plate, pins)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	fmt.Printf("faces: %d, live bodies: %d\n", len(drilled.Faces()), len(doc.Bodies()))
	// Output:
	// faces: 12, live bodies: 1
}
