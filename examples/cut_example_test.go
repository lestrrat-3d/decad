package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Cut removes one body (the tool) from another (the target). Use it for holes,
// slots and pockets: model the shape to remove as its own body, then subtract
// it. Both inputs are consumed and the result is a new body.
func Example_cut() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The target is a 20 x 20 x 8 mm plate.
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := plateSketch.CreateRectangle(0, 0, 20, 20)
	plateSketch.Fix(rect.A)
	if _, err := plateSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), doc, plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}

	// The tool is a 4 mm wide pin, longer than the plate is thick.
	pinSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := pinSketch.CreatePoint(14, 6)
	pinSketch.Fix(center)
	pinSketch.CreateCircle(center, 2)
	if _, err := pinSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	pin, err := decad.Extrude(context.Background(), doc, pinSketch, pinSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude pin: %s\n", err)
		return
	}

	// Move the pin down 6 mm so it sticks out of both plate faces. A tool
	// that ends exactly on a plate face is a harder case for the cut.
	down, err := r3.Translation(r3.Vec{Z: -6})
	if err != nil {
		fmt.Printf("failed to build translation: %s\n", err)
		return
	}
	tool, err := decad.Transform(context.Background(), pin, down)
	if err != nil {
		fmt.Printf("failed to move tool: %s\n", err)
		return
	}

	drilled, err := decad.Cut(context.Background(), plate, tool)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	volume, err := drilled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate held 3200 mm^3. The hole removes about 100.5 mm^3.
	fmt.Printf("faces: %d\n", len(drilled.Faces()))
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 7
	// volume: 3099.5 mm^3
}
