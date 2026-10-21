package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A blind pocket is a Cut whose tool stops inside the part, so the cavity does
// not go all the way through. Use it for recesses, seats and pockets. Place the
// tool's sketch on an offset plane to choose how deep the pocket starts.
func Example_cut_blind_pocket() {
	w := sketch.NewWorld()
	doc := decad.New()

	// Start with the part to pocket: a cube.
	cubeSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	cubeRect := cubeSketch.CreateRectangle(0, 0, 10, 10)
	cubeSketch.Fix(cubeRect.A)
	if _, err := cubeSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	cube, err := decad.Extrude(context.Background(), doc, cubeSketch, cubeSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude cube: %s\n", err)
		return
	}

	// The tool is the shape of the pocket: a 4 x 4 mm block filling z = 6 to 10,
	// flush with the cube's top. Sketching it on a plane 6 mm up sets where the
	// pocket floor is, so the pocket is 4 mm deep and stops inside the part.
	plane, err := w.CreateOffsetPlane(w.XY(), 6)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	toolSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	toolRect := toolSketch.CreateRectangle(3, 3, 7, 7)
	toolSketch.Fix(toolRect.A)
	if _, err := toolSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	tool, err := decad.Extrude(context.Background(), doc, toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}

	pocketed, err := decad.Cut(context.Background(), cube, tool)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	volume, err := pocketed.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// 1000 mm^3 minus the 4 x 4 x 4 pocket.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 936 mm^3
}
