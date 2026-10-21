package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
)

// Loft joins two profiles on different planes with a smooth skin, making a
// solid that changes shape from one end to the other. Use it for tapered
// parts, adapters and transitions, such as going from a large square to a
// small one.
func Example_loft() {
	// The bottom profile is a 40 x 40 mm square on the XY plane.
	w := sketch.NewWorld()
	bottom, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	bottomRect := bottom.CreateRectangle(-20, -20, 20, 20)
	bottom.Fix(bottomRect.A)
	if _, err := bottom.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The top profile is a smaller 20 x 20 mm square, drawn on a plane 10 mm
	// above the first. Loft needs its two profiles on different planes.
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	top, err := w.CreateSketch(topPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	topRect := top.CreateRectangle(-10, -10, 10, 10)
	top.Fix(topRect.A)
	if _, err := top.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Loft from the bottom profile to the top one. The result is a frustum:
	// a pyramid with its tip cut off.
	doc := decad.New()
	frustum, err := decad.Loft(context.Background(), doc, bottom, bottom.Profiles()[0], top, top.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to loft: %s\n", err)
		return
	}
	volume, err := frustum.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\n", len(frustum.Faces()))
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 10
	// volume: 9333.3 mm^3
}
