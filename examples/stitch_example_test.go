package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Stitch welds sheets along their shared edges, and if the result encloses a
// volume it becomes a solid. Use it to build a solid from surfaces: walls from
// a surface extrude, closed by a patch at each end.
func Example_stitch() {
	w := sketch.NewWorld()
	ctx := context.Background()

	// The bottom outline, which is both the profile we extrude and the shape
	// of the bottom cap.
	bottomSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	bottomRect := bottomSketch.CreateRectangle(0, 0, 100, 60)
	bottomSketch.Fix(bottomRect.A)
	if _, err := bottomSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The top outline: the same rectangle, 10 mm up. The top cap is built
	// from this one, so its edges coincide exactly with the walls' top edges.
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	topSketch, err := w.CreateSketch(topPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	topRect := topSketch.CreateRectangle(0, 0, 100, 60)
	topSketch.Fix(topRect.A)
	if _, err := topSketch.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Three sheets: the four walls, and one flat cap for each end.
	doc := decad.New()
	walls, err := decad.Extrude(ctx, doc, bottomSketch, bottomSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude walls: %s\n", err)
		return
	}
	bottom, err := decad.PatchProfile(ctx, doc, bottomSketch, bottomSketch.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch bottom: %s\n", err)
		return
	}
	top, err := decad.PatchProfile(ctx, doc, topSketch, topSketch.Profiles()[0])
	if err != nil {
		fmt.Printf("failed to patch top: %s\n", err)
		return
	}

	// Weld them. The sheets close into a box, so the result is a solid.
	box, err := decad.Stitch(ctx, walls, bottom, top)
	if err != nil {
		fmt.Printf("failed to stitch: %s\n", err)
		return
	}
	volume, err := box.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("solid: %v, faces: %d\n", box.IsSolid(), len(box.Faces()))
	fmt.Printf("volume: %s\n", volume.Value)
	// Output:
	// solid: true, faces: 6
	// volume: 60000 mm^3
}
