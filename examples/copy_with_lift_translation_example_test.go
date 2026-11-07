package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithCopyLiftTranslation moves a copy by an offset measured in a lift's own
// coordinates. The lift is the placement the parts of an assembly share, for
// example a tilted plate that carries the holes the copy must seat in. The
// world translation is the lift's linear part times the local offset, formed
// exactly, so the copy stays on the lift's grid.
func Example_copy_with_lift_translation() {
	ctx := context.Background()
	// The lift is a quarter turn about the Z axis through the origin: the lift's
	// local X axis points along world Y.
	lift, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	if err != nil {
		fmt.Printf("failed to build lift: %s\n", err)
		return
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	master, err := decad.Extrude(ctx, doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Move the copy 20 mm along the lift's local X axis, which is world Y.
	moved, err := decad.Copy(ctx, master,
		decad.WithCopyLiftTranslation(lift, r3.Vec{X: 10}, r3.Vec{X: 30}))
	if err != nil {
		fmt.Printf("failed to copy: %s\n", err)
		return
	}
	centroid, err := moved.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	fmt.Printf("copy centroid: (%.1f, %.1f, %.1f)\n", centroid.Value.X, centroid.Value.Y, centroid.Value.Z)
	// Output:
	// copy centroid: (5.0, 25.0, 2.0)
}
