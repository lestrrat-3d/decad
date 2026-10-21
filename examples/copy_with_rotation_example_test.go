package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithCopyRotation places a copy by turning it about an axis, given as a
// center point, a direction and an angle. Use it when the copy should be
// swung into position instead of slid, for example around a shaft.
func Example_copy_with_rotation() {
	// The master part is a small peg to copy.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	master, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Turn a copy a quarter turn about the Z axis through the origin. That
	// swings the peg from the first quadrant (x and y positive) into the
	// second (x negative, y positive).
	turned, err := decad.Copy(context.Background(), master,
		decad.WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90)))
	if err != nil {
		fmt.Printf("failed to copy: %s\n", err)
		return
	}
	centroid, err := turned.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	fmt.Printf("copy centroid: (%.1f, %.1f, %.1f)\n", centroid.Value.X, centroid.Value.Y, centroid.Value.Z)
	// Output:
	// copy centroid: (-5.0, 5.0, 2.0)
}
