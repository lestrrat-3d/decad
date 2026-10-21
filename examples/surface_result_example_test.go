package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithSurfaceResult makes a feature return a sheet instead of a solid: only the
// walls, with no end caps. Use a sheet when you want a surface to work with
// (trim it, patch it, offset it, stitch it into a solid) rather than a closed
// volume. The option works the same way on Extrude, Revolve, Loft and Sweep.
func Example_surface_result() {
	// Sketch a rectangle to extrude (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Extrude as usual, adding WithSurfaceResult. The result is a four-wall
	// tube with open ends.
	doc := decad.New()
	tube, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// A sheet has an area but no volume. Asking for the volume returns
	// ErrNotSolid, so the program can tell the two kinds of body apart.
	area, err := tube.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	_, volumeErr := tube.Volume()
	fmt.Printf("is a sheet: %v, faces: %d\n", tube.Kind() == decad.BodySheet, len(tube.Faces()))
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("volume is unavailable: %v\n", errors.Is(volumeErr, decad.ErrNotSolid))
	// Output:
	// is a sheet: true, faces: 4
	// area: 3200 mm^2
	// volume is unavailable: true
}
