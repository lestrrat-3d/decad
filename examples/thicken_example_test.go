package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Thicken turns a sheet into a solid by giving it thickness. Model a surface
// first, then thicken it to make a wall, a shell or a plate of known
// thickness. WithThickenSide chooses which side of the sheet the material
// grows toward.
func Example_thicken() {
	// The sheet is a four-wall tube: a rectangle extruded with
	// WithSurfaceResult (see Example_surface_result).
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
	doc := decad.New()
	tube, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Give the walls 5 mm of thickness growing inward, so the outside stays
	// at 100 x 60 mm.
	walls, err := decad.Thicken(context.Background(), tube, units.Millimeters(5),
		decad.WithThickenSide(decad.ThickenNegative))
	if err != nil {
		fmt.Printf("failed to thicken: %s\n", err)
		return
	}
	volume, err := walls.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("solid: %v\n", walls.Kind() == decad.BodySolid)
	fmt.Printf("volume: %s\n", volume.Value)
	// Output:
	// solid: true
	// volume: 15000 mm^3
}
