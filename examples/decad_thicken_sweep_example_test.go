package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A one-span sweep sheet can be thickened on either side or centered. The
// resulting solid keeps Sweep's path and face roles for later operations.
func Example_decad_thickenSweep() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 10)})
	if err != nil {
		fmt.Printf("failed to create path: %s\n", err)
		return
	}
	doc := decad.New()
	sheet, err := doc.Sweep(context.Background(), s, s.Profiles()[0], path, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	solid, err := sheet.Thicken(context.Background(), units.Millimeters(1),
		decad.WithThickenSide(decad.ThickenNegative))
	if err != nil {
		fmt.Printf("failed to thicken: %s\n", err)
		return
	}
	volume, err := solid.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mesh, err := solid.Tessellate(context.Background(), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	fmt.Printf("solid: %t\n", solid.IsSolid())
	fmt.Printf("volume: %s\n", volume.Value)
	fmt.Printf("mesh volume verified: %t\n", mesh.VolumeVerified())
	// Output:
	// solid: true
	// volume: 3160 mm^3
	// mesh volume verified: true
}
