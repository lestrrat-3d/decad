package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Revolve spins a sketch profile about an axis and evaluates the solid of
// revolution analytically: volume and surface area come from Pappus's
// theorems on the recorded region's exact moments, so every measurement is
// closed form. A rectangle with one edge ON the axis revolves into a solid
// cylinder — the on-axis edge sweeps nothing, so there is no inner face, and
// a full revolution has no caps and no seam.
func Example_decad_revolve() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// The generating section: 10 mm long, reaching 8 mm from the axis.
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own u axis, stated in plane coordinates.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	volume, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	area, err := body.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	surface, err := area.Value.In(units.SquareMillimeter)
	if err != nil {
		fmt.Printf("failed to convert area: %s\n", err)
		return
	}
	c, err := body.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}

	// π·8²·10 = 640π mm³; the centroid sits ON the axis.
	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("volume: %.4f mm³ (%s)\n", volume, vol.Exactness)
	fmt.Printf("area: %.4f mm²\n", surface)
	fmt.Printf("centroid: (%.4f, %.4f, %.4f)\n", c.Value.X, c.Value.Y, c.Value.Z)
	// Output:
	// solid: true, faces: 3
	// volume: 2010.6193 mm³ (Exact)
	// area: 904.7787 mm²
	// centroid: (5.0000, 0.0000, 0.0000)
}
