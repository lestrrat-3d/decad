package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Coil screws a sketch profile along an axis in its own plane: the section
// turns about the axis and slides along it by one pitch per turn, staying in
// the axis plane throughout. A 1 mm square wire 2 mm from the axis, coiled
// two turns at a 1.5 mm pitch, is a square-wire spring. Volume, area and
// centroid come from closed forms over the profile's own moments, so the
// volume is the revolve's Θ·∫ρ dA with Θ = 2π·turns. The extent is stated in
// turns: a spring of length L at pitch p passes units.Scalar(L/p), and the
// body then denotes pitch × that float as its height.
func Example_decad_coil() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// The wire section: u from 2 to 3 mm, v from 0 to 1 mm.
	rect := s.CreateRectangle(2, 0, 3, 1)
	s.Fix(rect.A)
	s.Fix(rect.C)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own v axis; the coil advances along +v.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

	doc := decad.New()
	spring, err := doc.Coil(context.Background(), s, s.Profiles()[0], axis,
		units.Millimeters(1.5), units.Scalar(2))
	if err != nil {
		fmt.Printf("failed to coil: %s\n", err)
		return
	}

	vol, err := spring.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	volume, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	c, err := spring.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	// 2π·2 turns · (2.5 mm · 1 mm²) = 10π mm³. After two whole turns the
	// centroid sits on the axis, half the 3 mm advance plus the section's own
	// 0.5 mm mean height up it — exactly.
	fmt.Printf("solid: %v, faces: %d\n", spring.IsSolid(), len(spring.Faces()))
	fmt.Printf("volume: %.4f mm³ (%s)\n", volume, vol.Exactness)
	fmt.Printf("centroid: (%.4f, %.4f, %.4f) (%s)\n", c.Value.X, c.Value.Y, c.Value.Z, c.Exactness)
	fmt.Printf("verify: %s\n", report.Status)
	// Output:
	// solid: true, faces: 6
	// volume: 31.4159 mm³ (Approximate)
	// centroid: (0.0000, 2.0000, 0.0000) (Exact)
	// verify: Sound
}
