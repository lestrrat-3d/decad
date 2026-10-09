package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A round-wire spring is a circle coiled about an axis beside it. The wire's
// section stays in the axis plane, so its volume is the revolve's
// Θ·∫ρ dA = 2π·turns · 3 mm · π·0.25 mm² for a 0.5 mm wire 3 mm from the
// axis. The wire's own wall has no elementary antiderivative; its area is a
// bracket that integrates a Taylor expansion of the area element piece by
// piece and charges the remainder, so the area bound stays near 1e-10 of the
// area. A whole circle's rim is a closed Circle3 edge on each end cap.
func Example_decad_coil_round_wire() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := s.CreatePoint(3, 0)
	s.Fix(center)
	s.CreateCircle(center, 0.5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// The axis is the sketch's own v axis; five turns at a 1.5 mm pitch.
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	doc := decad.New()
	spring, err := doc.Coil(context.Background(), s, s.Profiles()[0], axis,
		units.Millimeters(1.5), units.Scalar(5))
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
	area, err := spring.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	areaMM, err := area.Value.In(units.SquareMillimeter)
	if err != nil {
		fmt.Printf("failed to convert area: %s\n", err)
		return
	}
	relative := area.Bound.Base() / area.Value.Base()
	rims := 0
	for _, e := range spring.Edges() {
		if _, ok := e.Curve().(decad.Circle3); ok {
			rims++
		}
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	// The volume is 7.5π² mm³.
	fmt.Printf("faces: %d, circular rims: %d\n", len(spring.Faces()), rims)
	fmt.Printf("volume: %.4f mm³\n", volume)
	fmt.Printf("area: %.4f mm², bound below 1e-9 of it: %v\n", areaMM, relative < 1e-9)
	fmt.Printf("verify: %s\n", report.Status)
	// Output:
	// faces: 3, circular rims: 2
	// volume: 74.0220 mm³
	// area: 298.1304 mm², bound below 1e-9 of it: true
	// verify: Sound
}
