package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// SweepPair reports SweepFractionFloor when the requested TimeResolution is so
// fine, relative to the duration, that the search cannot name an instant that
// small as a dyadic fraction of the duration. The outcome is SweepUndecided: no
// impact is claimed, and the cause says which limit stopped the search.
func Example_contact_sweep_fraction_floor() {
	ctx := context.Background()
	doc := decad.New()

	box := func(x0, x1 float64) (*decad.Body, error) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, 0, x1, 10)
		s.Fix(rect.A)
		if _, err := s.Solve(ctx); err != nil {
			return nil, err
		}
		return decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	}
	a, err := box(0, 10)
	if err != nil {
		fmt.Printf("failed to create box: %s\n", err)
		return
	}
	b, err := box(20, 30)
	if err != nil {
		fmt.Printf("failed to create box: %s\n", err)
		return
	}

	drift := func(vx float64) decad.RigidDriftSegment {
		return decad.RigidDriftSegment{
			From: r3.Identity(),
			LinearVelocity: decad.QuantityVec{
				X: units.MillimetersPerSecond(vx),
				Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(0),
			},
			AngularVelocity: decad.QuantityVec{
				X: units.RadiansPerSecond(0),
				Y: units.RadiansPerSecond(0),
				Z: units.RadiansPerSecond(0),
			},
			Duration: units.Seconds(0.2),
		}
	}
	contact := decad.ContactRequest{
		PointResolution:  units.Millimeters(1e-6),
		NormalResolution: units.Degrees(1),
	}
	req := decad.SweepRequest{
		ContactRequest: contact,
		// 1e-30 s over a 0.2 s duration is finer than 2^-60 of it.
		TimeResolution:     units.Seconds(1e-30),
		MaxPoseEvaluations: 8,
	}
	report, err := doc.SweepPair(ctx, a, b, drift(100), drift(0), req)
	if err != nil {
		fmt.Printf("failed to sweep: %s\n", err)
		return
	}
	fmt.Println(report.Outcome == decad.SweepUndecided)
	fmt.Println(report.Cause == decad.SweepFractionFloor)
	// Output:
	// true
	// true
}
