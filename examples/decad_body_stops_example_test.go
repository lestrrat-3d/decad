package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A through-all extrude has no distance of its own: the sweep runs through
// the far side of every live body it meets. The stop is resolved at the
// call, and each stop body's StepRef is recorded in the step's Inputs — the
// recipe stays a complete graph, with no ambient body-set dependency. The
// stop body is depended on, never consumed: it stays live.
func Example_decad_extrude_through_all() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// Two profiles on one sketch: a 100 x 60 base plate, and a 20 x 20 pin
	// footprint beside it.
	plate := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(plate.A)
	s.CreateRectangle(120, 0, 140, 20)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var plateProfile, pinProfile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Area > 1000 {
			plateProfile = p
		} else {
			pinProfile = p
		}
	}

	doc := decad.New()
	// Step 0: the plate, 10 mm thick — the body the through-all sweep meets.
	if _, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}
	// Step 1: the pin, through all — it stops at the plate's far side, so
	// its height is exactly the plate's thickness.
	pin, err := doc.Extrude(s, pinProfile, decad.ThroughAll{Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude pin: %s\n", err)
		return
	}

	vol, err := pin.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	bounds, err := pin.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}

	step := doc.Recipe().Steps[1]
	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	fmt.Printf("pin spans z: %v .. %v\n", bounds.Min.Z, bounds.Max.Z)
	fmt.Printf("recorded inputs: %v\n", step.Inputs)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// volume: 4000 mm^3 (Exact)
	// pin spans z: 0 .. 10
	// recorded inputs: [0]
	// live bodies: 2
}
