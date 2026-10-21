package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ThroughAll extrudes without a stated distance: the sweep runs until it has
// passed through every live body in its path. Use it when a feature must reach
// across whatever is there, such as a pin or a cut that has to go all the way
// through. The bodies it passes through are left untouched.
func Example_extrude_through_all() {
	// Draw both parts in one sketch: a large rectangle for the plate and a
	// small square for the pin. Sketch reports two profiles, one per closed shape.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	plateRect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(plateRect.A)
	s.CreateRectangle(120, 0, 140, 20)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var plateProfile, pinProfile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Area > 1000 {
			plateProfile = p // the large one is the plate
		} else {
			pinProfile = p
		}
	}

	// Extrude the plate first, so there is a body for the pin to pass through.
	doc := decad.New()
	if _, err := decad.Extrude(context.Background(), doc, s, plateProfile,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}); err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}

	// Then extrude the pin through all. It stops at the plate's far side,
	// so its height equals the plate's thickness.
	pin, err := decad.Extrude(context.Background(), doc, s, pinProfile, decad.ThroughAll{Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude pin: %s\n", err)
		return
	}
	bounds, err := pin.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("pin spans z: %v .. %v\n", bounds.Min.Z, bounds.Max.Z)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// pin spans z: 0 .. 10
	// live bodies: 2
}
