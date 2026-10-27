package leadin_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LeadIn chamfers the threaded face's ends at 45 degrees before the thread is cut,
// so a screw starts into its nut without a sharp edge to catch. Here an M10 rod
// gets a 0.8 mm lead-in on both ends.
func Example_thread_lead_in() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	threaded, err := decad.Thread(context.Background(), rod, decad.ThreadSpec{
		Face:   decad.Faces(decad.Cylindrical()),
		Pitch:  units.Millimeters(1.5),
		LeadIn: units.Millimeters(0.8),
	})
	if err != nil {
		fmt.Printf("failed to thread: %s\n", err)
		return
	}
	volume, err := threaded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1306 mm^3
}

// LeadInAngle sets the slope of the lead-in's bevel: LeadIn is its distance along
// the end face and the bevel runs LeadIn·tan(angle) along the rod's side, so a
// 30 degree bevel is shallow, a gentle start for a screw.
func Example_thread_lead_in_angle() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	threaded, err := decad.Thread(context.Background(), rod, decad.ThreadSpec{
		Face:        decad.Faces(decad.Cylindrical()),
		Pitch:       units.Millimeters(1.5),
		LeadIn:      units.Millimeters(1),
		LeadInAngle: units.Degrees(30),
	})
	if err != nil {
		fmt.Printf("failed to thread: %s\n", err)
		return
	}
	volume, err := threaded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1306 mm^3
}
