package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Thread cuts a standard metric thread into a round rod or a bore from its
// pitch, so you do not have to draw the groove and coil it yourself. A rod's
// face is the thread's major diameter and a bore's face is its minor diameter.
func Example_thread() {
	// Make a 10 mm rod, 20 mm long: sketch a circle and extrude it (see
	// Example_extrude for these steps).
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

	// An M10 x 1.5 thread along the rod's one cylindrical wall.
	threaded, err := decad.Thread(context.Background(), rod, decad.ThreadSpec{
		Face:  decad.Faces(decad.Cylindrical()),
		Pitch: units.Millimeters(1.5),
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
	// The plain rod holds 1571 mm^3. The groove removes about 258 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 1313 mm^3
}

// LeftHand and Length change the thread's hand and how much of the face it
// covers. A left-hand thread screws the other way, and a Length shorter than the
// face stops the thread there, leaving the rest of the rod plain.
func Example_thread_left_hand_length() {
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

	// A left-hand M10 x 1.5 thread over the first 8 mm of the rod.
	threaded, err := decad.Thread(context.Background(), rod, decad.ThreadSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		Pitch:    units.Millimeters(1.5),
		LeftHand: true,
		Length:   units.Millimeters(8),
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
	// volume: 1476 mm^3
}

// Form replaces the ISO groove with a trapezoidal one, such as an Acme lead
// screw thread: its included flank angle, its depth and the width of its root.
// Here a 29 degree Acme-style groove 1.1 mm deep with a 0.74 mm root is cut into
// a 10 mm rod at a 2 mm pitch.
func Example_thread_form() {
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
		Face:  decad.Faces(decad.Cylindrical()),
		Pitch: units.Millimeters(2),
		Form: decad.ThreadForm{
			Angle: units.Degrees(29),
			Depth: units.Millimeters(1.1),
			Flat:  units.Millimeters(0.74),
		},
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
	// volume: 1252 mm^3
}

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

// A buttress groove has a steep flank on one side and a shallow one on the other:
// set FlankLead, the flank toward the face's start, and FlankTrail, the one toward
// its far end, in place of Angle. Here a 10 mm rod takes a buttress thread of pitch
// 3 mm, 45 degrees on the lead flank and 7 on the trail.
func Example_thread_buttress() {
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
		Face:  decad.Faces(decad.Cylindrical()),
		Pitch: units.Millimeters(3),
		Form: decad.ThreadForm{
			FlankLead:  units.Degrees(45),
			FlankTrail: units.Degrees(7),
			Depth:      units.Millimeters(1.5),
			Flat:       units.Millimeters(0.4),
		},
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
	// volume: 1225 mm^3
}
