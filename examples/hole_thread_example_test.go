package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A HoleSpec's Thread field taps the hole in the same call that drills it. The
// hole's Diameter is then the tap drill, the thread's minor diameter. Here an
// M5 x 0.8 thread goes into a 10 mm thick plate through a 4.2 mm drill.
func Example_hole_thread() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	tapped, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(4.2),
		Depth:    decad.ThroughHole{},
		Thread:   &decad.HoleThread{Pitch: units.Millimeters(0.8)},
	})
	if err != nil {
		fmt.Printf("failed to tap: %s\n", err)
		return
	}
	volume, err := tapped.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 4000 mm^3. The drill removes about 139 mm^3 and the thread
	// groove about 35 mm^3 more.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 3827 mm^3
}
