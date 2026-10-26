package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A hole does not have to be round. Set HoleSpec.Shape to a RectangularHole
// to cut a slot-like opening, such as a window for a connector. The shape
// replaces Diameter, which stays zero, and takes no counterbore or countersink.
func Example_hole_rectangular() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 30)
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

	// Cut a 12 mm by 8 mm window through the middle of the top face, turned
	// 30 degrees about the hole axis.
	cut, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:  decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:    r3.Vec{X: 20, Y: 15, Z: 10},
		Depth: decad.ThroughHole{},
		Shape: decad.RectangularHole{
			Width:  units.Millimeters(12),
			Height: units.Millimeters(8),
			Angle:  units.Degrees(30),
		},
	})
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}
	volume, err := cut.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate held 12000 mm^3. The window removes 12 * 8 * 10 = 960 mm^3,
	// whatever its turn.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 11040 mm^3
}
