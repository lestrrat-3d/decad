package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A SlotHole shape cuts a stadium: a strip with a half circle on each end.
// Use it for an adjustable screw slot, where the screw can sit anywhere along
// the opening. Length counts end to end, so the strip between the two end
// centres is Length minus Width long.
func Example_hole_slot() {
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

	// Cut a 16 mm long, 6 mm wide slot through the middle of the top face,
	// along the X axis.
	cut, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:  decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:    r3.Vec{X: 20, Y: 15, Z: 10},
		Depth: decad.ThroughHole{},
		Shape: decad.SlotHole{
			Length: units.Millimeters(16),
			Width:  units.Millimeters(6),
			Angle:  units.Degrees(0),
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
	// The plate held 12000 mm^3. The slot's area is a 10 x 6 strip plus a
	// circle of radius 3, and it removes that area over 10 mm.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 11117 mm^3
}
