package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// An OutlineHole cuts any simple polygon, given by its corners about the hole axis in
// the face frame: a keyed or L-shaped bore, say. Here an L of 48 mm^2 is cut 5 mm
// into a plate.
func Example_hole_outline() {
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
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	cut, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:  decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:    r3.Vec{X: 20, Y: 15, Z: 10},
		Depth: decad.BlindHole{Depth: units.Millimeters(5)},
		Shape: decad.OutlineHole{Corners: []decad.Point2{
			{U: -4, V: -4}, {U: 4, V: -4}, {U: 4, V: 0}, {U: 0, V: 0}, {U: 0, V: 4}, {U: -4, V: 4},
		}},
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
	// The L has an area of 48 mm^2, so the pocket removes 240 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 11760 mm^3
}
