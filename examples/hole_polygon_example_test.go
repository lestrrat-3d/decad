package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A PolygonHole cuts a regular polygon of any 3 to 64 sides, sized across its
// flats: a hexagon for a nut that must not turn, or a square or an octagon for a
// plug. A flat faces the face frame's U axis until Angle turns it. Here a
// hexagon 10 mm across flats, deep enough for an M6 nut, is cut 5 mm into a plate.
func Example_hole_polygon() {
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
		Shape: decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(10), Angle: units.Degrees(0)},
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
	// A hexagon 10 mm across flats has an area of 86.6 mm^2, so the pocket removes 433 mm^3.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 11567 mm^3
}
