package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Hole drills a round hole into a flat face at a point you choose. Use it for
// bolt holes and screw holes instead of sketching and cutting a cylinder
// yourself. The spec states the hole's diameter, how deep it goes, and an
// optional head: a counterbore (a flat-bottomed recess for a bolt head) or a
// countersink (a cone for a flat-head screw).
func Example_hole() {
	// Make the plate to drill: sketch a rectangle and extrude it (see
	// Example_extrude for these steps).
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

	// Drill a 6 mm hole all the way through, from the middle of the top face
	// (the extrude's end cap, at z = 10). The head is a 12 mm wide
	// counterbore, 4 mm deep, so a bolt head can sit flush.
	drilled, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 15, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.ThroughHole{},
		Head:     decad.Counterbore{Diameter: units.Millimeters(12), Depth: units.Millimeters(4)},
	})
	if err != nil {
		fmt.Printf("failed to drill: %s\n", err)
		return
	}
	volume, err := drilled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate held 12000 mm^3. The hole and its recess remove the rest.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 11378 mm^3
}

// A HoleSpec's Draft leans the bore's wall, as a cast or moulded hole is made so
// that its core pulls out. A positive angle narrows the bore toward its bottom.
// Here a 6 mm hole 6 mm deep, drafted 3 degrees, is about 5.37 mm across at its floor.
func Example_hole_draft() {
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
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	drilled, err := decad.Hole(context.Background(), plate, decad.HoleSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.BlindHole{Depth: units.Millimeters(6)},
		Draft:    units.Degrees(3),
	})
	if err != nil {
		fmt.Printf("failed to drill: %s\n", err)
		return
	}
	volume, err := drilled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 4000 mm^3 and the bore is a frustum of radii 3 and 2.69 mm.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 3848 mm^3
}
