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

// A hole can carry its counterbore on a cylinder too. The recess is cut from the plane
// tangent to the rod at the hole's centre, so on a rod it is shallower where the wall
// falls away. Here a 10 mm radius, 16 mm long rod takes a 3 mm hole 6 mm deep with a
// 6 mm counterbore 2 mm deep.
func Example_hole_counterbore_on_a_cylinder() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(16), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	drilled, err := decad.Hole(context.Background(), rod, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.Vec{X: 10, Y: 0, Z: 8},
		Diameter: units.Millimeters(3),
		Depth:    decad.BlindHole{Depth: units.Millimeters(6)},
		Head:     decad.Counterbore{Diameter: units.Millimeters(6), Depth: units.Millimeters(2)},
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
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 4945 mm^3
}

// A threaded hole can start in a bore's wall too, for a screw that goes into a
// boss from the inside. The thread starts at the plane tangent to the wall at At.
// Here a 12 mm bore through a 40 x 40 x 10 block takes an M5 hole, 8 mm deep.
func Example_hole_thread_on_a_cylinder() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-20, -20, 20, 20)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(0, 0), 6)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var block *decad.Body
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			block, err = decad.Extrude(context.Background(), decad.New(), s, p,
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			if err != nil {
				fmt.Printf("failed to extrude: %s\n", err)
				return
			}
		}
	}
	tapped, err := decad.Hole(context.Background(), block, decad.HoleSpec{
		Face:     decad.Faces(decad.Cylindrical()),
		At:       r3.Vec{X: 6, Y: 0, Z: 5},
		Diameter: units.Millimeters(4.2),
		Depth:    decad.BlindHole{Depth: units.Millimeters(8)},
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
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 14730 mm^3
}
