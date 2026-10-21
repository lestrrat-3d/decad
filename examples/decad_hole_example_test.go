package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Hole drills into a planar face at a world point. Here a counterbored
// through hole goes into the top of a 40 x 30 x 10 plate, so the volume is the
// plate minus the bore and minus the wider recess around its mouth.
func Example_decad_hole() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}

	// The extrude's end cap is the plate's top face, at z = 10.
	part, err := decad.Hole(ctx, plate, decad.HoleSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 15, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.ThroughHole{},
		Head:     decad.Counterbore{Diameter: units.Millimeters(12), Depth: units.Millimeters(4)},
	})
	if err != nil {
		fmt.Printf("failed to drill plate: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure part: %s\n", err)
		return
	}
	fmt.Printf("volume: %.3f mm^3 (%s)\n", volume.Value.Base(), volume.Exactness)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// volume: 11377.965 mm^3 (Approximate)
	// live bodies: 1
}
