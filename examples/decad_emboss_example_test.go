package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Emboss raises or engraves closed profiles on a planar face. Here a 40 x 30 x
// 10 plate takes a raised label on its top face, then an engraved mark on its
// bottom face. Each sketch lies in the plane of its face, and the direction of
// each relief comes from the face, not from the sketch.
func Example_decad_emboss() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()
	base, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := base.CreateRectangle(0, 0, 40, 30)
	base.Fix(rect.A)
	if _, err := base.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	plate, err := decad.Extrude(ctx, doc, base, base.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}
	// Facing selects a planar face by its outward normal. Each of these faces is
	// the only one of its direction on the body it is read from.
	top := decad.Faces(decad.Facing(r3.Vec{Z: 1}))
	bottom := decad.Faces(decad.Facing(r3.Vec{Z: -1}))

	// The sketch for the reliefs lies in the plane of the top face, z = 10.
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	marks, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	label := marks.CreateRectangle(5, 5, 20, 15)
	marks.Fix(label.A)
	if _, err := marks.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	raised, err := decad.Emboss(ctx, plate, marks, marks.Profiles(), top, decad.EmbossRaised, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to raise label: %s\n", err)
		return
	}

	// A second sketch, in the plane of the bottom face, holds the engraved mark.
	cuts, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	mark := cuts.CreateRectangle(25, 5, 35, 25)
	cuts.Fix(mark.A)
	if _, err := cuts.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	part, err := decad.Emboss(ctx, raised, cuts, cuts.Profiles(), bottom, decad.EmbossEngraved, units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to engrave mark: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure part: %s\n", err)
		return
	}
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 12100.0 mm^3
}
