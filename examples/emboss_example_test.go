package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Emboss raises a shape out of a flat face, or engraves it into the face. Use
// it for labels, logos and marks. You draw closed shapes in a sketch that lies
// in the plane of the face; Emboss reads the direction from the face, so a
// raised shape always stands out of the body and an engraved one always cuts
// into it.
func Example_emboss() {
	// Make the plate to mark: sketch a rectangle and extrude it (see
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

	// Raise a label on the top face. Draw it in a sketch on the plane z = 10,
	// where the top face lies, and pick the face by the way it points.
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	label, err := w.CreateSketch(topPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	labelRect := label.CreateRectangle(5, 5, 20, 15)
	label.Fix(labelRect.A)
	if _, err := label.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	raised, err := decad.Emboss(context.Background(), plate, label, label.Profiles(),
		decad.Faces(decad.Facing(r3.Vec{Z: 1})), decad.EmbossRaised, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to raise label: %s\n", err)
		return
	}

	// Engrave a mark 1 mm deep into the bottom face, which lies in the plane
	// z = 0. The same call engraves, with EmbossEngraved in place of EmbossRaised.
	mark, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	markRect := mark.CreateRectangle(25, 5, 35, 25)
	mark.Fix(markRect.A)
	if _, err := mark.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	part, err := decad.Emboss(context.Background(), raised, mark, mark.Profiles(),
		decad.Faces(decad.Facing(r3.Vec{Z: -1})), decad.EmbossEngraved, units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to engrave mark: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate held 12000 mm^3. The label adds 300 and the mark removes 200.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 12100 mm^3
}

// WithEmbossDraft leans the relief's walls so a moulded or cast part releases
// from its tool. A positive angle makes a raised relief narrower toward its top.
// Here a 10 mm square label, raised 3 mm with a 5 degree draft, is 10 mm wide on
// the plate and about 9.5 mm wide at its top.
func Example_emboss_draft() {
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
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	label, err := w.CreateSketch(topPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	labelRect := label.CreateRectangle(10, 10, 20, 20)
	label.Fix(labelRect.A)
	if _, err := label.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	raised, err := decad.Emboss(context.Background(), plate, label, label.Profiles(),
		decad.Faces(decad.Facing(r3.Vec{Z: 1})), decad.EmbossRaised, units.Millimeters(3),
		decad.WithEmbossDraft(units.Degrees(5)))
	if err != nil {
		fmt.Printf("failed to emboss: %s\n", err)
		return
	}
	volume, err := raised.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 12000 mm^3. A straight label would add 300 mm^3; the draft
	// leaves about 285.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 12285 mm^3
}

// WithEmbossRimFillet rounds the rim of a relief, so a raised badge or a nameplate
// has no sharp top edge to catch on. Here a round 8 mm badge 2 mm tall gets a
// 0.5 mm round on its top rim.
func Example_emboss_rim_fillet() {
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
	topPlane, err := w.CreateOffsetPlane(w.XY(), 10)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	badge, err := w.CreateSketch(topPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := badge.CreatePoint(20, 15)
	badge.Fix(centre)
	badge.CreateCircle(centre, 4)
	if _, err := badge.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	raised, err := decad.Emboss(context.Background(), plate, badge, badge.Profiles(),
		decad.Faces(decad.Facing(r3.Vec{Z: 1})), decad.EmbossRaised, units.Millimeters(2),
		decad.WithEmbossRimFillet(units.Millimeters(0.5)))
	if err != nil {
		fmt.Printf("failed to emboss: %s\n", err)
		return
	}
	volume, err := raised.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 12000 mm^3, the badge adds 100.5, and the round takes off about 1.8.
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 12099.2 mm^3
}
