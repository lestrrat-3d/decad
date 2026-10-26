package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Boss raises a round post on a flat face, with a blind hole down its middle
// for a screw to bite into. Use it for the screw posts of a housing instead of
// extruding, joining and drilling by hand.
func Example_boss() {
	// Make a 40 x 40 x 5 mm plate: sketch a rectangle and extrude it (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// An 8 mm post, 8 mm tall, in the middle of the top face (the extrude's end
	// cap, at z = 5), with a 3 mm pilot hole 6 mm deep from the post's top.
	boss, err := decad.Boss(context.Background(), plate, decad.BossSpec{
		Face:     decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate))),
		At:       r3.Vec{X: 20, Y: 20, Z: 5},
		Diameter: units.Millimeters(8),
		Height:   units.Millimeters(8),
		Bore:     &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(6)},
	})
	if err != nil {
		fmt.Printf("failed to raise the boss: %s\n", err)
		return
	}
	volume, err := boss.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	// The plate holds 8000 mm^3. The post adds 16π x 8 and the pilot removes
	// 2.25π x 6.
	fmt.Printf("volume: %.0f mm^3\n", volume.Value.Base())
	// Output:
	// volume: 8360 mm^3
}
