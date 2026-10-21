package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ToFace extrudes until the sketch reaches a planar face of another body. Use
// it when a feature must meet an existing part, so that moving the stop later
// moves the feature with it instead of leaving a gap.
func Example_extrude_to_face() {
	w := sketch.NewWorld()
	doc := decad.New()

	// The stop is a 60 x 60 x 5 mm plate whose bottom face sits at z = 30.
	stopFrame, err := r3.NewFrame(r3.NewVec(0, 0, 30), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		fmt.Printf("failed to build frame: %s\n", err)
		return
	}
	stopPlane, err := w.CreatePlaneFromFrame(stopFrame)
	if err != nil {
		fmt.Printf("failed to create plane: %s\n", err)
		return
	}
	stopSketch, err := w.CreateSketch(stopPlane)
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	stopRect := stopSketch.CreateRectangle(-30, -30, 30, 30)
	stopSketch.Fix(stopRect.A)
	if _, err := stopSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	stop, err := decad.Extrude(context.Background(), doc, stopSketch, stopSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude stop: %s\n", err)
		return
	}

	// The part is a 20 x 20 mm column that starts at z = 0.
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(-10, -10, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Extrude up to the stop's bottom face (the start cap of its extrude).
	end := decad.ToFace{
		Body: stop,
		Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(stop))),
	}
	column, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0], end)
	if err != nil {
		fmt.Printf("failed to extrude to face: %s\n", err)
		return
	}
	bounds, err := column.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("column spans z: %v .. %v\n", bounds.Min.Z, bounds.Max.Z)
	// Output:
	// column spans z: 0 .. 30
}
