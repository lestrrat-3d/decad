package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A through hole followed by a wider blind cut leaves an analytic shoulder.
func Example_decad_counterbore() {
	w := sketch.NewWorld()
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create plate sketch: %s\n", err)
		return
	}
	rect := plateSketch.CreateRectangle(-10, -10, 10, 10)
	plateSketch.Fix(rect.A)
	if _, err := plateSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve plate: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := doc.Extrude(plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}

	throughSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create through-hole sketch: %s\n", err)
		return
	}
	center := throughSketch.CreatePoint(0, 0)
	throughSketch.Fix(center)
	throughSketch.CreateCircle(center, 2)
	if _, err := throughSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve through hole: %s\n", err)
		return
	}
	throughTool, err := doc.Extrude(throughSketch, throughSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude through hole: %s\n", err)
		return
	}
	drilled, err := decad.Cut(context.Background(), plate, throughTool)
	if err != nil {
		fmt.Printf("failed to drill plate: %s\n", err)
		return
	}

	shoulderPlane, err := w.CreateOffsetPlane(w.XY(), 7)
	if err != nil {
		fmt.Printf("failed to create counterbore plane: %s\n", err)
		return
	}
	shoulderSketch, err := w.CreateSketch(shoulderPlane)
	if err != nil {
		fmt.Printf("failed to create counterbore sketch: %s\n", err)
		return
	}
	shoulderCenter := shoulderSketch.CreatePoint(0, 0)
	shoulderSketch.Fix(shoulderCenter)
	shoulderSketch.CreateCircle(shoulderCenter, 4)
	if _, err := shoulderSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve counterbore: %s\n", err)
		return
	}
	shoulderTool, err := doc.Extrude(shoulderSketch, shoulderSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude counterbore: %s\n", err)
		return
	}
	part, err := decad.Cut(context.Background(), drilled, shoulderTool)
	if err != nil {
		fmt.Printf("failed to cut counterbore: %s\n", err)
		return
	}
	volume, err := part.Volume()
	if err != nil {
		fmt.Printf("failed to measure part: %s\n", err)
		return
	}
	mesh, err := part.Tessellate(context.Background(), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to mesh part: %s\n", err)
		return
	}
	fmt.Printf("volume: %.3f mm^3\n", volume.Value.Base())
	fmt.Printf("faces: %d\n", len(part.Faces()))
	fmt.Printf("verified mesh: %t\n", mesh.VolumeVerified())
	// Output:
	// volume: 3761.239 mm^3
	// faces: 9
	// verified mesh: true
}
