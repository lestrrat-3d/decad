package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Cut removes the tool from the target. This placed tool takes the mesh path,
// which stitches a watertight faceted boundary and publishes a proven volume
// bound. A same-plane blind tool is shown in Example_decad_blind_cut.
func Example_decad_cut() {
	w := sketch.NewWorld()
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := plateSketch.CreateRectangle(0, 0, 20, 20)
	plateSketch.Fix(rect.A)
	if _, err := plateSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	holeSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	center := holeSketch.CreatePoint(14, 6)
	holeSketch.Fix(center)
	holeSketch.CreateCircle(center, 2)
	if _, err := holeSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	plate, err := doc.Extrude(plateSketch, plateSketch.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}
	pin, err := doc.Extrude(holeSketch, holeSketch.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}
	// Drop this placed tool so it pierces both plate faces, keeping its caps
	// clear of the mesh path's face-on-face contact case.
	down, err := r3.Translation(r3.Vec{Z: -6})
	if err != nil {
		fmt.Printf("failed to build translation: %s\n", err)
		return
	}
	tool, err := pin.Placed(context.Background(), down)
	if err != nil {
		fmt.Printf("failed to place tool: %s\n", err)
		return
	}

	drilled, err := decad.Cut(context.Background(), plate, tool)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}

	vol, err := drilled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	volMM, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	boundMM, err := vol.Bound.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert bound: %s\n", err)
		return
	}

	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	// The analytic answer is 3200 − π·2²·8 ≈ 3099.47 mm³; the held mesh's
	// exact integral lands inside the proven bound of it.
	fmt.Printf("faces: %d, lumps: %d\n", len(drilled.Faces()), len(drilled.Lumps()))
	fmt.Printf("volume: %.2f mm^3 (%s, bound %.3f mm^3)\n", volMM, vol.Exactness, boundMM)
	fmt.Printf("status: %s, trustworthy: %v\n", report.Status, report.Passed())
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// faces: 7, lumps: 1
	// volume: 3099.51 mm^3 (Approximate, bound 0.115 mm^3)
	// status: Sound, trustworthy: true
	// live bodies: 1
}
