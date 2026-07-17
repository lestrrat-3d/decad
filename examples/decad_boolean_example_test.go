package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Cut removes the tool from the target — an explicit boolean, never folded
// into a feature. The result is a Faceted body: the operation runs on the
// operands' tessellations with exact predicates, so the stitched boundary is
// watertight by construction, and every measurement says how far it can be
// trusted — the volume reads Approximate with a PROVEN error bound, and
// Verify accepts that bound when it is within the requested tolerance.
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
	// Drop the tool so it pierces both plate faces; a tool whose cap merely
	// rests ON a plate face is a face-on-face contact the boolean rejects.
	down, err := r3.Translation(r3.Vec{Z: -6})
	if err != nil {
		fmt.Printf("failed to build translation: %s\n", err)
		return
	}
	tool, err := pin.Placed(down)
	if err != nil {
		fmt.Printf("failed to place tool: %s\n", err)
		return
	}

	drilled, err := decad.Cut(plate, tool)
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
	fmt.Printf("status: %s, trustworthy: %v\n", report.Status, report.Trustworthy())
	fmt.Printf("recipe steps: %d\n", len(doc.Recipe().Steps))
	// Output:
	// faces: 7, lumps: 1
	// volume: 3099.51 mm^3 (Approximate, bound 0.189 mm^3)
	// status: Sound, trustworthy: true
	// recipe steps: 4
}
