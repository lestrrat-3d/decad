package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Extrude converts the sketch profile to structural records and evaluates the
// prism it defines: an analytic B-rep body whose mass properties are closed-form.
// Every measurement carries its exactness — the evaluator never hands back a
// number it cannot vouch for.
func Example_decad_extrude() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 10 mm along the sketch normal.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	area, err := body.Area()
	if err != nil {
		fmt.Printf("failed to measure area: %s\n", err)
		return
	}
	c, err := body.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}

	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("volume: %s (%s)\n", vol.Value, vol.Exactness)
	fmt.Printf("area: %s\n", area.Value)
	fmt.Printf("centroid: %v\n", c.Value)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// solid: true, faces: 6
	// volume: 60000 mm^3 (Exact)
	// area: 15200 mm^2
	// centroid: {50 30 5}
	// live bodies: 1
}

// Placed applies a rigid motion to a body: the moved body is a new body with
// fresh provenance, and the original is retired —
// it stays readable, but takes no further operations.
func Example_decad_placed() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	motion, err := r3.Translation(r3.NewVec(200, 0, 25))
	if err != nil {
		fmt.Printf("failed to build motion: %s\n", err)
		return
	}
	placed, err := body.Placed(context.Background(), motion)
	if err != nil {
		fmt.Printf("failed to place: %s\n", err)
		return
	}

	c, err := placed.Centroid()
	if err != nil {
		fmt.Printf("failed to measure centroid: %s\n", err)
		return
	}
	bounds, err := placed.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}

	fmt.Printf("centroid: %v\n", c.Value)
	fmt.Printf("bounds: %v .. %v\n", bounds.Min, bounds.Max)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))

	// The original was consumed by the placement.
	if _, err := body.Placed(context.Background(), motion); err != nil {
		fmt.Printf("moving the original again: %s\n", err)
	}
	// Output:
	// centroid: {250 30 30}
	// bounds: {200 0 25} .. {300 60 35}
	// live bodies: 1
	// moving the original again: decad: body has been retired from its document
}
