package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A rounded outline (straight edges and arcs that meet tangentially on one
// face) is filleted as one chain. Here a box with rounded corners is cross
// drilled by a tool that Transform moved, which makes the result a mesh
// Boolean result; WithTangentChain expands the one named edge to the whole
// outline of the top face.
func Example_fillet_rounded_outline() {
	ctx := context.Background()
	w := sketch.NewWorld()
	doc := decad.New()

	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	rounded, err := decad.Fillet(ctx, box, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4),
		units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to round the corners: %s\n", err)
		return
	}

	// A cylinder along y through (20, ·, 10), moved into place by a translation.
	ts, err := w.CreateSketch(w.XZ())
	if err != nil {
		fmt.Printf("failed to create tool sketch: %s\n", err)
		return
	}
	c := ts.CreatePoint(20, 10)
	ts.Fix(c)
	ts.CreateCircle(c, 4)
	if _, err := ts.Solve(ctx); err != nil {
		fmt.Printf("failed to solve tool sketch: %s\n", err)
		return
	}
	cyl, err := decad.Extrude(ctx, doc, ts, ts.Profiles()[0], decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude tool: %s\n", err)
		return
	}
	move, err := r3.Translation(r3.Vec{Y: 30})
	if err != nil {
		fmt.Printf("failed to build the move: %s\n", err)
		return
	}
	tool, err := decad.Transform(ctx, cyl, move)
	if err != nil {
		fmt.Printf("failed to move the tool: %s\n", err)
		return
	}
	drilled, err := decad.Cut(ctx, rounded, tool)
	if err != nil {
		fmt.Printf("failed to drill: %s\n", err)
		return
	}
	before, err := drilled.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}

	// The edge from (3, 0, 20) to (37, 0, 20) is one of eight in the outline.
	edge := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(3, 0, 20))).Exactly(1)
	filleted, err := decad.Fillet(ctx, drilled, edge, units.Millimeters(1), decad.WithTangentChain())
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	after, err := filleted.Volume()
	if err != nil {
		fmt.Printf("failed to measure: %s\n", err)
		return
	}
	cylinders, tori := 0, 0
	for _, f := range filleted.Faces() {
		switch f.Surface().(type) {
		case decad.Cylinder:
			cylinders++
		case decad.Torus:
			tori++
		}
	}
	fmt.Printf("cylinders: %d, tori: %d\n", cylinders, tori)
	fmt.Printf("removed: %.0f mm^3\n", before.Value.Mag()-after.Value.Mag())
	// Output:
	// cylinders: 9, tori: 4
	// removed: 24 mm^3
}
