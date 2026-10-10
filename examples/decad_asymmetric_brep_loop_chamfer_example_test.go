package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A cross-drilled plate is a brep. Its complete top loop can take one
// setback across the top face and another down the surrounding walls.
func Example_decad_asymmetric_brep_loop_chamfer() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create plate sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve plate sketch: %s\n", err)
		return
	}
	doc := decad.New()
	plate, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude plate: %s\n", err)
		return
	}
	rounded, err := plate.Fillet(context.Background(),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to round plate: %s\n", err)
		return
	}

	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	if err != nil {
		fmt.Printf("failed to create drill plane: %s\n", err)
		return
	}
	drillSketch, err := w.CreateSketch(plane)
	if err != nil {
		fmt.Printf("failed to create drill sketch: %s\n", err)
		return
	}
	center := drillSketch.CreatePoint(20, 10)
	drillSketch.Fix(center)
	drillSketch.CreateCircle(center, 3)
	if _, err := drillSketch.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve drill sketch: %s\n", err)
		return
	}
	drill, err := doc.Extrude(drillSketch, drillSketch.Profiles()[0],
		decad.Symmetric{D: units.Millimeters(11)})
	if err != nil {
		fmt.Printf("failed to extrude drill: %s\n", err)
		return
	}
	part, err := decad.Cut(context.Background(), rounded, drill)
	if err != nil {
		fmt.Printf("failed to drill plate: %s\n", err)
		return
	}

	tops, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).SelectFaces(part)
	if err != nil || len(tops) != 1 {
		fmt.Printf("failed to select top face: %v, count %d\n", err, len(tops))
		return
	}
	top := tops[0]
	loop := top.Loops()[0]
	var edges *decad.EdgeQuery
	for _, edge := range loop.Edges() {
		a, b := edge.Start().Position().Value, edge.End().Position().Value
		predicates := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(b)}
		if _, straight := edge.Curve().(decad.Line3); straight {
			predicates = append(predicates, decad.ParallelTo(b.Sub(a)))
		} else {
			predicates = append(predicates, decad.Circular())
		}
		if edges == nil {
			edges = decad.Edges(predicates...)
		} else {
			edges.Or(predicates...)
		}
	}
	beveled, err := part.Chamfer(context.Background(), edges.Exactly(len(loop.Edges())),
		units.Millimeters(1.5),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(top.Origins()[0])),
			units.Millimeters(2)))
	if err != nil {
		fmt.Printf("failed to chamfer top loop: %s\n", err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.3f mm^3\n", mm3)
	// Output:
	// volume: 15112.438 mm^3
}
