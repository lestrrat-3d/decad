package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A blind pocket makes a stacked body. Its outer top loop accepts different
// setbacks across the top cap and down the surrounding walls.
func Example_decad_asymmetric_stacked_chamfer() {
	w := sketch.NewWorld()
	doc := decad.New()
	extrudeRectangle := func(x0, y0, x1, y1, z, height float64) (*decad.Body, error) {
		plane, err := w.CreateOffsetPlane(w.XY(), z)
		if err != nil {
			return nil, err
		}
		s, err := w.CreateSketch(plane)
		if err != nil {
			return nil, err
		}
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return nil, err
		}
		return doc.Extrude(s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	}
	plate, err := extrudeRectangle(0, 0, 40, 40, 0, 10)
	if err != nil {
		fmt.Println(err)
		return
	}
	tool, err := extrudeRectangle(10, 15, 30, 25, 5, 5)
	if err != nil {
		fmt.Println(err)
		return
	}
	pocket, err := decad.Cut(context.Background(), plate, tool)
	if err != nil {
		fmt.Println(err)
		return
	}
	tops, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pocket))).Exactly(1).SelectFaces(pocket)
	if err != nil {
		fmt.Println(err)
		return
	}
	top := tops[0]
	loop := top.Loops()[0]
	var edges *decad.EdgeQuery
	for _, edge := range loop.Edges() {
		a, b := edge.Start().Position().Value, edge.End().Position().Value
		predicates := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(b), decad.ParallelTo(b.Sub(a))}
		if edges == nil {
			edges = decad.Edges(predicates...)
		} else {
			edges.Or(predicates...)
		}
	}
	beveled, err := pocket.Chamfer(context.Background(), edges.Exactly(len(loop.Edges())), units.Millimeters(1.5),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(top.Origins()[0])), units.Millimeters(2)))
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := beveled.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("volume: %.0f mm^3\n", mm3)
	// Output:
	// volume: 14766 mm^3
}
