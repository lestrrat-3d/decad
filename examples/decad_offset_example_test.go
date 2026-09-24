package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A sheet and its offset are two live bodies of one document: Offset depends on
// its source and consumes nothing, so the pair can be measured against each
// other or stitched afterwards. Offsetting a rectangular prism sheet outward
// rounds each convex corner with an arc of the offset distance.
func Example_body_offset() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create the sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve the sketch: %s\n", err)
		return
	}
	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude the sheet: %s\n", err)
		return
	}
	grown, err := sheet.Offset(context.Background(), units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to offset the sheet: %s\n", err)
		return
	}
	box, err := grown.Bounds()
	if err != nil {
		fmt.Printf("failed to read the bounds: %s\n", err)
		return
	}
	// Four straight walls plus one corner arc each, and the source is still
	// live beside its offset.
	fmt.Println(grown.Kind() == decad.BodySheet, len(grown.Faces()))
	fmt.Println(box.Min, box.Max)
	fmt.Println(len(doc.Bodies()))
	// Output:
	// true 8
	// {-3 -3 0} {103 63 10}
	// 2
}
