package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A recorded prism sheet can grow inward to form a solid wall.
func Example_body_thicken_prism() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	if err != nil {
		fmt.Println(err)
		return
	}
	solid, err := sheet.Thicken(context.Background(), units.Millimeters(5),
		decad.WithThickenSide(decad.ThickenNegative))
	if err != nil {
		fmt.Println(err)
		return
	}
	volume, err := solid.Volume()
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Println(solid.Kind() == decad.BodySolid, len(solid.Faces()))
	fmt.Println(volume.Value)
	// Output:
	// true 10
	// 15000 mm^3
}
