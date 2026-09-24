package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A recorded planar patch grows into a solid along its positive normal.
func Example_body_thicken_patch() {
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
	patch, err := doc.Patch(context.Background(), s, s.Profiles()[0])
	if err != nil {
		fmt.Println(err)
		return
	}
	solid, err := patch.Thicken(context.Background(), units.Millimeters(2))
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
	// true 6
	// 12000 mm^3
}
