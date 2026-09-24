package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A ribbon from an open sketch curve grows into a solid wall. The section the
// walk sweeps is closed in place: the walk's two copies joined by one cap at
// each free end, with an inserted arc where the walk turns away from the side
// that receives the material.
func Example_body_thicken_ribbon() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	corner := s.CreatePoint(40, 0)
	s.Fix(corner)
	start := s.CreatePoint(0, 0)
	end := s.CreatePoint(40, 30)
	s.CreateLine(start, corner)
	s.CreateLine(corner, end)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	ribbon, err := doc.ExtrudeChain(s, s.Chains()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	if err != nil {
		fmt.Println(err)
		return
	}
	// The inward side mitres the corner, so its 68 mm^2 section is the 70 mm
	// walk times 2 mm less the 2x2 mm square the mitre removes.
	solid, err := ribbon.Thicken(context.Background(), units.Millimeters(2),
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
	fmt.Println(volume.Value, volume.Exactness)
	// Output:
	// true 8
	// 1360 mm^3 Exact
}
