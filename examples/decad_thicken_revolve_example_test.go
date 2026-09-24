package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A revolve sheet grows into a hollow ring: the meridian offsets in its own
// plane, and the offset annulus spins through the sheet's own interval. The
// eroded meridian sweeps a cavity, so the one lump carries a second, void
// shell beside its outer one.
func Example_body_thicken_revolve() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	// The meridian rectangle sits 10 mm out from the axis and is 10 mm tall.
	rect := s.CreateRectangle(10, 0, 20, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	sheet, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Println(err)
		return
	}
	solid, err := sheet.Thicken(context.Background(), units.Millimeters(2),
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
	voids := 0
	for _, shell := range solid.Shells() {
		if shell.IsVoid() {
			voids++
		}
	}
	fmt.Println(solid.Kind() == decad.BodySolid, len(solid.Faces()), voids)
	// Pappus: 2*pi*(1500 - 540) mm^3, the two meridian rectangles' own first
	// moments about the axis.
	fmt.Printf("%.4f mm^3\n", volume.Value.Base())
	// Output:
	// true 8 1
	// 6031.8579 mm^3
}
