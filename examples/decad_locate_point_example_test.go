package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_locate_point() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Println(err)
		return
	}
	for _, point := range []r3.Vec{
		r3.NewVec(5, 5, 5), r3.NewVec(12, 5, 5), r3.NewVec(0, 5, 5),
	} {
		location, err := box.LocatePoint(context.Background(), point, units.Millimeters(0.1))
		fmt.Println(location, err)
	}
	// Output:
	// Inside <nil>
	// Outside <nil>
	// OnBoundary <nil>
}
