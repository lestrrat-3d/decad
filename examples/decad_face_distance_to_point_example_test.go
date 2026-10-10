package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_decad_face_distance_to_point() {
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
	box, err := decad.New().Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Println(err)
		return
	}
	faces, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(1).SelectFaces(box)
	if err != nil {
		fmt.Println(err)
		return
	}
	distance, err := faces[0].DistanceToPoint(context.Background(),
		r3.NewVec(5, 5, 5), units.Millimeters(0.1))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("%.1f mm, %s, bound %.1f mm\n",
		distance.Value.Base(), distance.Exactness, distance.Bound.Base())
	// Output:
	// 5.0 mm, Exact, bound 0.0 mm
}
