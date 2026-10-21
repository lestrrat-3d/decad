package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Example_cone_from_face builds the cone that extends one conical side
// of a revolved blank, moved off the blank by one float step.
func Example_cone_from_face() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	corners := [][2]float64{{1, 0}, {1, 6}, {11, 2}, {11, 0}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		fmt.Println(err)
		return
	}
	doc := decad.New()
	blank, err := decad.Revolve(ctx, doc, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	if err != nil {
		fmt.Println(err)
		return
	}
	var side decad.FeatureRef
	for _, face := range blank.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			side = face.Origins()[0]
		}
	}
	cone, err := decad.ConeFromFace(ctx, blank,
		decad.Faces(decad.FaceCreatedBy(side)).Exactly(1), units.Millimeters(-5))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("solid %t; bodies %d; blank solid %t\n", cone.IsSolid(), len(doc.Bodies()), blank.IsSolid())
	// Output: solid true; bodies 2; blank solid true
}
