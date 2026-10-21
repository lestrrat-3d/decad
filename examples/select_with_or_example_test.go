package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Or adds another branch to a selector, so the query matches faces that fit any
// branch. Use it when no single description fits all the faces you want. A
// quarter-turn revolve has two flat end faces in different planes, so one
// branch per end names both.
func Example_select_with_or() {
	// A ring is this cross-section spun about the X axis. The cross-section is
	// 20 mm long and reaches from radius 5 to radius 10.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	corners := [][2]float64{{0, 5}, {20, 5}, {20, 10}, {0, 10}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	ring, err := decad.Revolve(context.Background(), doc, s, s.Profiles()[0], axis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// The body has four flat faces: two at the ends of the turn and two at
	// the ends of the ring. Select only the two at the ends of the turn.
	ends := decad.Faces(decad.FaceCreatedBy(decad.CapStart(ring))).
		Or(decad.FaceCreatedBy(decad.CapEnd(ring))).
		Exactly(2)
	faces, err := ends.SelectFaces(ring)
	if err != nil {
		fmt.Printf("failed to select: %s\n", err)
		return
	}
	fmt.Printf("selected %d faces\n", len(faces))
	// Output:
	// selected 2 faces
}
