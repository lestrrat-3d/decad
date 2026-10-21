package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ConeFromFace builds the full cone that a conical face of a revolved body
// belongs to, extended to a chosen plane. Use it when you need the whole cone
// rather than the truncated piece, for example as a tool to trim or cut other
// bodies along the same taper. The original body is kept.
func Example_cone_from_face() {
	// A revolved part with a conical side: draw a tapered outline and spin
	// it about the X axis. The slanted edge from (1, 6) to (11, 2) becomes
	// the cone.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
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
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	part, err := decad.Revolve(context.Background(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// Find the conical face, and remember where it came from so a query can
	// select it.
	var side decad.FeatureRef
	for _, face := range part.Faces() {
		if _, ok := face.Surface().(decad.Cone); ok {
			side = face.Origins()[0]
		}
	}

	// Build the full cone from that face. The last argument is the far plane,
	// as an X coordinate below the origin, where the cone stops.
	cone, err := decad.ConeFromFace(context.Background(), part,
		decad.Faces(decad.FaceCreatedBy(side)).Exactly(1), units.Millimeters(-5))
	if err != nil {
		fmt.Printf("failed to build cone: %s\n", err)
		return
	}
	bounds, err := cone.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("cone starts at x = %v\n", bounds.Min.X)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// cone starts at x = -5
	// live bodies: 2
}
