package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A fitted-spline meridian stays a free-form surface after Revolve. The
// profile must have proven clearance from the axis, and VerifyAll certifies
// the chorded boundary and occupied-volume difference.
func Example_decad_revolveFitSpline() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Println(err)
		return
	}
	start := s.CreatePoint(-1, -1)
	middle := s.CreatePoint(0, -1.25)
	end := s.CreatePoint(1, -1)
	if _, err := s.CreateFitSpline(start, middle, end); err != nil {
		fmt.Println(err)
		return
	}
	rightTop := s.CreatePoint(1, 1)
	leftTop := s.CreatePoint(-1, 1)
	s.CreateLine(end, rightTop)
	s.CreateLine(rightTop, leftTop)
	s.CreateLine(leftTop, start)

	axis := decad.SketchLine{
		Start: decad.Point2{U: -5, V: -5},
		End:   decad.Point2{U: -5, V: 5},
	}
	body, err := decad.New().Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Println(err)
		return
	}
	freeformWall := false
	for _, face := range body.Faces() {
		freeformWall = freeformWall || face.Surface().Kind() == decad.KindNURBS
	}
	mesh, err := body.Tessellate(context.Background(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Printf("free-form wall: %t\n", freeformWall)
	fmt.Printf("boundary verified: %t\n", mesh.BoundaryVerified())
	fmt.Printf("volume verified: %t\n", mesh.VolumeVerified())
	// Output:
	// free-form wall: true
	// boundary verified: true
	// volume verified: true
}
