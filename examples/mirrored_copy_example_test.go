package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// MirroredCopy reflects a body across a plane and keeps the original. Use it
// for parts that come in left and right versions. The mirror plane is named by
// one of the body's own flat faces, so it follows the part if the part changes.
func Example_mirrored_copy() {
	// An L-shaped profile has a clear left/right asymmetry, so the mirror image
	// is easy to tell apart from the original.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}
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
	l, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The upright's outer wall is the only flat face that faces -X, and it
	// lies in the plane x = 0. Mirror across it.
	wall := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))
	image, err := decad.MirroredCopy(context.Background(), l, decad.MirrorFace{Body: l, Face: wall})
	if err != nil {
		fmt.Printf("failed to mirror: %s\n", err)
		return
	}
	bounds, err := image.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("mirror image spans x: %v .. %v\n", bounds.Min.X, bounds.Max.X)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// mirror image spans x: -20 .. 0
	// live bodies: 2
}
