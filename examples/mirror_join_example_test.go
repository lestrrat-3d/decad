package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// WithJoin makes Mirrored return one body: the original and its mirror image
// merged. Use it to model a symmetric part as a single half and mirror it
// into the whole, so that changing the half changes both sides.
func Example_mirror_join() {
	// Model only half of the part. The profile's left edge lies on the plane
	// x = 0, which will be the mirror plane, so the two halves meet exactly there.
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
	half, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Mirror across the half's -X wall and join the two sides into one body.
	wall := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))
	whole, err := decad.Mirrored(context.Background(), half,
		decad.MirrorFace{Body: half, Face: wall}, decad.WithJoin())
	if err != nil {
		fmt.Printf("failed to mirror: %s\n", err)
		return
	}
	bounds, err := whole.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	fmt.Printf("spans x: %v .. %v\n", bounds.Min.X, bounds.Max.X)
	fmt.Printf("pieces: %d, live bodies: %d\n", len(whole.Lumps()), len(doc.Bodies()))
	// Output:
	// spans x: -20 .. 20
	// pieces: 1, live bodies: 1
}
