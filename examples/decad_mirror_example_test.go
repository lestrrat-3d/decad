package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A body is mirrored across a plane named by one of its own planar faces.
// MirroredCopy keeps the source live; Mirrored would retire it, as Placed
// does. The face selector must name exactly one face: the L below has two
// walls facing +x, so naming "the +x wall" is a cardinality error, while its
// single -x wall names the plane x = 0.
func Example_decad_mirror() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	// An L: a 5 mm upright along x = 0 and a 5 mm foot along y = 0.
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	l, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	plusX := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))
	_, err = l.MirroredCopy(ctx, decad.MirrorFace{Body: l, Face: plusX})
	fmt.Printf("mirror across the +x wall: cardinality error: %t\n", errors.Is(err, decad.ErrCardinality))

	minusX := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))
	image, err := l.MirroredCopy(ctx, decad.MirrorFace{Body: l, Face: minusX})
	if err != nil {
		fmt.Printf("failed to mirror: %s\n", err)
		return
	}
	volume, err := image.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	box, err := image.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	src, err := l.Centroid()
	if err != nil {
		fmt.Printf("failed to measure the source centroid: %s\n", err)
		return
	}
	mirrored, err := image.Centroid()
	if err != nil {
		fmt.Printf("failed to measure the image centroid: %s\n", err)
		return
	}
	fmt.Printf("image volume: %.0f mm^3 (%s)\n", mm3, volume.Exactness)
	fmt.Printf("image x range: [%g, %g]\n", box.Min.X, box.Max.X)
	fmt.Printf("centroid x: source %.4f, image %.4f\n", src.Value.X, mirrored.Value.X)
	fmt.Printf("live bodies: %d\n", len(doc.Bodies()))
	// Output:
	// mirror across the +x wall: cardinality error: true
	// image volume: 1750 mm^3 (Exact)
	// image x range: [-20, 0]
	// centroid x: source 6.7857, image -6.7857
	// live bodies: 2
}
