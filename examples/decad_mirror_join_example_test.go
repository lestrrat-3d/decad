package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A symmetric part is modelled as one half ending on the mirror plane, then
// joined with its own image. WithJoin rewrites the half's section instead of
// running a boolean, so the joined L below is one exact T-shaped prism that
// later modify operations accept.
func Example_decad_mirror_join() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	// Half of a T: its wall x = 0 lies on the mirror plane.
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
	half, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	wall := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))
	tee, err := half.Mirrored(ctx, decad.MirrorFace{Body: half, Face: wall}, decad.WithJoin())
	if err != nil {
		fmt.Printf("failed to join: %s\n", err)
		return
	}
	volume, err := tee.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := volume.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	box, err := tee.Bounds()
	if err != nil {
		fmt.Printf("failed to measure bounds: %s\n", err)
		return
	}
	// The joined body fillets like any drawn prism.
	rounded, err := tee.Fillet(ctx, decad.Edges(decad.Convex(), decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3 (%s)\n", mm3, volume.Exactness)
	fmt.Printf("x range: [%g, %g], lumps: %d, faces: %d\n", box.Min.X, box.Max.X, len(tee.Lumps()), len(tee.Faces()))
	fmt.Printf("filleted faces: %d, live bodies: %d\n", len(rounded.Faces()), len(doc.Bodies()))
	// Output:
	// volume: 3500 mm^3 (Exact)
	// x range: [-20, 20], lumps: 1, faces: 10
	// filleted faces: 16, live bodies: 1
}
