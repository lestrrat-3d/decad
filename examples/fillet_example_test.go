package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Fillet rounds edges. Use it to remove sharp corners that concentrate stress
// or cut hands. Here the four vertical corners of a box are rounded. Edges are
// chosen with a query, so the call says which edges it means (here: those
// parallel to Z) instead of pointing at them by index.
func Example_fillet() {
	// A 100 x 60 mm plate, extruded 20 mm.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	doc := decad.New()
	box, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Round the four vertical edges with a 10 mm radius. Each rounded corner
	// becomes a cylindrical face, so the box gains four faces.
	rounded, err := decad.Fillet(context.Background(), box,
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(10))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	volume, err := rounded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d\n", len(rounded.Faces()))
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 10
	// volume: 118283.2 mm^3
}
