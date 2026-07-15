package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Fillet rounds the convex lateral edges of a straight prism. The reduction is
// a rewrite of the recorded 2D section — each rounded corner becomes a tangent
// arc — so the blend walls are true cylinders and every measurement stays
// Exact. The step records the unresolved edge query and the radius, retiring
// the receiver, so the recipe replays deterministically.
func Example_decad_fillet() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate, extruded 20 mm — a box with four lateral edges.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Round all four lateral edges (straight, parallel to the z sweep) with a
	// 10 mm radius.
	body, err := box.Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(10))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}

	blends := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			blends++
		}
	}

	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("blend cylinders: %d\n", blends)
	fmt.Printf("volume: %.3f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("recipe steps: %d\n", len(doc.Recipe().Steps))
	// Output:
	// solid: true, faces: 10
	// blend cylinders: 4
	// volume: 118283.185 mm^3 (Exact)
	// recipe steps: 2
}
