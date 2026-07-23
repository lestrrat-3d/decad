package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Chamfer bevels the convex lateral edges of a straight prism. Like the fillet,
// the reduction is a rewrite of the recorded 2D section — but each corner is cut
// off by a straight chord between the two setback feet, so the bevel walls are
// planes, not cylinders, and irrational lengths carry proven bounds. The step records the
// unresolved edge query and setback distance, retiring the receiver, so the
// recipe replays deterministically.
func Example_decad_chamfer() {
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

	// Bevel all four lateral edges (straight, parallel to the z sweep) with a
	// 10 mm setback.
	body, err := box.Chamfer(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(10))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
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

	bevels := 0
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamfer(") {
				bevels++
			}
		}
	}

	fmt.Printf("solid: %v, faces: %d\n", body.IsSolid(), len(body.Faces()))
	fmt.Printf("bevel planes: %d\n", bevels)
	fmt.Printf("volume: %.3f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("recipe steps: %d\n", len(doc.Recipe().Steps))
	// Output:
	// solid: true, faces: 10
	// bevel planes: 4
	// volume: 116000.000 mm^3 (Exact)
	// recipe steps: 2
}
