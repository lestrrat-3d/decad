package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// PatchEdges fills the holes in a sheet. Select the edges that form the open
// boundary, and PatchEdges adds a flat face across each closed loop. Use it to
// cap the ends of a tube, or to close a gap before stitching a sheet into a
// solid.
func Example_patch_edges() {
	// A tube is a sheet with two open ends, so build one: extrude a rectangle
	// with WithSurfaceResult (see Example_surface_result).
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
	tube, err := decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Free edges are edges that belong to only one face, which is what an open
	// boundary is made of. The tube has eight: four at each end. Both loops
	// match the query, so one call caps both ends.
	capped, err := decad.PatchEdges(context.Background(), tube,
		decad.Edges(decad.Free()).Exactly(8))
	if err != nil {
		fmt.Printf("failed to patch: %s\n", err)
		return
	}

	// The tube had 4 walls and now has 2 caps as well. No free edges remain.
	_, freeErr := decad.Edges(decad.Free()).SelectEdges(capped)
	fmt.Printf("faces: %d\n", len(capped.Faces()))
	fmt.Printf("free edges remain: %v\n", freeErr == nil)
	// Output:
	// faces: 6
	// free edges remain: false
}
