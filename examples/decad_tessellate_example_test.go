package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Tessellate approximates a body's boundary as a watertight triangle mesh —
// an output for export, never the representation. Faces that share a curved
// edge chord it at the same samples, so the mesh has no cracks, and the mesh
// reports the chord deviation it proves. STL and OBJ writers build on it.
func Example_decad_tessellate() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A 100 x 60 plate with a round hole, extruded 8 mm.
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Chord the hole at a 0.5 mm tolerance. The context reaches every
	// chording and triangulation phase.
	mesh, err := body.TessellateContext(context.Background(), units.Millimeters(0.5))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	fmt.Printf("vertices: %d, triangles: %d\n", len(mesh.Vertices()), len(mesh.Triangles()))
	fmt.Printf("proven chord bound within tolerance: %v\n", mesh.Bound().Mag() <= 0.5)

	var stl strings.Builder
	if err := body.STL(&stl, decad.WithChordTolerance(units.Millimeters(0.5))); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}
	fmt.Printf("stl facets: %d\n", strings.Count(stl.String(), "facet normal"))
	// Output:
	// vertices: 28, triangles: 56
	// proven chord bound within tolerance: true
	// stl facets: 56
}
