package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A chain-fed ribbon mints no cap, so its mesh is every wall's own exact
// planar quad read straight off the body's topology — no chording, and no
// tolerance for one to bind. A straight (LineSeg-only) chain's ribbon
// tessellates and exports; a curved or free-form wall is not yet chorded.
func Example_decad_extrude_chain_tessellate() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A rectangle with one side erased: three straight walls, the ribbon
	// this section builds from an open walk.
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 6)
	d := s.CreatePoint(0, 6)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.ExtrudeChain(s, s.Chains()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude chain: %s\n", err)
		return
	}

	mesh, err := body.Tessellate(context.Background(), units.Millimeters(0.1))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d, bound: %v mm\n", len(mesh.Triangles()), mesh.Bound().Mag())
	fmt.Printf("boundary verified: %v, volume verified: %v\n", mesh.BoundaryVerified(), mesh.VolumeVerified())

	var stl strings.Builder
	if err := body.STL(&stl); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}
	fmt.Printf("stl facets: %d\n", strings.Count(stl.String(), "facet normal"))
	// Output:
	// triangles: 6, bound: 0 mm
	// boundary verified: true, volume verified: false
	// stl facets: 6
}
