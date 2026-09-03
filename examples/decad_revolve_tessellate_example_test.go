package examples_test

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A revolved body meshes from its meridian section and ONE global angular
// sequence, so every generator face shares its whole latitude edge and a full
// turn closes with no seam. A generator that reaches the axis sweeps a pole,
// and a pole is a single interned vertex fanned to by its whole ring — never a
// ring of coincident vertices and never a collapsed quad.
//
// The mesh is an approximation of a curved boundary, so its volume sits UNDER
// the analytic one the evaluator measured in closed form; the gap is the
// chording the requested tolerance bought. A revolve mesh serves export today
// and is not yet admitted to the mesh boolean, which needs a proof of the
// volume the mesh and the body differ by.
func Example_decad_revolve_tessellate() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}

	// A right triangle with its base on the axis: revolved, a cone of radius
	// 5 mm and height 10 mm, with its apex ON the axis.
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 5)
	s.CreateLine(o, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, o)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	mesh, err := body.TessellateContext(context.Background(), units.Millimeters(0.5))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}

	// Two fans — one over the cone wall, one over the base disk — sharing the
	// single ring between them.
	fmt.Printf("vertices: %d, triangles: %d\n", len(mesh.Vertices()), len(mesh.Triangles()))
	fmt.Printf("bound within tolerance: %v\n", mesh.Bound().Mag() <= 0.5)

	vol, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	analytic, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	held := 0.0
	verts := mesh.Vertices()
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		held += a.Dot(b.Cross(c)) / 6
	}
	fmt.Printf("chorded volume under the analytic volume: %v\n", held < analytic)

	var stl strings.Builder
	if err := body.STL(&stl, decad.WithChordTolerance(units.Millimeters(0.5))); err != nil {
		fmt.Printf("failed to write STL: %s\n", err)
		return
	}
	fmt.Printf("stl facets: %d\n", strings.Count(stl.String(), "facet normal"))
	// Output:
	// vertices: 10, triangles: 16
	// bound within tolerance: true
	// chorded volume under the analytic volume: true
	// stl facets: 16
}
