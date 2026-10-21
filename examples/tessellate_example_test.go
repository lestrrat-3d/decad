package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Tessellate turns a body into a mesh of triangles. Use it when you need
// triangles, for example to render the part, to feed a slicer, or to run your
// own mesh code. The tolerance sets how far the triangles may stray from a
// curved surface: a smaller number gives more triangles. The mesh reports the
// worst-case deviation it actually has, so you can check it against your limit.
func Example_tessellate() {
	// A plate with a round hole gives the mesh a curved edge to approximate.
	// Sketch it, then pick the profile that has the hole (see Example_extrude
	// for the sketch steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var plate *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			plate = p
		}
	}
	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, plate,
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Ask for triangles no more than 0.5 mm from the true surface.
	mesh, err := body.Tessellate(context.Background(), units.Millimeters(0.5))
	if err != nil {
		fmt.Printf("failed to tessellate: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d\n", len(mesh.Triangles()))
	fmt.Printf("within the 0.5 mm tolerance: %v\n", mesh.Bound().Mag() <= 0.5)
	// Output:
	// triangles: 56
	// within the 0.5 mm tolerance: true
}
