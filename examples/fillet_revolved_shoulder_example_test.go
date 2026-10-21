package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Fillet also works on revolved parts. A turned shaft has a sharp inside
// corner where a thick section steps down to a thin one, which concentrates
// stress. Rounding that corner with a fillet is standard practice.
func Example_fillet_revolved_shoulder() {
	// Draw half of the shaft's side view above the X axis: a radius-10
	// section from x = 0 to 10, then a radius-5 section to x = 20.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {10, 5}, {10, 10}, {0, 10}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	// Spin the drawing a full turn about the X axis.
	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	shaft, err := decad.Revolve(context.Background(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}

	// The shoulder's inside corner is the only circular edge that is
	// concave, so that query selects it. Round it with a 2 mm radius.
	rounded, err := decad.Fillet(context.Background(), shaft,
		decad.Edges(decad.Circular(), decad.Concave()), units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to fillet: %s\n", err)
		return
	}
	for _, f := range rounded.Faces() {
		if torus, ok := f.Surface().(decad.Torus); ok {
			fmt.Printf("fillet is a torus: major radius %s, tube radius %s\n", torus.Major, torus.Minor)
		}
	}
	// Output:
	// fillet is a torus: major radius 7 mm, tube radius 2 mm
}
