package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// A profile bounded by a fit spline extrudes like any other. Use a fit spline
// when the outline must pass through chosen points, such as a curved edge
// taken from a drawing. The curved side wall becomes a free-form (NURBS)
// surface, and the body is still a measured solid.
func Example_extrude_fit_spline() {
	// A fit spline passes through the points you give it, which is how you
	// reproduce a curved outline taken from a drawing. Here it runs through three
	// points, and a straight line closes the shape.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	start := s.CreatePoint(0, 0)
	middle := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	if _, err := s.CreateFitSpline(start, middle, end); err != nil {
		fmt.Printf("failed to create spline: %s\n", err)
		return
	}
	s.CreateLine(end, start)

	// Sketch finds the closed regions itself. Take the valid one.
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			profile = p
			break
		}
	}
	if profile == nil {
		fmt.Println("no closed profile found")
		return
	}

	doc := decad.New()
	body, err := decad.Extrude(context.Background(), doc, s, profile,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	curved := 0
	for _, f := range body.Faces() {
		if f.Surface().Kind() == decad.KindNURBS {
			curved++
		}
	}
	volume, err := body.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("faces: %d, free-form walls: %d\n", len(body.Faces()), curved)
	fmt.Printf("volume: %.1f mm^3\n", volume.Value.Base())
	// Output:
	// faces: 4, free-form walls: 1
	// volume: 150.0 mm^3
}
