package examples_test

import (
	"context"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// extrudeBlock adds a rectangular block to doc and returns it. The block spans
// x0..x1 and y0..y1 in millimeters, starts at height z0, and is height mm
// tall. The motion, linkage and dynamics examples use it to build the scene
// around the part they are testing (walls, arms, floors), so each of those
// examples can focus on its own feature. The steps inside are the same as in
// Example_extrude: sketch a rectangle, then extrude it.
func extrudeBlock(doc *decad.Document, x0, y0, x1, y1, z0, height float64) (*decad.Body, error) {
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(plane)
	if err != nil {
		return nil, err
	}
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, err
	}
	return decad.Extrude(context.Background(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
}
