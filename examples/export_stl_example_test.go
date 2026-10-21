package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// export.STL writes a body as a mesh of triangles in the STL format. Use it
// for 3D printing and for viewers that only understand triangles. Unlike STEP,
// an STL file has no exact surfaces, so curved parts are approximated.
func Example_export_stl() {
	// Make a body to export: sketch a rectangle and extrude it into a box
	// (see Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 2, 3)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The tolerance bounds how far the triangles may stray from the true
	// surface. The box has flat faces, so it needs no approximation.
	var out bytes.Buffer
	if err := export.STL(context.Background(), &out, box, units.Millimeters(0.1)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	// Each of the six faces is two triangles.
	fmt.Printf("triangles: %d\n", strings.Count(out.String(), "facet normal"))
	// Output:
	// triangles: 12
}
