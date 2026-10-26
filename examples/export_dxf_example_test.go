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

// export.DXF writes a 2D drawing of a body, the format CAD and drafting tools
// open. It projects the body's edges along a direction you give, at full scale
// in millimetres, and puts the edges a surface hides on a separate dashed layer.
func Example_export_dxf() {
	// Make a body to draw: sketch a rectangle and extrude it into a box (see
	// Example_extrude for these steps).
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// An isometric view of the box. Write to any io.Writer.
	var out bytes.Buffer
	if err := export.DXF(context.Background(), &out, box, units.Millimeters(0.1), export.IsometricDrawingView()); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	file := out.String()
	// A box seen from a corner shows nine edges and hides three.
	fmt.Printf("visible edges: %d\n", strings.Count(file, "LINE\n  8\n"+export.DXFVisibleLayer+"\n"))
	fmt.Printf("hidden edges: %d\n", strings.Count(file, "LINE\n  8\n"+export.DXFHiddenLayer+"\n"))
	// Output:
	// visible edges: 9
	// hidden edges: 3
}
