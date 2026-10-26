package examples_test

import (
	"bytes"
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
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

// export.DXFViews puts several views of a body on one drawing sheet, side by side
// with a gap between them, as the front, top and side views of a drawing are laid
// out. Each view rests on the same baseline.
func Example_export_dxf_views() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	box, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// The top view, looking down with +Y up the sheet, and the front view,
	// looking along +Y with +Z up, 5 mm apart. The top view is 20 by 30 mm and
	// the front view 20 by 10 mm, so the front view starts at x = 25.
	top := export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	front := export.DrawingView{Direction: r3.NewVec(0, 1, 0), Up: r3.NewVec(0, 0, 1)}
	var out bytes.Buffer
	if err := export.DXFViews(context.Background(), &out, box, units.Millimeters(0.1),
		[]export.DrawingView{top, front}, units.Millimeters(5)); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("layers: %d visible entities, ends with EOF: %t\n",
		strings.Count(out.String(), "VISIBLE\n 10"), strings.HasSuffix(out.String(), "EOF\n"))
	// Output:
	// layers: 8 visible entities, ends with EOF: true
}

// export.DXFSheet draws views at a stated scale on a sheet of stated paper size,
// inside a border with a title block, and can dimension each view with the
// model's true width and height. Here a 20 x 30 x 10 mm block goes on A4
// landscape paper at 1:2, so the 20 mm top view is drawn 10 mm wide and its
// dimension still reads 20.
func Example_export_dxf_sheet() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 20, 30)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	block, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	var out bytes.Buffer
	if err := export.DXFSheet(context.Background(), &out, block, units.Millimeters(0.1), export.DXFPaper{
		Views: []export.DrawingView{
			{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)},
			{Direction: r3.NewVec(0, 1, 0), Up: r3.NewVec(0, 0, 1)},
		},
		Width:      units.Millimeters(297),
		Height:     units.Millimeters(210),
		Scale:      units.Scalar(0.5),
		Margin:     units.Millimeters(10),
		Gap:        units.Millimeters(15),
		Title:      "BLOCK",
		Dimensions: true,
	}); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("border lines: %d, texts: %d\n",
		strings.Count(out.String(), "BORDER\n 10"), strings.Count(out.String(), "TEXT\n  8"))
	// Output:
	// border lines: 6, texts: 6
}
