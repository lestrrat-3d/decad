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

// A circular edge that faces the view is a circle in the drawing, and DXF writes it
// as a CIRCLE entity, not a polygon of short lines, so a CAD program can snap to its
// centre and dimension its radius. Here a 10 mm rod seen from above is two circles,
// the top rim and the bottom rim behind it, and nothing else.
func Example_export_dxf_circles() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	top := export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	var out bytes.Buffer
	if err := export.DXF(context.Background(), &out, rod, units.Millimeters(0.1), top); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("circles: %d, lines: %d\n", strings.Count(out.String(), "CIRCLE\n"), strings.Count(out.String(), "LINE\n"))
	// Output:
	// circles: 2, lines: 0
}

// A rounded corner is an arc in the drawing, and DXF writes it as an ARC entity with
// its centre, radius and angles. Here a 40 x 30 plate with 5 mm corners, seen from
// above, draws four arcs for its top rim (and four dashed ones for its bottom rim,
// behind them) and the four straight sides as lines.
func Example_export_dxf_arcs() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	pt := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	s.CreateLine(pt(5, 0), pt(35, 0))
	s.CreateArc(pt(35, 5), pt(35, 0), pt(40, 5))
	s.CreateLine(pt(40, 5), pt(40, 25))
	s.CreateArc(pt(35, 25), pt(40, 25), pt(35, 30))
	s.CreateLine(pt(35, 30), pt(5, 30))
	s.CreateArc(pt(5, 25), pt(5, 30), pt(0, 25))
	s.CreateLine(pt(0, 25), pt(0, 5))
	s.CreateArc(pt(5, 5), pt(0, 5), pt(5, 0))
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	plate, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	top := export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	var out bytes.Buffer
	if err := export.DXF(context.Background(), &out, plate, units.Millimeters(0.1), top); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("arcs: %d, lines: %d\n", strings.Count(out.String(), "ARC\n"), strings.Count(out.String(), "LINE\n"))
	// Output:
	// arcs: 8, lines: 8
}

// DXFSheet with Dimensions states the diameter of each whole circle a view draws
// and the radius of each arc, beside the overall width and height. Here a rod seen
// from above on A4 paper at 1:2 gets its 10 mm diameter.
func Example_export_dxf_sheet_dimensions() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	rod, err := decad.Extrude(context.Background(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	var out bytes.Buffer
	if err := export.DXFSheet(context.Background(), &out, rod, units.Millimeters(0.1), export.DXFPaper{
		Views:      []export.DrawingView{{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}},
		Width:      units.Millimeters(297),
		Height:     units.Millimeters(210),
		Scale:      units.Scalar(0.5),
		Margin:     units.Millimeters(10),
		Gap:        units.Millimeters(0),
		Title:      "ROD",
		Dimensions: true,
	}); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("diameter stated: %t\n", strings.Contains(out.String(), "DIA 10"))
	// Output:
	// diameter stated: true
}

// DXFPaper.Fields adds labelled entries to the title block, such as the part
// number, the author and the revision, beside the title.
func Example_export_dxf_title_fields() {
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
		Views:  []export.DrawingView{{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}},
		Width:  units.Millimeters(297),
		Height: units.Millimeters(210),
		Scale:  units.Scalar(1),
		Margin: units.Millimeters(10),
		Gap:    units.Millimeters(0),
		Title:  "BLOCK",
		Fields: []export.DXFField{{Label: "PART", Value: "BK-100"}, {Label: "REV", Value: "C"}},
	}); err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("part number written: %t, revision written: %t\n",
		strings.Contains(out.String(), "PART: BK-100"), strings.Contains(out.String(), "REV: C"))
	// Output:
	// part number written: true, revision written: true
}

// export.DXFSection draws the cut of a body by a plane, with the cut's outline and
// its hatching, the way a section view of an engineering drawing does. Here a plate
// with a round hole is cut through its middle: the hatch fills the plate and leaves
// the hole empty.
func Example_export_dxf_section() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 40, 40)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(20, 20), 6)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var plate *decad.Body
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			plate, err = decad.Extrude(context.Background(), decad.New(), s, p,
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			if err != nil {
				fmt.Printf("failed to extrude: %s\n", err)
				return
			}
		}
	}
	// The cutting plane is z = 5, its X and Y along the world's.
	plane, err := r3.NewFrame(r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		fmt.Printf("failed to make the plane: %s\n", err)
		return
	}
	var out bytes.Buffer
	err = export.DXFSection(context.Background(), &out, plate, plane, units.Millimeters(0.1),
		export.DXFHatch{Spacing: units.Millimeters(2)})
	if err != nil {
		fmt.Printf("failed to export: %s\n", err)
		return
	}
	fmt.Printf("outline: %t, hatch: %t\n",
		strings.Contains(out.String(), export.DXFSectionLayer), strings.Contains(out.String(), export.DXFHatchLayer))
	// Output:
	// outline: true, hatch: true
}
