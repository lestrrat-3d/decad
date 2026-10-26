package export

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"math"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// The layers of a [DXFPaper] sheet besides VISIBLE and HIDDEN.
const (
	DXFBorderLayer    = "BORDER"
	DXFTextLayer      = "TEXT"
	DXFDimensionLayer = "DIMENSION"
)

// Sheet geometry in paper millimetres: the title block's height and the room a
// view's dimensions take left of it and below it.
const (
	paperTitleHeight = 14.0
	paperDimRoom     = 12.0
	paperTextHeight  = 3.5
	paperTick        = 1.5
)

// DXFPaper describes a drawing sheet for [DXFSheet]: its paper size, the scale
// of the views on it, a border with a title block, and optionally the overall
// size of each view as a dimension.
type DXFPaper struct {
	// Views are laid in a row, left to right, in the order given.
	Views []DrawingView
	// Width and Height are the paper's size, such as 297 by 210 mm for A4 landscape.
	Width, Height units.Value
	// Scale is a dimensionless ratio of paper to model: 0.5 draws at 1:2 and 2 at 2:1.
	Scale units.Value
	// Margin is the border's inset from the paper's edge, Gap the clear distance
	// between views. Both are non-negative lengths.
	Margin, Gap units.Value
	// Title is written in the title block. It holds printable ASCII only.
	Title string
	// Dimensions adds, for each view, its overall width below it and its overall
	// height left of it, as the model's true size and not the drawn one.
	Dimensions bool
}

// DXFSheet writes several views of body at a stated scale on a sheet of stated
// paper size, inside a border with a title block, as an ASCII DXF file
// (agent-docs/dxf-export-design.md §6). Coordinates are paper millimetres, the
// paper's lower left corner at the origin. Each view is drawn as [DXF] draws it
// and then scaled and placed; the row of views is centred in the area above the
// title block, and when paper.Dimensions is set each view carries its overall
// width and height.
//
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. ctx, w and body MUST NOT be nil and paper.Views must hold a
// view valid as for [DXF], otherwise decad.ErrDegenerate. Width, Height, Scale
// must be positive: a zero is decad.ErrDegenerate, a negative value
// decad.ErrNegativeMagnitude and a wrong unit kind decad.ErrUnitKind, and Margin
// and Gap must not be negative. A title with a character other than printable
// ASCII, and views that do not fit the paper with their dimensions, are
// decad.ErrDegenerate; a sheet that is too small writes nothing.
func DXFSheet(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, paper DXFPaper, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: DXF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: DXF: %w: nil writer", decad.ErrDegenerate)
	}
	if len(paper.Views) == 0 {
		return fmt.Errorf("export: DXF: %w: a sheet needs at least one view", decad.ErrDegenerate)
	}
	width, err := positiveLength(paper.Width, "the paper width")
	if err != nil {
		return err
	}
	height, err := positiveLength(paper.Height, "the paper height")
	if err != nil {
		return err
	}
	margin, err := sectionrecord.MagnitudeIn(paper.Margin, units.Length, units.Millimeter, "the sheet margin")
	if err != nil {
		return fmt.Errorf("export: DXF: %w", err)
	}
	gap, err := sectionrecord.MagnitudeIn(paper.Gap, units.Length, units.Millimeter, "the sheet gap")
	if err != nil {
		return fmt.Errorf("export: DXF: %w", err)
	}
	scale, err := positiveRatio(paper.Scale)
	if err != nil {
		return err
	}
	for _, r := range paper.Title {
		if r < 32 || r > 126 {
			return fmt.Errorf("export: DXF: %w: the sheet title holds a character that is not printable ASCII", decad.ErrDegenerate)
		}
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	type placed struct {
		lines          drawing.Lines
		minX, minY     float64
		trueW, trueH   float64
		paperW, paperH float64
	}
	views := make([]placed, len(paper.Views))
	rowW, rowH := gap*float64(len(views)-1), 0.0
	for i, view := range paper.Views {
		if err := ctx.Err(); err != nil {
			return err
		}
		lines, err := projectDrawing(ctx, mesh, view)
		if err != nil {
			return err
		}
		minX, maxX, minY := extent(lines)
		maxY := minY
		for _, group := range [][]drawing.Segment{lines.Visible, lines.Hidden} {
			for _, s := range group {
				maxY = math.Max(maxY, math.Max(s.A[1], s.B[1]))
			}
		}
		p := placed{lines: lines, minX: minX, minY: minY, trueW: maxX - minX, trueH: maxY - minY}
		p.paperW, p.paperH = p.trueW*scale, p.trueH*scale
		views[i] = p
		rowW += p.paperW
		rowH = math.Max(rowH, p.paperH)
	}
	dimLeft, dimBelow := 0.0, 0.0
	if paper.Dimensions {
		dimLeft, dimBelow = paperDimRoom, paperDimRoom
		if len(views) > 1 && gap < paperDimRoom {
			return fmt.Errorf("export: DXF: %w: views with dimensions need a gap of at least %g mm for them", decad.ErrDegenerate, paperDimRoom)
		}
	}
	areaW := width - 2*margin
	areaH := height - 2*margin - paperTitleHeight
	if rowW+dimLeft > areaW || rowH+dimBelow > areaH {
		return fmt.Errorf("export: DXF: %w: the views at scale %g need %g by %g mm and the sheet leaves %g by %g mm",
			decad.ErrDegenerate, scale, rowW+dimLeft, rowH+dimBelow, areaW, areaH)
	}
	var out bytes.Buffer
	out.WriteString(dxfPaperHeader)
	line := func(layer string, ax, ay, bx, by float64) {
		fmt.Fprintf(&out, "  0\nLINE\n  8\n%s\n 10\n%s\n 20\n%s\n 30\n0\n 11\n%s\n 21\n%s\n 31\n0\n",
			layer, dxfNumber(ax), dxfNumber(ay), dxfNumber(bx), dxfNumber(by))
	}
	text := func(x, y, h, rotation float64, s string) {
		fmt.Fprintf(&out, "  0\nTEXT\n  8\n%s\n 10\n%s\n 20\n%s\n 30\n0\n 40\n%s\n  1\n%s\n 50\n%s\n",
			DXFTextLayer, dxfNumber(x), dxfNumber(y), dxfNumber(h), s, dxfNumber(rotation))
	}
	left, right, bottom, top := margin, width-margin, margin, height-margin
	for _, e := range [][4]float64{
		{left, bottom, right, bottom}, {right, bottom, right, top}, {right, top, left, top}, {left, top, left, bottom},
		{left, bottom + paperTitleHeight, right, bottom + paperTitleHeight},
	} {
		line(DXFBorderLayer, e[0], e[1], e[2], e[3])
	}
	divide := math.Max(left, right-60)
	line(DXFBorderLayer, divide, bottom, divide, bottom+paperTitleHeight)
	text(left+3, bottom+5, 5, 0, paper.Title)
	text(divide+3, bottom+5, 5, 0, "SCALE "+scaleLabel(scale))
	x := left + (areaW-rowW-dimLeft)/2 + dimLeft
	y := bottom + paperTitleHeight + (areaH-rowH-dimBelow)/2 + dimBelow
	move := func(p placed, group []drawing.Segment, layer string) {
		for _, s := range group {
			line(layer, (s.A[0]-p.minX)*scale+x, (s.A[1]-p.minY)*scale+y, (s.B[0]-p.minX)*scale+x, (s.B[1]-p.minY)*scale+y)
		}
	}
	for _, p := range views {
		move(p, p.lines.Visible, DXFVisibleLayer)
		move(p, p.lines.Hidden, DXFHiddenLayer)
		if paper.Dimensions {
			// The width: extension lines down from both sides, a dimension line with
			// a tick at each end, and the true width centred above it.
			dy := y - 6
			line(DXFDimensionLayer, x, y-1, x, y-8)
			line(DXFDimensionLayer, x+p.paperW, y-1, x+p.paperW, y-8)
			line(DXFDimensionLayer, x, dy, x+p.paperW, dy)
			line(DXFDimensionLayer, x-paperTick/2, dy-paperTick/2, x+paperTick/2, dy+paperTick/2)
			line(DXFDimensionLayer, x+p.paperW-paperTick/2, dy-paperTick/2, x+p.paperW+paperTick/2, dy+paperTick/2)
			label := dimensionLabel(p.trueW)
			text(x+p.paperW/2-float64(len(label))*paperTextHeight/4, dy+1, paperTextHeight, 0, label)
			// The height: the same turned a quarter, left of the view.
			dx := x - 6
			line(DXFDimensionLayer, x-1, y, x-8, y)
			line(DXFDimensionLayer, x-1, y+p.paperH, x-8, y+p.paperH)
			line(DXFDimensionLayer, dx, y, dx, y+p.paperH)
			line(DXFDimensionLayer, dx-paperTick/2, y-paperTick/2, dx+paperTick/2, y+paperTick/2)
			line(DXFDimensionLayer, dx-paperTick/2, y+p.paperH-paperTick/2, dx+paperTick/2, y+p.paperH+paperTick/2)
			label = dimensionLabel(p.trueH)
			text(dx-1, y+p.paperH/2-float64(len(label))*paperTextHeight/4, paperTextHeight, 90, label)
		}
		x += p.paperW + gap
	}
	out.WriteString("  0\nENDSEC\n  0\nEOF\n")
	if err := ctx.Err(); err != nil {
		return err
	}
	if n, err := w.Write(out.Bytes()); err != nil {
		return fmt.Errorf("export: DXF write failed: %w", err)
	} else if n != out.Len() {
		return fmt.Errorf("export: DXF write failed: %w", io.ErrShortWrite)
	}
	return nil
}

// positiveLength reads v as a length in millimetres that must be positive.
func positiveLength(v units.Value, what string) (float64, error) {
	mm, err := sectionrecord.MagnitudeIn(v, units.Length, units.Millimeter, what)
	if err != nil {
		return 0, fmt.Errorf("export: DXF: %w", err)
	}
	if mm == 0 {
		return 0, fmt.Errorf("export: DXF: %w: %s must be positive", decad.ErrDegenerate, what)
	}
	return mm, nil
}

// positiveRatio reads a dimensionless scale that must be positive and finite.
func positiveRatio(v units.Value) (float64, error) {
	if v.Kind() != units.Dimensionless {
		return 0, fmt.Errorf("export: DXF: %w: the sheet scale must be dimensionless", decad.ErrUnitKind)
	}
	s := v.Base()
	switch {
	case math.IsNaN(s) || math.IsInf(s, 0):
		return 0, fmt.Errorf("export: DXF: %w: the sheet scale is not finite", decad.ErrNotFinite)
	case s < 0:
		return 0, fmt.Errorf("export: DXF: %w: the sheet scale is negative", decad.ErrNegativeMagnitude)
	case s == 0:
		return 0, fmt.Errorf("export: DXF: %w: the sheet scale must be positive", decad.ErrDegenerate)
	}
	return s, nil
}

// scaleLabel writes a scale as a ratio: 1:2 for a half, 2:1 for double.
func scaleLabel(s float64) string {
	if s < 1 {
		return "1:" + strconv.FormatFloat(round2(1/s), 'f', -1, 64)
	}
	return strconv.FormatFloat(round2(s), 'f', -1, 64) + ":1"
}

// dimensionLabel writes a model length in millimetres to two decimals.
func dimensionLabel(v float64) string { return strconv.FormatFloat(round2(v), 'f', -1, 64) }

func round2(v float64) float64 { return math.Round(v*100) / 100 }

// dxfPaperHeader holds the sections before a sheet's entities: the two
// linetypes and the five layers.
const dxfPaperHeader = "  0\nSECTION\n  2\nHEADER\n  9\n$ACADVER\n  1\nAC1009\n  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nTABLES\n" +
	"  0\nTABLE\n  2\nLTYPE\n 70\n     2\n" +
	"  0\nLTYPE\n  2\nCONTINUOUS\n 70\n     0\n  3\nSolid line\n 72\n    65\n 73\n     0\n 40\n0.0\n" +
	"  0\nLTYPE\n  2\nHIDDEN\n 70\n     0\n  3\n__ __ __ __\n 72\n    65\n 73\n     2\n 40\n0.75\n 49\n0.5\n 49\n-0.25\n" +
	"  0\nENDTAB\n" +
	"  0\nTABLE\n  2\nLAYER\n 70\n     5\n" +
	"  0\nLAYER\n  2\n" + DXFVisibleLayer + "\n 70\n     0\n 62\n     7\n  6\nCONTINUOUS\n" +
	"  0\nLAYER\n  2\n" + DXFHiddenLayer + "\n 70\n     0\n 62\n     8\n  6\nHIDDEN\n" +
	"  0\nLAYER\n  2\n" + DXFBorderLayer + "\n 70\n     0\n 62\n     7\n  6\nCONTINUOUS\n" +
	"  0\nLAYER\n  2\n" + DXFTextLayer + "\n 70\n     0\n 62\n     7\n  6\nCONTINUOUS\n" +
	"  0\nLAYER\n  2\n" + DXFDimensionLayer + "\n 70\n     0\n 62\n     3\n  6\nCONTINUOUS\n" +
	"  0\nENDTAB\n" +
	"  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nENTITIES\n"
