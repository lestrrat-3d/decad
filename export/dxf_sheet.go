package export

import (
	"context"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// DXFViews writes several views of body on one DXF sheet, laid out in a row
// (agent-docs/dxf-export-design.md §5). Each view is drawn as [DXF] draws it, then
// moved so that its lower left corner sits on the sheet's baseline: the first
// view's left edge is at x = 0, and each later view starts gap to the right of the
// one before. The views share the VISIBLE and HIDDEN layers. Coordinates are
// millimetres at full scale.
//
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. ctx, w and body MUST NOT be nil, and views MUST hold at least
// one view, each valid as for [DXF], otherwise decad.ErrDegenerate. gap is a
// non-negative length: a negative gap is decad.ErrNegativeMagnitude and a wrong
// unit kind decad.ErrUnitKind.
func DXFViews(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, views []DrawingView, gap units.Value, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: DXF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: DXF: %w: nil writer", decad.ErrDegenerate)
	}
	if len(views) == 0 {
		return fmt.Errorf("export: DXF: %w: a sheet needs at least one view", decad.ErrDegenerate)
	}
	space, err := sectionrecord.MagnitudeIn(gap, units.Length, units.Millimeter, "the sheet gap")
	if err != nil {
		return fmt.Errorf("export: DXF: %w", err)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	var sheet drawing.Lines
	x := 0.0
	for _, view := range views {
		lines, err := projectBodyDrawing(ctx, body, mesh, view)
		if err != nil {
			return err
		}
		minX, maxX, minY := extent(lines)
		move := func(segments []drawing.Segment) []drawing.Segment {
			out := make([]drawing.Segment, len(segments))
			for i, s := range segments {
				out[i] = drawing.Segment{
					A: [2]float64{s.A[0] - minX + x, s.A[1] - minY},
					B: [2]float64{s.B[0] - minX + x, s.B[1] - minY},
				}
			}
			return out
		}
		sheet.Visible = append(sheet.Visible, move(lines.Visible)...)
		sheet.Hidden = append(sheet.Hidden, move(lines.Hidden)...)
		for _, c := range lines.VisibleCircles {
			sheet.VisibleCircles = append(sheet.VisibleCircles, drawing.Circle{Centre: [2]float64{c.Centre[0] - minX + x, c.Centre[1] - minY}, Radius: c.Radius})
		}
		for _, c := range lines.HiddenCircles {
			sheet.HiddenCircles = append(sheet.HiddenCircles, drawing.Circle{Centre: [2]float64{c.Centre[0] - minX + x, c.Centre[1] - minY}, Radius: c.Radius})
		}
		x += maxX - minX + space
	}
	return writeDXFFile(ctx, w, sheet)
}

// extent returns the smallest x, the largest x and the smallest y of the
// segments, or zeros for none.
func extent(lines drawing.Lines) (minX, maxX, minY float64) {
	minX, maxX, minY = math.Inf(1), math.Inf(-1), math.Inf(1)
	for _, group := range [][]drawing.Segment{lines.Visible, lines.Hidden} {
		for _, s := range group {
			for _, p := range [2][2]float64{s.A, s.B} {
				minX, maxX, minY = math.Min(minX, p[0]), math.Max(maxX, p[0]), math.Min(minY, p[1])
			}
		}
	}
	for _, group := range [][]drawing.Circle{lines.VisibleCircles, lines.HiddenCircles} {
		for _, c := range group {
			minX, maxX, minY = math.Min(minX, c.Centre[0]-c.Radius), math.Max(maxX, c.Centre[0]+c.Radius), math.Min(minY, c.Centre[1]-c.Radius)
		}
	}
	if math.IsInf(minX, 1) {
		return 0, 0, 0
	}
	return minX, maxX, minY
}
