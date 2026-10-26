package export

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"strconv"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// DrawingView is the direction a drawing looks along. Direction is the way the
// camera looks, from the eye into the scene, and Up sets the drawing's vertical;
// neither need be a unit vector. The drawing's X axis is Direction × Up and its
// Y axis is up the sheet.
type DrawingView struct {
	Direction, Up r3.Vec
}

// IsometricDrawingView looks along (-1, -1, -1) with +Z up the sheet, as
// [IsometricView] does.
func IsometricDrawingView() DrawingView {
	return DrawingView{Direction: r3.NewVec(-1, -1, -1), Up: r3.NewVec(0, 0, 1)}
}

// Names of the DXF layers DXF writes.
const (
	// DXFVisibleLayer holds the edges a viewer would see.
	DXFVisibleLayer = "VISIBLE"
	// DXFHiddenLayer holds the edges behind a surface, in a dashed linetype.
	DXFHiddenLayer = "HIDDEN"
)

// DXF writes a 2D drawing of body as an ASCII DXF file, release 12, at the
// requested chord tolerance (agent-docs/dxf-export-design.md). The drawing is an
// orthographic projection along view: every feature edge of the body's
// tessellation becomes LINE entities in millimetres at full scale, on the
// VISIBLE layer where a surface does not hide it and on the dashed HIDDEN layer
// where one does. A feature edge is a boundary or non-manifold edge, a crease of
// more than 30 degrees, or a silhouette; a curved face is drawn as the chords
// of its tessellation, so a circle is a polygon. No dimensions, text, hatching
// or title block are drawn.
//
// The mesh defaults to decad.VerifyNone; pass decad.WithVerification to demand
// stronger proofs. Hidden-line removal reads a 2048-pixel depth buffer, so an
// edge passing behind a surface by less than a few pixels of the drawing's size
// reads as visible. ctx, w and body MUST NOT be nil, and view MUST have a
// direction and an up vector not parallel to it, each otherwise
// decad.ErrDegenerate.
func DXF(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, view DrawingView, opts ...decad.TessellateOption) error {
	if ctx == nil {
		return fmt.Errorf("export: DXF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: DXF: %w: nil writer", decad.ErrDegenerate)
	}
	mesh, err := tessellateForExport(ctx, body, tol, opts)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	lines, err := projectBodyDrawing(ctx, body, mesh, view)
	if err != nil {
		return err
	}
	return writeDXFFile(ctx, w, lines)
}

// writeDXFDrawing writes the mesh's feature edges as a DXF drawing.
func writeDXFDrawing(ctx context.Context, w io.Writer, mesh meshSource, view DrawingView) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	lines, err := projectDrawing(ctx, mesh, view)
	if err != nil {
		return err
	}
	return writeDXFFile(ctx, w, lines)
}

// projectBodyDrawing is projectDrawing with the whole circles of body that face
// the view drawn as circles: a circular edge whose axis lies along the view
// direction is a true circle in the drawing, and where every chord of its polygon
// is a whole visible (or hidden) segment, those chords are replaced by the circle.
func projectBodyDrawing(ctx context.Context, body *decad.Body, mesh meshSource, view DrawingView) (drawing.Lines, error) {
	lines, err := projectDrawing(ctx, mesh, view)
	if err != nil {
		return drawing.Lines{}, err
	}
	right, up, dir := raster.Camera{Direction: view.Direction, Up: view.Up}.Axes()
	var candidates []drawing.Circle
	for _, e := range body.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok || e.Start() != e.End() {
			continue
		}
		axis, ok := c.Axis.Normalize()
		if !ok || axis.Cross(dir).Len() > 1e-9 {
			continue
		}
		radius, err := c.Radius.In(units.Millimeter)
		if err != nil {
			continue
		}
		candidates = append(candidates, drawing.Circle{Centre: [2]float64{right.Dot(c.Center), up.Dot(c.Center)}, Radius: radius})
	}
	lines.Circulate(candidates)
	return lines, nil
}

// projectDrawing projects the feature edges of mesh along view.
func projectDrawing(ctx context.Context, mesh meshSource, view DrawingView) (drawing.Lines, error) {
	lines, err := drawing.Project(ctx, mesh.Vertices(), mesh.Triangles(), raster.Camera{Direction: view.Direction, Up: view.Up})
	if errors.Is(err, raster.ErrDegenerate) {
		return drawing.Lines{}, fmt.Errorf("export: DXF: %w: %s", decad.ErrDegenerate, err)
	}
	return lines, err
}

// writeDXFFile builds the DXF file for lines and writes it with one Write.
func writeDXFFile(ctx context.Context, w io.Writer, lines drawing.Lines) error {
	var out bytes.Buffer
	writeDXF(&out, lines)
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

// dxfHeader holds the sections before the entities: the version, the linetypes
// and the two layers.
const dxfHeader = "  0\nSECTION\n  2\nHEADER\n  9\n$ACADVER\n  1\nAC1009\n  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nTABLES\n" +
	"  0\nTABLE\n  2\nLTYPE\n 70\n     2\n" +
	"  0\nLTYPE\n  2\nCONTINUOUS\n 70\n     0\n  3\nSolid line\n 72\n    65\n 73\n     0\n 40\n0.0\n" +
	"  0\nLTYPE\n  2\nHIDDEN\n 70\n     0\n  3\n__ __ __ __\n 72\n    65\n 73\n     2\n 40\n0.75\n 49\n0.5\n 49\n-0.25\n" +
	"  0\nENDTAB\n" +
	"  0\nTABLE\n  2\nLAYER\n 70\n     2\n" +
	"  0\nLAYER\n  2\n" + DXFVisibleLayer + "\n 70\n     0\n 62\n     7\n  6\nCONTINUOUS\n" +
	"  0\nLAYER\n  2\n" + DXFHiddenLayer + "\n 70\n     0\n 62\n     8\n  6\nHIDDEN\n" +
	"  0\nENDTAB\n" +
	"  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nENTITIES\n"

func writeDXF(out *bytes.Buffer, lines drawing.Lines) {
	out.WriteString(dxfHeader)
	for _, layer := range []struct {
		name    string
		circles []drawing.Circle
	}{{DXFVisibleLayer, lines.VisibleCircles}, {DXFHiddenLayer, lines.HiddenCircles}} {
		for _, c := range layer.circles {
			writeDXFCircle(out, layer.name, c)
		}
	}
	for _, layer := range []struct {
		name     string
		segments []drawing.Segment
	}{{DXFVisibleLayer, lines.Visible}, {DXFHiddenLayer, lines.Hidden}} {
		for _, s := range layer.segments {
			fmt.Fprintf(out, "  0\nLINE\n  8\n%s\n 10\n%s\n 20\n%s\n 30\n0\n 11\n%s\n 21\n%s\n 31\n0\n",
				layer.name, dxfNumber(s.A[0]), dxfNumber(s.A[1]), dxfNumber(s.B[0]), dxfNumber(s.B[1]))
		}
	}
	out.WriteString("  0\nENDSEC\n  0\nEOF\n")
}

// writeDXFCircle writes one CIRCLE entity on layer.
func writeDXFCircle(out *bytes.Buffer, layer string, c drawing.Circle) {
	fmt.Fprintf(out, "  0\nCIRCLE\n  8\n%s\n 10\n%s\n 20\n%s\n 30\n0\n 40\n%s\n",
		layer, dxfNumber(c.Centre[0]), dxfNumber(c.Centre[1]), dxfNumber(c.Radius))
}

// dxfNumber formats x in plain decimal, never in exponent form, with negative
// zero as zero.
func dxfNumber(x float64) string {
	if x == 0 {
		x = 0
	}
	return strconv.FormatFloat(x, 'f', -1, 64)
}
