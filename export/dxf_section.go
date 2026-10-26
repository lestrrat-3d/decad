package export

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Names of the DXF layers DXFSection writes.
const (
	// DXFSectionLayer holds the outline of the cut.
	DXFSectionLayer = "SECTION"
	// DXFHatchLayer holds the hatching of the cut.
	DXFHatchLayer = "HATCH"
)

// DXFHatch is the hatching of a section drawn by [DXFSection].
type DXFHatch struct {
	// Spacing is the distance between neighbouring hatch lines, a positive length.
	Spacing units.Value
	// Angle is the angle of the hatch lines from the drawing's X axis. The zero
	// units.Value is 45 degrees, the usual hatch; any other must be an angle, and
	// units.Degrees(0) is lines along X.
	Angle units.Value
}

// DXFSection writes the cut of body by plane as an ASCII DXF file, release 12,
// with the cut hatched, as a section view of an engineering drawing
// (agent-docs/dxf-export-design.md §10). The drawing's X and Y axes are the
// plane's U and V in millimetres at full scale, from the plane's origin. The
// outlines of the cut, read from [decad.Body.SectionOutline], are LINE entities
// on the SECTION layer, and the material between them is filled by parallel LINE
// entities on the HATCH layer: a hole inside an outline is left empty, and the
// lines lie at whole multiples of the spacing across their direction, so two
// sections hatched alike carry one family of lines.
//
// The outline is the cut of the body's tessellation at tol, a reading of the
// mesh as SectionOutline says, so a curved wall is drawn as chords. A plane that
// misses the body writes a file with no entities. ctx, w and body MUST NOT be
// nil and plane MUST be valid, each otherwise decad.ErrDegenerate. Spacing MUST
// be positive: a zero is decad.ErrDegenerate, a negative value
// decad.ErrNegativeMagnitude and a wrong unit kind decad.ErrUnitKind, as is the
// wrong unit kind for Angle. A spacing so fine that the cut needs over 100000
// lines is decad.ErrDegenerate. Every failure writes nothing. The tessellation
// options of [decad.Body.SectionOutline] are not offered: it verifies the
// mesh's boundary itself.
func DXFSection(ctx context.Context, w io.Writer, body *decad.Body, plane r3.Frame, tol units.Value, hatch DXFHatch) error {
	if ctx == nil {
		return fmt.Errorf("export: DXF: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: DXF: %w: nil writer", decad.ErrDegenerate)
	}
	if body == nil {
		return fmt.Errorf("export: DXF: %w: nil body", decad.ErrDegenerate)
	}
	spacing, err := positiveLength(hatch.Spacing, "the hatch spacing")
	if err != nil {
		return err
	}
	angle := math.Pi / 4
	if hatch.Angle != (units.Value{}) {
		angle, err = sectionrecord.MagnitudeIn(hatch.Angle, units.Angle, units.Radian, "the hatch angle")
		if err != nil {
			// A negative angle is a direction, not a magnitude: read it as the signed angle.
			if !errors.Is(err, decad.ErrNegativeMagnitude) {
				return fmt.Errorf("export: DXF: %w", err)
			}
			angle = hatch.Angle.Base()
		}
	}
	outline, err := body.SectionOutline(ctx, plane, tol)
	if err != nil {
		return err
	}
	loops := make([][][2]float64, len(outline.Loops))
	for i, loop := range outline.Loops {
		loops[i] = make([][2]float64, len(loop))
		for j, p := range loop {
			local := plane.ToLocal(p)
			loops[i][j] = [2]float64{local.X, local.Y}
		}
	}
	lines, err := drawing.Hatch(loops, spacing, angle)
	if errors.Is(err, raster.ErrDegenerate) {
		return fmt.Errorf("export: DXF: %w: %s", decad.ErrDegenerate, err)
	}
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	var out bytes.Buffer
	out.WriteString(dxfSectionHeader)
	line := func(layer string, a, b [2]float64) {
		fmt.Fprintf(&out, "  0\nLINE\n  8\n%s\n 10\n%s\n 20\n%s\n 30\n0\n 11\n%s\n 21\n%s\n 31\n0\n",
			layer, dxfNumber(a[0]), dxfNumber(a[1]), dxfNumber(b[0]), dxfNumber(b[1]))
	}
	for _, loop := range loops {
		for i, p := range loop {
			line(DXFSectionLayer, p, loop[(i+1)%len(loop)])
		}
	}
	for _, s := range lines {
		line(DXFHatchLayer, s.A, s.B)
	}
	out.WriteString("  0\nENDSEC\n  0\nEOF\n")
	if n, err := w.Write(out.Bytes()); err != nil {
		return fmt.Errorf("export: DXF write failed: %w", err)
	} else if n != out.Len() {
		return fmt.Errorf("export: DXF write failed: %w", io.ErrShortWrite)
	}
	return nil
}

// dxfSectionHeader holds the sections before a section view's entities: the
// version, the linetype and the two layers.
const dxfSectionHeader = "  0\nSECTION\n  2\nHEADER\n  9\n$ACADVER\n  1\nAC1009\n  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nTABLES\n" +
	"  0\nTABLE\n  2\nLTYPE\n 70\n     1\n" +
	"  0\nLTYPE\n  2\nCONTINUOUS\n 70\n     0\n  3\nSolid line\n 72\n    65\n 73\n     0\n 40\n0.0\n" +
	"  0\nENDTAB\n" +
	"  0\nTABLE\n  2\nLAYER\n 70\n     2\n" +
	"  0\nLAYER\n  2\n" + DXFSectionLayer + "\n 70\n     0\n 62\n     7\n  6\nCONTINUOUS\n" +
	"  0\nLAYER\n  2\n" + DXFHatchLayer + "\n 70\n     0\n 62\n     8\n  6\nCONTINUOUS\n" +
	"  0\nENDTAB\n" +
	"  0\nENDSEC\n" +
	"  0\nSECTION\n  2\nENTITIES\n"
