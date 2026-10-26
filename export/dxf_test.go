package export_test

import (
	"bytes"
	"context"
	"math"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// dxfLine is one LINE entity read back from a file.
type dxfLine struct {
	layer string
	a, b  [2]float64
}

// readDXFLines returns the LINE entities of an ASCII DXF file, and the names of
// its sections in order. It reads group-code and value line pairs.
func readDXFLines(t *testing.T, file string) ([]dxfLine, []string) {
	t.Helper()
	raw := strings.Split(strings.TrimRight(file, "\n"), "\n")
	require.Zero(t, len(raw)%2, "a DXF file is group-code and value line pairs")
	var lines []dxfLine
	var sections []string
	var current *dxfLine
	flush := func() {
		if current != nil {
			lines = append(lines, *current)
			current = nil
		}
	}
	number := func(v string) float64 {
		f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
		require.NoError(t, err)
		return f
	}
	for i := 0; i < len(raw); i += 2 {
		code, value := strings.TrimSpace(raw[i]), strings.TrimSpace(raw[i+1])
		switch {
		case code == "0":
			flush()
			if value == "LINE" {
				current = &dxfLine{}
			}
		case code == "2" && len(sections) < 3 && (value == "HEADER" || value == "TABLES" || value == "ENTITIES"):
			sections = append(sections, value)
		case current == nil:
		case code == "8":
			current.layer = value
		case code == "10":
			current.a[0] = number(value)
		case code == "20":
			current.a[1] = number(value)
		case code == "11":
			current.b[0] = number(value)
		case code == "21":
			current.b[1] = number(value)
		}
	}
	flush()
	return lines, sections
}

func TestDXFDrawsACubeFromACorner(t *testing.T) {
	t.Parallel()
	cube := decadtest.NewBlock(t, decad.New(), 0, 0, 10, 10, units.Millimeters(10))
	view := export.DrawingView{Direction: r3.NewVec(0.55, 0.8, 0.45), Up: r3.NewVec(0, 0, 1)}
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, cube, units.Millimeters(0.1), view))
	file := out.String()
	require.True(t, strings.HasSuffix(file, "  0\nEOF\n"))
	lines, sections := readDXFLines(t, file)
	require.Equal(t, []string{"HEADER", "TABLES", "ENTITIES"}, sections)

	var visible, hidden []dxfLine
	for _, l := range lines {
		switch l.layer {
		case export.DXFVisibleLayer:
			visible = append(visible, l)
		case export.DXFHiddenLayer:
			hidden = append(hidden, l)
		default:
			t.Fatalf("a LINE on layer %q", l.layer)
		}
	}
	require.Len(t, visible, 9, "nine edges of a cube show from a corner")
	require.Len(t, hidden, 3, "and the three that meet at the far corner are hidden")

	// The drawing is at full scale: x = right·p and y = up·p with right =
	// direction × up. The far corner (10,10,10) projects to the point where
	// every hidden edge ends.
	dir, up := view.Direction, view.Up
	dir, _ = dir.Normalize()
	right, _ := dir.Cross(up).Normalize()
	trueUp := right.Cross(dir)
	far := r3.NewVec(10, 10, 10)
	farX, farY := right.Dot(far), trueUp.Dot(far)
	for _, l := range hidden {
		meets := math.Hypot(l.a[0]-farX, l.a[1]-farY) < 1e-6 || math.Hypot(l.b[0]-farX, l.b[1]-farY) < 1e-6
		require.True(t, meets, "a hidden edge ends at the far corner: %+v", l)
	}
	// Each visible edge is 10 mm of the cube projected, so none is longer.
	for _, l := range visible {
		require.LessOrEqual(t, math.Hypot(l.b[0]-l.a[0], l.b[1]-l.a[1]), 10+1e-9)
	}
	require.Contains(t, file, "$ACADVER\n  1\nAC1009\n")
	require.Contains(t, file, "HIDDEN\n 70\n     0\n  3\n__ __ __ __\n", "the dashed linetype is defined")
}

func TestDXFOfACylinderSpansItsDiameterAndHeight(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyForDXF(t, doc)
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, rod, units.Millimeters(0.05),
		export.DrawingView{Direction: r3.NewVec(0, 1, 0), Up: r3.NewVec(0, 0, 1)}))
	lines, _ := readDXFLines(t, out.String())
	minX, maxX, minY, maxY := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, l := range lines {
		if l.layer != export.DXFVisibleLayer {
			continue
		}
		for _, p := range [][2]float64{l.a, l.b} {
			minX, maxX = math.Min(minX, p[0]), math.Max(maxX, p[0])
			minY, maxY = math.Min(minY, p[1]), math.Max(maxY, p[1])
		}
	}
	// The chords of the tessellation span the diameter, within the chord error.
	require.InDelta(t, 10, maxX-minX, 0.05)
	require.InDelta(t, 20, maxY-minY, 1e-9)
}

func TestDXFIsDeterministic(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	var a, b bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &a, body, units.Millimeters(0.1), export.IsometricDrawingView()))
	require.NoError(t, export.DXF(t.Context(), &b, body, units.Millimeters(0.1), export.IsometricDrawingView()))
	require.Equal(t, a.Bytes(), b.Bytes())
}

func TestDXFRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	good := export.IsometricDrawingView()
	var nilContext context.Context
	require.ErrorIs(t, export.DXF(nilContext, &bytes.Buffer{}, body, units.Millimeters(0.1), good), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXF(t.Context(), nil, body, units.Millimeters(0.1), good), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXF(t.Context(), &bytes.Buffer{}, nil, units.Millimeters(0.1), good), decad.ErrDegenerate)
	var out bytes.Buffer
	require.ErrorIs(t, export.DXF(t.Context(), &out, body, units.Millimeters(0.1),
		export.DrawingView{Up: r3.NewVec(0, 0, 1)}), decad.ErrDegenerate, "no direction")
	require.ErrorIs(t, export.DXF(t.Context(), &out, body, units.Millimeters(0.1),
		export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 0, 1)}), decad.ErrDegenerate, "up along the view")
	require.Zero(t, out.Len())
	require.ErrorIs(t, export.DXF(t.Context(), &out, body, units.Degrees(1), good), decad.ErrUnitKind)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, export.DXF(canceled, &out, body, units.Millimeters(0.1), good), context.Canceled)
	require.Zero(t, out.Len())
}

// circleBodyForDXF extrudes a circle of radius 5 mm by 20 mm along +Z.
func circleBodyForDXF(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	centre := s.CreatePoint(0, 0)
	s.Fix(centre)
	s.CreateCircle(centre, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return body
}
