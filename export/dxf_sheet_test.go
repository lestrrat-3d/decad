package export_test

import (
	"bytes"
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/dxf-export-design.md §5's tests for DXFViews, on a
// 20 x 30 x 10 mm block: seen from above it is 20 by 30 and from the front 20 by 10.

var (
	topView   = export.DrawingView{Direction: r3.NewVec(0, 0, -1), Up: r3.NewVec(0, 1, 0)}
	frontView = export.DrawingView{Direction: r3.NewVec(0, 1, 0), Up: r3.NewVec(0, 0, 1)}
)

// sheetBounds is the extent of the visible lines.
func sheetBounds(lines []dxfLine) (minX, maxX, minY, maxY float64) {
	minX, maxX, minY, maxY = math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, l := range lines {
		for _, p := range [][2]float64{l.a, l.b} {
			minX, maxX = math.Min(minX, p[0]), math.Max(maxX, p[0])
			minY, maxY = math.Min(minY, p[1]), math.Max(maxY, p[1])
		}
	}
	return minX, maxX, minY, maxY
}

func TestDXFViewsLaysViewsInARow(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var out bytes.Buffer
	require.NoError(t, export.DXFViews(t.Context(), &out, body, units.Millimeters(0.1),
		[]export.DrawingView{topView, frontView}, units.Millimeters(5)))
	lines, sections := readDXFLines(t, out.String())
	require.Equal(t, []string{"HEADER", "TABLES", "ENTITIES"}, sections)
	// The top view takes x 0 to 20 and the front view x 25 to 45, both resting on y = 0.
	var top, front []dxfLine
	for _, l := range lines {
		if l.a[0] <= 20+1e-9 && l.b[0] <= 20+1e-9 {
			top = append(top, l)
		} else {
			front = append(front, l)
		}
	}
	minX, maxX, minY, maxY := sheetBounds(top)
	require.InDelta(t, 0, minX, 1e-9)
	require.InDelta(t, 20, maxX, 1e-9)
	require.InDelta(t, 0, minY, 1e-9)
	require.InDelta(t, 30, maxY, 1e-9)
	minX, maxX, minY, maxY = sheetBounds(front)
	require.InDelta(t, 25, minX, 1e-9, "the gap of 5 mm after the first view's 20 mm")
	require.InDelta(t, 45, maxX, 1e-9)
	require.InDelta(t, 0, minY, 1e-9)
	require.InDelta(t, 10, maxY, 1e-9)
}

func TestDXFViewsOfOneViewMovesItToTheOrigin(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 5, 7, 25, 37, units.Millimeters(10))
	var out bytes.Buffer
	require.NoError(t, export.DXFViews(t.Context(), &out, body, units.Millimeters(0.1), []export.DrawingView{topView}, units.Millimeters(0)))
	lines, _ := readDXFLines(t, out.String())
	minX, maxX, minY, maxY := sheetBounds(lines)
	require.InDelta(t, 0, minX, 1e-9)
	require.InDelta(t, 20, maxX, 1e-9)
	require.InDelta(t, 0, minY, 1e-9)
	require.InDelta(t, 30, maxY, 1e-9)
}

func TestDXFViewsKeepsHiddenEdgesOnTheirLayer(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var out bytes.Buffer
	require.NoError(t, export.DXFViews(t.Context(), &out, body, units.Millimeters(0.1),
		[]export.DrawingView{export.IsometricDrawingView(), topView}, units.Millimeters(2)))
	lines, _ := readDXFLines(t, out.String())
	layers := map[string]int{}
	for _, l := range lines {
		layers[l.layer]++
	}
	require.NotZero(t, layers[export.DXFVisibleLayer])
	require.NotZero(t, layers[export.DXFHiddenLayer], "the isometric view hides three edges of the block")
}

func TestDXFViewsRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
	var nilContext context.Context
	tol := units.Millimeters(0.1)
	views := []export.DrawingView{topView}
	gap := units.Millimeters(1)
	require.ErrorIs(t, export.DXFViews(nilContext, &bytes.Buffer{}, body, tol, views, gap), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFViews(t.Context(), nil, body, tol, views, gap), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFViews(t.Context(), &bytes.Buffer{}, nil, tol, views, gap), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFViews(t.Context(), &bytes.Buffer{}, body, tol, nil, gap), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFViews(t.Context(), &bytes.Buffer{}, body, tol, views, units.Millimeters(-1)), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, export.DXFViews(t.Context(), &bytes.Buffer{}, body, tol, views, units.Degrees(1)), decad.ErrUnitKind)
	require.ErrorIs(t, export.DXFViews(t.Context(), &bytes.Buffer{}, body, tol, []export.DrawingView{{}}, gap), decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	var out bytes.Buffer
	require.ErrorIs(t, export.DXFViews(canceled, &out, body, tol, views, gap), context.Canceled)
	require.Zero(t, out.Len(), "a canceled export writes nothing")
}
