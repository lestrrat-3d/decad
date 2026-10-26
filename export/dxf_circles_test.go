package export_test

import (
	"bytes"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/dxf-export-design.md §7's tests for the true circles
// of a drawing, on the 5 mm radius, 20 mm long rod of dxf_test.go.

// dxfCircle is one CIRCLE entity read back from a file.
type dxfCircle struct {
	layer string
	at    [2]float64
	r     float64
}

// readDXFCircles returns the CIRCLE entities of an ASCII DXF file.
func readDXFCircles(t *testing.T, file string) []dxfCircle {
	t.Helper()
	raw := strings.Split(strings.TrimRight(file, "\n"), "\n")
	var out []dxfCircle
	var cur *dxfCircle
	number := func(v string) float64 {
		f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
		require.NoError(t, err)
		return f
	}
	for i := 0; i+1 < len(raw); i += 2 {
		code, value := strings.TrimSpace(raw[i]), strings.TrimSpace(raw[i+1])
		switch {
		case code == "0":
			if cur != nil {
				out = append(out, *cur)
				cur = nil
			}
			if value == "CIRCLE" {
				cur = &dxfCircle{}
			}
		case cur == nil:
		case code == "8":
			cur.layer = value
		case code == "10":
			cur.at[0] = number(value)
		case code == "20":
			cur.at[1] = number(value)
		case code == "40":
			cur.r = number(value)
		}
	}
	if cur != nil {
		out = append(out, *cur)
	}
	return out
}

func TestDXFDrawsTheRimsOfARodAsCirclesFromTheTop(t *testing.T) {
	t.Parallel()
	rod := circleBodyForDXF(t, decad.New())
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, rod, units.Millimeters(0.1), topView))
	circles := readDXFCircles(t, out.String())
	// The top rim is seen and the bottom rim lies behind it, a hidden circle of the same size.
	require.Len(t, circles, 2)
	byLayer := map[string]dxfCircle{}
	for _, c := range circles {
		byLayer[c.layer] = c
	}
	for _, layer := range []string{export.DXFVisibleLayer, export.DXFHiddenLayer} {
		c, ok := byLayer[layer]
		require.True(t, ok, "a circle on %s", layer)
		require.InDelta(t, 5, c.r, 1e-9)
		require.InDelta(t, 0, c.at[0], 1e-9)
		require.InDelta(t, 0, c.at[1], 1e-9)
	}
	lines, _ := readDXFLines(t, out.String())
	require.Empty(t, lines, "the rod's chords are the circles: nothing else is drawn from above")
}

func TestDXFKeepsTheChordsOfACircleSeenEdgeOn(t *testing.T) {
	t.Parallel()
	rod := circleBodyForDXF(t, decad.New())
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, rod, units.Millimeters(0.1), frontView))
	require.Empty(t, readDXFCircles(t, out.String()), "from the side a circle is a line, not a circle")
	lines, _ := readDXFLines(t, out.String())
	require.NotEmpty(t, lines)
}

func TestDXFViewsAndSheetMoveAndScaleTheCircles(t *testing.T) {
	t.Parallel()
	rod := circleBodyForDXF(t, decad.New())
	var views bytes.Buffer
	require.NoError(t, export.DXFViews(t.Context(), &views, rod, units.Millimeters(0.1),
		[]export.DrawingView{topView, frontView}, units.Millimeters(5)))
	circles := readDXFCircles(t, views.String())
	require.Len(t, circles, 2)
	for _, c := range circles {
		// The top view is 10 by 10 mm with its corner at the origin: the circle is centred at (5, 5).
		require.InDelta(t, 5, c.at[0], 1e-9)
		require.InDelta(t, 5, c.at[1], 1e-9)
		require.InDelta(t, 5, c.r, 1e-9)
	}
	var sheet bytes.Buffer
	paper := a4Sheet(false)
	paper.Views = []export.DrawingView{topView}
	paper.Title = "ROD"
	require.NoError(t, export.DXFSheet(t.Context(), &sheet, rod, units.Millimeters(0.1), paper))
	scaled := readDXFCircles(t, sheet.String())
	require.Len(t, scaled, 2)
	for _, c := range scaled {
		require.InDelta(t, 2.5, c.r, 1e-9, "5 mm at 1:2")
	}
}
