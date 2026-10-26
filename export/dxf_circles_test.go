package export_test

import (
	"bytes"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
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

// dxfArc is one ARC entity read back from a file, its angles in degrees.
type dxfArc struct {
	layer      string
	at         [2]float64
	r          float64
	start, end float64
}

// readDXFArcs returns the ARC entities of an ASCII DXF file.
func readDXFArcs(t *testing.T, file string) []dxfArc {
	t.Helper()
	raw := strings.Split(strings.TrimRight(file, "\n"), "\n")
	var out []dxfArc
	var cur *dxfArc
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
			if value == "ARC" {
				cur = &dxfArc{}
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
		case code == "50":
			cur.start = number(value)
		case code == "51":
			cur.end = number(value)
		}
	}
	if cur != nil {
		out = append(out, *cur)
	}
	return out
}

// roundedPlate is a 40 x 30 plate 10 mm thick with corners of radius 5.
func roundedPlate(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pt := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	// Counter-clockwise from the bottom edge: each side, then the corner arc about its centre.
	s.CreateLine(pt(5, 0), pt(35, 0))
	s.CreateArc(pt(35, 5), pt(35, 0), pt(40, 5))
	s.CreateLine(pt(40, 5), pt(40, 25))
	s.CreateArc(pt(35, 25), pt(40, 25), pt(35, 30))
	s.CreateLine(pt(35, 30), pt(5, 30))
	s.CreateArc(pt(5, 25), pt(5, 30), pt(0, 25))
	s.CreateLine(pt(0, 25), pt(0, 5))
	s.CreateArc(pt(5, 5), pt(0, 5), pt(5, 0))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return plate
}

// arcKey names an arc by its layer, centre and angles to the nearest degree, for comparing sets.
func arcKey(a dxfArc) string {
	return a.layer + " (" + strconv.FormatFloat(a.at[0], 'f', 0, 64) + "," + strconv.FormatFloat(a.at[1], 'f', 0, 64) + ") r" +
		strconv.FormatFloat(a.r, 'f', 0, 64) + " " + strconv.FormatFloat(a.start, 'f', 0, 64) + ">" + strconv.FormatFloat(a.end, 'f', 0, 64)
}

func TestDXFDrawsTheCornersOfARoundedPlateAsArcs(t *testing.T) {
	t.Parallel()
	plate := roundedPlate(t)
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, plate, units.Millimeters(0.1), topView))
	var got []string
	for _, a := range readDXFArcs(t, out.String()) {
		got = append(got, arcKey(a))
	}
	// The top rim's four corners are visible and the bottom rim's lie behind them, each arc
	// counter-clockwise in the drawing from its start to its end.
	var want []string
	for _, layer := range []string{export.DXFVisibleLayer, export.DXFHiddenLayer} {
		want = append(want,
			layer+" (35,5) r5 270>0", layer+" (35,25) r5 0>90", layer+" (5,25) r5 90>180", layer+" (5,5) r5 180>270")
	}
	require.ElementsMatch(t, want, got)
	lines, _ := readDXFLines(t, out.String())
	require.Len(t, lines, 8, "the four straight sides, visible and hidden, are still lines")
}

func TestDXFArcsMirrorWhenSeenFromBelow(t *testing.T) {
	t.Parallel()
	plate := roundedPlate(t)
	bottom := export.DrawingView{Direction: r3.NewVec(0, 0, 1), Up: r3.NewVec(0, 1, 0)}
	var out bytes.Buffer
	require.NoError(t, export.DXF(t.Context(), &out, plate, units.Millimeters(0.1), bottom))
	var got []string
	for _, a := range readDXFArcs(t, out.String()) {
		got = append(got, arcKey(a))
	}
	// Seen from below x runs the other way, so each corner is mirrored and its counter-clockwise
	// arc is the world's clockwise one.
	var want []string
	for _, layer := range []string{export.DXFVisibleLayer, export.DXFHiddenLayer} {
		want = append(want,
			layer+" (-5,5) r5 270>0", layer+" (-35,5) r5 180>270", layer+" (-35,25) r5 90>180", layer+" (-5,25) r5 0>90")
	}
	require.ElementsMatch(t, want, got)
}

func TestDXFViewsPlacesTheArcsByTheirOwnBounds(t *testing.T) {
	t.Parallel()
	plate := roundedPlate(t)
	var out bytes.Buffer
	require.NoError(t, export.DXFViews(t.Context(), &out, plate, units.Millimeters(0.1), []export.DrawingView{topView}, units.Millimeters(5)))
	// The plate is 40 by 30 with its corner at the origin, so the arcs keep their places.
	var got []string
	for _, a := range readDXFArcs(t, out.String()) {
		if a.layer == export.DXFVisibleLayer {
			got = append(got, arcKey(a))
		}
	}
	require.ElementsMatch(t, []string{"VISIBLE (35,5) r5 270>0", "VISIBLE (35,25) r5 0>90", "VISIBLE (5,25) r5 90>180", "VISIBLE (5,5) r5 180>270"}, got)
}

func sheetTexts(t *testing.T, body *decad.Body, paper export.DXFPaper) []string {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), paper))
	var values []string
	for _, tx := range readDXFTexts(t, out.String()) {
		values = append(values, tx.value)
	}
	return values
}

func TestDXFSheetDimensionsACircleAndItsLeader(t *testing.T) {
	t.Parallel()
	rod := circleBodyForDXF(t, decad.New())
	paper := a4Sheet(true)
	paper.Views = []export.DrawingView{topView}
	paper.Title = "ROD"
	// The overall 10 and 10 of the view, and the circle's diameter once: the hidden rim behind it has none.
	require.ElementsMatch(t, []string{"ROD", "SCALE 1:2", "10", "10", "DIA 10"}, sheetTexts(t, rod, paper))
	paper.Dimensions = false
	require.ElementsMatch(t, []string{"ROD", "SCALE 1:2"}, sheetTexts(t, rod, paper))
}

func TestDXFSheetDimensionsTheRadiusOfEachCorner(t *testing.T) {
	t.Parallel()
	plate := roundedPlate(t)
	paper := a4Sheet(true)
	paper.Views = []export.DrawingView{topView}
	paper.Title = "PLATE"
	texts := sheetTexts(t, plate, paper)
	var radii int
	for _, v := range texts {
		if v == "R5" {
			radii++
		}
	}
	require.Equal(t, 4, radii, "one radius for each corner arc: %v", texts)
	require.Contains(t, texts, "40", "the overall width is still stated")
	require.Contains(t, texts, "30")
}
