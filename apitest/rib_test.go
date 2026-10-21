package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/rib-design.md's public-surface tests for Rib and
// Web. The part is a 40 x 40 x 20 block with a 30 x 30 pocket cut 15 deep from
// its top, so the pocket walls stand at x, y = 5 and 35 and its floor at z = 5.

const (
	ribBlockVolume  = 40 * 40 * 20
	ribPocketVolume = 30 * 30 * 15
	ribPartVolume   = ribBlockVolume - ribPocketVolume
)

// ribPart returns the pocketed block.
func ribPart(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	ctx := tb.Context()
	doc := decad.New()
	block := decadtest.NewBlock(tb, doc, 0, 0, 40, 40, units.Millimeters(20))
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	r := s.CreateRectangle(5, 5, 35, 35)
	s.Fix(r.A)
	_, err = s.Solve(ctx)
	require.NoError(tb, err)
	pocket, err := decad.Extrude(ctx, doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
	require.NoError(tb, err)
	part, err := decad.Cut(ctx, block, pocket)
	require.NoError(tb, err)
	decadtest.MeasuresVolume(tb, part, units.CubicMillimeters(ribPartVolume), decadtest.Exactly())
	return doc, part
}

// ribSketch returns a sketch on the plane z = 4, below the pocket floor, and
// the lines (x0, y0)-(x1, y1) drawn in it.
func ribSketch(tb testing.TB, segments ...[4]float64) (*sketch.Sketch, []*sketch.Line) {
	tb.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 4)
	require.NoError(tb, err)
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	var lines []*sketch.Line
	for _, seg := range segments {
		a := s.CreatePoint(seg[0], seg[1])
		b := s.CreatePoint(seg[2], seg[3])
		s.Fix(a)
		s.Fix(b)
		lines = append(lines, s.CreateLine(a, b))
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	return s, lines
}

func TestRibAcrossPocket(t *testing.T) {
	t.Parallel()
	doc, part := ribPart(t)
	s, lines := ribSketch(t, [4]float64{3, 20, 37, 20})
	out, err := decad.Rib(t.Context(), part, s, lines[0], units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the part and the rib tool leave the document")
	// The rib stands from the floor at z = 5 to z = 15 between the pocket
	// walls x = 5 and 35: 30 x 2 x 10 of new material.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+30*2*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 40, Y: 40, Z: 20})
}

func TestWebCrossesRibs(t *testing.T) {
	t.Parallel()
	doc, part := ribPart(t)
	s, lines := ribSketch(t, [4]float64{3, 20, 37, 20}, [4]float64{20, 3, 20, 37})
	out, err := decad.Web(t.Context(), part, s, lines, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	// Two 30 x 2 x 10 ribs share a 2 x 2 x 10 crossing.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+2*30*2*10-2*2*10))
}

func TestRibOverrunBuriesTheEnds(t *testing.T) {
	t.Parallel()
	_, part := ribPart(t)
	// The line stops exactly at the pocket walls, so the rib's end faces lie in
	// the wall faces; an overrun of 1 mm buries them.
	s, lines := ribSketch(t, [4]float64{5, 20, 35, 20})
	out, err := decad.Rib(t.Context(), part, s, lines[0], units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along}, decad.WithRibOverrun(units.Millimeters(1)))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+30*2*10))
}

func TestRibRefusals(t *testing.T) {
	t.Parallel()
	good := [4]float64{3, 20, 37, 20}
	thick := units.Millimeters(2)
	e := decad.Distance{D: units.Millimeters(11), Dir: decad.Along}
	cases := []struct {
		name string
		seg  [4]float64
		th   units.Value
		opts []decad.RibOption
		want error
		text string
	}{
		{"zero thickness", good, units.Millimeters(0), nil, decad.ErrDegenerate, ""},
		{"thickness of the wrong kind", good, units.Degrees(2), nil, decad.ErrUnitKind, ""},
		{"negative overrun", good, thick, []decad.RibOption{decad.WithRibOverrun(units.Millimeters(-1))}, decad.ErrNegativeMagnitude, ""},
		{"line without length", [4]float64{20, 20, 20, 20}, thick, nil, decad.ErrDegenerate, ""},
		{"rib beside the body", [4]float64{60, 20, 80, 20}, thick, nil, decad.ErrDegenerate, "would not attach"},
		{"rib inside the material", [4]float64{1, 20, 4, 20}, thick, nil, decad.ErrDegenerate, "adds no material"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, part := ribPart(t)
			s, lines := ribSketch(t, tc.seg)
			_, err := decad.Rib(t.Context(), part, s, lines[0], tc.th, e, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, []*decad.Body{part}, doc.Bodies(), "a refused rib leaves the part live and alone")
		})
	}
}

func TestRibRefusesForeignLine(t *testing.T) {
	t.Parallel()
	doc, part := ribPart(t)
	s, _ := ribSketch(t, [4]float64{3, 20, 37, 20})
	_, other := ribSketch(t, [4]float64{3, 10, 37, 10})
	_, err := decad.Rib(t.Context(), part, s, other[0], units.Millimeters(2), decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Web(t.Context(), part, s, nil, units.Millimeters(2), decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, []*decad.Body{part}, doc.Bodies())
}
