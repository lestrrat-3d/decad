package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's meridian
// separator: a turned spindle with a collar, revolved about the world y axis,
// and a bored bearing on that axis that sits flush on the collar with its bore
// flush on the spindle. Two faces coincide, so no mesh decides the pair, but
// the two meridian regions touch only along their edges.

// collarSpindle revolves a Ø8 spindle along y from 0 to 40 with a Ø12 collar
// from y = 10 to 12, drawn in (x, y) and turned about the y axis.
func collarSpindle(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{{0, 0}, {4, 0}, {4, 10}, {6, 10}, {6, 12}, {4, 12}, {4, 40}, {0, 40}})
	body, err := decad.Revolve(t.Context(), doc, s, profile,
		decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// yBearing is an 8 mm annulus of outer radius 11 and bore radius bore about
// the line x = cx, z = 0, along y from y0.
func yBearing(t *testing.T, doc *decad.Document, cx, y0, bore float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	for _, r := range []float64{11, bore} {
		c := s.CreatePoint(cx, 0)
		s.Fix(c)
		s.CreateCircle(c, r)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	box, err := body.Bounds()
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Y: y0 - box.Min.Y})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	return placed
}

// TestVerifyMeridianSeparator holds the separator to exact meridians: a
// bearing flush on the collar and on the spindle is decided; one sunk 0.5 mm
// into the collar, bored smaller than the spindle, or 0.01 mm off its axis is
// not.
//
// Leg shown to fail: reading no meridian at a history leaf leaves the flush
// pair with a diagnostic.
func TestVerifyMeridianSeparator(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		cx, y0    float64
		bore      float64
		separated bool
	}{
		{"bearing flush on the collar", 0, 12, 4, true},
		{"bearing sunk into the collar", 0, 11.5, 4, false},
		{"bore smaller than the spindle", 0, 12, 3.9, false},
		{"bearing off the spindle's axis", 0.01, 12, 4, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			spindle := collarSpindle(t, doc)
			bearing := yBearing(t, doc, tc.cx, tc.y0, tc.bore)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, spindle, bearing)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the meridian separator decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}

// bandWrap revolves a Ø1 circle at (cu, 0) half a turn about the y axis: a
// band wrapped round a pulley.
func bandWrap(t *testing.T, doc *decad.Document, cu float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(cu, 0)
	s.Fix(c)
	s.CreateCircle(c, 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.SketchLine{End: decad.Point2{V: 1}}, decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestVerifyMeridianBandInGroove seats a half-turn band of Ø1 wire in the
// flat-bottomed groove of a revolved pulley: the groove floor at ρ = 10, its
// walls at y = ±1. The band's circle rests on the floor, so the two bodies
// touch along a circle no held mesh decides; a band sunk 0.1 mm into the floor
// is not separated.
//
// Leg shown to fail: reading only full revolutions of whole lines leaves the
// resting band with a pair diagnostic.
func TestVerifyMeridianBandInGroove(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		cu        float64
		separated bool
	}{
		{"band resting on the floor", 10.5, true},
		{"band sunk into the floor", 10.4, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, profile := polygonSketch(t, [][2]float64{{0, -3}, {12, -3}, {12, -1}, {10, -1}, {10, 1}, {12, 1}, {12, 3}, {0, 3}})
			pulley, err := decad.Revolve(t.Context(), doc, s, profile,
				decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
			require.NoError(t, err)
			band := bandWrap(t, doc, tc.cu)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, pulley, band)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the meridian separator decides the band")
				return
			}
			require.Positive(t, interferences+diagnostics)
		})
	}
}

// TestVerifyMeridianSharedPlacement turns a collared spindle, revolved in the
// XY plane, and a bored bearing, extruded from the XZ plane, by one inexact
// rotation about x. Their frames differ, and the rotation leaves neither lift
// exactly orthonormal, but the one placement they share carries both, so
// they are compared before it: the bearing sitting flush under the collar,
// its bore flush on the shank, is decided. TestCommonAxesSharedPlacement
// holds the comparison to the placement being one bit for bit.
//
// Leg shown to fail: comparing only equal lifts or orthonormal ones leaves
// the turned pair with a diagnostic.
func TestVerifyMeridianSharedPlacement(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		bore      float64
		separated bool
	}{
		{"bearing flush under the collar", 4, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, profile := polygonSketch(t, [][2]float64{{0, -10}, {4, -10}, {4, 0}, {6, 0}, {6, 2}, {4, 2}, {4, 30}, {0, 30}})
			spindle, err := decad.Revolve(t.Context(), doc, s, profile,
				decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
			require.NoError(t, err)
			w := sketch.NewWorld()
			bs, err := w.CreateSketch(w.XZ())
			require.NoError(t, err)
			for _, r := range []float64{11, tc.bore} {
				c := bs.CreatePoint(0, 0)
				bs.Fix(c)
				bs.CreateCircle(c, r)
			}
			_, err = bs.Solve(t.Context())
			require.NoError(t, err)
			bearing, err := decad.Extrude(t.Context(), doc, bs, bs.Profiles()[0],
				decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
			require.NoError(t, err)
			spindle, bearing = turned(t, spindle, math.Pi/3), turned(t, bearing, math.Pi/3)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, spindle, bearing)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the shared placement carries both meridians")
				return
			}
			require.Positive(t, interferences+diagnostics)
		})
	}
}
