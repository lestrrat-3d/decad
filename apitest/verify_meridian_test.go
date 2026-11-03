package apitest_test

import (
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
