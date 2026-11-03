package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's end-cap
// separator: a Ø2 set screw along x whose seat lies in the plane x = 4,
// tangent to a Ø8 shaft. The seat touches the shaft along a line, so no mesh
// decides the pair, but the seat's plane holds the shaft on its other side.

// xScrew is a Ø2 cylinder along x from x = x0 to x0 + 4 about the line
// y = cy, z = cz.
func xScrew(t *testing.T, doc *decad.Document, x0, cy, cz float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	box, err := body.Bounds()
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: x0 - box.Min.X, Y: cy, Z: cz})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	return placed
}

// TestVerifyEndCapSeparator holds the separator to exact seats: a screw
// seated on a held shaft, or on a revolved spindle away from its collar, is
// decided; one sunk 0.01 mm into either, or seated over the collar, is not.
//
// Leg shown to fail: reading no end cap at a history leaf leaves both seated
// pairs with a diagnostic.
func TestVerifyEndCapSeparator(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		revolved  bool
		x0, at    float64
		separated bool
	}{
		{"screw seated on a held shaft", false, 4, 35, true},
		{"screw sunk into a held shaft", false, 3.99, 35, false},
		{"screw seated on a revolved spindle", true, 4, 20, true},
		{"screw sunk into a revolved spindle", true, 3.99, 20, false},
		{"screw over the spindle's collar", true, 4, 11, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			var shaft, screw *decad.Body
			if tc.revolved {
				shaft = collarSpindle(t, doc)
				screw = xScrew(t, doc, tc.x0, tc.at, 0)
			} else {
				shaft = keyedShaft(t, doc)
				screw = xScrew(t, doc, tc.x0, 0, tc.at)
			}
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, shaft, screw)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the end-cap separator decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}
