package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's box algebra
// separator: a rail along x and a plate across it, half-lapped. Each is cut
// where the other stays, so they meet only on the plane z = 0, which no held
// mesh decides, and the intersection of their stocks lies in the union of the
// two laps.

// lapBox is the box x in [x0, x1], y in [y0, y1] and z in [z0, z0 + 6].
func lapBox(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	s.CreateRectangle(x0, y0, x1, y1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestVerifyBoxAlgebra holds the separator to the laps' union: laps that meet
// on z = 0 decide the pair; a rail lap that stops 0.5 mm short of the plate's
// plane leaves a sliver of rail inside the plate, and is not decided.
//
// Leg shown to fail: reading no tool boxes leaves the flush pair with a pair
// diagnostic.
func TestVerifyBoxAlgebra(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		lapBottom float64
		separated bool
	}{
		{"laps meet on z = 0", 0, true},
		{"rail lap 0.5 mm short", 0.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rail := lapBox(t, doc, -40, -3, 40, 3, -3)
			plate := lapBox(t, doc, 0, -10, 20, 10, -3)
			railLap := lapBox(t, doc, 0, -10, 20, 10, tc.lapBottom)
			plateLap := lapBox(t, doc, -40, -3, 40, 3, -6)
			rail, err := decad.Cut(t.Context(), rail, railLap)
			require.NoError(t, err)
			plate, err = decad.Cut(t.Context(), plate, plateLap)
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, rail, plate)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the laps' union holds the stocks' intersection")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}
