package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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

// TestVerifyBoxAlgebraRotatedRail turns the rail and its own lap tool by one
// inexact angle about z while the plate and its lap stay square to the axes.
// The rail's box and the rail lap's are one box in the turned frame, so the
// lap holds the rail exactly, and the plate's lap holds the rest; the union of
// a turned tool and a square one covers the shared region. A rail lap that
// stops 0.5 mm short is not decided.
//
// Leg shown to fail: reading only boxes on the reference's axes leaves the turned
// pair with a pair diagnostic.
func TestVerifyBoxAlgebraRotatedRail(t *testing.T) {
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
			turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Radians(0.6))
			require.NoError(t, err)
			turned := func(b *decad.Body) *decad.Body {
				out, err := decad.Transform(t.Context(), b, turn)
				require.NoError(t, err)
				return out
			}
			rail := turned(lapBox(t, doc, -40, -3, 40, 3, -3))
			railLap := lapBox(t, doc, 0, -10, 20, 10, tc.lapBottom)
			plate := lapBox(t, doc, 0, -10, 20, 10, -3)
			plateLap := turned(lapBox(t, doc, -40, -3, 40, 3, -6))
			rail, err = decad.Cut(t.Context(), rail, railLap)
			require.NoError(t, err)
			plate, err = decad.Cut(t.Context(), plate, plateLap)
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, rail, plate)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the turned and square laps cover the shared region")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}
