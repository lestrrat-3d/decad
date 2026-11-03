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

// These fixtures pin agent-docs/interference-design.md §3.2's history
// certificate: a rail glued into the socket its own longer stock cut through
// a bored leg. The rail's faces coincide with the socket's, and the leg's bore
// keeps the exact planar arm out, so only the history decides the pair.

// slab extrudes the rectangle (x0, y0)-(x1, y1) on the XY plane offset to z0
// by height, then moves it by move.
func slab(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, height float64, move r3.Transform) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	return placed
}

// boredLeg is a 40×20×20 block with a Ø6 bore along z at (30, 10).
func boredLeg(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	c := s.CreatePoint(30, 10)
	s.Fix(c)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	leg, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return leg
}

// TestVerifyHistorySeparator holds the history certificate to containment
// the records prove: a rail inside the stock that cut its socket is decided,
// on one placement or another stock of the same frame; a rail longer than the
// stock, taller than it, or on another placement is not.
//
// Leg shown to fail: dropping the history check leaves the flush rail with a
// pair diagnostic.
func TestVerifyHistorySeparator(t *testing.T) {
	t.Parallel()
	lift, err := r3.Translation(r3.Vec{Z: 0.001})
	require.NoError(t, err)
	other, err := r3.Translation(r3.Vec{Z: 0.002})
	require.NoError(t, err)
	for _, tc := range []struct {
		name       string
		x0, x1     float64
		z0, height float64
		move       r3.Transform
		separated  bool
	}{
		{"rail inside the stock", 0, 40, 5, 10, lift, true},
		{"rail shorter than the stock", 10, 30, 5, 10, lift, true},
		{"rail past the stock's end", -8, 40, 5, 10, lift, false},
		{"rail taller than the stock", 0, 40, 4, 12, lift, false},
		{"rail on another placement", 0, 40, 5, 10, other, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stock := slab(t, doc, -5, 5, 45, 12, 5, 10, lift)
			leg, err := decad.Cut(t.Context(), boredLeg(t, doc), stock)
			require.NoError(t, err)
			require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(leg))
			rail := slab(t, doc, tc.x0, 5, tc.x1, 12, tc.z0, tc.height, tc.move)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, leg, rail)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the history decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}

// TestVerifyHistoryFlushPrisms holds the leaf rule for prisms of one frame
// and placement: a block glued flat against a bored rail, its face on the
// rail's, is decided; one sunk 0.5 mm into the rail, or on another
// placement, is not.
//
// Leg shown to fail: reading only containment leaves the flush block with a
// pair diagnostic.
func TestVerifyHistoryFlushPrisms(t *testing.T) {
	t.Parallel()
	lift, err := r3.Translation(r3.Vec{Z: 0.001})
	require.NoError(t, err)
	other, err := r3.Translation(r3.Vec{Z: 0.002})
	require.NoError(t, err)
	for _, tc := range []struct {
		name      string
		y0        float64
		move      r3.Transform
		separated bool
	}{
		{"block flush on the rail", 10, lift, true},
		{"block sunk into the rail", 9.5, lift, false},
		{"block on another placement", 10, other, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rail, err := decad.Cut(t.Context(), slab(t, doc, 0, 0, 40, 10, 5, 10, lift),
				placedAlongZ(t, coaxialSection(t, doc, 20, 5, 30, 2), 1))
			require.NoError(t, err)
			require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(rail))
			block := slab(t, doc, 10, tc.y0, 30, 15, 5, 10, tc.move)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, rail, block)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the history decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}
