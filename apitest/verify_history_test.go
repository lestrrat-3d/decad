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

// TestVerifyHistoryFlushPrisms holds the leaf rule for prisms of one exact
// linear lift: a block glued flat against a bored rail, its face on the
// rail's, is decided, also when its placement differs from the rail's by an
// exact offset; one sunk 0.5 mm into the rail is not.
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
		{"block flush on another placement", 10, other, true},
		{"block sunk on another placement", 9.5, other, false},
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

// TestVerifyHistoryAnalyticCut holds the walk to analytic Boolean results: a
// block made by cutting a rail-shaped prism of the rail's own section and
// frame out of a blank sits flush on the bored rail, and the analytic Cut's
// record decides the pair. A blank cut by a prism on another placement is not
// decided.
//
// Leg shown to fail: reading only mesh Boolean results leaves the analytic
// Cut a leaf, and the flush pair keeps a diagnostic.
func TestVerifyHistoryAnalyticCut(t *testing.T) {
	t.Parallel()
	lift := r3.Identity()
	other, err := r3.Translation(r3.Vec{Z: 0.25})
	require.NoError(t, err)
	for _, tc := range []struct {
		name      string
		toolMove  r3.Transform
		separated bool
	}{
		{"block cut by the rail's own section", lift, true},
		{"block cut by a prism on another placement", other, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			rail, err := decad.Cut(t.Context(), slab(t, doc, 0, 0, 40, 10, 5, 10, lift),
				placedAlongZ(t, coaxialSection(t, doc, 20, 5, 30, 2), 1))
			require.NoError(t, err)
			block, err := decad.Cut(t.Context(), slab(t, doc, 10, 5, 30, 15, 5, 10, lift),
				slab(t, doc, 0, 0, 40, 10, 5, 10, tc.toolMove))
			require.NoError(t, err)
			if tc.separated {
				require.NotEqual(t, "decad.facetedPayload", decadtest.PayloadClass(block), "premise: the block's Cut is analytic")
			}
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, rail, block)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the analytic Cut's record decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}

// TestVerifyHistoryPinInItsBore drills a 10 mm cube with a Ø2 tool along z
// from z = −1 to 11 and seats a Ø2 pin in the bore. The pin's circle is the
// tool's circle, centre and radius alike, so the pin lies in the tool once its
// levels do: a pin from z = 2 to 8 is decided, and one that runs out to
// z = 11.5, past the tool, is decided by the coaxial check instead; one
// 0.01 mm off the tool's centre is not.
//
// Leg shown to fail: without the identical-circle reading the seated pin keeps
// its pair diagnostic.
func TestVerifyHistoryPinInItsBore(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		cx, z1    float64
		separated bool
	}{
		{"pin inside the tool", 5, 8, true},
		// The history walk cannot place this pin in the tool, but the
		// coaxial check reads the drilled cube outside the tool's cylinder.
		{"pin past the tool", 5, 11.5, true},
		{"pin off the tool's centre", 5.01, 8, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			cube := slab(t, doc, 0, 0, 10, 10, 0, 10, r3.Identity())
			tool := coaxialSection(t, doc, 5, 5, 12, 1)
			tool = placedAlongZ(t, tool, -1)
			drilled, err := decad.Cut(t.Context(), cube, tool)
			require.NoError(t, err)
			pin := placedAlongZ(t, coaxialSection(t, doc, tc.cx, 5, tc.z1-2, 1), 2)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, drilled, pin)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the pin lies in the tool that drilled its bore")
				return
			}
			require.Positive(t, interferences+diagnostics)
		})
	}
}
