package apitest_test

import (
	"math"
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
// on one placement or another stock of the same frame; a rail taller than the
// stock or on another placement is not. A rail longer than the stock is decided
// too, by the box algebra: its overhang lies outside the leg's block, so only
// the part inside the stock's box meets the leg.
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
		{"rail past the stock's end, clear of the leg", -8, 40, 5, 10, lift, true},
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

// hookRing revolves a Ø1.6 circle three quarters of a turn about the z axis
// line x = 10, y = 0, its centre 1.5 from the axis at height z.
func hookRing(t *testing.T, doc *decad.Document, z float64) *decad.Body {
	t.Helper()
	return hookRingSweep(t, doc, z, 270)
}

func hookRingSweep(t *testing.T, doc *decad.Document, z, degrees float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(11.5, z)
	s.Fix(c)
	s.CreateCircle(c, 0.8)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ring, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: 10}, End: decad.Point2{U: 10, V: 1}},
		decad.AngleExtent{A: units.Degrees(degrees), Dir: decad.Along})
	require.NoError(t, err)
	return ring
}

// TestVerifyHistoryFarOperand seats the Ø2 stem of a hook in a bore of a
// 20×10×10 block: the stem lies in the Ø2 tool that drilled the bore from
// z = −1 to 16. The stem is unioned with a ring at its top whose box lies
// above the block's. No separator reads the ring against the block, but the
// Union misses the block when every operand does: the stem through the
// drilling tool, the ring because its box is clear of the block's. A ring
// sunk to z = 9 meets the block and is not decided.
//
// Leg shown to fail: without the box reading the far ring keeps the hook's
// pair diagnostic.
func TestVerifyHistoryFarOperand(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		ringZ     float64
		separated bool
	}{
		{"ring above the block", 15.5, true},
		{"ring sunk into the block", 9.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			block := slab(t, doc, 0, -5, 20, 5, 0, 10, r3.Identity())
			tool := placedAlongZ(t, coaxialSection(t, doc, 10, 0, 17, 1), -1)
			drilled, err := decad.Cut(t.Context(), block, tool)
			require.NoError(t, err)
			stem := placedAlongZ(t, coaxialSection(t, doc, 10, 0, 14, 1), 2)
			hook, err := decad.Union(t.Context(), stem, hookRing(t, doc, tc.ringZ))
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, drilled, hook)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "every operand of the hook misses the block")
				return
			}
			require.Positive(t, interferences+diagnostics)
		})
	}
}

// TestVerifyHistoryTurnedFarOperand is TestVerifyHistoryFarOperand turned 45°
// about the x axis: block, drilling tool, stem and ring alike. The ring then
// stands clear of the block's top face in the block's own frame, but the
// world boxes of the two overlap, so only the block's frame planes separate
// them. A ring sunk into the block is not decided.
//
// Leg shown to fail: reading only world boxes leaves the turned hook with a
// pair diagnostic.
func TestVerifyHistoryTurnedFarOperand(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		ringZ     float64
		separated bool
	}{
		{"ring above the block", 15.5, true},
		{"ring sunk into the block", 9.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			turn := func(b *decad.Body) *decad.Body { return turned(t, b, math.Pi/4) }
			block := turn(slab(t, doc, 0, -5, 20, 5, 0, 10, r3.Identity()))
			tool := turn(placedAlongZ(t, coaxialSection(t, doc, 10, 0, 17, 1), -1))
			drilled, err := decad.Cut(t.Context(), block, tool)
			require.NoError(t, err)
			stem := turn(placedAlongZ(t, coaxialSection(t, doc, 10, 0, 14, 1), 2))
			hook, err := decad.Union(t.Context(), stem, turn(hookRing(t, doc, tc.ringZ)))
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, drilled, hook)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the block's top face plane holds the ring off")
				return
			}
			require.Positive(t, interferences+diagnostics)
		})
	}
}

// TestVerifyHistoryRefinedOperand refines the hook of
// TestVerifyHistoryFarOperand before Verify. Refine moves no vertex and only
// tightens the bound its mesh publishes, so the refined hook is the same
// solid as its Union and the history walk reads it as that Union: the stem
// through its drilling tool and the ring by its box.
//
// Leg shown to fail: without reading a Refine as its source the refined
// hook is a leaf no rule separates.
func TestVerifyHistoryRefinedOperand(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := slab(t, doc, 0, -5, 20, 5, 0, 10, r3.Identity())
	tool := placedAlongZ(t, coaxialSection(t, doc, 10, 0, 17, 1), -1)
	drilled, err := decad.Cut(t.Context(), block, tool)
	require.NoError(t, err)
	stem := placedAlongZ(t, coaxialSection(t, doc, 10, 0, 14, 1), 2)
	hook, err := decad.Union(t.Context(), stem, hookRing(t, doc, 15.5))
	require.NoError(t, err)
	refined, err := decad.Refine(t.Context(), hook, units.Millimeters(0.05))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	interferences, diagnostics := pairFindings(report, drilled, refined)
	require.Zero(t, interferences)
	require.Zero(t, diagnostics, "the refined hook is read as its Union")
}
