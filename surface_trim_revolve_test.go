package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T180, T182 and T183:
// docs/surface-intersection-design.md's PR4, Body.Trim and Body.Extend over
// the revolve family. Every fixture spins its meridian about the sketch
// plane's own v axis, so the two operands resolve one frame and one axis and
// S4 reads them equal on the stored floats.

// trimRevolveAxis is the sketch plane's own v axis, the line every fixture
// below spins about. Two bodies built from it on two sketches of the same
// plane resolve axisFrame's eight generator fields identically, which is what
// S4's revolve arm compares.
func trimRevolveAxis() decad.SketchLine {
	return decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
}

// trimRevolveRing spins the axis-aligned rectangle meridian (u0,v0)-(u1,v1) a
// full turn about that axis. surface selects the sheet — a full turn mints no
// cap to omit, so WithSurfaceResult returns a CLOSED sheet (docs/surface-design.md
// §2.1's closed-sheet rule) — and its absence the solid.
func trimRevolveRing(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64, surface bool) *decad.Body {
	t.Helper()
	return trimRevolveRingAxis(t, doc, u0, v0, u1, v1, surface, trimRevolveAxis())
}

func trimRevolveRingAxis(t *testing.T, doc *decad.Document, u0, v0, u1, v1 float64, surface bool, axis decad.Axis) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var opts []decad.RevolveOption
	if surface {
		opts = append(opts, decad.WithSurfaceResult())
	}
	body, err := doc.Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{}, opts...)
	require.NoError(t, err)
	return body
}

// trimRevolvePartial spins the same rectangle through a partial angle, for
// S6's own span relation.
func trimRevolvePartial(t *testing.T, doc *decad.Document, u0, v0, u1, v1, degrees float64, surface bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var opts []decad.RevolveOption
	if surface {
		opts = append(opts, decad.WithSurfaceResult())
	}
	body, err := doc.Revolve(s, s.Profiles()[0], trimRevolveAxis(),
		decad.AngleExtent{A: units.Degrees(degrees), Dir: decad.Along}, opts...)
	require.NoError(t, err)
	return body
}

// TestSurfaceTrimRevolveKeepOutsideSplitsIntoTwoWalks is T180: the closed
// tube sheet over the meridian r in [3, 5], z in [0, 20] — 352π mm² — trimmed
// KeepOutside by a solid revolve of r in [2, 8], z in [5, 10] about the
// identical axis on the identical frame. The tool cuts both cylinder walls at
// z = 5 and z = 10 and leaves the two annuli whole, so six fragments survive
// and chain into two open meridian walks of three segments each.
func TestSurfaceTrimRevolveKeepOutsideSplitsIntoTwoWalks(t *testing.T) {
	doc := decad.New()
	sheet := trimRevolveRing(t, doc, 3, 0, 5, 20, true)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	sheetArea, err := sheet.Area()
	require.NoError(t, err)
	require.InDelta(t, 352*math.Pi, sheetArea.Value.Base(), 1e-9)

	tool := trimRevolveRing(t, doc, 2, 5, 8, 10, false)
	trimmed, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, trimmed.Kind())
	require.Len(t, trimmed.Lumps(), 2)
	requireLumpFaceCounts(t, trimmed, 3)
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(trimmed)
	require.NoError(t, err)
	require.Len(t, free, 4)

	_, err = trimmed.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	// The area interval must ENCLOSE the analytic answer, and its bound must
	// clear the revolve's own float-summation noise: §7.1's fold is what puts
	// the cut parameters' own rounding into the Pappus moment, and a zero bound
	// there would be the claim that a cut meridian is exact.
	//
	// The threshold is set above a MEASURED floor rather than pinned to the
	// bound itself, which differs between amd64 and arm64 under FMA. Forcing
	// trimRevolve's own sectionDelta to 0 leaves this fixture's area bound at
	// 1.3e-13 mm² — the arithmetic noise a per-walk-then-across-walks
	// boundedAdd carries at zero displacement — while the charged bound
	// measures 3.7e-12 mm², twenty-eight times it. 1e-12 sits between the two
	// with a factor of seven either side.
	//
	// Shown-to-fail: that same forcing turns this assertion red. The ENCLOSURE
	// assertion below does NOT go red under it — the noise bound of 1.3e-13
	// still covers the 1.1e-13 the held sum sits from the analytic answer — so
	// the threshold is the leg that proves the fold is charged, and the
	// enclosure is what proves the charge is not covering a wrong answer.
	area, err := trimmed.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	bound := area.Bound.Base()
	require.Greater(t, bound, 1e-12)
	value := area.Value.Base()
	want := 272 * math.Pi
	require.LessOrEqual(t, value-bound, want)
	require.GreaterOrEqual(t, value+bound, want)

	// The box takes §7.1's FIFTH mechanism, and that term is the WHOLE of its
	// bound here: an untrimmed full revolution about this axis-aligned frame
	// reads Exact at zero. Shown-to-fail: forcing sectionDelta to 0 publishes
	// Exact at a zero bound and turns both assertions red.
	box, err := trimmed.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, box.Exactness)
	require.Positive(t, box.Bound.Base())

	// A revolve sheet's mesh waits on surface increment 4, trimmed or not.
	_, err = trimmed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestSurfaceTrimRevolveRefusalsNameTheirOwnGate is T182: three revolve pairs
// this evaluator refuses, each read by MESSAGE TEXT so the refusal is pinned
// to its own gate rather than only to the sentinel every one of them shares.
func TestSurfaceTrimRevolveRefusalsNameTheirOwnGate(t *testing.T) {
	// Shown-to-fail for both S4 sub-cases: replacing revolveAxisIdentical's
	// stored-float comparison with an angle comparison admits BOTH pairs, and
	// no later gate refuses either. The parallel-axis pair then returns a body
	// whose area reads 854.513 mm² — the answer for a genuinely co-axial tool —
	// for a tool that is not co-axial at all, so the loosened gate publishes a
	// wrong answer rather than a wider bound. That is what makes the arm
	// load-bearing: the one-ulp pair alone would not show it, since a nudge
	// that small sits under every bound this fixture publishes.
	t.Run("S4 refuses an axis one ulp away", func(t *testing.T) {
		doc := decad.New()
		sheet := trimRevolveRing(t, doc, 3, 0, 5, 20, true)
		nudged := decad.SketchLine{
			Start: decad.Point2{U: math.Nextafter(0, 1), V: 0},
			End:   decad.Point2{U: math.Nextafter(0, 1), V: 1},
		}
		tool := trimRevolveRingAxis(t, doc, 2, 5, 8, 10, false, nudged)
		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "do not spin about the same axis, exactly")
		require.Len(t, doc.Bodies(), 2)
	})

	t.Run("S4 refuses a parallel axis two millimetres away", func(t *testing.T) {
		doc := decad.New()
		sheet := trimRevolveRing(t, doc, 3, 0, 5, 20, true)
		// Same DIRECTION, different ANCHOR — the case an angle comparison
		// cannot see at all, and the reason S4 compares the anchor's own two
		// fields beside the direction's.
		shifted := decad.SketchLine{Start: decad.Point2{U: -2, V: 0}, End: decad.Point2{U: -2, V: 1}}
		tool := trimRevolveRingAxis(t, doc, 2, 5, 8, 10, false, shifted)
		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "do not spin about the same axis, exactly")
		require.Len(t, doc.Bodies(), 2)
	})

	t.Run("S6 refuses a tool that stops inside the angular span", func(t *testing.T) {
		doc := decad.New()
		sheet := trimRevolveRing(t, doc, 3, 0, 5, 20, true)
		tool := trimRevolvePartial(t, doc, 2, 5, 8, 10, 90, false)
		_, err := sheet.Trim(t.Context(), tool, decad.KeepOutside)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "does not cover the receiver's")
		require.Len(t, doc.Bodies(), 2)
	})

	t.Run("RS13 refuses Split over the revolve family", func(t *testing.T) {
		doc := decad.New()
		target := trimRevolveRing(t, doc, 3, 0, 5, 20, false)
		tool := trimRevolveRing(t, doc, 2, 5, 8, 10, true)
		_, err := doc.Split(t.Context(), target, tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "Split over the revolve family")
		require.Len(t, doc.Bodies(), 2)
	})
}

// extendRevolveRibbon builds T183's receiver: the line meridian from (3, 0) to
// (3, 100) cut at v = 40 by a crossing line, its lower fragment spun about the
// same v axis every fixture above spins about. The recorded range is TStart 0,
// TEnd 0.4 — NARROWER than the line's own natural domain, which is the one
// exception S7 admits, and the only shape Extend has anything to lengthen.
func extendRevolveRibbon(t *testing.T, doc *decad.Document, a decad.AngularExtent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	anchor := s.CreatePoint(3, 0)
	s.Fix(anchor)
	line := s.CreateLine(anchor, s.CreatePoint(3, 100))
	cut := s.CreatePoint(0, 40)
	s.Fix(cut)
	s.CreateLine(cut, s.CreatePoint(10, 40))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, chain := range s.Chains() {
		if len(chain.Edges) != 1 || chain.Edges[0].Entity != line {
			continue
		}
		record, _, err := decad.RecordChain(s, chain)
		require.NoError(t, err)
		require.Len(t, record.Segments, 1)
		seg, ok := record.Segments[0].(decad.LineSeg)
		require.True(t, ok)
		if seg.TStart != 0 {
			continue // the upper fragment, anchored at the line's far end
		}
		require.InDelta(t, 0.4, seg.TEnd, 1e-12)
		body, err := doc.RevolveChain(s, chain, trimRevolveAxis(), a)
		require.NoError(t, err)
		return body
	}
	t.Fatal("sketch published no lower fragment of the source meridian")
	return nil
}

// extendRevolveFreeRim returns a full-turn revolve ribbon's two free latitude
// circles and the index of the one standing at axial level v. The meridian's
// two free ends sweep exactly those two circles under a full revolution
// (docs/surface-design.md Table G), so naming one names a meridian end.
func extendRevolveFreeRim(t *testing.T, b *decad.Body, v float64) ([]*decad.Edge, int) {
	t.Helper()
	edges, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(b)
	require.NoError(t, err)
	at := -1
	for i, e := range edges {
		if math.Abs(e.Start().Position().Value.Y-v) > 1e-9 {
			continue
		}
		require.Equal(t, -1, at, "two free rims stand at v = %v", v)
		at = i
	}
	require.NotEqual(t, -1, at, "no free rim stands at v = %v", v)
	return edges, at
}

// TestSurfaceExtendRevolveRibbonToNearestCut is T183: the radius-3 cylinder
// band over v ∈ [0, 40], lengthened along its own meridian carrier until it
// meets a co-axial revolve solid whose meridian rectangle spans r ∈ [2, 8],
// v ∈ [60, 80]. The tool crosses the carrier twice; the nearer crossing at
// v = 60 wins, and the band becomes v ∈ [0, 60] — 360π mm².
func TestSurfaceExtendRevolveRibbonToNearestCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ribbon := extendRevolveRibbon(t, doc, decad.FullRevolution{})
	require.Equal(t, decad.BodySheet, ribbon.Kind())
	before, err := ribbon.Area()
	require.NoError(t, err)
	require.InDelta(t, 240*math.Pi, before.Value.Base(), 1e-9)

	rims, moved := extendRevolveFreeRim(t, ribbon, 40)
	untouchedBefore := rims[1-moved].Start().Position().Value
	query := decad.Edges(decad.Free(),
		decad.EndpointAt(rims[moved].Start().Position().Value)).Exactly(1)

	tool := trimRevolveRing(t, doc, 2, 60, 8, 80, false)
	result, err := ribbon.Extend(t.Context(), query, tool)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, result.Kind())

	// The area interval must ENCLOSE the analytic answer, and its bound must
	// clear this fixture's own float-summation noise: §7.1's fold is what puts
	// the cut parameter's rounding into the Pappus moment, and a zero bound
	// there would be the claim that a lengthened meridian is exact.
	//
	// The threshold sits above a MEASURED floor rather than being pinned to the
	// bound itself, which differs between amd64 and arm64 under FMA. Forcing
	// the result's sectionDelta to 0 leaves the area bound at 2.06e-13 mm²,
	// while the charged bound measures 3.55e-12 mm² — seventeen times it. 1e-12
	// sits between the two with a factor of five either side. Shown-to-fail:
	// that same forcing turns this assertion red.
	area, err := result.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Greater(t, area.Bound.Base(), 1e-12,
		"the extended area bound must carry the cut parameter's own displacement")
	require.LessOrEqual(t, math.Abs(area.Value.Base()-360*math.Pi), area.Bound.Base(),
		"the area interval must enclose the 360π mm² band")

	// The box takes §7.1's FIFTH mechanism. Measured floor: 7.11e-15 mm with
	// the charge forced off against 3.62e-13 mm with it, fifty-one times.
	box, err := result.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, box.Exactness)
	require.Greater(t, box.Bound.Base(), 1e-13,
		"the extended box bound must carry the cut endpoint's own displacement")
	require.InDelta(t, 60, box.Max.Y, box.Bound.Base())
	require.InDelta(t, 0, box.Min.Y, box.Bound.Base())

	// The v = 0 end is the one the extension never named. It is read by
	// ELIMINATION against the end that moved, never by a selector clause keyed
	// on the untouched coordinate, so a nudge of it reaches the byte comparison
	// instead of being filtered out before it.
	after, movedAfter := extendRevolveFreeRim(t, result, 60)
	untouchedAfter := after[1-movedAfter].Start().Position().Value
	require.Equal(t, math.Float64bits(untouchedBefore.X), math.Float64bits(untouchedAfter.X))
	require.Equal(t, math.Float64bits(untouchedBefore.Y), math.Float64bits(untouchedAfter.Y))
	require.Equal(t, math.Float64bits(untouchedBefore.Z), math.Float64bits(untouchedAfter.Z))
	t.Logf("extended area %g ± %g mm², box y [%g, %g] ± %g mm",
		area.Value.Base(), area.Bound.Base(), box.Min.Y, box.Max.Y, box.Bound.Base())
}

// TestSurfaceExtendRevolveRefusalsNameTheirOwnGate is T183's refusal half:
// four revolve pairs Extend declines, each read by MESSAGE TEXT so the refusal
// is pinned to its own gate rather than only to the sentinel they share.
func TestSurfaceExtendRevolveRefusalsNameTheirOwnGate(t *testing.T) {
	t.Parallel()
	t.Run("RS4 refuses a tool with no cut past the named end", func(t *testing.T) {
		doc := decad.New()
		ribbon := extendRevolveRibbon(t, doc, decad.FullRevolution{})
		rims, moved := extendRevolveFreeRim(t, ribbon, 40)
		query := decad.Edges(decad.Free(),
			decad.EndpointAt(rims[moved].Start().Position().Value)).Exactly(1)
		// v ∈ [10, 20] lies INSIDE the band's own covered range, so its two
		// crossings sit before the named end rather than past it.
		tool := trimRevolveRing(t, doc, 2, 10, 8, 20, false)
		before := doc.Bodies()
		_, err := ribbon.Extend(t.Context(), query, tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "carrier's own natural domain")
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("a partial revolution refuses by name", func(t *testing.T) {
		doc := decad.New()
		ribbon := extendRevolveRibbon(t, doc,
			decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		tool := trimRevolvePartial(t, doc, 2, 60, 8, 80, 90, false)
		_, err := ribbon.Extend(t.Context(), decad.Edges(decad.Free()), tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "admits a full revolution alone")
		require.Len(t, doc.Bodies(), 2)
	})

	t.Run("S4 refuses a parallel axis two millimetres away", func(t *testing.T) {
		doc := decad.New()
		ribbon := extendRevolveRibbon(t, doc, decad.FullRevolution{})
		rims, moved := extendRevolveFreeRim(t, ribbon, 40)
		query := decad.Edges(decad.Free(),
			decad.EndpointAt(rims[moved].Start().Position().Value)).Exactly(1)
		shifted := decad.SketchLine{Start: decad.Point2{U: -2, V: 0}, End: decad.Point2{U: -2, V: 1}}
		tool := trimRevolveRingAxis(t, doc, 2, 60, 8, 80, false, shifted)
		_, err := ribbon.Extend(t.Context(), query, tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "do not spin about the same axis, exactly")
		require.Len(t, doc.Bodies(), 2)
	})

	t.Run("a closed meridian has no free end to lengthen", func(t *testing.T) {
		doc := decad.New()
		sheet := trimRevolveRing(t, doc, 3, 0, 5, 20, true)
		tool := trimRevolveRing(t, doc, 2, 60, 8, 80, false)
		// Every edge, not the free ones: a closed tube sheet has no free edge
		// at all, so a Free() clause would refuse at the SELECTOR and never
		// reach the gate this case is about.
		_, err := sheet.Extend(t.Context(), decad.Edges(), tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "open meridian")
		require.Len(t, doc.Bodies(), 2)
	})
}
