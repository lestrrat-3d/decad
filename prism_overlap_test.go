package decad_test

import (
	"errors"
	"math"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §4.5's own required-tests suite
// (§15's multi-region rows) and docs/interference-design.md §10.2's
// multi-region rows, exercised end to end through Document.Verify and the
// public booleans. The internal admission/decline mechanics (the exactly-
// tangent decline, the arrangement cap, cancellation, and the G1-G4-style
// regression fallbacks) are prism_overlap_internal_test.go's own, white-box
// suite.
//
// polyPrismBody's U-and-bar fixture reproduces .tmp/multilump/main.go's own
// measured case: a U-shaped prism (a rectangle with a rectangular notch cut
// from one side, drawn as one outer loop, no holes) crossed by a bar so the
// two outlines overlap in exactly two disjoint 12 mm² regions over a 5 mm
// sweep — true overlap volume 2*12*5 = 120 mm^3.

// polyPrismBody extrudes a closed polygon (drawn as one outer loop, no
// holes) from z=0 to z=h, with every vertex pinned exactly at its authored
// coordinate.
func polyPrismBody(t *testing.T, doc *decad.Document, pts [][2]float64, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// uShapePts and barPts are .tmp/multilump/main.go's own case B: the U
// straddles the bar's ends, so the two outlines overlap in exactly two
// disjoint 12 mm² regions.
var (
	uShapePts = [][2]float64{
		{0, 0}, {4, 0}, {4, 8}, {6, 8}, {6, 0}, {10, 0}, {10, 12}, {0, 12},
	}
	barPts = [][2]float64{{-2, 2}, {12, 2}, {12, 5}, {-2, 5}}
)

// uAndBarSweep is every fixture's own sweep height in this file, matching
// .tmp/multilump/main.go's own case B.
const uAndBarSweep = 5.0

// uAndBarBodies builds the U-and-bar fixture into doc, both swept
// uAndBarSweep.
func uAndBarBodies(t *testing.T, doc *decad.Document) (u, bar *decad.Body) {
	t.Helper()
	u = polyPrismBody(t, doc, uShapePts, uAndBarSweep)
	bar = polyPrismBody(t, doc, barPts, uAndBarSweep)
	return u, bar
}

// TestVerifyMultiRegionOverlapReportsSummedVolume is §4.5's own headline test
// (§15's "the headline" row, docs/interference-design.md §10.2's multi-region
// row): a U-shaped prism crossed by a bar, whose overlap covers two disjoint
// regions, now reports one Interference row rather than falling to the mesh
// path's coplanar refusal.
func TestVerifyMultiRegionOverlapReportsSummedVolume(t *testing.T) {
	doc := decad.New()
	u, bar := uAndBarBodies(t, doc)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)

	row := report.Interferences[0]
	require.Same(t, u, row.A)
	require.Same(t, bar, row.B)
	require.InDelta(t, 120.0, row.Volume.Value.Base(), 1e-9)
	require.Equal(t, decad.Approximate, row.Volume.Exactness)
	// §7's own reading: the bound's whole content is the cut parameters' own
	// rounding (delta_cut alone), which is orders of magnitude below a
	// chord-derived term (~1e-2 mm^3 on a curved operand) and close to the
	// single-region analogue's own measured 3.55e-12 mm^3.
	require.Less(t, row.Volume.Bound.Base(), 1e-9)
	require.Greater(t, row.Volume.Value.Base()-row.Volume.Bound.Base(), 0.0,
		"§6's positive-volume gate must still hold")

	// The mesh path's own coplanar refusal is what a regression back to it
	// would emit; its absence here is the proof the analytic multi-region
	// reading answered (§4.5's own "what proves the analytic path answered").
	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagUnsupportedPairContact, d.Code,
			`a published row on a coplanar pair can only have come from the analytic reading`)
		require.NotEqual(t, decad.DiagUnsupportedPair, d.Code)
	}
	requireDocumentUnchanged(t, doc, before)
}

// exactRectOverlapArea is the exact rational area shared by two axis-aligned
// rectangles [x0,y0,x1,y1], each edge taken exactly over the operands' own
// recorded float coordinates — never a second float computation.
func exactRectOverlapArea(a, b [4]float64) *big.Rat {
	ratOf := func(f float64) *big.Rat {
		r := new(big.Rat)
		if r.SetFloat64(f) == nil {
			panic("non-finite coordinate in a test fixture")
		}
		return r
	}
	span := func(alo, ahi, blo, bhi float64) *big.Rat {
		lo, hi := ratOf(alo), ratOf(ahi)
		if ratOf(blo).Cmp(lo) > 0 {
			lo = ratOf(blo)
		}
		if ratOf(bhi).Cmp(hi) < 0 {
			hi = ratOf(bhi)
		}
		d := new(big.Rat).Sub(hi, lo)
		if d.Sign() < 0 {
			return new(big.Rat)
		}
		return d
	}
	return new(big.Rat).Mul(
		span(a[0], a[2], b[0], b[2]),
		span(a[1], a[3], b[1], b[3]),
	)
}

// TestVerifyMultiRegionOverlapSumMatchesIndependentRationalAnswer is §15's
// "cell-sum soundness against an independent answer" row: the U shape
// decomposes, by hand, into three axis-aligned rectangles — left [0,4]x[0,12],
// right [6,10]x[0,12], and the bridge over the notch [4,6]x[8,12] — a
// decomposition read directly off the polygon's own authored vertices, never
// off anything decad computed. The true overlap with the bar [-2,12]x[2,5] is
// the exact rational sum of each rectangle's own overlap with the bar,
// computed over math/big.Rat from the fixture's own float coordinates.
func TestVerifyMultiRegionOverlapSumMatchesIndependentRationalAnswer(t *testing.T) {
	doc := decad.New()
	uAndBarBodies(t, doc)

	left := [4]float64{0, 0, 4, 12}
	right := [4]float64{6, 0, 10, 12}
	bridge := [4]float64{4, 8, 6, 12}
	bar := [4]float64{-2, 2, 12, 5}

	area := new(big.Rat)
	for _, rect := range [][4]float64{left, right, bridge} {
		area.Add(area, exactRectOverlapArea(rect, bar))
	}
	height := new(big.Rat).SetInt64(5)
	truth := new(big.Rat).Mul(area, height)
	truthFloat, _ := truth.Float64()
	require.InDelta(t, 120.0, truthFloat, 1e-12, "premise: the hand decomposition matches the stated 120 mm^3")

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]

	lo := new(big.Rat).SetFloat64(row.Volume.Value.Base() - row.Volume.Bound.Base())
	hi := new(big.Rat).SetFloat64(row.Volume.Value.Base() + row.Volume.Bound.Base())
	require.LessOrEqual(t, lo.Cmp(truth), 0, "the published interval must contain the true overlap")
	require.GreaterOrEqual(t, hi.Cmp(truth), 0)
}

// TestVerifySingleRegionOverlapOrderIsNormative is §15's "the single-region
// reading is untouched" row: two coplanar boxes overlapping in ONE region
// still report 125 mm^3 through evaluateAnalyticIntersect, not through §4.5's
// reading — pinned here by asserting the SAME fixture as
// TestVerifyCrossingCoplanarBoxesReportsInterference
// (prism_boolean_crossing_test.go) still reports the identical value and a
// tiny bound after this PR, which is what §5's normative order (the twin
// runs first) requires.
func TestVerifySingleRegionOverlapOrderIsNormative(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 5)
	b := boxBody(t, doc, 5, 5, 15, 15, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, a, row.A)
	require.Same(t, b, row.B)
	require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)
	require.Equal(t, decad.Approximate, row.Volume.Exactness)
	require.Less(t, row.Volume.Bound.Base(), 1e-9)
}

// combTeethPts is a comb-shaped prism (a base bar with three upward teeth,
// drawn as one outer loop, no holes) crossed by a horizontal bar entirely
// inside the teeth's own vertical span but outside the base's: the overlap is
// exactly three disjoint 2x2 = 4 mm^2 regions, one per tooth.
var combTeethPts = [][2]float64{
	{0, 0}, {16, 0}, {16, 2}, {15, 2}, {15, 10}, {13, 10}, {13, 2},
	{9, 2}, {9, 10}, {7, 10}, {7, 2}, {3, 2}, {3, 10}, {1, 10}, {1, 2}, {0, 2},
}

var combBarPts = [][2]float64{{-2, 4}, {18, 4}, {18, 6}, {-2, 6}}

// TestVerifyThreeRegionOverlapReportsSummedVolume is §15's "three regions,
// not two" row: a comb-shaped prism with three teeth crossed by a bar
// overlaps in three disjoint regions of known area, so a two-region-only
// implementation cannot pass this fixture by accident.
func TestVerifyThreeRegionOverlapReportsSummedVolume(t *testing.T) {
	const h = 3.0
	doc := decad.New()
	comb := polyPrismBody(t, doc, combTeethPts, h)
	bar := polyPrismBody(t, doc, combBarPts, h)
	before := snapshotDocument(t, doc)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, comb, row.A)
	require.Same(t, bar, row.B)

	// Three teeth, each 2 mm wide (x in [1,3], [7,9], [13,15]) fully spanning
	// the bar's y in [4,6]: 3 * (2*2) * h = 36 mm^3 at h=3.
	const want = 3 * 2 * 2 * h
	require.InDelta(t, want, row.Volume.Value.Base(), row.Volume.Bound.Base())
	require.Less(t, row.Volume.Bound.Base(), 1e-9)
	require.Equal(t, decad.Approximate, row.Volume.Exactness)
	requireDocumentUnchanged(t, doc, before)
}

// TestVerifyExactlyTangentPairStaysUndecided is §15's "the exactly-tangent
// pair stays undecided" row: two coplanar prisms whose sections meet along a
// shared wall but enclose no common area report no Interference row — §4.5's
// reading never turns a contact into a zero-volume row.
func TestVerifyExactlyTangentPairStaysUndecided(t *testing.T) {
	doc := decad.New()
	boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBody(t, doc, 10, 0, 20, 10, 10)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Empty(t, report.Interferences, "a shared-wall contact must never publish a zero-volume row")
	for _, row := range report.Interferences {
		require.Positive(t, row.Volume.Value.Base())
	}
}

// TestVerifyFacetedMultiRegionPairStaysAsBefore is §15's "a bored (faceted)
// pair stays undecided" row: G1 excludes a facetedPayload operand from every
// analytic path (the twin and §4.5's reading alike), so a pair whose operands
// were forced through the mesh boolean reports exactly what it did before
// this PR — no new row.
func TestVerifyFacetedMultiRegionPairStaysAsBefore(t *testing.T) {
	doc := decad.New()
	u, bar := uAndBarBodies(t, doc)

	// A small notch, cut from a z-span ([1,3]) that does not fully span
	// either operand's own height ([0,5]), fails Cut's G5 z-interval-spans
	// gate, so the analytic path never admits it and Cut falls to the
	// (unchanged) mesh path.
	notch := translated(t, boxBody(t, doc, 1, 1, 2, 2, 2), 0, 0, 1)
	notch2 := translated(t, boxBody(t, doc, -1, 3, 0, 4, 2), 0, 0, 1)
	uBored, err := decad.Cut(u, notch)
	require.NoError(t, err)
	barBored, err := decad.Cut(bar, notch2)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(uBored), "the bore must force the mesh path")
	require.True(t, anyFaceIsFaceted(barBored), "the bore must force the mesh path")

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Empty(t, report.Interferences,
		"neither operand is a prismPayload, so G1 excludes the pair from every analytic path")
}

// TestPublicBooleansUnchangedOnMultiRegionPair is §15's "public booleans
// unchanged" row: decad.Intersect of the U prism and the bar still returns
// the mesh path's existing refusal, since the public surface owes its caller
// a Body and §4.5's reading builds none (docs/prism-boolean-design.md §13).
// Cut hits the identical mesh-path coplanar-contact refusal (neither the
// clean-nesting match nor the crossing sub-case resolves a genuinely
// multi-region selection, §4.4), and Union — whose select-all path merges
// every arrangement cell into ONE simple loop regardless of how many
// disjoint regions the pair's INTERSECTION would cover — is unaffected and
// keeps building analytically. None of the three is touched by this PR: it
// adds no capability to performBoolean's dispatch at all (§4.5's own "The
// reading runs only from the read-only interference path, never from
// performBoolean").
func TestPublicBooleansUnchangedOnMultiRegionPair(t *testing.T) {
	t.Run("intersect refuses through the mesh path", func(t *testing.T) {
		doc := decad.New()
		u, bar := uAndBarBodies(t, doc)
		_, err := decad.Intersect(u, bar)
		require.Error(t, err)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		var be *decad.BooleanError
		require.True(t, errors.As(err, &be))
		require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
	})

	t.Run("cut refuses through the same mesh-path contact refusal", func(t *testing.T) {
		doc := decad.New()
		u, bar := uAndBarBodies(t, doc)
		_, err := decad.Cut(u, bar)
		require.Error(t, err)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		var be *decad.BooleanError
		require.True(t, errors.As(err, &be))
		require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
	})

	t.Run("union still builds analytically", func(t *testing.T) {
		doc := decad.New()
		u, bar := uAndBarBodies(t, doc)
		got, err := decad.Union(u, bar)
		require.NoError(t, err)
		require.False(t, anyFaceIsFaceted(got),
			"Union's select-all path merges every cell into one loop and is unaffected by §4.5")
		vol, err := got.Volume()
		require.NoError(t, err)
		require.Positive(t, vol.Value.Base())
	})
}

// The rest of this file is the MULTIREGION-TASKS.md Task 3 fixture: a pair of
// interleaved comb prisms sized to sit past the consumer's own real-scale
// gear pair (gearScaleConsumerSegments), proving §4.5's reading answers
// at that scale and recording what prismMaxArrangementSegments (4096,
// prism_boolean.go) does to a pair that crosses it. Task 1 and Task 2 already
// shipped the production code this exercises; this file adds no new
// capability, tests and measurement only.
//
// combUpPts builds a comb polygon with its base at the bottom (y in
// [0, yBase]) and n teeth of width tw rising from it at pitch p (left edge of
// tooth k at x0+mL+k*p) up to tip height yTop, with base margins mL/mR beyond
// the end teeth. Point order generalizes combTeethPts's own hand-authored
// 3-tooth comb (prism_overlap_test.go's TestVerifyThreeRegionOverlapReports
// SummedVolume fixture) to any tooth count, right-to-left per tooth.
func combUpPts(n int, x0, tw, p, mL, mR, yBase, yTop float64) [][2]float64 {
	total := mL + float64(n-1)*p + tw + mR
	pts := [][2]float64{{x0, 0}, {x0 + total, 0}, {x0 + total, yBase}}
	for k := n - 1; k >= 0; k-- {
		left := x0 + mL + float64(k)*p
		right := left + tw
		pts = append(pts,
			[2]float64{right, yBase}, [2]float64{right, yTop},
			[2]float64{left, yTop}, [2]float64{left, yBase})
	}
	return append(pts, [2]float64{x0, yBase})
}

// combDownPts is combUpPts's vertical mirror about baseTop: base at the top
// (y in [baseTop-baseThk, baseTop]), n teeth hanging down to teethBottom.
func combDownPts(n int, x0, tw, p, mL, mR, baseThk, teethBottom, baseTop float64) [][2]float64 {
	up := combUpPts(n, x0, tw, p, mL, mR, baseThk, baseTop-teethBottom)
	down := make([][2]float64, len(up))
	for i, pt := range up {
		down[i] = [2]float64{pt[0], baseTop - pt[1]}
	}
	return down
}

// subdividePolygon replaces each of pts's edges with m collinear
// sub-segments, multiplying the polygon's own segment count by m without
// moving a single one of its vertices or changing its area at all — the same
// way a real gear tooth flank arrives as many short polyline points rather
// than one long straight one, which is what pushes the consumer's own pair to
// gearScaleConsumerSegments in the first place.
func subdividePolygon(pts [][2]float64, m int) [][2]float64 {
	n := len(pts)
	out := make([][2]float64, 0, n*m)
	for i := range n {
		p0, p1 := pts[i], pts[(i+1)%n]
		for k := range m {
			t := float64(k) / float64(m)
			out = append(out, [2]float64{
				p0[0] + (p1[0]-p0[0])*t,
				p0[1] + (p1[1]-p0[1])*t,
			})
		}
	}
	return out
}

// gearScale* fixes the interleaved-comb fixture's own geometry: comb A points
// n teeth up from a base at the bottom, comb B points n teeth down from a
// base at the top, and B's teeth are shifted gearScaleXOffset to the right of
// A's own — less than the shared tooth width, so tooth k of each comb
// overlaps the OTHER comb's tooth k in exactly one small rectangle, and never
// its neighbours (mirroring a gear pair pressed slightly past nominal centre
// distance, engaging several tooth pairs by a sliver each). Every edge is
// then subdivided gearScaleSubdiv-fold (subdividePolygon), so the combined
// segment count scales with n at a fixed, known rate.
const (
	gearScaleToothWidth   = 2.0
	gearScalePitch        = 4.0
	gearScaleXOffset      = 1.0
	gearScaleMarginL      = 1.0
	gearScaleMarginR      = 1.0
	gearScaleABase        = 2.0  // comb A: base thickness, and its teeth's own bottom y
	gearScaleATeethTop    = 12.0 // comb A: teeth tip y
	gearScaleBBaseThk     = 2.0  // comb B: base thickness
	gearScaleBTeethBottom = 6.0  // comb B: teeth tip y (teeth point down)
	gearScaleBTopY        = 16.0 // comb B: base top y
	gearScaleSweepH       = 4.0
	gearScaleSubdiv       = 13
)

// gearScaleXOverlap and gearScaleYOverlap are each interleaved region's own
// known width and height, derived from the gearScale* geometry constants
// above rather than restated as independent numbers.
var (
	gearScaleXOverlap = gearScaleToothWidth - gearScaleXOffset
	gearScaleYOverlap = math.Min(gearScaleATeethTop, gearScaleBTopY-gearScaleBBaseThk) -
		math.Max(gearScaleABase, gearScaleBTeethBottom)
)

// gearScaleSegmentCount is the combined LineSeg count prismSceneWithinWorkCap
// (prism_boolean.go) charges an n-tooth interleavedCombBodies pair: 2 combs,
// each with combUpPts's own 4+4n raw points-and-edges, each edge subdivided
// gearScaleSubdiv-fold.
func gearScaleSegmentCount(n int) int {
	return 2 * (4 + 4*n) * gearScaleSubdiv
}

// gearScaleConsumerSegments is the consumer's own real-scale gear pair's
// measured combined polyline segment count: 312 segments on one gear and 520
// on the other (MULTIREGION-TASKS.md's "already measured" note). This is the
// only site in the tree that spells those numbers out; every other mention
// names this constant, so the pair's measured size has one owner.
const gearScaleConsumerSegments = 312 + 520

// gearScaleEightToothSegments is the combined segment count the 8-tooth
// headline fixture is claimed to stand at, i.e. what gearScaleSegmentCount(8)
// must come to: 2 combs, each with 4+4*8 raw edges, each subdivided
// gearScaleSubdiv-fold. Pinning it keeps that fixture at the scale the test's
// own claim rests on — without the pin, lowering gearScaleSubdiv would drop
// the pair below gearScaleConsumerSegments while every geometric assertion
// still passed, since subdividePolygon moves no vertex and changes no area.
const gearScaleEightToothSegments = 936

// interleavedCombBodies builds the gearScale* fixture's two comb prisms into
// doc, both swept gearScaleSweepH, overlapping in exactly n disjoint regions
// of gearScaleXOverlap*gearScaleYOverlap mm² each.
func interleavedCombBodies(t *testing.T, doc *decad.Document, n int) (a, b *decad.Body) {
	t.Helper()
	aPts := combUpPts(n, 0, gearScaleToothWidth, gearScalePitch,
		gearScaleMarginL, gearScaleMarginR, gearScaleABase, gearScaleATeethTop)
	bPts := combDownPts(n, 0, gearScaleToothWidth, gearScalePitch,
		gearScaleMarginL+gearScaleXOffset, gearScaleMarginR,
		gearScaleBBaseThk, gearScaleBTeethBottom, gearScaleBTopY)
	a = polyPrismBody(t, doc, subdividePolygon(aPts, gearScaleSubdiv), gearScaleSweepH)
	b = polyPrismBody(t, doc, subdividePolygon(bPts, gearScaleSubdiv), gearScaleSweepH)
	return a, b
}

// TestVerifyGearScaleEightRegionOverlapReportsSummedVolume is Task 3's own
// headline (MULTIREGION-TASKS.md Task 3, observable test 1): an 8-tooth
// interleaved-comb pair at gearScaleEightToothSegments combined segments —
// above the consumer's own gearScaleConsumerSegments, and safely below
// prismMaxArrangementSegments' 4096 cap — still reports one summed
// Interference row within a tiny bound. The wall-clock cost is logged for
// comparison against the real gear pair's own measured ~16s Suspect
// (MULTIREGION-TASKS.md's "already measured" note).
func TestVerifyGearScaleEightRegionOverlapReportsSummedVolume(t *testing.T) {
	const n = 8
	segs := gearScaleSegmentCount(n)
	require.Greater(t, segs, gearScaleConsumerSegments,
		"premise: case 1's combined segment count (%d) must exceed the consumer's own measured gear pair (%d segments)",
		segs, gearScaleConsumerSegments)
	require.Equal(t, gearScaleEightToothSegments, segs,
		"premise: case 1 must stand at its stated combined segment count; gearScaleSubdiv or the comb's own point count moved the fixture off that scale")
	require.LessOrEqual(t, segs, 4096, "premise: case 1's combined segment count stays at or under the 4096 arrangement cap")

	doc := decad.New()
	a, b := interleavedCombBodies(t, doc, n)
	before := snapshotDocument(t, doc)

	start := time.Now()
	report, err := doc.Verify(t.Context())
	elapsed := time.Since(start)
	t.Logf("gear-scale Verify: %d teeth, %d combined line segments, %s wall clock", n, segs, elapsed)

	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.Same(t, a, row.A)
	require.Same(t, b, row.B)

	want := float64(n) * gearScaleXOverlap * gearScaleYOverlap * gearScaleSweepH
	require.InDelta(t, want, row.Volume.Value.Base(), row.Volume.Bound.Base())
	require.Less(t, row.Volume.Bound.Base(), 1e-9)
	require.Equal(t, decad.Approximate, row.Volume.Exactness)

	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagUnsupportedPairContact, d.Code,
			"a published row on a coplanar pair can only have come from the analytic reading")
		require.NotEqual(t, decad.DiagUnsupportedPair, d.Code)
		require.NotEqual(t, decad.DiagUnsupportedPairPipeline, d.Code)
	}
	requireDocumentUnchanged(t, doc, before)
}

// TestVerifyGearScaleOverArrangementCapStaysSuspect is Task 3's observable
// test 2: the same interleaved-comb fixture built one tooth larger — 39 teeth,
// 4160 combined segments, over the 4096 arrangement cap — reports Suspect
// through the pipeline-unsupported diagnostic rather than an error or a
// crash, pinning that the cap degrades §4.5's reading rather than breaking
// it.
func TestVerifyGearScaleOverArrangementCapStaysSuspect(t *testing.T) {
	const n = 39
	segs := gearScaleSegmentCount(n)
	require.Greater(t, segs, 4096, "premise: case 2's combined segment count crosses the 4096 arrangement cap")

	doc := decad.New()
	a, b := interleavedCombBodies(t, doc, n)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err, "the cap must report Suspect, never fail Verify itself")
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Interferences)

	var sawPipeline, sawLegacy bool
	for _, d := range report.Diagnostics {
		if d.Pair == nil || d.Pair.A != a || d.Pair.B != b {
			continue
		}
		switch d.Code {
		case decad.DiagUnsupportedPairPipeline:
			sawPipeline = true
		case decad.DiagUnsupportedPair:
			sawLegacy = true
		}
	}
	require.True(t, sawPipeline, "the arrangement cap must report the pipeline-unsupported cause code")
	require.True(t, sawLegacy, "the deprecated broad unsupported-pair code must still accompany it")
}
