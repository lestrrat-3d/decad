package decad_test

import (
	"errors"
	"math/big"
	"testing"

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
