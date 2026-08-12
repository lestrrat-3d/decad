package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §4.2's crossing sub-case black-box
// test suite for Cut and Intersect (prism_boolean_crossing.go): two coplanar
// prisms whose 2D outlines genuinely cross, neither nested inside the other.
// boxBody/discBody (clearance_test.go, prism_boolean_bounds_test.go) and
// anyFaceIsFaceted/boundMM3/volumeMM (prism_boolean_test.go, boolean_test.go)
// are the shared helpers. The randomized analytic/mesh property test lives in
// prism_boolean_crossing_internal_test.go, which needs the internal
// evaluateBoolean entry point to force the mesh path for comparison.

// TestPrismIntersectCrossingBoxesResolvesAnalytically is docs/prism-boolean-design.md
// §14 PR3's own required test: two coplanar boxes offset diagonally so their
// footprints genuinely cross (neither is nested) build analytically, not
// through the mesh path, and their overlap is the exact 5x5 square the two
// footprints share.
func TestPrismIntersectCrossingBoxesResolvesAnalytically(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 5)
	b := boxBody(t, doc, 5, 5, 15, 15, 5)

	got, err := decad.Intersect(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the crossing sub-case must build analytically`)

	vol, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, 125.0, volumeMM(t, vol), 1e-9) // the 5x5 shared square, times h=5
	require.Equal(t, decad.Approximate, vol.Exactness,
		`the merge splits four walls, exactly like the Union sibling of this same pair`)
	require.Positive(t, vol.Bound.Base())
	require.Less(t, boundMM3(t, vol), 1e-9)
}

// TestPrismCutCrossingBoxesResolvesAnalytically is the Cut arm of the same
// required test: the target loses exactly the shared 5x5 square over its own
// height.
func TestPrismCutCrossingBoxesResolvesAnalytically(t *testing.T) {
	doc := decad.New()
	target := boxBody(t, doc, 0, 0, 10, 10, 5)
	tool := boxBody(t, doc, 5, 5, 15, 15, 5)

	got, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the crossing sub-case must build analytically`)

	vol, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, 375.0, volumeMM(t, vol), 1e-9) // 100 - 25 (the 5x5 overlap), times h=5
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	require.Less(t, boundMM3(t, vol), 1e-9)
}

// TestPrismIntersectCrossingCylindersMatchesClosedFormLens is §14 PR3's own
// required test for a curved pair: two coplanar r10 cylinders with centers
// 8 mm apart overlap in a lens, and the closed-form lens area (two circular
// segments) times the shared height is what the analytic path must publish,
// well inside the 1e-9 mm^3 bound that would betray a chorded mesh answer.
func TestPrismIntersectCrossingCylindersMatchesClosedFormLens(t *testing.T) {
	const r, d, h = 10.0, 8.0, 5.0
	doc := decad.New()
	a := discBody(t, doc, 0, r, h)
	b := discBody(t, doc, d, r, h)

	got, err := decad.Intersect(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), `the crossing sub-case must build analytically`)

	// The half-angle at each center subtended by the chord through the two
	// crossing points: cos(theta) = (d/2)/r = 0.4, and the lens area is twice
	// one circular segment's area, 2*(r^2*acos(d/2r) - (d/4)*sqrt(4r^2-d^2)).
	want := h * (2*r*r*math.Acos(d/(2*r)) - (d/2)*math.Sqrt(4*r*r-d*d))
	require.InDelta(t, 792.673, want, 1e-3, `premise: the closed-form lens volume`)

	vol, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volumeMM(t, vol), boundMM3(t, vol))
	require.Less(t, boundMM3(t, vol), 1e-9, `proves the analytic path, not a chorded mesh answer`)
}

// TestVerifyCrossingCoplanarBoxesReportsInterference is the consumer's own
// "done when" (REMAINING-TASKS.md's Task 1): Document.Verify on two crossing
// coplanar boxes now proves the overlap analytically, reporting exactly one
// Interference row and no unsupported_pair_contact diagnostic — before this
// increment the same pair read Suspect with that diagnostic
// (TestVerifyDiagnosticsUnsupportedPairStagedContact's sibling shape, but
// sharing the extrusion base plane rather than a side face).
func TestVerifyCrossingCoplanarBoxesReportsInterference(t *testing.T) {
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 5)
	b := boxBody(t, doc, 5, 5, 15, 15, 5)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)

	row := report.Interferences[0]
	require.Same(t, a, row.A)
	require.Same(t, b, row.B)
	require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)

	for _, d := range report.Diagnostics {
		require.NotEqual(t, decad.DiagUnsupportedPairContact, d.Code,
			`the crossing pair no longer stages a boolean contact`)
		require.NotEqual(t, decad.DiagUnsupportedPair, d.Code,
			`the crossing pair no longer trips the broad compatibility code either`)
	}
}

// TestPrismCutIntersectNonCoplanarPairStillTakesMeshPath is a regression
// check (§14 PR3's own required test): a pair whose composed world planes
// are NOT the same plane (G3 miss) reaches the unchanged mesh path for Cut
// and Intersect, exactly as it did before this increment — the crossing
// classifier never runs on a pair the entry gate already excludes.
func TestPrismCutIntersectNonCoplanarPairStillTakesMeshPath(t *testing.T) {
	doc := decad.New()
	target := boxBody(t, doc, 0, 0, 10, 10, 10)
	toolSrc := boxBody(t, doc, 5, 5, 15, 15, 10)
	shift, err := r3.Translation(r3.Vec{Z: 3})
	require.NoError(t, err)
	tool, err := toolSrc.Placed(shift)
	require.NoError(t, err)

	gotCut, err := decad.Cut(target, tool)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(gotCut), `a non-coplanar pair must still take the mesh path`)

	doc2 := decad.New()
	a := boxBody(t, doc2, 0, 0, 10, 10, 10)
	bSrc := boxBody(t, doc2, 5, 5, 15, 15, 10)
	b, err := bSrc.Placed(shift)
	require.NoError(t, err)
	gotIntersect, err := decad.Intersect(a, b)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(gotIntersect), `a non-coplanar pair must still take the mesh path`)
}

// TestPrismCutIntersectRotatedSectionStillTakesMeshPath is the crossing
// sub-case's own version of §3.3's rotated-section regression: a tooth
// placed via r3.RotationAround at an inexact step count still misses G3 and
// falls back to the mesh path's own coplanar-contact refusal for Cut and
// Intersect, unaffected by this increment — matching
// TestPrismUnionRotatedToothFallback's own shape for Union.
func TestPrismCutIntersectRotatedSectionStillTakesMeshPath(t *testing.T) {
	const r, r2, th1, th2, h = 15.0, 19.0, 0.05, 0.25, 8.0
	const n, k = 17, 10
	angle := units.Radians(2 * math.Pi * float64(k) / float64(n))
	tr, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), angle)
	require.NoError(t, err)

	doc := decad.New()
	hub := hubBody(t, doc, r, h)
	toothSrc := toothBody(t, doc, r, r2, th1, th2, h)
	tooth, err := toothSrc.Placed(tr)
	require.NoError(t, err)

	_, err = decad.Cut(hub, tooth)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	doc2 := decad.New()
	hub2 := hubBody(t, doc2, r, h)
	toothSrc2 := toothBody(t, doc2, r, r2, th1, th2, h)
	tooth2, err := toothSrc2.Placed(tr)
	require.NoError(t, err)
	_, err = decad.Intersect(hub2, tooth2)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
