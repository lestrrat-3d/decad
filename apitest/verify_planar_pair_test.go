package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests drive Verify's exact planar pair arm (clearance_planar.go)
// through the public API alone, on mtilt's support-tree fixtures: a part
// with an overhanging arm, and mitred, tapered 16-gon branches that stand
// under it, inside its bounding box, which the analytic clearance kernel
// cannot model.

// overhangPart is a ⊐-shaped part: a base over X 0..40 and Z 0..10, a back
// wall X 0..10, and an arm over X 0..40 at Z 40..50, 20 mm deep along Y. The
// outline is drawn in XY and turned so that its sketch Y becomes world Z.
func overhangPart(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {10, 10}, {10, 40}, {40, 40}, {40, 50}, {0, 50}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	turn, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)},
		r3.NewVec(0, 20, 0))
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), turn)
	require.NoError(t, err)
	return body
}

// mitredBranch sweeps a 16-gon of circumradius radii[0] from the origin
// along the polyline through pts (relative to pts[0]), tapering to radii[k]
// at pts[k], and moves it to pts[0] — mtilt's branch recipe.
func mitredBranch(t *testing.T, doc *decad.Document, pts []r3.Vec, radii []float64) *decad.Body {
	t.Helper()
	s, profile := polygonSweepSketch(t, 16, radii[0])
	segments := make([]decad.PathSegment, 0, len(pts)-1)
	factors := make([]float64, 0, len(pts)-1)
	for i := 1; i < len(pts); i++ {
		segments = append(segments, decad.LineTo{End: pts[i].Sub(pts[0])})
		factors = append(factors, radii[i]/radii[0])
	}
	path, err := decad.NewPath(r3.Vec{}, segments...)
	require.NoError(t, err)
	body, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins(), decad.WithSectionScale(scalars(factors...)...))
	require.NoError(t, err)
	move, err := r3.Translation(pts[0])
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), move)
	require.NoError(t, err)
	return body
}

// leaningBranchPoints is mtilt's branch under the arm: it rises outside the
// base at X 46, leans in under the arm, and ends with a vertical tip at
// height top.
func leaningBranchPoints(top float64) []r3.Vec {
	lean := math.Tan(30 * math.Pi / 180)
	return []r3.Vec{
		r3.NewVec(46, 10, 0), r3.NewVec(46, 10, 14),
		r3.NewVec(46-12*lean, 10, 26), r3.NewVec(46-12*lean, 10, top-2), r3.NewVec(46-12*lean, 10, top),
	}
}

var leaningBranchRadii = []float64{2.0, 1.8, 1.4, 1.0, 0.5}

// requireSoundWithClearance verifies doc with WithClearances and requires a
// Sound report whose one Clearance row names a and b with a gap proven
// positive and within tol of want.
func requireSoundWithClearance(t *testing.T, doc *decad.Document, a, b *decad.Body, want, tol float64) {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	for _, d := range report.Diagnostics {
		t.Logf("diagnostic %s: %s", d.Code, d.Message)
	}
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Interferences)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, a, row.A)
	require.Equal(t, b, row.B)
	gap, bound := row.Gap.Value.Base(), row.Gap.Bound.Base()
	require.Positive(t, gap-bound, "the clearance must be proven positive")
	require.InDelta(t, want, gap, tol)
	require.LessOrEqual(t, math.Abs(want-gap), bound+tol, "the true gap lies within the published bound")
}

// TestVerifyMitredSweepUnderPrismArmIsDisjoint is mtilt's follow-up A: a
// mitred, tapered branch whose box overlaps the part's reads Sound when it
// stops 0.2 mm or 5 mm short of the arm. At 0.2 mm the arm's underside is
// the nearest feature; at 5 mm the branch's foot, 4 mm beside the base's
// end face, is.
func TestVerifyMitredSweepUnderPrismArmIsDisjoint(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		gap  float64
		want float64
	}{
		{"0.2 mm under the arm", 0.2, 0.2},
		{"5 mm under the arm", 5, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			part := overhangPart(t, doc)
			branch := mitredBranch(t, doc, leaningBranchPoints(40-tc.gap), leaningBranchRadii)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			require.Empty(t, report.Diagnostics)
			require.Empty(t, report.Interferences)

			requireSoundWithClearance(t, doc, part, branch, tc.want, 1e-9)
		})
	}
}

// TestVerifyMitredBranchUnionUnderPrismArmIsDisjoint unions a trunk and a
// twig into one faceted tree, whose held mesh carries a positive
// displacement, and verifies it 0.2 mm under the arm.
func TestVerifyMitredBranchUnionUnderPrismArmIsDisjoint(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	part := overhangPart(t, doc)
	lean := math.Tan(30 * math.Pi / 180)
	trunk := mitredBranch(t, doc, leaningBranchPoints(39.8), leaningBranchRadii)
	twig := mitredBranch(t, doc, []r3.Vec{
		r3.NewVec(46-12*lean, 10, 26.5), r3.NewVec(46-12*lean, 10, 27.5),
		r3.NewVec(46-12*lean-4, 14, 33), r3.NewVec(46-12*lean-4, 14, 37.8), r3.NewVec(46-12*lean-4, 14, 39.8),
	}, []float64{1.2, 1.2, 1.0, 0.9, 0.5})
	tree, err := decad.Union(t.Context(), trunk, twig)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)

	requireSoundWithClearance(t, doc, part, tree, 0.2, 1e-9)
}

// TestVerifyMitredSweepMeetingPrismArmIsNotSound keeps the other side of
// the partition honest: a branch whose tip reaches the arm's underside
// exactly is undecided — its held vertices carry a displacement, so a held
// touch proves nothing about the true pair — and one driven 1 mm into the
// arm is proven to interfere.
func TestVerifyMitredSweepMeetingPrismArmIsNotSound(t *testing.T) {
	t.Parallel()

	t.Run("touching", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		overhangPart(t, doc)
		mitredBranch(t, doc, leaningBranchPoints(40), leaningBranchRadii)

		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Clearances, "a touch within the held displacement proves no gap")
		require.Empty(t, report.Interferences)
		// The planar arm leaves the pair undecided, and the read-only mesh
		// intersection then names the contact it cannot classify; either
		// way the pair is diagnosed, never passed.
		require.Len(t, report.Diagnostics, 1)
		d := report.Diagnostics[0]
		require.Equal(t, decad.Suspect, d.Status)
		require.NotNil(t, d.Pair)
		require.Contains(t, []decad.DiagnosticCode{decad.DiagUndecidedPair, decad.DiagUnsupportedPairContact}, d.Code)
	})

	t.Run("overlapping", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		part := overhangPart(t, doc)
		branch := mitredBranch(t, doc, leaningBranchPoints(41), leaningBranchRadii)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)
		row := report.Interferences[0]
		require.Equal(t, part, row.A)
		require.Equal(t, branch, row.B)
		require.Positive(t, row.Volume.Value.Base()-row.Volume.Bound.Base())
		// The tip is a 1 mm tall frustum of radii 0.75 and 0.5 (16-gon),
		// so its volume is below the circumscribed cone frustum's.
		require.Less(t, row.Volume.Value.Base(), math.Pi*(0.75*0.75+0.75*0.5+0.5*0.5)/3)
	})
}
