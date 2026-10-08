package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures are docs/helix-design.md §13's PR 4 rows: the exact planar
// arm (clearance_planar.go) serves a coil against a prism, with the held gap
// widened by the coil's δ. The spring is coilSquare's (ρ ∈ [2, 3], two turns
// at pitch 1.5, so y ∈ [0, 4] about the Y axis) and the partner is a box
// [-1, 1] × [0, 4] × [-1, 1] standing in its core. The two boxes overlap, so
// box separation never answers; the nearest spring points are on the inner
// wall ρ = 2 at 45°, against the box's edge at ρ = √2, a true gap of
// 2 − √2. Legs shown to fail, each deleted once:
//
//   - the δ added to the interval's upper end in planarDisjointResult: the
//     chords sit inside the true wall, toward the box, so the held gap is
//     smaller than the true one and the proven interval stops holding it;
//   - the coil case of planarSnapshotOf: the pair reads undecided.
//
// The partner restriction of planarPairAdmits is pinned by the refusal rows.

func coilCoreFixture(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	s, p := coilSquare(t)
	spring, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	core := planarBox(t, doc, -1, 0, -1, 1, 4, 1)
	return doc, spring, core
}

func TestPlanarPairVerdictCoilAgainstPrism(t *testing.T) {
	t.Parallel()
	_, spring, core := coilCoreFixture(t)
	payload, ok := spring.payload.(coilPayload)
	require.True(t, ok)
	require.Positive(t, payload.delta)

	want := 2 - math.Sqrt2
	for name, pair := range map[string][2]*Body{"coil first": {spring, core}, "prism first": {core, spring}} {
		res, ok, err := planarPairVerdict(t.Context(), pair[0], pair[1])
		require.NoError(t, err, name)
		require.True(t, ok, name)
		require.Equal(t, pairDisjoint, res.verdict, name)
		require.LessOrEqual(t, res.lo, want, name)
		require.GreaterOrEqual(t, res.hi, want, name)
		// The held gap is within δ of the true one and the interval adds δ
		// on each side, so its width is at most about 4δ.
		require.LessOrEqual(t, res.hi-res.lo, 4*payload.delta+1e-9, name)
		require.Positive(t, res.lo, name)
		require.False(t, res.exact, name)
	}
}

func TestVerifyCoilClearanceAgainstPrism(t *testing.T) {
	t.Parallel()
	doc, spring, core := coilCoreFixture(t)
	payload := spring.payload.(coilPayload)
	report, err := doc.Verify(t.Context(), WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Interferences)
	require.Len(t, report.Clearances, 1)
	c := report.Clearances[0]
	require.ElementsMatch(t, []*Body{spring, core}, []*Body{c.A, c.B})
	want := 2 - math.Sqrt2
	gap, err := c.Gap.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound := c.Gap.Bound.Base()
	require.InDelta(t, want, gap, bound)
	require.LessOrEqual(t, bound, 2*payload.delta+1e-9)
	require.InDelta(t, want, gap, 2*payload.delta, "the gap is within δ of the hand value")
}

func TestVerifyCoilClearanceAgainstRevolveStaysSuspect(t *testing.T) {
	t.Parallel()
	doc := New()
	s, p := coilSquare(t)
	spring, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	// A ring beside the spring: the planar arm does not serve a revolve
	// partner, so the pair stays undecided and no Clearance row appears.
	rs, rp := coilLoopsSketch(t, [][2]float64{{10, 0}, {11, 0}, {11, 1}, {10, 1}})
	ring, err := doc.Revolve(rs, rp, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	require.False(t, planarPairAdmits(spring, ring))

	report, err := doc.Verify(t.Context(), WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Clearances)
	require.Equal(t, Suspect, report.Status)
	found := false
	for _, d := range report.Diagnostics {
		found = found || d.Code == DiagUndecidedPair && d.Status == Suspect
	}
	require.True(t, found, "the undecided pair is reported: %v", report.Diagnostics)
}

func TestPlanarPairAdmitsCoilPartners(t *testing.T) {
	t.Parallel()
	doc, spring, core := coilCoreFixture(t)
	s, p := coilSquare(t)
	other, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	faceted, err := Union(t.Context(), core, planarBox(t, doc, 0, 1, 0, 6, 2, 0.5))
	require.NoError(t, err)
	_, isFaceted := faceted.payload.(facetedPayload)
	require.True(t, isFaceted)

	require.True(t, planarPairAdmits(spring, core))
	require.False(t, planarPairAdmits(spring, other), "a coil pair stays undecided")
	require.False(t, planarPairAdmits(spring, faceted), "a faceted partner is not a prism or a stitched solid")
}
