package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file pins the fix for docs/spline-design.md §5.2's own discipline:
// within ONE evalPrismContext call, buildLoopSidesAs,
// profileCoordinateEnvelope (via prismCentroidGeometryBound and, four times
// over, prismBoundsContext's per-axis extentBoundedAlong) and
// boundaryExtremesBoundedContext (three times, also via extentBoundedAlong)
// each used to call walkOf on the SAME recorded segment, so one free-form
// segment's conversion-and-bracket charge was spent eight times over instead
// of once. profileWalks (extrude.go) resolves every segment's walk exactly
// once per evaluation and lets every consumer read it back.

// involuteFitProfile is the requester's own reproduction fixture: 15
// endpoint-inclusive samples of one involute gear-tooth flank
// (involuteFitPoints, spline_convexity_internal_test.go), closed by a
// straight chord back to its start. The construction — a LineSeg from the
// first fit point to the last, followed by the FitSplineSeg walked in
// REVERSE (TStart=1, TEnd=0) — is the same reversed pairing
// spline_fit_test.go's own TestFitSplineTerminalDedupRefusesUnclosedLoopReversed
// uses to reproduce a real recorded record exactly, and it is what gives this
// loop positive net area: the forward pairing (spline first, chord back)
// winds the opposite way and evalPrismContext refuses it as ErrDegenerate.
func involuteFitProfile() ProfileRecord {
	fit := involuteFitPoints()
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: fit[0], End: fit[len(fit)-1], TStart: 0, TEnd: 1},
		FitSplineSeg{Fit: fit, TStart: 1, TEnd: 0},
	}}}
}

// involuteFitPrismPayload builds the payload evalPrism evaluates: an
// unplaced 5 mm prism over involuteFitProfile, the same synthetic-payload
// pattern extrude_freeform_internal_test.go, extrude_work_test.go and
// verify_freeform_internal_test.go already use for a free-form-walled prism,
// since no evaluator path builds one through a live sketch profile yet.
func involuteFitPrismPayload(t *testing.T) prismPayload {
	t.Helper()
	return prismPayload{
		profile: involuteFitProfile(),
		frame:   identityFrame(t),
		z1:      5,
		xform:   r3.Identity(),
	}
}

// TestPrismWalkOnceInvoluteRecordFitsWorkBudget is the observable fix itself:
// before profileWalks, this exact record's walk was resolved eight times
// over within one evalPrismContext call (230,168 units each), for
// 1,841,344 units against a 1,048,576 ceiling — refused as ErrUnsupported.
// With the walk resolved once and every consumer reading it back, the whole
// build's charge fits comfortably inside the ceiling and the body builds.
//
// 959,408 is this file's own pinned measurement, taken directly from this
// exact fixture (involuteFitPrismPayload evaluated through evalPrism, the
// same call every other free-form-prism test in this package uses). It is
// LOWER than the 975,600 the bug report measured on the requester's real
// record by exactly one moments-preflight charge (16,192,
// evaluatorIntegralsUncheckedContext's own — see evaluatorIntegralsUncheckedContext's
// call in evalPrismContext): the requester's number was measured through the
// public Document.Extrude path, whose RecordProfile seam runs its own
// independent-implementation area check (§5.2's "sketch computes its own
// free-form area... decad integrates its OWN records... the two agreeing is
// the §1 falsifier") before Extrude's own evalPrismContext runs its own
// preflight — a second whole-record preflight this synthetic-payload fixture
// never pays, since it calls evalPrism directly on a hand-built
// prismPayload/ProfileRecord rather than through RecordProfile. What both
// numbers agree on is the walk resolving exactly ONCE per evaluation instead
// of eight times: resolveProfileWalks's own charge is
// TestResolveProfileWalksChargesSegmentOnce's 230,168, one occurrence of
// which is common to both totals.
func TestPrismWalkOnceInvoluteRecordFitsWorkBudget(t *testing.T) {
	pp := involuteFitPrismPayload(t)
	work := newFreeformWork()

	body, err := evalPrism(New(), 0, pp, work)
	require.NoError(t, err, "the deduplicated charge must fit inside the work budget")
	require.NotNil(t, body)
	require.Greater(t, body.volume.Value.Mag(), 0.0, "the built prism must enclose positive volume")

	require.Less(t, work.spent, freeformWorkLimit,
		"the whole build's deduplicated charge must sit strictly below the ceiling")
	// The charge is asserted as a BAND, not an exact equality. Most of it is
	// count-driven and identical everywhere, but §6.5's certificate subdivides
	// on a mixed Bernstein sign, and how deep it goes is decided by float
	// coefficients that sketch's own solve produces — Go may contract a*b+c
	// into a fused multiply-add on arm64 and not on amd64, so a coefficient
	// sitting near zero can cost one extra split on one host and not another.
	// Pinning the exact figure would pin the host's FMA contraction rather
	// than the deduplication this test is for. The band is tight enough to
	// fail loudly if the walk were resolved even twice: a second resolution
	// alone adds 230,168 units, far outside it.
	require.Greater(t, work.spent, uint64(900000),
		"the build must still do its real free-form work, not silently skip it")
	require.Less(t, work.spent, uint64(1000000),
		"measured 959,408 on amd64; a second walk resolution would add 230,168 and break this")
}

// TestResolveProfileWalksChargesSegmentOnce is the mechanism
// TestPrismWalkOnceInvoluteRecordFitsWorkBudget's fix rests on: resolving a
// profile's walks charges each segment's conversion and bracket cost exactly
// ONCE, never once per loop iteration or once per caller. 230,168 is the
// segment's own single-resolution cost (this file's involuteFitProfile doc
// comment, and the bug report's own measurement of walkOf on this exact
// segment); the closing LineSeg is analytic and charges nothing
// (walkOf's own doc comment: "An analytic segment charges nothing").
func TestResolveProfileWalksChargesSegmentOnce(t *testing.T) {
	work := newFreeformWork()
	pw, err := resolveProfileWalks(involuteFitProfile(), work)
	require.NoError(t, err)
	require.NotNil(t, pw)
	require.Equal(t, uint64(230168), work.spent,
		"one resolution of one free-form segment must charge exactly its own single-walk cost")
}

// TestProfileWalksMismatchRefuses pins the coarsest half of constraint 5's
// reject-only guard: a *profileWalks resolved from one profile must never be
// read against a profile of a DIFFERENT shape, silently or otherwise. The
// same-shape half — a profile that differs only in its segment DATA — is
// TestProfileWalksSegmentDataMismatchRefuses below. Either set applied anyway
// would read another section's geometry as this one's, a plumbing bug a nil
// fallback would hide behind a correct-looking answer, so every consumer
// refuses instead.
func TestProfileWalksMismatchRefuses(t *testing.T) {
	// A 4-segment square, deliberately a different outer segment count than
	// involuteFitProfile's 2 (a LineSeg and a FitSplineSeg).
	square := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 0}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	wrongWalks, err := resolveProfileWalks(square, newFreeformWork())
	require.NoError(t, err)

	involute := involuteFitProfile()
	require.NotEqual(t, len(square.Outer.Segments), len(involute.Outer.Segments),
		"premise: the two profiles must have different shapes for the guard to fire")
	require.False(t, wrongWalks.matches(involute), "matches must catch the shape mismatch")

	t.Run("profileCoordinateEnvelope", func(t *testing.T) {
		_, err := profileCoordinateEnvelope(involute, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("profileCoordinateUpper", func(t *testing.T) {
		_, err := profileCoordinateUpper(involute, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("boundaryExtremesBoundedContext", func(t *testing.T) {
		_, _, _, err := boundaryExtremesBoundedContext(t.Context(), involute, 1, 0, newFreeformWork(), wrongWalks)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// lineEndProfile is a one-segment profile whose only difference from another
// built the same way is WHERE its line ends: same variant, same start, same
// range, one differing coordinate. Two of them have identical shape — one
// outer segment, no holes — so shape alone cannot tell them apart.
func lineEndProfile(end Point2) ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: end, TStart: 0, TEnd: 1},
	}}}
}

// TestProfileWalksSegmentDataMismatchRefuses pins the guard on the reading it
// exists for: a *profileWalks resolved from one profile must not be read
// against a DIFFERENT profile of the same shape. Comparing counts alone let
// such a pair through, and the cached read then published the first section's
// geometry as the second's — profileCoordinateEnvelope reporting 1 for a
// section whose own envelope is 2, with no error anywhere. The comparison is
// over the recorded segments themselves, exactly: the one-ulp subtest below
// pins that it carries no tolerance and no "close enough" arm.
func TestProfileWalksSegmentDataMismatchRefuses(t *testing.T) {
	near := lineEndProfile(Point2{U: 1, V: 0})
	far := lineEndProfile(Point2{U: 2, V: 0})
	require.Len(t, far.Outer.Segments, len(near.Outer.Segments),
		"premise: the two profiles must have the same shape for the data comparison to be the thing under test")
	require.Empty(t, near.Holes)
	require.Empty(t, far.Holes)

	pw, err := resolveProfileWalks(near, newFreeformWork())
	require.NoError(t, err)
	require.True(t, pw.matches(near), "premise: the resolved profile itself must still read back")
	require.False(t, pw.matches(far), "a same-shaped profile with different segment data is not the resolved profile")

	// The two readings the mismatch used to conflate: near's coordinate
	// envelope is 1, far's is 2.
	nearUpper, err := profileCoordinateEnvelope(near, newFreeformWork(), pw)
	require.NoError(t, err)
	require.Equal(t, 1.0, nearUpper, "near's own envelope, read through its own resolved walks")
	farUpper, err := profileCoordinateEnvelope(far, newFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, 2.0, farUpper, "far's true envelope, resolved from far's own segment")

	t.Run("profileCoordinateEnvelope", func(t *testing.T) {
		_, err := profileCoordinateEnvelope(far, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("profileCoordinateUpper", func(t *testing.T) {
		_, err := profileCoordinateUpper(far, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("boundaryExtremesBoundedContext", func(t *testing.T) {
		_, _, _, err := boundaryExtremesBoundedContext(t.Context(), far, 1, 0, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("buildLoopSidesAs", func(t *testing.T) {
		pp := prismPayload{profile: near, frame: identityFrame(t), z1: 5, xform: r3.Identity()}
		body := &Body{doc: New(), solid: true}
		_, _, _, _, err := buildLoopSidesAs(t.Context(), body, 0, pp, 0, false, far.Outer, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("one ulp apart", func(t *testing.T) {
		ulp := lineEndProfile(Point2{U: math.Nextafter(1, 2), V: 0})
		require.False(t, pw.matches(ulp), "the comparison is exact: one ulp of difference is a mismatch")
		_, err := profileCoordinateEnvelope(ulp, newFreeformWork(), pw)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
	t.Run("hole data", func(t *testing.T) {
		withHole := near
		withHole.Holes = []LoopRecord{lineEndProfile(Point2{U: 1, V: 1}).Outer}
		holed, err := resolveProfileWalks(withHole, newFreeformWork())
		require.NoError(t, err)
		other := near
		other.Holes = []LoopRecord{lineEndProfile(Point2{U: 1, V: 3}).Outer}
		require.False(t, holed.matches(other), "a hole loop's own segment data is compared too")
		_, err = profileCoordinateEnvelope(other, newFreeformWork(), holed)
		require.ErrorIs(t, err, errResolvedWalksMismatch)
	})
}

// TestProfileWalksReadBackMatchesFreshResolution is the other half of the
// guard: the profile a set WAS resolved from still reads back, and what it
// hands each consumer is the walk walkOf itself resolves for that segment,
// field for field. The refusal above must cost the matching path nothing.
func TestProfileWalksReadBackMatchesFreshResolution(t *testing.T) {
	profile := involuteFitProfile()
	pw, err := resolveProfileWalks(profile, newFreeformWork())
	require.NoError(t, err)
	require.True(t, pw.matches(profile))
	require.True(t, pw.loopMatches(0, profile.Outer))

	for si, seg := range profile.Outer.Segments {
		fresh, err := walkOf(seg, newFreeformWork())
		require.NoError(t, err)
		require.Equal(t, fresh, pw.at(0, si),
			"segment %d's read-back walk must be exactly the walk walkOf resolves for it", si)
	}

	// The whole cached-read path still produces the reading it did before:
	// the involute section's own coordinate envelope, unchanged by the guard.
	cached, err := profileCoordinateEnvelope(profile, newFreeformWork(), pw)
	require.NoError(t, err)
	direct, err := profileCoordinateEnvelope(profile, newFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, direct, cached, "reading the cache must give the resolve-every-segment answer")
	require.Greater(t, cached, 0.0)
}
