package decad

import (
	"context"
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// boxLoftVerts and boxLoftTris are an untwisted, unit-square-to-unit-square
// loft's own vertex table and wall triangle set (docs/loft-design.md §5):
// V0..V3 the bottom loop (indices 0-3), W0..W3 the top loop (indices 4-7),
// each wall cell j split into lower(V_j, V_{j+1}, W_{j+1}) and
// upper(V_j, W_{j+1}, W_j). No twist, so every wall quad is planar and its
// own lower/upper pair is the coplanar-diagonal case §6 names.
func boxLoftVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0), // V0..V3
		r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(1, 1, 1), r3.NewVec(0, 1, 1), // W0..W3
	}
}

func boxLoftTris() [][3]int {
	var tris [][3]int
	for j := range 4 {
		v0, v1 := j, (j+1)%4
		w0, w1 := 4+j, 4+(j+1)%4
		tris = append(tris, [3]int{v0, v1, w1}) // lower_j: side(0,j,0)
		tris = append(tris, [3]int{v0, w1, w0}) // upper_j: side(0,j,1)
	}
	return tris
}

// TestLoftCrossingAuditAdmitsUntwistedBox proves the whole eight-triangle
// lateral surface of an untwisted box loft passes: every consecutive-wall
// vertex pair, every rung and diagonal edge pair, and every non-adjacent
// disjoint pair, all classify as their recorded adjacency expects.
func TestLoftCrossingAuditAdmitsUntwistedBox(t *testing.T) {
	budget := newWorkBudget(t.Context())
	err := loftCrossingAudit(budget, boxLoftVerts(), boxLoftTris())
	require.NoError(t, err)
}

// TestLoftCrossingAuditAdmitsCoplanarSharedEdge proves a wall cell's own
// lower/upper diagonal pair — coplanar, sharing the diagonal V0->W1 — is
// admitted through triTriCoplanarSharedEdge, AND that the identical pair
// still reports contactRegion from triTriClassify directly: the audit-only
// helper does not change mesh-boolean contact classification
// (docs/loft-design.md §6, required test).
func TestLoftCrossingAuditAdmitsCoplanarSharedEdge(t *testing.T) {
	verts := boxLoftVerts()
	tris := boxLoftTris()
	lower0, upper0 := tris[0], tris[1] // cell 0's own diagonal pair

	require.NoError(t, auditLoftPair(verts, tris, 0, 1))

	ta := loftTriCorners(verts, lower0)
	tb := loftTriCorners(verts, upper0)
	xta := loftXTriCorners(verts, lower0)
	xtb := loftXTriCorners(verts, upper0)
	na := xcross(xsub(xta[1], xta[0]), xsub(xta[2], xta[0]))
	nb := xcross(xsub(xtb[1], xtb[0]), xsub(xtb[2], xtb[0]))
	contact, err := triTriClassify(ta, tb, xta, xtb, na, nb)
	require.NoError(t, err)
	require.Equal(t, contactRegion, contact.kind,
		"the coplanar diagonal pair must still classify as contactRegion under triTriClassify itself")
}

// sameSideApexesFixture is docs/loft-design.md §13's own worked fixture: p0
// frame U=(1,0,0), V=(0,1,0), origin (0,0,0); p1 frame U=(-1,0,0), V=(0,1,0),
// origin (1,0,1); the same local square. Cell 0's two triangles share the
// recorded diagonal V0-W1 (two shared vertex indices — S7's two-shared-vertex
// case) but their apexes (V1 and W0) fall on the SAME side of that edge's
// line, so the two triangles overlap in area rather than meeting only along
// the diagonal.
func sameSideApexesFixture() ([]r3.Vec, [][3]int) {
	p0 := func(u, v float64) r3.Vec { return r3.NewVec(u, v, 0) }
	p1 := func(u, v float64) r3.Vec { return r3.NewVec(1-u, v, 1) }

	v0, v1 := p0(0, 0), p0(1, 0)
	w0, w1 := p1(0, 0), p1(1, 0)
	verts := []r3.Vec{v0, v1, w0, w1} // 0=V0, 1=V1, 2=W0, 3=W1
	tris := [][3]int{
		{0, 1, 3}, // lower0: V0, V1, W1
		{0, 3, 2}, // upper0: V0, W1, W0
	}
	return verts, tris
}

// TestLoftCrossingAuditRejectsSameSideApexes runs sameSideApexesFixture
// through both auditLoftPair directly and the whole audit: S7 refuses.
func TestLoftCrossingAuditRejectsSameSideApexes(t *testing.T) {
	verts, tris := sameSideApexesFixture()

	err := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "triangles 0 and 1",
		"the refusal must name the specific triangle pair the predicate found")

	budget := newWorkBudget(t.Context())
	err = loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrDegenerate)
}

// genuineCrossingFixture builds two triangles with NO shared vertex index —
// S7's zero-shared-vertex case, which therefore expects contactNone — that
// nonetheless cross transversally along a positive-length segment on the
// line x=0, y=0: triangle A lies wholly in the y=0 plane, triangle B wholly
// in the x=0 plane, and their own (x,z)/(y,z) footprints overlap on that
// shared line for z in [0,1].
func genuineCrossingFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(-1, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 2), // triangle A (y=0 plane)
		r3.NewVec(0, -1, 1), r3.NewVec(0, 1, 1), r3.NewVec(0, 0, -1), // triangle B (x=0 plane)
	}
	tris := [][3]int{
		{0, 1, 2},
		{3, 4, 5},
	}
	return verts, tris
}

// TestLoftCrossingAuditRejectsGenuineCrossing runs genuineCrossingFixture
// through both auditLoftPair directly and the whole audit: S7 refuses,
// naming the pair it found.
func TestLoftCrossingAuditRejectsGenuineCrossing(t *testing.T) {
	verts, tris := genuineCrossingFixture()

	err := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "triangles 0 and 1",
		"the refusal must name the specific triangle pair the predicate found")

	budget := newWorkBudget(t.Context())
	err = loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrDegenerate)
}

// vertexCrossesAwayFixture builds two coplanar triangles sharing exactly one
// recorded vertex (index 0) — S7's one-shared-vertex case — whose interiors
// nonetheless overlap in positive area away from that vertex, the case
// docs/loft-design.md §6 calls out: "vertex-sharing pairs need this check
// because they can cross away from their recorded vertex."
func vertexCrossesAwayFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(1, 0, 2), // triangle A
		r3.NewVec(1, 0, -1), r3.NewVec(1, 0, 3), // triangle B's other two corners
	}
	tris := [][3]int{
		{0, 1, 2},
		{0, 3, 4},
	}
	return verts, tris
}

// TestLoftCrossingAuditRejectsVertexPairThatCrossesAway runs
// vertexCrossesAwayFixture through auditLoftPair: S7 refuses.
func TestLoftCrossingAuditRejectsVertexPairThatCrossesAway(t *testing.T) {
	verts, tris := vertexCrossesAwayFixture()

	err := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestLoftCrossingAuditRejectsCollapsedTriangle is S6: a triangle whose three
// recorded vertices are exactly collinear has no interior, so no such solid
// exists — ErrDegenerate, before any pair is tested.
func TestLoftCrossingAuditRejectsCollapsedTriangle(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0),
	}
	tris := [][3]int{{0, 1, 2}}

	budget := newWorkBudget(t.Context())
	err := loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrDegenerate)
}

// syntheticLoftTriangles builds n cheap, non-degenerate, mutually far-apart
// triangles: each holds its own three fresh vertex indices, so no pair
// shares an index and no genuine adjacency exists.
func syntheticLoftTriangles(n int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, 3*n)
	tris := make([][3]int, 0, n)
	for i := range n {
		base := float64(i) * 10
		vi := len(verts)
		verts = append(verts,
			r3.NewVec(base, 0, 0),
			r3.NewVec(base+1, 0, 0),
			r3.NewVec(base+1, 1, 0),
		)
		tris = append(tris, [3]int{vi, vi + 1, vi + 2})
	}
	return verts, tris
}

// TestLoftCrossingAuditRefusesOverBudgetBeforeAnyPairTest is S8: a synthetic
// set sized so F*(F-1)/2 exceeds maxFacetPairTestsPerCall must refuse before
// a single pair test runs. An instrumented counting budget proves it: the
// step count after refusal equals exactly the triangle count (S6's own
// per-triangle scan), never more — no pair test was ever trusted.
func TestLoftCrossingAuditRefusesOverBudgetBeforeAnyPairTest(t *testing.T) {
	// 4001*4000/2 = 8_002_000 > maxFacetPairTestsPerCall (8_000_000).
	const n = 4001
	verts, tris := syntheticLoftTriangles(n)

	calls := 0
	budget := &workBudget{
		stepFn: func() error { calls++; return nil },
		errFn:  func() error { return nil },
	}

	err := loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Equal(t, n, calls,
		"the budget must be spent only on S6's per-triangle scan; S8 must refuse before any pair test")
}

// TestLoftCrossingAuditCancellation proves cancellation through the shared
// budget returns ctx.Err() rather than a sentinel of the audit's own.
func TestLoftCrossingAuditCancellation(t *testing.T) {
	verts, tris := syntheticLoftTriangles(4)

	calls := 0
	budget := &workBudget{
		stepFn: func() error {
			calls++
			if calls == 2 {
				return context.Canceled
			}
			return nil
		},
		errFn: func() error { return nil },
	}

	err := loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, context.Canceled)
}

// TestLoftCrossingAuditPollsAfterFinalPair proves the audit observes a context
// cancelled after the S6 boundary check even when no step call ever reaches a
// poll. The budget here mirrors newWorkBudget's real semantics — step observes
// the context only on every workPollInterval-th call, err observes it
// unconditionally — so three triangles (three S6 steps, three S7 pair steps)
// finish the whole audit without a single step poll landing. The trailing
// budget.err() after S7 is the only thing that can return ctx.Err() here.
func TestLoftCrossingAuditPollsAfterFinalPair(t *testing.T) {
	verts, tris := syntheticLoftTriangles(3)

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()

	const finalPairStep = 6 // 3 triangles in S6, then 3 pairs in S7
	steps, errs := 0, 0
	budget := &workBudget{
		stepFn: func() error {
			steps++
			if steps == finalPairStep {
				cancel()
			}
			if steps%workPollInterval == 0 {
				return ctx.Err()
			}
			return nil
		},
		errFn: func() error {
			errs++
			return ctx.Err()
		},
	}

	err := loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, context.Canceled,
		"a context cancelled on the final S7 step must come back from the audit")
	require.Equal(t, finalPairStep, steps,
		"no step call may reach a poll at this size, so the trailing err is the only observer")
	require.Equal(t, 3, errs,
		"err runs at entry, at the S6 boundary, and once more after the S7 loops")
	require.ErrorIs(t, ctx.Err(), context.Canceled)
}

// --- the S7 shortcuts (loft_audit.go's loftAuditShortcuts) ---
//
// loftAuditReference is the audit's independent reference path: no
// broad-phase tier and no certificate, so every pair S8 admits reaches
// triTriClassify's exact contact classification. Every equivalence assertion
// below is against THIS, never against production run through a second
// wrapper. loftAuditProduction is exactly what loftCrossingAudit itself
// passes, so an equivalence proven here is a statement about the shipped
// path.
//
// FALSIFICATION LOG (docs/loft-design.md's own "prove the mechanism can
// fail" discipline). Each leg below was actually broken in loft_audit.go,
// the suite re-run to confirm a RED failure, then reverted before this file
// was committed:
//
//   - Leg 1, "let the short-circuit reach a required-contact pair": the S7
//     loop's guard was widened from `len(shared) == 0` to `len(shared) <= 1`.
//     Both TestLoftCrossingAuditBroadPhaseNeverSkipsARequiredContactPair and
//     TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit stayed GREEN —
//     this leg could NOT be made to fail, and the reason is provable rather
//     than a gap in the fixtures: a pair sharing a recorded vertex INDEX
//     shares the identical coordinate there, so (a) boxesOverlap's own <=
//     always reports the two boxes overlapping at that point (equality
//     satisfies <=, on every axis), and (b) orientSign against the OTHER
//     triangle's plane, evaluated at that shared vertex, is the signed
//     volume of a tetrahedron with a repeated point — exactly zero, every
//     time — so allOneSide (which demands all three signs strictly
//     same-sign) can never be true for it. Both of loftPlaneSeparated's own
//     sign arrays therefore always contain at least one zero for a
//     shared-vertex pair, and boxesOverlap can never separate it either.
//     Both tiers of loftCrossingAudit's CURRENT broad-phase are structurally
//     incapable of reporting "no contact" for a pair sharing a vertex INDEX,
//     independent of the len(shared) guard — so this leg is genuinely
//     redundant against the mechanism as shipped. The explicit guard stays
//     in the source anyway: it is defense in depth against a FUTURE tier
//     that does not share this structural property, and Table
//     TestLoftCrossingAuditBroadPhaseNeverSkipsARequiredContactPair still
//     pins the call's own skip count at 0 for both required-contact fixtures.
//   - Leg 2, "round the bounding box INWARD instead of outward": boxesOverlap
//     compares with <=; a local copy using < (strict) was substituted in the
//     S7 loop, and boundaryTouchingFixture (below) — two triangles sharing NO
//     recorded vertex INDEX but touching at a coordinate exactly on both
//     boxes' shared boundary plane — went RED: the strict test wrongly
//     proved the boxes disjoint, skipping a pair the full audit refuses
//     (ErrDegenerate for touching with no recorded adjacency), so the
//     broad-phase-on run wrongly returned nil.
//     TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit caught it.
//   - Leg 3, "the filter always reports disjoint": the S7 loop's skip
//     condition was replaced with an unconditional `true` (still gated by
//     len(shared) == 0). TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit
//     went RED on genuineCrossingFixture: the broad-phase-on run wrongly
//     returned nil where the pair genuinely crosses and the full audit
//     refuses.
//
// Legs 2 and 3 were both caught by a distinct fixture already in this
// section; leg 1 is the one argued redundant above, on a structural proof
// rather than an untried fixture.

// loftAuditReference and loftAuditProduction are the two shortcut settings
// every equivalence assertion in this section is written against; the section
// header above says what each one means.
var (
	loftAuditReference  = loftAuditShortcuts{}
	loftAuditProduction = loftAuditShortcuts{broadPhase: true, certificates: true}
)

// boundaryTouchingFixture builds two triangles that share NO recorded vertex
// INDEX (so S7 expects contactNone, same as genuineCrossingFixture) but whose
// bounding boxes touch EXACTLY on a shared boundary plane (x=1) rather than
// overlapping with slack: triangle A's vertex 1 and triangle B's vertex 3
// hold the identical coordinate (1,0,0) at two DIFFERENT table indices, so
// the two triangles genuinely touch at that point while sharing no index.
// This is the fixture leg 2 above needs: boxesOverlap's own <= must treat
// the shared boundary as overlap (not disjoint), or the pair is wrongly
// skipped.
func boundaryTouchingFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), // triangle A
		r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(1, 1, 0), // triangle B (vertex 3 duplicates vertex 1's coordinate at a different index)
	}
	tris := [][3]int{
		{0, 1, 2},
		{3, 4, 5},
	}
	return verts, tris
}

// requireLoftCrossingAuditVerdictsMatch runs the audit over the same
// verts/tris under the reference shortcuts and under each shortcut
// combination production can reach, and asserts every run lands on the
// IDENTICAL verdict: all nil, or all an error with the exact same message
// (sentinel, wrapped text and the specific triangle indices it names, all
// included, since Error() renders every one of them). This is loft_audit.go's
// own soundness argument, exercised rather than assumed: a shortcut may only
// ever change WHETHER a pair reaches the exact classification, never WHAT
// that classification (or its absence) decides.
//
// Running the broad-phase and the certificates separately as well as together
// is what keeps a failure attributable: an isolated arm names which of the two
// mechanisms disagreed with the reference.
func requireLoftCrossingAuditVerdictsMatch(t *testing.T, verts []r3.Vec, tris [][3]int) {
	t.Helper()

	_, refErr := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditReference)
	for _, arm := range []struct {
		name      string
		shortcuts loftAuditShortcuts
	}{
		{name: "broad-phase only", shortcuts: loftAuditShortcuts{broadPhase: true}},
		{name: "certificates only", shortcuts: loftAuditShortcuts{certificates: true}},
		{name: "production", shortcuts: loftAuditProduction},
	} {
		_, gotErr := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, arm.shortcuts)
		if refErr == nil {
			require.NoError(t, gotErr, "%s must not turn a passing audit into a failing one", arm.name)
			continue
		}
		require.Error(t, gotErr, "%s must not turn a failing audit into a passing one", arm.name)
		require.Equal(t, refErr.Error(), gotErr.Error(),
			"%s must reach the identical error, triangle indices included", arm.name)
	}
}

// TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit is the central
// equivalence test: every fixture below — one that PASSES, one with many
// legitimately-skippable far-apart pairs, and one failing each of S7's three
// distinct ways (a zero-shared-vertex pair that touches, a one-shared-vertex
// pair that crosses away from its vertex, a two-shared-vertex pair that
// crosses off its edge) — must reach the identical verdict whether the
// broad-phase runs or not.
func TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit(t *testing.T) {
	t.Run("passes: untwisted box", func(t *testing.T) {
		requireLoftCrossingAuditVerdictsMatch(t, boxLoftVerts(), boxLoftTris())
	})
	t.Run("passes: many far-apart zero-shared-vertex pairs, genuinely skippable", func(t *testing.T) {
		verts, tris := syntheticLoftTriangles(40)
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	})
	t.Run("fails: zero-shared-vertex pair that touches", func(t *testing.T) {
		verts, tris := genuineCrossingFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	})
	t.Run("fails: zero-shared-vertex pair touching only at a shared box boundary", func(t *testing.T) {
		verts, tris := boundaryTouchingFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	})
	t.Run("fails: one-shared-vertex pair crossing away from its vertex", func(t *testing.T) {
		verts, tris := vertexCrossesAwayFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	})
	t.Run("fails: two-shared-vertex pair crossing off its edge", func(t *testing.T) {
		verts, tris := sameSideApexesFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	})
}

// TestLoftCrossingAuditBroadPhaseSkipsFarApartPairs proves the broad-phase is
// not a no-op: over syntheticLoftTriangles' far-apart, zero-shared-vertex
// triangles it actually skips pairs (the call's own work counts them), so the
// equivalence test above is exercising the short-circuit and not vacuously
// agreeing because it never ran.
func TestLoftCrossingAuditBroadPhaseSkipsFarApartPairs(t *testing.T) {
	verts, tris := syntheticLoftTriangles(40)

	work, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
	require.NoError(t, err)
	require.Positive(t, work.skips,
		"40 mutually far-apart triangles must exercise the short-circuit at least once")
}

// TestLoftCrossingAuditWorkCountsAreIndependentPerCall proves the audit's work
// counts are per-call state rather than a shared package counter: two audits
// run concurrently over the same triangle set each report the SAME totals they
// would report alone, and `go test -race` sees no write shared between them.
// A package-level counter fails this test twice over — the race detector flags
// the increments, and either goroutine's total absorbs the other's pairs.
func TestLoftCrossingAuditWorkCountsAreIndependentPerCall(t *testing.T) {
	verts, tris := syntheticLoftTriangles(40)

	alone, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
	require.NoError(t, err)
	require.Positive(t, alone.skips)
	require.Equal(t, 40*39/2, alone.skips+alone.edgeCerts+alone.vertexCerts+alone.classifications,
		"every admitted pair is skipped by a broad-phase tier, admitted by a certificate, or classified")

	var wg sync.WaitGroup
	got := make([]loftAuditWork, 2)
	errs := make([]error, 2)
	for k := range got {
		wg.Go(func() {
			got[k], errs[k] = loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
		})
	}
	wg.Wait()

	for k := range got {
		require.NoError(t, errs[k])
		require.Equal(t, alone, got[k],
			"a concurrent audit must count only its own pairs")
	}
}

// TestLoftCrossingAuditBroadPhaseNeverSkipsARequiredContactPair proves the
// short-circuit is unreachable for a pair required to touch: both
// vertexCrossesAwayFixture (one shared vertex) and sameSideApexesFixture (two
// shared vertices) hold exactly one pair each, so if the call's own skip count
// stays 0 after the audit runs, that one pair was decided by auditLoftPair
// and not by the broad-phase — by construction, since the S7 loop's guard
// gates both tiers behind len(shared) == 0 and neither fixture's pair has
// zero shared vertices.
func TestLoftCrossingAuditBroadPhaseNeverSkipsARequiredContactPair(t *testing.T) {
	t.Run("one shared vertex", func(t *testing.T) {
		verts, tris := vertexCrossesAwayFixture()
		work, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
		require.ErrorIs(t, err, ErrDegenerate)
		require.Zero(t, work.skips,
			"a pair sharing one recorded vertex is required to touch there; the broad-phase must never decide it")
	})

	t.Run("two shared vertices", func(t *testing.T) {
		verts, tris := sameSideApexesFixture()
		work, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
		require.ErrorIs(t, err, ErrDegenerate)
		require.Zero(t, work.skips,
			"a pair sharing two recorded vertices is required to touch along that edge; the broad-phase must never decide it")
	})
}

// TestLoftCrossingAuditBroadPhaseStillCatchesACrossing re-runs
// sameSideApexesFixture — the same-side-apexes crossing docs/loft-design.md
// §13 names — through loftCrossingAudit, the production entry point where the
// broad-phase always runs, and asserts the refusal names the same triangle
// pair auditLoftPair itself finds.
func TestLoftCrossingAuditBroadPhaseStillCatchesACrossing(t *testing.T) {
	verts, tris := sameSideApexesFixture()

	want := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, want, ErrDegenerate)

	got := loftCrossingAudit(newWorkBudget(t.Context()), verts, tris)
	require.ErrorIs(t, got, ErrDegenerate)
	require.Equal(t, want.Error(), got.Error())
}

// chordedWedgeTriangles assembles the F~230 hand-chorded spline wedge's own
// wall-and-cap triangle set — the very verts/tris loftCrossingAudit is handed
// inside a Document.Loft of that shape — by running the same pipeline evalLoft
// runs ahead of the audit (recordProfile, validateLoftRecords, loftPairings,
// assembleLoft) and stopping at its output. It reuses
// loft_chord_calibration_internal_test.go's wedgePlanes/chordedWedgeProfile
// harness, so the geometry is that file's own m=112-station wedge.
//
// Assembling the set once and auditing it twice is what makes the comparison
// below a comparison: both runs see byte-identical geometry, so the only
// difference between their work counts is the broad-phase itself.
func chordedWedgeTriangles(t testing.TB, pts [][2]float64) ([]r3.Vec, [][3]int) {
	t.Helper()
	w, base, top := wedgePlanes(t)
	s0, p0 := chordedWedgeProfile(t, w, base, pts)
	s1, p1 := chordedWedgeProfile(t, w, top, pts)

	profile0, plane0, _, err := recordProfile(s0, p0)
	require.NoError(t, err)
	profile1, plane1, _, err := recordProfile(s1, p1)
	require.NoError(t, err)

	work0, work1 := newFreeformWork(), newFreeformWork()
	offsets, walks0, walks1, err := validateLoftRecords(profile0, profile1, plane0, plane1, nil, work0, work1)
	require.NoError(t, err)
	target, err := loftChordTarget(profile0, profile1, walks0, walks1)
	require.NoError(t, err)
	pairs, _, _, stationRound, err := loftPairings(profile0, profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	require.NoError(t, err)
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	require.NoError(t, err)

	a, err := assembleLoft(t.Context(), pairs, frame0, frame1, plane0, r3.Identity(), stationRound)
	require.NoError(t, err)
	return a.verts, a.tris
}

// loftBroadPhaseWorkDivisor is how much of the S7 pair loop's exact
// classification work the broad-phase must still be removing on the F~230
// wedge below for the filter to count as doing its job: the broad-phase-on run
// may reach auditLoftPair on at most one QUARTER of the pairs the
// broad-phase-off run reaches it on. It is a floor with room under it, not a
// measured residual — the wedge's real reduction is far larger, and this
// divisor exists so an ordinary future change to the triangle set does not
// have to re-pin a number.
const loftBroadPhaseWorkDivisor = 4

// TestLoftCrossingAuditBroadPhaseCutsClassificationWork is the evidence that
// the broad-phase actually removes work: it assembles the F~230 hand-chorded
// spline wedge's triangle set once (m=112 stations) and audits that ONE set
// twice, with the short-circuit off and then on, comparing how many pairs each
// run pushed through auditLoftPair's exact classification.
//
// The instrument is a COUNT, not a wall clock. Both counts depend only on the
// geometry, so they are identical on every host and across repeats, and the
// test fails outright if the broad-phase is disabled, mis-gated, or made
// ineffective — the case where the two runs classify the same pairs. A timing
// assertion could not do that here: the gap between the two arms' wall clocks
// is smaller than the per-host spread
// loft_chord_calibration_internal_test.go's own loftChordBuildCeiling comment
// records for a loft this size, so asserting it would fail the suite on a slow
// host while proving nothing about the code.
//
// The count is NOT a proxy for the runtime and must never be read as one. The
// pairs the broad-phase removes are the cheap ones — a box test rejects them
// in a handful of float comparisons — while the pairs it keeps are the ones
// that were always going to dominate, so the classification reduction this
// test asserts is several times larger than the wall-clock reduction that
// follows from it. BenchmarkLoftCrossingAuditBroadPhase below measures both
// quantities on one host and records what each came to.
func TestLoftCrossingAuditBroadPhaseCutsClassificationWork(t *testing.T) {
	const stations = 112
	fs := wedgeFitSpline(t)
	verts, tris := chordedWedgeTriangles(t, wedgeSplinePoints(fs, stations))

	off, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditReference)
	require.NoError(t, err)
	on, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
	require.NoError(t, err)

	t.Logf("F~230 wedge: stations=%d triangles=%d classifications off=%d on=%d skips=%d edgeCerts=%d vertexCerts=%d",
		stations, len(tris), off.classifications, on.classifications, on.skips, on.edgeCerts, on.vertexCerts)

	require.Zero(t, off.skips,
		"with the short-circuit off, no pair may be skipped")
	require.Zero(t, off.edgeCerts+off.vertexCerts,
		"with the certificates off, no pair may be admitted by one")
	require.Equal(t, off.classifications, on.classifications+on.skips+on.edgeCerts+on.vertexCerts,
		"a shortcut only moves a pair out of the classified column; the pair count is the same either way")
	require.Positive(t, on.classifications,
		"the wedge's own adjacent and nearby pairs must still reach the exact classification")
	require.Less(t, on.classifications, off.classifications/loftBroadPhaseWorkDivisor,
		"the broad-phase must still remove the bulk of the exact classification work")
}

// BenchmarkLoftCrossingAuditBroadPhase times the F~230 wedge audit with the
// short-circuit off and on, over the SAME pre-assembled triangle set, so the
// two arms differ in nothing but the broad-phase. It reports rather than
// asserts: a benchmark has no pass/fail, so it can measure wall clock without
// the host-dependent flakiness a timing assertion would carry, and it does not
// run under a plain go test ./... at all.
//
// Run it with:
//
//	go test . -run '^$' -bench BenchmarkLoftCrossingAuditBroadPhase -benchtime 5x
//
// Each arm reports its own ns/op alongside classifications/op and skips/op, so
// the two quantities the audit's speedup is spoken about in are visible side by
// side. This is the defining site for what they came to. Three repeats of the
// command above, on one otherwise idle development host (linux/amd64, AMD Ryzen
// 9 7900X3D), measured:
//
//	off: 4.05s, 3.30s, 4.31s per audit — 101926 classifications, 0 skips
//	on:  2.44s, 2.43s, 2.53s per audit —  15234 classifications, 86692 skips
//
// So the broad-phase removes 6.7x of the exact classification work and buys
// about 1.4x to 1.7x of wall clock for it on this host. Those are two different
// numbers about two different things, and the smaller one is the runtime claim:
// reading the 6.7x count as a runtime figure overstates the speedup by roughly
// 5x. The absolute figures are host-specific and will not reproduce elsewhere;
// the ratio is the portable part, and even it moves by a few tenths between
// repeats on the same host, which is exactly why no test asserts on it. See
// TestLoftCrossingAuditBroadPhaseCutsClassificationWork for the count the suite
// does assert on.
func BenchmarkLoftCrossingAuditBroadPhase(b *testing.B) {
	const stations = 112
	fs := wedgeFitSpline(b)
	verts, tris := chordedWedgeTriangles(b, wedgeSplinePoints(fs, stations))

	for _, arm := range []struct {
		name      string
		shortcuts loftAuditShortcuts
	}{
		{name: "off", shortcuts: loftAuditReference},
		{name: "broadphase", shortcuts: loftAuditShortcuts{broadPhase: true}},
		{name: "on", shortcuts: loftAuditProduction},
	} {
		b.Run(arm.name, func(b *testing.B) {
			budget := newWorkBudget(b.Context())
			var work loftAuditWork
			for b.Loop() {
				var err error
				work, err = loftCrossingAuditWork(budget, verts, tris, arm.shortcuts)
				if err != nil {
					b.Fatal(err)
				}
			}
			b.ReportMetric(float64(work.classifications), "classifications/op")
			b.ReportMetric(float64(work.skips), "skips/op")
			b.ReportMetric(float64(work.edgeCerts+work.vertexCerts), "certificates/op")
		})
	}
}

// This file is a10-plan.md Part 3 PR 9 Task 1c: an END-TO-END fixture for
// the wall's own three-leg AREA gap (loft_moments.go's
// m.chorded.areaExcess, area()'s own composition line
// `bound = absSumUpper(bound, m.chorded.areaExcess, m.chorded.capAreaExcess)`)
// — the leg zeroing left the ENTIRE repository suite green before this file
// existed. TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap
// (loft_arc_pairs_internal_test.go) already proves computeLoftChordedAllow's
// own RETURNED areaExcess value is sound; it does NOT prove area() actually
// SPENDS it — a regression deleting that composition term, or swapping which
// field it reads, is invisible to a test that calls computeLoftChordedAllow
// directly and never goes through the real Body.Area(). Every test below
// goes through Document.Loft and Body.Area(), the real published path, for
// the BODY UNDER TEST. The dense REFERENCE each test converges against is
// computed separately, described below.
//
// TWO REVIEW FINDINGS fixed the first version of this file:
//
//  1. THE CONVERGENCE CRITERION MUST NOT READ THE BOUND UNDER TEST. An
//     earlier version gated convergence on `diff <= 1% * area.Bound.Base()`
//     — the very quantity a leg-zeroing corrupts. Verified by hand: zeroing
//     the wall leg shrinks the published Area.Bound, which TIGHTENS that
//     convergence gate, which can fail the SWEEP itself (the reference
//     "does not converge") before the enclosure assertion ever runs — a red
//     test for the wrong reason, and a green one on a shape whose own cap
//     term happens to mask the wall leg regardless (the untwisted wedge:
//     confirmed by hand that zeroing the wall leg still leaves it PASSING,
//     residual 5.6408e-3 against a shrunk bound of 1.1521e-2 — cap slack
//     alone covers it, the identical masking
//     TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap's own doc
//     comment already reports for the cone frustum). So convergence below
//     is judged entirely on the REFERENCE's own successive-refinement
//     agreement, at a FIXED relative tolerance untied to anything this file
//     is checking, and the untwisted-wedge fixture is kept only as a
//     baseline — TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference
//     and TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference are the
//     two that are actually load-bearing for the wall leg.
//  2. THE REFERENCE MUST NOT PAY THE PRODUCTION AUDIT. Routing every
//     reference count through Document.Loft made this file cost 323s (the
//     package suite was about 65s before it), because each hand-chorded
//     build pays loft_audit.go's O(F^2) exact-rational crossing-audit cost
//     for a manifold proof the REFERENCE never needed — it is a plain
//     surface-area sum, not a solid this file certifies watertight.
//     denseChordedArea below computes the identical Table B geometry
//     (loft_build.go's assembleLoft: the diagonal-split lower/upper
//     triangle pair per cell, both caps' own shoelace) directly, in plain
//     float64 arithmetic, with no sketch and no Document.Loft call at all —
//     which is also a cleaner independence for a reference to have, and
//     removes the audit cost entirely, so the sweep can run to a much
//     finer, genuinely converged count for a small fraction of the
//     original cost. The BODY UNDER TEST is unaffected: every test still
//     builds it through the real Document.Loft and reads its Area from the
//     real Body.Area().
//
// FALSIFICATION LEDGER (verified by hand during review: temporarily zero
// the named leg in loft_moments.go's `area()`, rerun the named test,
// confirm RED, restore). Each entry quotes the ACTUAL failing assertion
// line, not merely "RED":
//
//   - m.chorded.areaExcess (the wall leg) zeroed
//     (`bound = absSumUpper(bound, 0, m.chorded.capAreaExcess)`):
//     TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.002446194848033656" is not less than or equal to "0.0004608435108288873"`
//     (residual=2.4462e-3, shrunk bound=4.6084e-4) — on "the loft's own
//     Area must enclose the converged densely-chorded reference on a
//     wall-dominated shape", the enclosure assertion itself.
//     TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.031970386054553046" is not less than or equal to "0.006082653033925895"`
//     (residual=3.1970e-2, shrunk bound=6.0827e-3) — on the same enclosure
//     assertion, "...under a large in-plane shear".
//     TestLoftArcWedgeAreaBoundEnclosesConvergedReference (the untwisted,
//     radius-5 wedge) stays GREEN under this same leg-zeroing (residual
//     5.7333e-3 against a shrunk bound of 1.1521e-2) — its own cap term is
//     large enough, relative to its short wall, to mask a missing wall term
//     entirely; it is kept as a baseline sanity check, never as the wall
//     leg's own falsifier.
//   - m.chorded.capAreaExcess (the cap leg) zeroed
//     (`bound = absSumUpper(bound, m.chorded.areaExcess, 0)`):
//     TestLoftArcWedgeAreaBoundEnclosesConvergedReference fails —
//     `"0.0057332610270179885" is not less than or equal to "0.0038222448743720373"`
//     (residual=5.7333e-3, shrunk bound=3.8222e-3) — on "the loft's own Area
//     must enclose the converged densely-chorded reference", the enclosure
//     assertion. TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference
//     and TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference both
//     stay GREEN under this same leg-zeroing — their own wall term
//     dominates and masks a missing cap term, the complementary shape to
//     the wall-leg case above. The PRE-EXISTING
//     TestLoftConeFrustumWallAreaEnclosed (loft_arc_pairs_internal_test.go)
//     also stays GREEN under this same leg-zeroing (gap 5.9387e-2 against
//     an untouched bound of 5.9587e+2) — CONTRARY to what an earlier
//     version of that test's own doc comment claimed about VIOLATION 1: at
//     its own r0=20 that fixture's bound is dominated by other slack (the
//     wall summation's own sumSlop term at that scale) by four orders of
//     magnitude, so it is not, in fact, a working falsifier for this leg —
//     a finding surfaced here rather than papered over. This file's own
//     untwisted-wedge fixture is the one that actually falsifies the cap
//     leg.

// triArea3 is a plain float64 triangle-area formula in 3D — the same
// (1/2)|cross| form triAreaFloat (loft_arc_pairs_internal_test.go) already
// uses, restated here so this file has no dependency on that one beyond the
// shared package.
func triArea3(a, b, c r3.Vec) float64 {
	return 0.5 * b.Sub(a).Cross(c.Sub(a)).Len()
}

// shoelace2DAbs is the plain (unsigned) shoelace area of a closed 2D
// polygon, vertices in either winding order.
func shoelace2DAbs(ring [][2]float64) float64 {
	sum := 0.0
	n := len(ring)
	for i := range n {
		x0, y0 := ring[i][0], ring[i][1]
		x1, y1 := ring[(i+1)%n][0], ring[(i+1)%n][1]
		sum += x0*y1 - x1*y0
	}
	return math.Abs(sum) / 2
}

// denseChordedArea computes the SAME wall-plus-cap area a hand-chorded
// LineSeg-only wedge loft (chordedWedgeProfile's own outline: origin ->
// botPts[0] -> ... -> botPts[m] -> origin, bottom and top) would publish
// through the real Document.Loft/Body.Area() — loft_build.go's assembleLoft
// own Table B construction (the diagonal-split lower/upper triangle pair
// per wall cell, `{vj,vjn,wjn}` then `{vj,wjn,wj}`) and both caps' own
// shoelace area — computed DIRECTLY, with no sketch and no Document.Loft
// call, so a reference sweep pays none of the O(F^2) crossing-audit cost
// the production build needs for its own manifold proof but a plain area
// reference does not.
//
// botPts/topPts are each the m+1 arc points (or radius/shear variant)
// wedgeCirclePoints(m) and its twins return; the ORIGIN each hand-chorded
// profile also draws through is prepended here, exactly matching
// chordedWedgeProfile's own vertex order. The bottom plane is z=0 with the
// identity (u,v)->(x,y) lift wedgePlanes' own base plane uses; the top
// plane is z=height with the SAME U/V basis (an offset plane, never a
// rotation — wedgePlanes' own CreateOffsetPlane), shifted in-plane by
// (dx, dy) — 0 for the untwisted fixtures, nonzero for the sheared one.
func denseChordedArea(botPts, topPts [][2]float64, height, dx, dy float64) float64 {
	botRing := append([][2]float64{{0, 0}}, botPts...)
	topRing := append([][2]float64{{0, 0}}, topPts...)
	n := len(botRing)

	bot := func(p [2]float64) r3.Vec { return r3.NewVec(p[0], p[1], 0) }
	top := func(p [2]float64) r3.Vec { return r3.NewVec(p[0]+dx, p[1]+dy, height) }

	wall := 0.0
	for j := range n {
		jn := (j + 1) % n
		vLo, vHi := bot(botRing[j]), bot(botRing[jn])
		wLo, wHi := top(topRing[j]), top(topRing[jn])
		wall += triArea3(vLo, vHi, wHi) + triArea3(vLo, wHi, wLo)
	}

	return wall + shoelace2DAbs(botRing) + shoelace2DAbs(topRing)
}

// convergedDenseArea sweeps denseChordedArea at increasing station counts
// and returns the value at the finest count once two SUCCESSIVE counts
// agree to within a FIXED relative tolerance of the reference's own value —
// never of refBound, the quantity a test built on this helper is checking
// (finding 1 in this file's own header comment). Because denseChordedArea
// pays no audit cost, the sweep can run to genuine convergence cheaply: at
// convergeRelTol = 1e-9 every fixture in this file converges by m <= 16384,
// in milliseconds.
func convergedDenseArea(t *testing.T, botPtsAt, topPtsAt func(m int) [][2]float64, height, dx, dy float64) float64 {
	t.Helper()
	const convergeRelTol = 1e-9
	var prev float64
	havePrev := false
	for _, m := range []int{256, 512, 1024, 2048, 4096, 8192, 16384, 32768} {
		cur := denseChordedArea(botPtsAt(m), topPtsAt(m), height, dx, dy)
		if havePrev {
			diff := math.Abs(cur - prev)
			if diff <= convergeRelTol*math.Abs(cur) {
				t.Logf("dense reference converged at m=%d: area=%.15g diff-from-previous=%.6e (relative %.3e)", m, cur, diff, diff/math.Abs(cur))
				return cur
			}
		}
		prev, havePrev = cur, true
	}
	t.Fatalf("dense-chorded area reference did not converge to within relative %.3g", convergeRelTol)
	return 0
}

// TestLoftArcWedgeAreaBoundEnclosesConvergedReference is the untwisted
// A10a-shaped baseline: the real arc-paired loft's own Area must enclose
// the densely-converged reference — not merely the closed-form
// quarter-cylinder value TestLoftArcWedgeBuildsAndMatchesClosedForm already
// checks for Volume, and not merely computeLoftChordedAllow's own RETURNED
// value (TestComputeLoftChordedAllowWallLegEnclosesConeFrustumGap) — this
// goes through the real Document.Loft and Body.Area() end to end. Per this
// file's own ledger, this shape's own cap term masks a zeroed wall leg; it
// stands as a baseline sanity check and as the cap leg's own falsifier
// (alongside the pre-existing TestLoftConeFrustumWallAreaEnclosed).
func TestLoftArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	w, base, top := wedgePlanes(t)
	s0, p0 := wedgeArcSketch(t, w, base)
	s1, p1 := wedgeArcSketch(t, w, top)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePointsR(m, wedgeRadius) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, wedgeHeight, 0, 0)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference")
}

// wedgeCirclePointsR returns the m+1 chord vertices on a quarter circle of the
// given radius, k=0..m at k*sweep/m.
func wedgeCirclePointsR(m int, radius float64) [][2]float64 {
	pts := make([][2]float64, m+1)
	for k := 0; k <= m; k++ {
		theta := wedgeSweep * float64(k) / float64(m)
		pts[k] = [2]float64{radius * math.Cos(theta), radius * math.Sin(theta)}
	}
	return pts
}

// wedgePlanesH is wedgePlanes' own height-parametrized twin: two parallel
// planes sharing one U/V basis, offset by height rather than the fixed
// package constant wedgeHeight.
func wedgePlanesH(t *testing.T, height float64) (*sketch.World, *sketch.Plane, *sketch.Plane) {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := w.CreateOffsetPlane(base, height)
	require.NoError(t, err)
	return w, base, top
}

// TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference is this
// file's own primary Task 1c fixture: radius 1, height 60. Wall area scales
// with height and cap area with radius^2, so this shape's own cap excess is
// two orders of magnitude smaller than its wall excess — the wall leg is
// LOAD-BEARING here, unlike the untwisted baseline or the big-radius cone
// frustum fixture (both of whose cap slack masks a zeroed wall term, this
// file's own ledger).
func TestLoftTallThinArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	const radius, height = 1.0, 60.0
	w, base, top := wedgePlanesH(t, height)
	s0, p0 := wedgeArcSketchR(t, w, base, radius)
	s1, p1 := wedgeArcSketchR(t, w, top, radius)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePointsR(m, radius) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, height, 0, 0)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("tall-thin arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference on a wall-dominated shape")
}

// wedgeArcSketchShiftedR is wedgeArcSketchR's own sheared twin: the true
// (uncorded) quarter-arc outline of the given radius, translated in-plane by
// (dx, dy).
func wedgeArcSketchShiftedR(t *testing.T, w *sketch.World, plane *sketch.Plane, radius, dx, dy float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	origin := s.CreatePoint(dx, dy)
	s.Fix(origin)
	px := s.CreatePoint(radius+dx, dy)
	py := s.CreatePoint(dx, radius+dy)
	s.CreateLine(origin, px)
	s.CreateLine(py, origin)
	s.CreateArc(origin, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference is this file's
// own shear fixture: radius 1, height 60 — the SAME wall-dominated shape as
// the tall-thin fixture above, so the wall leg stays load-bearing (a
// radius-5 shear alone left the cap and other bound terms with enough slack
// to mask a zeroed wall leg too, verified by hand and corrected here) — with
// the top section's own construction points ALSO translated by (3, -2),
// several times the wedge's own radius, against the bottom section: the
// case where the ruling tilts hardest against the section tangent
// (cellTwistAreaAllow's own twist leg, a10-plan.md's own wording for this
// task).
func TestLoftShearedArcWedgeAreaBoundEnclosesConvergedReference(t *testing.T) {
	const radius, height = 1.0, 60.0
	const dx, dy = 3.0, -2.0
	w, base, top := wedgePlanesH(t, height)
	s0, p0 := wedgeArcSketchR(t, w, base, radius)
	s1, p1 := wedgeArcSketchShiftedR(t, w, top, radius, dx, dy)
	doc := New()
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	area, err := body.Area()
	require.NoError(t, err)

	ptsAt := func(m int) [][2]float64 { return wedgeCirclePointsR(m, radius) }
	ref := convergedDenseArea(t, ptsAt, ptsAt, height, dx, dy)

	residual := math.Abs(area.Value.Base() - ref)
	t.Logf("sheared arc wedge area: value=%.10g bound=%.6e ref=%.10g residual=%.6e", area.Value.Base(), area.Bound.Base(), ref, residual)
	require.LessOrEqual(t, residual, area.Bound.Base(),
		"the loft's own Area must enclose the converged densely-chorded reference under a large in-plane shear")
}
