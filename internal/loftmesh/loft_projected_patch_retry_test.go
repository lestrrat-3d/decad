package loftmesh

// CI: isolated-tests

import (
	"context"
	"errors"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectedPatchRetryLeavesProvenPatchUnchanged(t *testing.T) {
	g := newRetryGrid(8)
	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchHolds, got.old)
	requireRetryInvariants(t, g, got)
}

func TestProjectedPatchRetryDropsIsolatedBoundaryInversion(t *testing.T) {
	g := newRetryGrid(12)
	g.move(4, 0, 4, 1.6)
	inverted := g.invertedByCross()
	require.Len(t, inverted, 2)

	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchOrientation, got.old)
	requireRetryInvariants(t, g, got)
	require.True(t, got.ok)
	// Exactly the inverted triangles and one vertex ring around them are gone.
	require.Equal(t, g.ringExpectation(), got.reduced.Triangles)
	require.Less(t, len(got.reduced.Triangles), len(g.tris)-len(inverted))
}

func TestProjectedPatchRetryDropsTwoSeparateBoundaryInversions(t *testing.T) {
	g := newRetryGrid(16)
	g.move(3, 0, 3, 1.6)
	g.move(11, 0, 11, 1.6)
	require.Len(t, g.invertedByCross(), 4)

	got := retryDecide(t.Context(), g)
	requireRetryInvariants(t, g, got)
	require.True(t, got.ok)
	require.Equal(t, g.ringExpectation(), got.reduced.Triangles)
}

func TestProjectedPatchRetryRefusesRingThatPinchesBoundary(t *testing.T) {
	// The ring of an inversion next to the patch corner leaves a boundary that
	// touches itself at a vertex.
	g := newRetryGrid(12)
	g.move(10, 0, 10, 1.6)
	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchOrientation, got.old)
	requireRetryInvariants(t, g, got)
	require.False(t, got.ok)
}

func TestProjectedPatchRetryRefusesInteriorFold(t *testing.T) {
	// A vertex pushed across its neighbours inside the patch. Dropping its
	// ring leaves a hole, so the boundary is two loops.
	g := newRetryGrid(12)
	g.move(6, 6, 7.4, 7.4)
	require.NotEmpty(t, g.invertedByCross())
	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchOrientation, got.old)
	requireRetryInvariants(t, g, got)
	require.False(t, got.ok)
}

func TestProjectedPatchRetryRefusesInversionNextToAnotherFold(t *testing.T) {
	// A seam inversion whose ring reaches an interior fold: the interior fold
	// leaves a hole even though the seam one alone would prove.
	g := newRetryGrid(12)
	g.move(3, 0, 3, 1.6)
	g.move(8, 5, 9.4, 6.4)
	got := retryDecide(t.Context(), g)
	requireRetryInvariants(t, g, got)
	require.False(t, got.ok)
}

func TestProjectedPatchRetryRefusesMoreInvertedThanCap(t *testing.T) {
	// Folding a whole boundary row over its neighbour inverts 2n triangles.
	// Dropping them and their ring would leave a patch that proves, so only
	// the cap refuses it.
	g := newRetryGrid(20)
	for x := 0; x <= g.n; x++ {
		g.move(x, 0, float64(x), 1.6)
	}
	require.Greater(t, len(g.invertedByCross()), projectedPatchRetryMaxInverted)
	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchOrientation, got.old)
	requireRetryInvariants(t, g, got)
	require.False(t, got.ok)

	// At the cap the same shape is accepted.
	g = newRetryGrid(projectedPatchRetryMaxInverted / 2)
	for x := 0; x <= g.n; x++ {
		g.move(x, 0, float64(x), 1.6)
	}
	require.Len(t, g.invertedByCross(), projectedPatchRetryMaxInverted)
	got = retryDecide(t.Context(), g)
	requireRetryInvariants(t, g, got)
	require.True(t, got.ok)
}

func TestProjectedPatchRetryRefusesEvenSplitOfSigns(t *testing.T) {
	// The patch holds a grid and a separate mirrored copy of it, so as many
	// triangles are inverted as are not. Dropping the inverted half would leave
	// the first grid, which proves, so only the strict-majority check refuses it.
	g := newRetryGrid(3)
	half := len(g.tris)
	offset := len(g.verts)
	for _, v := range g.verts[:offset] {
		g.verts = append(g.verts, r3.Vec{X: v.X + 100, Y: v.Y, Z: v.Z})
	}
	for _, tri := range g.tris[:half] {
		g.tris = append(g.tris, [3]int{tri[0] + offset, tri[2] + offset, tri[1] + offset})
	}
	require.Len(t, g.invertedByCross(), half)
	require.Equal(t, half, len(g.tris)-half)
	require.LessOrEqual(t, half, projectedPatchRetryMaxInverted)

	first := newRetryGrid(3)
	remaining := uint64(100_000_000)
	require.Equal(t, ProjectedPatchHolds,
		projectedPatchProofLimited(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris,
			ProjectedPatch{Triangles: first.patch().Triangles, Axes: [2]int{0, 1}}, &remaining))

	got := retryDecide(t.Context(), g)
	require.Equal(t, ProjectedPatchOrientation, got.old)
	require.False(t, got.ok)
}

func TestProjectedPatchRetryRefusesStructureFailure(t *testing.T) {
	g := newRetryGrid(8)
	g.move(4, 0, 4, 1.6)
	g.tris[1] = g.tris[0] // a repeated cell fails on structure before the inversion is reached
	got := retryDecide(t.Context(), g)
	require.NotEqual(t, ProjectedPatchOrientation, got.old)
	require.False(t, got.ok)
}

func TestProjectedPatchRetryRefusesIndexOutsideMesh(t *testing.T) {
	g := newRetryGrid(8)
	g.move(4, 0, 4, 1.6)
	patch := g.patch()
	patch.Triangles = append(patch.Triangles, len(g.tris)+3)
	remaining := uint64(100_000_000)
	_, ok := provenProjectedPatch(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, patch, &remaining)
	require.False(t, ok)

	// A bad vertex after the first inverted triangle: the first proof stops at
	// the inversion and has not looked at it yet.
	g = newRetryGrid(8)
	g.move(1, 0, 1, 1.6)
	g.tris[len(g.tris)-1] = [3]int{0, 1, len(g.verts) + 5}
	remaining = 100_000_000
	_, ok = provenProjectedPatch(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, g.patch(), &remaining)
	require.False(t, ok)
}

func TestProjectedPatchRetryKeepsAuditVerdictAndDropsCandidates(t *testing.T) {
	g := newRetryGrid(12)
	g.move(4, 0, 4, 1.6)
	require.True(t, retryDecide(t.Context(), g).ok)

	work, withPatch, without := retryAuditVerdict(t, g, 1<<40)
	require.NoError(t, without)
	require.NoError(t, withPatch)
	noPatchWork, err := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, nil,
		loftGenericShortcuts, 1<<40, len(g.tris), false)
	require.NoError(t, err)
	require.Less(t, work.Candidates, noPatchWork.Candidates)
}

func TestProjectedPatchRetryKeepsDroppedTriangleContactInAudit(t *testing.T) {
	// The moved vertex is lifted so that a dropped triangle, which spans far
	// across the patch, pierces retained triangles. Both audits must refuse it,
	// with the same pair.
	g := newRetryGrid(12)
	g.move(4, 0, 8.3, 8.5)
	g.verts[g.vertex(4, 0)].Z = 0.3
	got := retryDecide(t.Context(), g)
	requireRetryInvariants(t, g, got)
	require.True(t, got.ok)

	_, withPatch, without := retryAuditVerdict(t, g, 1<<40)
	require.Error(t, without)
	require.Error(t, withPatch)
	require.EqualError(t, withPatch, without.Error())
	require.Contains(t, without.Error(), "share no recorded vertex, but make contact")
}

func TestProjectedPatchRetryKeepsWorkLimit(t *testing.T) {
	g := newRetryGrid(12)
	g.move(4, 0, 4, 1.6)
	require.True(t, retryDecide(t.Context(), g).ok)

	_, withPatch, without := retryAuditVerdict(t, g, 10)
	require.Error(t, without)
	require.Error(t, withPatch)
	var wantLimit, gotLimit *proofbound.WorkLimitError
	require.True(t, errors.As(without, &wantLimit))
	require.True(t, errors.As(withPatch, &gotLimit))
	require.Equal(t, wantLimit.Resource, gotLimit.Resource)
	require.Equal(t, wantLimit.Maximum, gotLimit.Maximum)
}

func TestProjectedPatchRetryObeysExhaustedCheckAllowance(t *testing.T) {
	g := newRetryGrid(12)
	g.move(4, 0, 4, 1.6)
	remaining := uint64(0)
	_, ok := provenProjectedPatch(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, g.patch(), &remaining)
	require.False(t, ok)
	require.Zero(t, remaining)
}

func TestProjectedPatchRetryObeysCancellation(t *testing.T) {
	g := newRetryGrid(12)
	g.move(4, 0, 4, 1.6)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	remaining := uint64(100_000_000)
	_, ok := provenProjectedPatch(proofbound.NewWorkBudget(ctx), g.verts, g.tris, g.patch(), &remaining)
	require.False(t, ok)

	err := LoftCrossingAuditBoundedProjectedPatches(proofbound.NewWorkBudget(ctx), g.verts, g.tris,
		[]ProjectedPatch{g.patch()}, len(g.tris), 1<<40)
	require.ErrorIs(t, err, context.Canceled)
}

func TestProjectedPatchRetryMatchesFirstProofOnRandomPatches(t *testing.T) {
	// A fixed pseudo-random corpus: grids with a few vertices moved by up to
	// two cells in the plane and lifted. For each grid the decision must
	// follow the invariants, and the audit verdict with the patch must equal
	// the verdict without any patch.
	rng := rand.New(rand.NewPCG(7, 11))
	var proven, retried, refused int
	for range 120 {
		g := newRetryGrid(10)
		for range 1 + rng.IntN(3) {
			x, y := rng.IntN(g.n+1), rng.IntN(g.n+1)
			if rng.IntN(2) == 0 {
				y = 0
			}
			g.move(x, y, float64(x)+3*rng.Float64()-1.5, float64(y)+3*rng.Float64()-1.5)
			g.verts[g.vertex(x, y)].Z += 0.4 * rng.Float64()
		}
		got := retryDecide(t.Context(), g)
		requireRetryInvariants(t, g, got)
		switch {
		case got.old == ProjectedPatchHolds:
			proven++
		case got.ok:
			retried++
		default:
			refused++
		}
		_, withPatch, without := retryAuditVerdict(t, g, 1<<40)
		require.Equal(t, without == nil, withPatch == nil)
		if without != nil {
			require.EqualError(t, withPatch, without.Error())
		}
	}
	// The corpus must exercise all three outcomes, or it proves nothing.
	require.NotZero(t, proven)
	require.NotZero(t, retried)
	require.NotZero(t, refused)
}
