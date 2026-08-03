package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
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

// TestLoftCrossingAuditRejectsSameSideApexes is docs/loft-design.md §13's own
// worked fixture: p0 frame U=(1,0,0), V=(0,1,0), origin (0,0,0); p1 frame
// U=(-1,0,0), V=(0,1,0), origin (1,0,1); the same local square. Cell 0's two
// triangles share the recorded diagonal V0-W1 but their apexes (V1 and W0)
// fall on the SAME side of that edge's line, so the two triangles overlap in
// area rather than meeting only along the diagonal: S7 refuses.
func TestLoftCrossingAuditRejectsSameSideApexes(t *testing.T) {
	p0 := func(u, v float64) r3.Vec { return r3.NewVec(u, v, 0) }
	p1 := func(u, v float64) r3.Vec { return r3.NewVec(1-u, v, 1) }

	v0, v1 := p0(0, 0), p0(1, 0)
	w0, w1 := p1(0, 0), p1(1, 0)
	verts := []r3.Vec{v0, v1, w0, w1} // 0=V0, 1=V1, 2=W0, 3=W1
	tris := [][3]int{
		{0, 1, 3}, // lower0: V0, V1, W1
		{0, 3, 2}, // upper0: V0, W1, W0
	}

	err := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "triangles 0 and 1",
		"the refusal must name the specific triangle pair the predicate found")

	budget := newWorkBudget(t.Context())
	err = loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestLoftCrossingAuditRejectsGenuineCrossing builds two triangles with NO
// shared vertex index — S7 therefore expects contactNone — that nonetheless
// cross transversally along a positive-length segment on the line x=0, y=0:
// triangle A lies wholly in the y=0 plane, triangle B wholly in the x=0
// plane, and their own (x,z)/(y,z) footprints overlap on that shared line
// for z in [0,1]. The audit must refuse this as S7, naming the pair it found.
func TestLoftCrossingAuditRejectsGenuineCrossing(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(-1, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 2), // triangle A (y=0 plane)
		r3.NewVec(0, -1, 1), r3.NewVec(0, 1, 1), r3.NewVec(0, 0, -1), // triangle B (x=0 plane)
	}
	tris := [][3]int{
		{0, 1, 2},
		{3, 4, 5},
	}

	err := auditLoftPair(verts, tris, 0, 1)
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "triangles 0 and 1",
		"the refusal must name the specific triangle pair the predicate found")

	budget := newWorkBudget(t.Context())
	err = loftCrossingAudit(budget, verts, tris)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestLoftCrossingAuditRejectsVertexPairThatCrossesAway builds two coplanar
// triangles sharing exactly one recorded vertex (index 0) whose interiors
// nonetheless overlap in positive area away from that vertex — the case
// docs/loft-design.md §6 calls out: "vertex-sharing pairs need this check
// because they can cross away from their recorded vertex."
func TestLoftCrossingAuditRejectsVertexPairThatCrossesAway(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(1, 0, 2), // triangle A
		r3.NewVec(1, 0, -1), r3.NewVec(1, 0, 3), // triangle B's other two corners
	}
	tris := [][3]int{
		{0, 1, 2},
		{0, 3, 4},
	}

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
