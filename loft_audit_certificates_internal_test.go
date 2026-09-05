package decad

import (
	"context"
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests loft_audit.go's ACCEPT-ONLY certificates — the shortcuts
// loftAuditShortcuts.certificates gates — against the audit's own reference
// path (loftAuditReference, defined with the rest of the shortcut vocabulary
// in loft_audit_internal_test.go). Every assertion here is one of two kinds:
//
//   - AGREEMENT. The certificate arm and the reference arm reach the identical
//     verdict on the same triangle set. A certificate may only change how soon
//     a pair's verdict is reached, never which verdict it is, and the
//     reference arm is the one that never runs a line of certificate code.
//   - EXECUTION. The certificate actually fired, and the pair it decided did
//     NOT reach the exact classification. Without this an agreement assertion
//     passes vacuously on a certificate that never runs.
//
// The fixtures are hand-built triangle sets rather than real lofts, because
// the cases a certificate must distinguish — a shared edge with the apex one
// ULP off the other triangle's plane, a shared vertex with a crossing
// elsewhere — are ones no ordinary loft produces on demand.

// FALSIFICATION LOG (docs/loft-design.md's own "prove the mechanism can fail"
// discipline). Each leg below was actually broken in loft_audit.go, this
// package re-run to confirm a RED failure, then reverted before this file was
// committed. Every leg names the test that caught it.
//
// Certificate A:
//
//   - "admit every shared-edge pair": the accept was hoisted above the exact
//     coplanarity sign, so a coplanar pair took it too.
//     TestLoftCrossingAuditEdgeCertificateNeverDecidesACoplanarPair went RED
//     on all three of its cases, and so did
//     TestLoftCrossingAuditRejectsSameSideApexes and the shared-edge arm of
//     TestLoftCrossingAuditBroadPhaseAgreesWithTheFullAudit — the certificate
//     was admitting two coplanar triangles overlapping in area.
//   - "widen the guard to one shared vertex": the shared-count test was
//     relaxed from two to at least one.
//     TestLoftCrossingAuditAdmitsUntwistedBox and
//     TestLoftCrossingAuditBroadPhaseCutsClassificationWork went RED.
//   - "never fire": the accept was made unreachable.
//     TestLoftCrossingAuditEdgeCertificateAdmitsAFoldedSharedEdge and
//     TestLoftCrossingAuditEdgeCertificateIsSymmetric went RED on their work
//     counts, which is what stops an agreement assertion passing vacuously.
//
// Certificate B:
//
//   - "admit every shared-vertex pair": isolatedSharedVertex was made to
//     return true unconditionally.
//     TestLoftCrossingAuditVertexCertificateNeverDecidesACrossingPair and the
//     three vertex fixtures in
//     TestLoftCrossingAuditCertificatesAgreeUnderScaleAndTranslation went RED.
//   - "count an on-plane vertex as one side": the strict sign test was
//     relaxed to a non-strict one. sharedVertexOnPlaneFixture went RED
//     through the same two tests — this is the leg that fixture exists for.
//   - "read one orientation only": the second isolatedSharedVertex call was
//     deleted. TestLoftCrossingAuditVertexCertificateIsSymmetric went RED on
//     its swapped-order cases, which is why isolatedSharedVertexFixture is
//     built so that only one orientation proves it.

// foldedSharedEdgeVerts and foldedSharedEdgeTris are certificate A's own case:
// two nondegenerate triangles sharing the vertex indices 0 and 1 — the segment
// (0,0,0)-(1,0,0) — folded about it, so triangle B's apex (0,1,1) is off
// triangle A's z=0 plane. Their exact intersection is precisely the shared
// edge, which is what the pair's recorded adjacency expects.
func foldedSharedEdgeVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 1, 1),
	}
}

func foldedSharedEdgeTris() [][3]int {
	return [][3]int{{0, 1, 2}, {0, 1, 3}}
}

// coplanarSharedEdgeFixture is the pair certificate A must NOT decide: the two
// triangles share the same edge (0,0,0)-(1,0,0) but both lie in z=0, so no
// exact sign proves their planes distinct. Their apexes fall on opposite sides
// of the edge's supporting line, so triTriCoplanarSharedEdge — the branch that
// predates the certificates — is what admits them.
func coplanarSharedEdgeFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0.5, -1, 0),
	}
	return verts, [][3]int{{0, 1, 2}, {0, 1, 3}}
}

// coplanarSameSideEdgeFixture is the refusal coplanarSharedEdgeFixture's own
// comment names: the same shared edge with BOTH apexes above the edge's
// supporting line, so the two closed triangles overlap in positive area rather
// than meeting only along the edge. No such solid exists (S7).
func coplanarSameSideEdgeFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0.5, 1, 0),
	}
	return verts, [][3]int{{0, 1, 2}, {0, 1, 3}}
}

// isolatedSharedVertexFixture is certificate B's own case, built ASYMMETRIC on
// purpose: two nondegenerate triangles share the vertex index 0 — the point
// (0,0,0) — and triangle B's two other vertices both sit strictly above
// triangle A's z=0 plane, so B meets that plane only at the shared vertex and
// the pair's whole intersection is that one point.
//
// Only ONE of the two orientations proves it. Triangle A's two other vertices
// fall on OPPOSITE sides of triangle B's own plane (its exact normal is
// (1,-1,0)), so reading A against B's plane decides nothing. A certificate
// that tried a single orientation would therefore admit this pair in one
// triangle order and miss it in the other.
func isolatedSharedVertexFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(1, 1, 1),
		r3.NewVec(2, 2, 3),
	}
	return verts, [][3]int{{0, 1, 2}, {0, 3, 4}}
}

// sharedVertexCrossingFixture is certificate B's own refusal: the two
// triangles share the vertex index 0, but triangle B's other two vertices
// straddle triangle A's z=0 plane, so B crosses that plane along a segment
// running into A's interior instead of touching it at the shared vertex
// alone. Neither orientation of the certificate can fire — A's own two other
// vertices straddle B's plane in turn — so the pair reaches the exact
// classification, which refuses it.
func sharedVertexCrossingFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0.3, 0.3, 1),
		r3.NewVec(0.3, 0.3, -1),
	}
	return verts, [][3]int{{0, 1, 2}, {0, 3, 4}}
}

// sharedVertexOnPlaneFixture is the variant that separates a STRICT sign from
// a merely non-negative one: triangle B shares the vertex index 0, its vertex
// 3 lies exactly ON triangle A's z=0 plane and inside A, and its vertex 4 sits
// strictly above. B therefore meets A along the whole segment from the shared
// vertex to (0.5, 0.2, 0), which the pair's shared vertex does not expect. A
// certificate that counted the on-plane vertex as being on the positive side
// would admit this pair.
func sharedVertexOnPlaneFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0.5, 0.2, 0),
		r3.NewVec(0.3, 0.3, 1),
	}
	return verts, [][3]int{{0, 1, 2}, {0, 3, 4}}
}

// duplicateTriangleFixture holds the same three vertex indices twice, and
// reversedTriangleFixture holds them twice with opposite winding. Both make a
// pair sharing THREE indices, which no adjacency expects: neither the
// broad-phase (gated on zero shared indices) nor either certificate (gated on
// two and on one) may touch it, and the audit refuses it under S7.
func duplicateTriangleFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)}
	return verts, [][3]int{{0, 1, 2}, {0, 1, 2}}
}

func reversedTriangleFixture() ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)}
	return verts, [][3]int{{0, 1, 2}, {0, 2, 1}}
}

// nearlyCoplanarSharedEdgeFixture is foldedSharedEdgeVerts' hardest variant:
// triangle B's apex is lifted off triangle A's plane by ONE ULP of 1.0, so the
// pair is noncoplanar by an exact rational hair. The float corners are
// indistinguishable from a coplanar pair to any tolerance, which is exactly
// why certificate A reads an exact sign and never a residual.
func nearlyCoplanarSharedEdgeFixture() ([]r3.Vec, [][3]int) {
	verts := foldedSharedEdgeVerts()
	verts[3] = r3.NewVec(0, 1, math.Nextafter(1, 2)-1)
	return verts, foldedSharedEdgeTris()
}

// reverseWinding returns tris with every triangle's last two indices swapped,
// which reverses each triangle's orientation and therefore the sign of every
// exact normal the audit derives from it. A certificate that reads a sign
// without proving what that sign means would come apart here.
func reverseWinding(tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for i, tri := range tris {
		out[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	return out
}

// swapPairOrder returns a two-triangle set with the two triangles exchanged,
// so the pair the audit reads as (i, j) becomes (j, i). Both certificates
// read one triangle's plane against the other's vertices, so an asymmetric
// implementation would disagree with the reference under this permutation.
func swapPairOrder(tris [][3]int) [][3]int {
	return [][3]int{tris[1], tris[0]}
}

// transformVerts applies a uniform scale then a translation to every vertex,
// which is what the "large translations, small scales" fixture row asks for:
// the same combinatorial pair at a coordinate magnitude where a float
// tolerance would decide differently from an exact sign.
func transformVerts(verts []r3.Vec, scale float64, shift r3.Vec) []r3.Vec {
	out := make([]r3.Vec, len(verts))
	for i, v := range verts {
		out[i] = r3.NewVec(v.X*scale+shift.X, v.Y*scale+shift.Y, v.Z*scale+shift.Z)
	}
	return out
}

// requireLoftAuditWork runs the audit under the given shortcuts and asserts
// the whole per-outcome breakdown of its pair loop, so a test states which
// path decided every pair rather than only how many pairs there were.
func requireLoftAuditWork(t *testing.T, verts []r3.Vec, tris [][3]int, shortcuts loftAuditShortcuts, want loftAuditWork) error {
	t.Helper()
	work, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, shortcuts)
	require.Equal(t, want, work)
	return err
}

// TestLoftCrossingAuditEdgeCertificateAdmitsAFoldedSharedEdge is certificate
// A's own execution-and-agreement test. The reference arm pushes the folded
// pair through the exact classification and admits it; the certificate arm
// admits the same pair without classifying it at all.
func TestLoftCrossingAuditEdgeCertificateAdmitsAFoldedSharedEdge(t *testing.T) {
	verts, tris := foldedSharedEdgeVerts(), foldedSharedEdgeTris()

	err := requireLoftAuditWork(t, verts, tris, loftAuditReference, loftAuditWork{classifications: 1})
	require.NoError(t, err, "the folded pair meets exactly along its shared edge, so the reference admits it")

	err = requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{edgeCerts: 1})
	require.NoError(t, err, "certificate A must admit the identical pair")
}

// TestLoftCrossingAuditEdgeCertificateIsSymmetric checks the certificate over
// every permutation of triangle order and winding: each one must still fire
// exactly once and still admit.
func TestLoftCrossingAuditEdgeCertificateIsSymmetric(t *testing.T) {
	base := foldedSharedEdgeTris()
	for _, tc := range []struct {
		name string
		tris [][3]int
	}{
		{name: "as built", tris: base},
		{name: "pair order swapped", tris: swapPairOrder(base)},
		{name: "winding reversed", tris: reverseWinding(base)},
		{name: "both", tris: reverseWinding(swapPairOrder(base))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			verts := foldedSharedEdgeVerts()
			requireLoftCrossingAuditVerdictsMatch(t, verts, tc.tris)
			err := requireLoftAuditWork(t, verts, tc.tris, loftAuditProduction, loftAuditWork{edgeCerts: 1})
			require.NoError(t, err)
		})
	}
}

// TestLoftCrossingAuditEdgeCertificateNeverDecidesACoplanarPair pins the
// certificate's own precondition from the other side: a coplanar shared-edge
// pair — admitted or refused — must reach the classified column, because the
// exact sign that proves two planes distinct is zero for it.
func TestLoftCrossingAuditEdgeCertificateNeverDecidesACoplanarPair(t *testing.T) {
	t.Run("apexes on opposite sides: admitted by the coplanar branch", func(t *testing.T) {
		verts, tris := coplanarSharedEdgeFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
		err := requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{classifications: 1})
		require.NoError(t, err)
	})

	t.Run("apexes on the same side: refused for overlapping in area", func(t *testing.T) {
		verts, tris := coplanarSameSideEdgeFixture()
		requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
		err := requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{classifications: 1})
		require.ErrorIs(t, err, ErrDegenerate)
	})

	t.Run("the box loft's own cell diagonal", func(t *testing.T) {
		// boxLoftTris' triangles 0 and 1 are wall cell 0's lower and upper
		// halves: coplanar, because an untwisted quad is planar, and sharing
		// that cell's diagonal. The pair the certificate must leave alone.
		verts, tris := boxLoftVerts(), boxLoftTris()
		outcome, err := auditLoftPairData(newLoftAuditData(verts, tris), tris, 0, 1, loftAuditProduction)
		require.NoError(t, err)
		require.Equal(t, loftPairClassified, outcome,
			"a coplanar shared-edge pair has no proven-distinct planes, so no certificate may decide it")

		// The box's OTHER shared-edge pairs — the rungs between consecutive
		// cells, which meet at a right angle — are the noncoplanar case, so
		// the whole audit still fires the certificate four times.
		work, err := loftCrossingAuditWork(newWorkBudget(t.Context()), verts, tris, loftAuditProduction)
		require.NoError(t, err)
		require.Equal(t, 4, work.edgeCerts,
			"one certificate per rung shared by two consecutive wall cells")
	})
}

// TestLoftCrossingAuditEdgeCertificateAgreesOnAOneULPFold is the fixture row
// that separates an exact sign from a tolerance: the fold is one ULP deep, so
// any float allowance would call the pair coplanar, and the certificate must
// still reach the reference's verdict.
func TestLoftCrossingAuditEdgeCertificateAgreesOnAOneULPFold(t *testing.T) {
	verts, tris := nearlyCoplanarSharedEdgeFixture()

	requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
	err := requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{edgeCerts: 1})
	require.NoError(t, err, "a one-ULP fold is still a proven fold; the pair meets along its shared edge")
}

// TestLoftCrossingAuditCertificatesAgreeUnderScaleAndTranslation sweeps every
// certificate fixture through large translations and small scales. The
// certificate arm must land on the reference arm's verdict at every
// magnitude, since both read exact rational signs of the same coordinates.
func TestLoftCrossingAuditCertificatesAgreeUnderScaleAndTranslation(t *testing.T) {
	fixtures := []struct {
		name    string
		fixture func() ([]r3.Vec, [][3]int)
	}{
		{name: "folded shared edge", fixture: func() ([]r3.Vec, [][3]int) { return foldedSharedEdgeVerts(), foldedSharedEdgeTris() }},
		{name: "coplanar shared edge", fixture: coplanarSharedEdgeFixture},
		{name: "coplanar same side", fixture: coplanarSameSideEdgeFixture},
		{name: "isolated shared vertex", fixture: isolatedSharedVertexFixture},
		{name: "vertex crossing away", fixture: vertexCrossesAwayFixture},
		{name: "vertex crossing through the plane", fixture: sharedVertexCrossingFixture},
		{name: "vertex with another vertex on the plane", fixture: sharedVertexOnPlaneFixture},
		{name: "same-side apexes", fixture: sameSideApexesFixture},
	}

	for _, scale := range []float64{1, 1e-6, 1e6} {
		for _, shift := range []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1e7, -3e7, 5e6)} {
			for _, f := range fixtures {
				verts, tris := f.fixture()
				t.Run(fmt.Sprintf("%s scale=%g shift=%g", f.name, scale, shift.X), func(t *testing.T) {
					requireLoftCrossingAuditVerdictsMatch(t, transformVerts(verts, scale, shift), tris)
				})
			}
		}
	}
}

// TestLoftCrossingAuditVertexCertificateAdmitsAnIsolatedSharedVertex is
// certificate B's own execution-and-agreement test, the vertex twin of
// certificate A's. The reference arm classifies the pair and admits it as a
// point contact at the shared vertex; the certificate arm admits the same
// pair without classifying it.
func TestLoftCrossingAuditVertexCertificateAdmitsAnIsolatedSharedVertex(t *testing.T) {
	verts, tris := isolatedSharedVertexFixture()

	err := requireLoftAuditWork(t, verts, tris, loftAuditReference, loftAuditWork{classifications: 1})
	require.NoError(t, err, "the pair meets only at its shared vertex, so the reference admits it")

	err = requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{vertexCerts: 1})
	require.NoError(t, err, "certificate B must admit the identical pair")
}

// TestLoftCrossingAuditVertexCertificateIsSymmetric runs certificate B over
// every permutation of triangle order and winding. Only ONE of the two
// triangles isolates the shared vertex in this fixture, so a certificate that
// tried a single orientation would fire on half these permutations and miss
// the other half.
func TestLoftCrossingAuditVertexCertificateIsSymmetric(t *testing.T) {
	verts, base := isolatedSharedVertexFixture()
	for _, tc := range []struct {
		name string
		tris [][3]int
	}{
		{name: "as built", tris: base},
		{name: "pair order swapped", tris: swapPairOrder(base)},
		{name: "winding reversed", tris: reverseWinding(base)},
		{name: "both", tris: reverseWinding(swapPairOrder(base))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			requireLoftCrossingAuditVerdictsMatch(t, verts, tc.tris)
			err := requireLoftAuditWork(t, verts, tc.tris, loftAuditProduction, loftAuditWork{vertexCerts: 1})
			require.NoError(t, err)
		})
	}
}

// TestLoftCrossingAuditVertexCertificateNeverDecidesACrossingPair pins
// certificate B's precondition from the other side. Both fixtures share one
// vertex and both are refused, one crossing within a common plane and one
// crossing through it, and neither may be admitted by a certificate.
func TestLoftCrossingAuditVertexCertificateNeverDecidesACrossingPair(t *testing.T) {
	for _, tc := range []struct {
		name    string
		fixture func() ([]r3.Vec, [][3]int)
	}{
		{name: "coplanar, overlapping away from the vertex", fixture: vertexCrossesAwayFixture},
		{name: "noncoplanar, crossing the plane away from the vertex", fixture: sharedVertexCrossingFixture},
		{name: "noncoplanar, with one vertex lying on the plane", fixture: sharedVertexOnPlaneFixture},
	} {
		t.Run(tc.name, func(t *testing.T) {
			verts, tris := tc.fixture()
			requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
			err := requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{classifications: 1})
			require.ErrorIs(t, err, ErrDegenerate)
		})
	}
}

// TestLoftCrossingAuditCertificatesNeverDecideAnUnexpectedSharedCount covers
// the fixture rows for duplicate and reversed triangles: a pair sharing three
// vertex indices is outside every shortcut's guard, so it reaches the
// classified column and is refused there.
func TestLoftCrossingAuditCertificatesNeverDecideAnUnexpectedSharedCount(t *testing.T) {
	for _, tc := range []struct {
		name    string
		fixture func() ([]r3.Vec, [][3]int)
	}{
		{name: "duplicate triangle", fixture: duplicateTriangleFixture},
		{name: "reversed winding of the same triangle", fixture: reversedTriangleFixture},
	} {
		t.Run(tc.name, func(t *testing.T) {
			verts, tris := tc.fixture()
			requireLoftCrossingAuditVerdictsMatch(t, verts, tris)
			err := requireLoftAuditWork(t, verts, tris, loftAuditProduction, loftAuditWork{classifications: 1})
			require.ErrorIs(t, err, ErrDegenerate)
		})
	}
}

// TestLoftCrossingAuditCertificatesPreserveCancellationPrecedence proves a
// shortcut does not change WHICH error the audit reports when the budget is
// already spent: cancellation still wins over any pair verdict, and it wins
// identically with the certificates on and off.
func TestLoftCrossingAuditCertificatesPreserveCancellationPrecedence(t *testing.T) {
	verts, tris := coplanarSameSideEdgeFixture()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	for _, arm := range []struct {
		name      string
		shortcuts loftAuditShortcuts
	}{
		{name: "reference", shortcuts: loftAuditReference},
		{name: "production", shortcuts: loftAuditProduction},
	} {
		t.Run(arm.name, func(t *testing.T) {
			_, err := loftCrossingAuditWork(newWorkBudget(ctx), verts, tris, arm.shortcuts)
			require.ErrorIs(t, err, context.Canceled,
				"a cancelled context outranks the pair verdict on every shortcut setting")
		})
	}
}
