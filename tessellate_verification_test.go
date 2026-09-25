package decad_test

import (
	"bytes"
	"context"
	"io"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These are docs/tessellation-design.md §14's verification-level obligations.
// The fixture throughout is a fat ring torus, because it is the shape whose
// facet-pair audit refuses at the very chord tolerance the exporter's own
// default picks for it (§3's ceiling), so one body exercises both the identity
// the levels must keep and the reach the lower ones buy.

// fatTorus is a ring torus whose own default export tolerance asks for more
// facets than the facet-pair audit's fixed ceiling admits.
func fatTorus(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return torusBody(t, doc, 8, 6)
}

func TestVerificationLevelsAgreeOnEveryVertexAndIndex(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)
	tol := units.Millimeters(0.2)

	proven, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	drawn, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)

	// The level changes which proofs are published, never the mesh itself.
	require.Equal(t, proven.Vertices(), drawn.Vertices())
	require.Equal(t, proven.Triangles(), drawn.Triangles())
	require.Equal(t, proven.SourceFaces(), drawn.SourceFaces())
	require.NotEmpty(t, drawn.Triangles())

	// Bound is published at every level and is the same proven figure: §10.1's
	// chain is per patch and per vertex and reads no facet pair. It is never
	// pinned to a literal here — the value differs between architectures.
	require.Equal(t, proven.Bound(), drawn.Bound())
	require.Positive(t, drawn.Bound().Mag())

	require.True(t, proven.BoundaryVerified())
	require.True(t, proven.VolumeVerified())
	require.False(t, drawn.BoundaryVerified())
	require.False(t, drawn.VolumeVerified())

	// VerifyBoundary runs the facet-contact audit and stops there.
	audited, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.Equal(t, proven.Vertices(), audited.Vertices())
	require.True(t, audited.BoundaryVerified())
	require.False(t, audited.VolumeVerified())
}

func TestTessellateDefaultsToVerifyAll(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.2))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// A restatement path copies its proof terms off the payload and publishes them
// unconditionally, so the level's own withholding is what keeps a VerifyNone
// mesh from stating a proof the call never ran
// (docs/tessellation-design.md §2). A faceted body — a boolean's own result —
// is the restatement whose payload always carries a volume proof.
func TestVerifyNoneWithholdsTheVolumeProofOnARestatedMesh(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	// The two boxes span different z ranges, so the analytic prism-boolean path
	// declines the pair and the result is a FACETED body — the restatement this
	// test is about.
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBodyAtZ(t, doc, 5, 5, 15, 15, 4, 8)
	fused, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	_, faceted := fused.Edges()[0].Curve().(decad.FacetedCurve)
	require.True(t, faceted, `this fixture must produce a faceted result, not an analytic prism`)

	tol := units.Millimeters(1)
	proven, err := fused.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, proven.VolumeVerified())

	drawn, err := fused.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.False(t, drawn.VolumeVerified())
	// The mesh itself is untouched, and its bound is the same proven figure.
	require.Equal(t, proven.Vertices(), drawn.Vertices())
	require.Equal(t, proven.Triangles(), drawn.Triangles())
	require.Equal(t, proven.Bound(), drawn.Bound())
	// A faceted restatement runs no facet-contact audit, so its boundary
	// reading does not move with the level.
	require.True(t, drawn.BoundaryVerified())
}

// core §12's nil-context rule binds every operation that takes one, and
// Tessellate polls its context through every chording, clearance,
// triangulation and audit phase.
func TestTessellateNilContextIsDegenerate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)

	// A nil-valued variable rather than an untyped nil literal: the call under
	// test is exactly the mistake staticcheck's SA1012 forbids writing, so the
	// value has to reach it past that check.
	var nilCtx context.Context
	_, err := body.Tessellate(nilCtx, units.Millimeters(0.2))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestTessellateRefusesAVerificationLevelItDoesNotKnow(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)
	_, err := body.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.Verification(7)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.Verification(-1)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// The facet-pair ceiling is a work budget, never a statement that the geometry
// cannot be meshed: the tolerance VerifyAll refuses for it is met by a level
// that runs no pair predicate (docs/tessellation-design.md §3).
func TestVerifyNoneMeetsAToleranceTheProvenPathRefuses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := fatTorus(t, doc)
	tol := units.Millimeters(0.0414)

	_, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "exact tests")

	drawn, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.Greater(t, len(drawn.Triangles()), 4000)
	require.False(t, drawn.BoundaryVerified())
	require.Positive(t, drawn.Bound().Mag())
}

// The one-entry cache keys on the level as well as the tolerance
// (docs/tessellation-design.md §1.1). Without it the second call at one
// tolerance would be answered by the first call's mesh, in either direction.
func TestTessellationCacheNeverCrossesVerificationLevels(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.2)

	t.Run("an unverified mesh never answers a verified request", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		body := torusBody(t, doc, 10, 3)
		drawn, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
		require.NoError(t, err)
		require.False(t, drawn.VolumeVerified())

		proven, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, proven.BoundaryVerified())
		require.True(t, proven.VolumeVerified())
	})

	t.Run("a verified mesh never answers an unverified request", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		body := torusBody(t, doc, 10, 3)
		proven, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, proven.VolumeVerified())

		drawn, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
		require.NoError(t, err)
		require.False(t, drawn.BoundaryVerified())
		require.False(t, drawn.VolumeVerified())
	})

	t.Run("one level repeated is still answered from the cache", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		body := torusBody(t, doc, 10, 3)
		first, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
		require.NoError(t, err)
		second, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
		require.NoError(t, err)
		require.Same(t, first, second)
	})
}

// A mesh built below VerifyBoundary keeps every audit that is not a facet-pair
// test, so a facet with no positive area is still refused. The whole mesh is
// the fixture, since the check is what the build runs rather than what a
// caller can hand it.
func TestVerifyNonePublishesOnlyPositiveAreaFacets(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)

	verts, tris := mesh.Vertices(), mesh.Triangles()
	require.NotEmpty(t, tris)
	for i, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		require.Positive(t, b.Sub(a).Cross(c.Sub(a)).Len()/2, `facet %d has no positive area`, i)
	}
}

// STL and OBJ default to VerifyNone, so they can write a dense mesh that the
// facet-pair audit refuses at VerifyAll (docs/api-design.md §11).
func TestExportWritesABodyTheProvenPathRefuses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := fatTorus(t, doc)
	bounds, err := body.Bounds()
	require.NoError(t, err)
	tol := units.Millimeters(bounds.Max.Sub(bounds.Min).Len() / 1000)

	var stl bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &stl, body, tol))
	require.Greater(t, strings.Count(stl.String(), "facet normal"), 4000)

	var obj bytes.Buffer
	require.NoError(t, export.OBJ(t.Context(), &obj, body, tol))
	require.Greater(t, strings.Count(obj.String(), "\nf "), 4000)

	err = export.STL(t.Context(), io.Discard, body, tol, decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "exact tests")
	err = export.OBJ(t.Context(), io.Discard, body, tol, decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// Where both levels succeed the bytes agree: the level publishes proofs, and
// neither writer reads one (docs/tessellation-design.md §1's Determinism row).
func TestExportBytesAreIdenticalAcrossVerificationLevels(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	body := torusBody(t, doc, 10, 3)
	tol := units.Millimeters(0.2)

	var drawn, proven bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &drawn, body, tol))
	require.NoError(t, export.STL(t.Context(), &proven, body, tol, decad.WithVerification(decad.VerifyAll)))
	require.Equal(t, drawn.String(), proven.String())
	require.NotEmpty(t, drawn.String())
}
