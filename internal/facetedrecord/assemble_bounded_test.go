package facetedrecord

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func boundedCube(t *testing.T) (Payload, *facetproof.MeshAudit) {
	t.Helper()
	var mesh Mesh
	addCube(&mesh, 0, 2, false, 3)
	audit, err := facetproof.AuditFacetedMesh(t.Context(), mesh.Verts, mesh.Tris)
	require.NoError(t, err)
	return Payload{Verts: mesh.Verts, VertexBound: mesh.VertexBound, Tris: mesh.Tris,
		Src: mesh.Src, Groups: mesh.Groups}, audit
}

func TestBuildWithProofBoundedCubeAndRefusals(t *testing.T) {
	pp, audit := boundedCube(t)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	volume := measurement.Measurement{Value: units.CubicMillimeters(8), Exactness: measurement.Exact}
	volRat := big.NewRat(8, 1)
	legacy, err := BuildWithProof(t.Context(), env, pp, audit, volume, volRat)
	require.NoError(t, err)
	bounded, ok, err := BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(65536),
		env, pp, audit, volume, volRat)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, legacy.Volume, bounded.Volume)
	require.Equal(t, legacy.Area, bounded.Area)
	require.Equal(t, legacy.Centroid, bounded.Centroid)
	require.Equal(t, legacy.Bounds, bounded.Bounds)
	require.Equal(t, legacy.Payload.(Payload).Diameter, bounded.Payload.(Payload).Diameter)
	require.Len(t, topobuild.Faces(bounded), 6)
	require.Len(t, topobuild.Edges(bounded), 12)
	steps := uint64(0)
	counted := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return nil },
		ErrFn:  func() error { return nil },
	}
	_, ok, err = BuildWithProofBounded(t.Context(), counted, env, pp, audit, volume, volRat)
	require.NoError(t, err)
	require.True(t, ok)
	require.Greater(t, steps, uint64(256+96*len(pp.Tris)+32*len(pp.Verts)))
	refused, ok, err := BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(steps-1),
		env, pp, audit, volume, volRat)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, refused)

	refused, ok, err = BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(10),
		env, pp, audit, volume, volRat)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, refused)
	withOrigins := pp
	withOrigins.Groups = append([]Group(nil), pp.Groups...)
	withOrigins.Groups[0].Origins = make([]provenance.Ref, 65537)
	refused, ok, err = BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(65536),
		env, withOrigins, audit, volume, volRat)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, refused)

	oversized := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8192))
	refused, ok, err = BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(65536),
		env, pp, audit, volume, oversized)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, refused)
	badAudit := *audit
	badAudit.Comp = append([]int(nil), audit.Comp...)
	badAudit.Comp[0] = len(audit.Members)
	refused, ok, err = BuildWithProofBounded(t.Context(), proofbound.NewWallWorkBudget(65536),
		env, pp, &badAudit, volume, volRat)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, refused)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	refused, ok, err = BuildWithProofBounded(ctx, proofbound.NewWallWorkBudget(65536),
		env, pp, audit, volume, volRat)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Nil(t, refused)
}

func TestBoundedAssemblyRefusesOldSkewTabAllowanceBeforeAllocation(t *testing.T) {
	// The retained 44-facet, 24-vertex skew-tab case has only 1,854 steps
	// left before the builder. Its storage reserve alone exceeds that.
	err := reserveBoundedAssembly(proofbound.NewWallWorkBudget(1854), 24, 44, 1, 0)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
}
