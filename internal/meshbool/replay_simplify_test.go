package meshbool

import (
	"context"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/coplanarcontact"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replayStitch(t *testing.T, ctx context.Context, kept []KeptFacet, budget *proofbound.ReplayBudget) (*StitchedMesh, error) {
	t.Helper()
	keys, err := NewReplayKeysBounded(ctx, budget)
	require.NoError(t, err)
	return StitchFacetsReplayWithKeys(ctx, kept, budget, keys)
}

// TestReplaySimplifyMatchesLive pins that the bounded stitch holds, vertex for
// vertex, what the live stitch holds when both simplify planar faces: on the
// slanted-slot fixture, where the simplification acts, and on a crossing box
// pair, where it does nothing.
func TestReplaySimplifyMatchesLive(t *testing.T) {
	ctx := coplanarcontact.With(t.Context(), true)
	cross := func() []KeptFacet {
		a := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{})
		b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{X: 0.5, Y: 0.4, Z: 0.3})
		kept, _, err := MeshBooleanStats(t.Context(), OpUnion, a, b, NewContactMemo(a, b))
		require.NoError(t, err)
		return kept
	}
	for name, kept := range map[string][]KeptFacet{"slot": dovetailCut(t, 0.125), "two boxes": cross()} {
		t.Run(name, func(t *testing.T) {
			live, err := StitchFacetsWith(ctx, kept, StitchOptions{SimplifyPlanar: true})
			require.NoError(t, err)
			got, err := replayStitch(t, ctx, kept, proofbound.NewReplayTrialBudget())
			require.NoError(t, err)
			require.Equal(t, live.Verts, got.Verts)
			require.Equal(t, live.Tris, got.Tris)
			require.Equal(t, live.Src, got.Src)
			require.Equal(t, live.VertexBound, got.VertexBound)
			require.Equal(t, live.Round, got.Round)
		})
	}
	// With the switch off the replay holds the unsimplified mesh.
	kept := dovetailCut(t, 0.125)
	plain, err := replayStitch(t, t.Context(), kept, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	simple, err := replayStitch(t, ctx, kept, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.Positive(t, plain.Round)
	require.Zero(t, simple.Round)
}

// TestReplaySimplifyBudgetExhaustion pins that the simplification charges the
// replay meter: a meter too small for it returns the budget error, and a large
// one lets it finish with the simplified mesh.
func TestReplaySimplifyBudgetExhaustion(t *testing.T) {
	kept := dovetailCut(t, 0.125)
	in := plannedMeshOf(t, kept)
	small, err := proofbound.NewReplayBudgetLimits(10, proofbound.ReplayTrialRecords, proofbound.ReplayTrialBytes)
	require.NoError(t, err)
	_, _, err = SimplifyPlanarFacesBudget(t.Context(), small.WorkBudget(t.Context()), in)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	big := proofbound.NewReplayTrialBudget()
	out, changed, err := SimplifyPlanarFacesBudget(t.Context(), big.WorkBudget(t.Context()), in)
	require.NoError(t, err)
	require.True(t, changed)
	require.Less(t, len(out.Xverts), len(in.Xverts))
}

// plannedMeshOf welds kept facets by exact key into the exact stitched mesh
// the simplification reads.
func plannedMeshOf(t *testing.T, kept []KeptFacet) PlanarMesh {
	t.Helper()
	index := map[string]int{}
	var in PlanarMesh
	for _, f := range kept {
		var tri [3]int
		for k := range 3 {
			key := f.V[k].Key()
			v, ok := index[key]
			if !ok {
				v = len(in.Xverts)
				index[key] = v
				in.Xverts = append(in.Xverts, f.V[k])
				in.Approx = append(in.Approx, f.V[k].Vec())
				in.Xbeta = append(in.Xbeta, 0)
			}
			tri[k] = v
			in.Xbeta[v] = max(in.Xbeta[v], f.Beta[k])
		}
		in.Tris = append(in.Tris, tri)
		in.Src = append(in.Src, f.Src)
		in.FacetBound = append(in.FacetBound, 0)
	}
	return in
}
