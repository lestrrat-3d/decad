package meshbool

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
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
	ctx := t.Context()
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

// needlePrism is a closed prism whose top, fanned round an inside vertex, holds
// a corner a hair below its neighbours' line. Re-triangulating the top on its
// corners clips the needle ear first.
func needlePrism() []KeptFacet {
	e := math.Ldexp(1, -50)
	pt := func(x, y, z float64) proof.Xpt { return proof.XptOf(r3.Vec{X: x, Y: y, Z: z}) }
	p1, v1, q1, r1 := pt(0, 0, 1), pt(1, -e, 1), pt(2, 0, 1), pt(1, 1, 1)
	c1 := pt(1, 0.25, 1)
	p0, v0, q0, r0 := pt(0, 0, 0), pt(1, -e, 0), pt(2, 0, 0), pt(1, 1, 0)
	f := func(src int, a, b, c proof.Xpt) KeptFacet { return KeptFacet{V: [3]proof.Xpt{a, b, c}, Src: src} }
	return []KeptFacet{
		f(0, v1, q1, c1), f(0, p1, v1, c1), f(0, q1, r1, c1), f(0, r1, p1, c1),
		f(1, v0, p0, r0), f(1, v0, r0, q0),
		f(2, p0, v0, v1), f(2, p0, v1, p1),
		f(3, v0, q0, q1), f(3, v0, q1, v1),
		f(4, q0, r0, r1), f(4, q0, r1, q1),
		f(5, r0, p0, p1), f(5, r0, p1, r1),
	}
}

// TestReplaySimplifyDeclinesWhatLiveFlips pins parity: the live stitch flips the
// needle the simplified top holds, which the bounded stitch cannot, so the
// bounded stitch declines instead of holding a different mesh.
func TestReplaySimplifyDeclinesWhatLiveFlips(t *testing.T) {
	ctx := t.Context()
	kept := needlePrism()
	_, err := StitchFacetsWith(ctx, kept, StitchOptions{SimplifyPlanar: true})
	require.NoError(t, err)
	_, err = replayStitch(t, ctx, kept, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, errReplaySimplifiedMeshNeedsAFlip)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// TestReplaySimplifyStitchBudget pins that the stitch, not only the
// simplification, answers a meter that is too small with its budget error: over
// a sweep of work limits every run either builds the simplified mesh or returns
// a WorkLimitError, never the plain mesh and never another refusal.
func TestReplaySimplifyStitchBudget(t *testing.T) {
	ctx := t.Context()
	kept := dovetailCut(t, 0.125)
	full := proofbound.NewReplayTrialBudget()
	_, err := replayStitch(t, ctx, kept, full)
	require.NoError(t, err)
	total, _ := full.Usage()
	built, refused := 0, 0
	limits := []uint64{total, 2 * total}
	for work := uint64(1); work < total; work += max(total/400, 1) {
		limits = append(limits, work)
	}
	for _, work := range limits {
		b, lerr := proofbound.NewReplayBudgetLimits(work, proofbound.ReplayTrialRecords, proofbound.ReplayTrialBytes)
		require.NoError(t, lerr)
		out, err := replayStitch(t, ctx, kept, b)
		var limit *proofbound.WorkLimitError
		switch {
		case err == nil:
			require.Zero(t, out.Round, "work %d: a built mesh is the simplified one", work)
			built++
		case errors.As(err, &limit):
			refused++
		default:
			require.Failf(t, "not a budget error", "work %d: %v", work, err)
		}
	}
	require.Positive(t, built)
	require.Positive(t, refused)
}
