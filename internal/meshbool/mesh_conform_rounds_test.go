package meshbool

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

func conformRoundsMeshes() []approxMesh {
	var out []approxMesh
	for seed := range uint64(300) {
		out = append(out, randomApproxMesh(rand.New(rand.NewPCG(seed, 53))))
		out = append(out, twinEdgeMesh(rand.New(rand.NewPCG(seed, 59)), 10+int(seed%30)))
	}
	return out
}

// TestConformRoundsMatchesPerRound holds the multi-round pass that carries its
// scan and edge memo to the pass that rebuilds both each round: same
// triangles, sources, bounds, xbeta, error and number of steps charged. Some
// mesh must need three or more rounds, so a later round meets triangles an
// earlier round produced.
//
// Leg shown to fail: dropping the StepN calls that stand in for the scan build
// in conformSession.once changes the step count of every mesh with a second round.
func TestConformRoundsMatchesPerRound(t *testing.T) {
	t.Parallel()
	multi, deep := 0, 0
	for i, base := range conformRoundsMeshes() {
		want := runConformRounds(t, base, 0, true)
		got := runConformRounds(t, base, 0, false)
		require.Equal(t, want.err, got.err, "mesh %d", i)
		require.Equal(t, want.steps, got.steps, "mesh %d", i)
		requireSameMesh(t, uint64(i), want.mesh, got.mesh)
		if want.err != nil {
			continue
		}
		if want.rounds >= 3 {
			multi++
		}
		if want.rounds >= 4 {
			deep++
		}
	}
	t.Logf("%d meshes needed three or more rounds, %d four or more", multi, deep)
	require.Positive(t, multi, "no mesh needed a third round")
}

// TestConformRoundsRefusesAtSameStep refuses the shared budget at every step
// count up to the run's total, for meshes of two or more rounds, and requires
// the same refusal, steps charged and mesh state from both loops.
//
// Leg shown to fail: charging one scan-build step too few on a reused scan
// moves every refusal that falls after the first round's scan.
func TestConformRoundsRefusesAtSameStep(t *testing.T) {
	t.Parallel()
	refusals, checked := 0, 0
	for i, base := range conformRoundsMeshes() {
		total := runConformRounds(t, base, 0, true)
		if total.err != nil || total.rounds < 2 || len(base.tris) > 40 {
			continue
		}
		checked++
		for limit := uint64(1); limit <= total.steps+1; limit++ {
			want := runConformRounds(t, base, limit, true)
			got := runConformRounds(t, base, limit, false)
			require.Equal(t, want.err, got.err, "mesh %d limit %d", i, limit)
			require.Equal(t, want.steps, got.steps, "mesh %d limit %d", i, limit)
			requireSameMesh(t, uint64(i), want.mesh, got.mesh)
			if want.err == errConformStep {
				refusals++
			}
		}
		if checked == 25 {
			break
		}
	}
	require.Positive(t, checked, "no multi-round mesh was small enough")
	require.Positive(t, refusals, "no limit refused a run")
}

// TestConformRoundsWalksOnlyNewEdges checks the saving: a later round walks the
// point tree only for edges no earlier round saw, so the walks of a whole run
// equal the distinct undirected edges that ever existed.
func TestConformRoundsWalksOnlyNewEdges(t *testing.T) {
	t.Parallel()
	for i, base := range conformRoundsMeshes() {
		ref := runConformRounds(t, base, 0, true)
		if ref.err != nil || ref.rounds < 3 {
			continue
		}
		m := base.clone()
		approx := base.rounded()
		var session conformSession
		seen := map[[2]int]struct{}{}
		for range ref.rounds {
			for _, tri := range m.tris {
				for k := range 3 {
					a, b := tri[k], tri[(k+1)%3]
					if a != b {
						seen[[2]int{min(a, b), max(a, b)}] = struct{}{}
					}
				}
			}
			_, err := session.once(t.Context(), newTestBudget(t), m.verts, approx, &m.tris, &m.src, &m.bounds, m.xbeta)
			require.NoError(t, err)
		}
		require.Equal(t, len(seen), session.memo.steps.Len(), "mesh %d", i)
		return
	}
	t.Fatal("no mesh needed three rounds")
}
