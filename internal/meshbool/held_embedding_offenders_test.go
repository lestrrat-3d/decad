package meshbool

import (
	"fmt"
	"maps"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// offendersReference is offenders as it stood before facetFailuresAfter: every
// candidate pair of every query facet goes through pairOK, which stores every
// pair it decides. The lockstep test holds offenders to it.
func (e *heldEmbedding) offendersReference(query []int) ([]int, []int, error) {
	movable := make([]bool, len(e.h.Verts))
	vertices := make([]int, 0)
	var failing []int
	for _, i := range query {
		bad, collapsed, err := e.facetFailuresReference(i)
		if err != nil {
			return nil, nil, err
		}
		if !collapsed && len(bad) == 0 {
			continue
		}
		failing = append(failing, i)
		e.addMovable(movable, &vertices, i)
		for _, j := range bad {
			e.addMovable(movable, &vertices, j)
		}
	}
	slices.Sort(vertices)
	return vertices, failing, nil
}

func (e *heldEmbedding) facetFailuresReference(i int) ([]int, bool, error) {
	if zeroXpt(e.normal(i)) {
		return nil, true, nil
	}
	cands, err := e.candidates(i)
	if err != nil {
		return nil, false, err
	}
	var bad []int
	for _, j := range cands {
		if err := e.budget.Step(); err != nil {
			return nil, false, err
		}
		if j == i || e.pairOK(i, j, true) {
			continue
		}
		bad = append(bad, j)
	}
	return bad, false, nil
}

// lockstepHeld is one side of the lockstep run: a repair state and the exact
// numbers of budget steps and Err polls it has taken.
type lockstepHeld struct {
	e     *heldEmbedding
	steps int
	polls int
}

func newLockstepHeld(h HeldRounding) *lockstepHeld {
	c := h
	c.Verts = slices.Clone(h.Verts)
	s := &lockstepHeld{}
	budget := &proofbound.WorkBudget{
		StepFn: func() error { s.steps++; return nil },
		ErrFn:  func() error { s.polls++; return nil },
	}
	s.e = newHeldEmbeddingScan(c, budget, true)
	return s
}

// requireLockstep compares what the two sides have done so far by bounded
// readings only: step and poll counts, stamps, the interned plane count and
// the held vertices' bits. offenders leaves out the pairs a float filter
// decided, so side a's memo may hold fewer entries, or older ones, than side
// b's; every entry of a that the current stamps replay must be b's word.
func requireLockstep(t *testing.T, name string, a, b *lockstepHeld) {
	t.Helper()
	require.Equal(t, b.steps, a.steps, `%s: steps`, name)
	require.Equal(t, b.polls, a.polls, `%s: polls`, name)
	require.Equal(t, b.e.nextGen, a.e.nextGen, `%s: next stamp`, name)
	require.True(t, slices.Equal(b.e.gen, a.e.gen), `%s: stamps`, name)
	require.LessOrEqual(t, len(a.e.pairs), len(b.e.pairs), `%s: memo size`, name)
	differ := 0
	for key, word := range a.e.pairs {
		lo, hi := key>>32, key&(1<<32-1)
		if word&^1 != heldPairWord(a.e.gen[lo], a.e.gen[hi], false) {
			continue
		}
		if w, ok := b.e.pairs[key]; !ok || w != word {
			differ++
		}
	}
	require.Zero(t, differ, `%s: replayable memo entries unlike the reference's`, name)
	require.Equal(t, len(b.e.planes), len(a.e.planes), `%s: interned planes`, name)
	for v := range a.e.h.Verts {
		p, q := a.e.h.Verts[v], b.e.h.Verts[v]
		require.True(t, math.Float64bits(p.X) == math.Float64bits(q.X) &&
			math.Float64bits(p.Y) == math.Float64bits(q.Y) &&
			math.Float64bits(p.Z) == math.Float64bits(q.Z), `%s: held vertex %d`, name, v)
	}
}

// runLockstep drives the repair loop of repairHeldEmbeddingSummary on two
// copies of h, one through offenders and one through offendersReference,
// and compares them after every offenders call and every move. It returns
// how many offenders calls ran and how many vertices moved.
func runLockstep(t *testing.T, name string, h HeldRounding) (int, int) {
	t.Helper()
	a, b := newLockstepHeld(h), newLockstepHeld(h)
	var query []int
	for i, tri := range h.Tris {
		if h.Moved[tri[0]] || h.Moved[tri[1]] || h.Moved[tri[2]] {
			query = append(query, i)
		}
	}
	for _, s := range []*lockstepHeld{a, b} {
		require.NoError(t, s.e.buildGrid())
		s.e.prepareRepair()
		s.e.pairs = make(map[uint64]uint64, 32*len(query))
	}
	calls, moved := 0, 0
	check := query
	offenders := func() ([]int, []int) {
		offA, failA, errA := a.e.offenders(check)
		offB, failB, errB := b.e.offendersReference(check)
		require.NoError(t, errA)
		require.NoError(t, errB)
		calls++
		require.Equal(t, offB, offA, `%s: offenders at call %d`, name, calls)
		require.Equal(t, failB, failA, `%s: failing at call %d`, name, calls)
		requireLockstep(t, fmt.Sprintf("%s call %d", name, calls), a, b)
		return offA, failA
	}
	for stage := range 2 {
		if stage == 1 {
			for _, s := range []*lockstepHeld{a, b} {
				s.e.reachOn = true
				require.NoError(t, s.e.buildGrid())
			}
		}
		for range heldEmbeddingSweeps {
			off, failing := offenders()
			if len(failing) == 0 {
				return calls, moved
			}
			next := map[int]struct{}{}
			for _, f := range failing {
				next[f] = struct{}{}
			}
			changed := false
			for _, v := range off {
				pa, errA := a.e.improve(v)
				pb, errB := b.e.improve(v)
				require.NoError(t, errA)
				require.NoError(t, errB)
				require.Equal(t, pb, pa, `%s: vertex %d placed`, name, v)
				requireLockstep(t, fmt.Sprintf("%s improve %d", name, v), a, b)
				if !pa {
					continue
				}
				moved++
				changed = true
				for _, f := range a.e.inc[v] {
					next[f] = struct{}{}
				}
			}
			check = slices.Sorted(maps.Keys(next))
			if !changed {
				break
			}
		}
		_, failing := offenders()
		if len(failing) == 0 || stage == 1 || h.Reach <= 0 {
			return calls, moved
		}
		check = slices.Sorted(slices.Values(failing))
	}
	return calls, moved
}

// TestHeldOffendersMatchReference runs the repair loop with offenders and
// with the pre-replay copy side by side over seeded sliver stacks, alone,
// beside a tube, and with a positive Reach, and requires every offenders
// call and every move to leave both sides with the same results, step and
// poll counts, stamps, memo and held vertices.
func TestHeldOffendersMatchReference(t *testing.T) {
	t.Parallel()
	var calls, moved, multi, reach int
	for seed := range uint64(20) {
		layers := 6 + int(seed%40)
		cases := map[string]HeldRounding{
			"layered": stitchLayered(rand.New(rand.NewPCG(seed, 1)), layers),
			"tube":    heldTubeWithFolds(24+int(seed%3)*8, layers, seed),
		}
		r := stitchLayered(rand.New(rand.NewPCG(seed, 2)), layers)
		r.Reach = 1e-3
		cases["reach"] = r
		for _, kind := range slices.Sorted(maps.Keys(cases)) {
			c, m := runLockstep(t, fmt.Sprintf("seed %d %s", seed, kind), cases[kind])
			calls += c
			moved += m
			if c > 2 {
				multi++
			}
			if kind == "reach" && c > 2 {
				reach++
			}
		}
	}
	require.Positive(t, moved, `the seeds must move vertices`)
	require.Positive(t, multi, `the seeds must run several offenders passes`)
	require.Positive(t, reach, `the reach seeds must run past the float box`)
	t.Logf("offenders calls %d, moves %d, multi-pass cases %d, reach cases %d", calls, moved, multi, reach)
}
