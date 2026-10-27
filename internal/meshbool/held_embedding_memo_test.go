package meshbool

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// heldPairWord is the memo word pairOK stores for stamps lo and hi.
func heldPairWord(lo, hi uint64, ok bool) uint64 {
	w := lo<<32 | hi<<1
	if ok {
		w |= 1
	}
	return w
}

// heldStampedPair is the pair memo entry as two full stamps and the answer,
// the form the packed word must agree with.
type heldStampedPair struct {
	genLo, genHi uint64
	ok           bool
}

// overlappingHeldPair returns a pair of facets of e whose boxes overlap, the
// lower index first, so pairOK reaches the memo for it.
func overlappingHeldPair(t *testing.T, e *heldEmbedding) (int, int) {
	t.Helper()
	for i := range e.h.Tris {
		for j := i + 1; j < len(e.h.Tris); j++ {
			if BoxesOverlap(e.triBox(i), e.triBox(j)) {
				return i, j
			}
		}
	}
	require.Fail(t, `the fixture must have two facets whose boxes overlap`)
	return 0, 0
}

// TestHeldPairMemoReplaysWhatStampsAllow walks seeded sliver stacks through
// trial moves undone with restamp, as improve makes them, accepted moves and
// pair queries with and without keepFloat, beside a memo that keeps both
// stamps in full. Before every query the packed word must replay exactly when
// the full stamps match, every answer must be the one the pair gives when
// decided afresh, and a pair is stored exactly when the exact predicates
// decided it or keepFloat is set.
func TestHeldPairMemoReplaysWhatStampsAllow(t *testing.T) {
	t.Parallel()
	var replays, decides, trials, skipped, exacts int
	for seed := range uint64(40) {
		h := stitchLayered(rand.New(rand.NewPCG(seed, 4)), 6+int(seed%20))
		e := newHeldEmbedding(cloneStitchHeld(h), proofbound.NewWorkBudget(t.Context()), true)
		require.NoError(t, e.buildGrid())
		e.pairs = map[uint64]uint64{}
		ref := map[uint64]heldStampedPair{}
		var movable []int
		for v, ok := range h.Movable {
			if ok {
				movable = append(movable, v)
			}
		}
		rng := rand.New(rand.NewPCG(seed, 5))
		query := func(i int) {
			cands, err := e.candidates(i)
			require.NoError(t, err)
			for _, j := range cands {
				if j == i || !BoxesOverlap(e.triBox(i), e.triBox(j)) {
					continue
				}
				lo, hi := min(i, j), max(i, j)
				key := uint64(lo)<<32 | uint64(hi)
				r, stored := ref[key]
				wantReplay := stored && r.genLo == e.gen[lo] && r.genHi == e.gen[hi]
				word, packed := e.pairs[key]
				gotReplay := packed && word&^1 == heldPairWord(e.gen[lo], e.gen[hi], false)
				require.Equal(t, wantReplay, gotReplay, `seed %d pair %d-%d replay`, seed, lo, hi)
				keep := rng.IntN(2) == 0
				got := e.pairOK(i, j, keep)
				fresh, exact := e.pairEmbeddedPath(i, j)
				require.Equal(t, fresh, got, `seed %d pair %d-%d answer`, seed, lo, hi)
				if wantReplay {
					replays++
					require.Equal(t, r.ok, got, `seed %d pair %d-%d replayed answer`, seed, lo, hi)
					continue
				}
				if !keep && !exact {
					// A float-decided pair asked without keepFloat leaves the memo alone.
					skipped++
					after, still := e.pairs[key]
					require.Equal(t, packed, still, `seed %d pair %d-%d kept nothing`, seed, lo, hi)
					require.Equal(t, word, after, `seed %d pair %d-%d kept nothing`, seed, lo, hi)
					continue
				}
				decides++
				if exact {
					exacts++
				}
				ref[key] = heldStampedPair{genLo: e.gen[lo], genHi: e.gen[hi], ok: got}
				require.Equal(t, heldPairWord(e.gen[lo], e.gen[hi], got), e.pairs[key], `seed %d pair %d-%d word`, seed, lo, hi)
			}
		}
		for range 300 {
			switch op := rng.IntN(4); {
			case op < 2 && len(movable) > 0:
				v := movable[rng.IntN(len(movable))]
				corners := FloatBoxCorners(e.h.Exact[v])
				c := corners[rng.IntN(len(corners))]
				if _, taken := e.taken[c]; taken {
					continue
				}
				orig, saved := e.h.Verts[v], e.stamps(v)
				e.setVert(v, c)
				for _, f := range e.inc[v] {
					query(f)
				}
				if op == 0 {
					// A trial: v goes back and its facets take back their stamps.
					e.setVert(v, orig)
					e.restamp(v, saved)
					trials++
				}
			default:
				query(rng.IntN(len(e.h.Tris)))
			}
		}
	}
	require.Positive(t, replays, `the walk must replay answers`)
	require.Positive(t, decides, `the walk must decide pairs`)
	require.Positive(t, trials, `the walk must undo trial moves`)
	require.Positive(t, skipped, `the walk must skip float-decided pairs`)
	require.Positive(t, exacts, `the walk must store exactly decided pairs`)
	t.Logf("replays %d, decided %d (%d exactly), skipped %d, trials %d", replays, decides, exacts, skipped, trials)
}

// TestHeldPairMemoDecidesStampsPastTheLimit plants a word holding the wrong
// answer, shows pairOK replays it while the stamps match, and then gives a
// facet a stamp at the limit whose packed form would alias the planted word.
// pairOK must decide that pair afresh and store nothing for it.
func TestHeldPairMemoDecidesStampsPastTheLimit(t *testing.T) {
	t.Parallel()
	h := stitchLayered(rand.New(rand.NewPCG(1, 1)), 8)
	for _, tc := range []struct {
		name           string
		planted        [2]uint64
		stamped        [2]uint64
		aliasesPlanted bool
	}{
		{"lower", [2]uint64{5, 9}, [2]uint64{5 + 1<<32, 9}, true},
		{"higher", [2]uint64{5, 9}, [2]uint64{4, 9 + 1<<31}, true},
		{"both at the limit", [2]uint64{5, 9}, [2]uint64{heldPairStampLimit, heldPairStampLimit}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			e := newHeldEmbedding(cloneStitchHeld(h), proofbound.NewWorkBudget(t.Context()), true)
			e.pairs = map[uint64]uint64{}
			i, j := overlappingHeldPair(t, e)
			want := e.pairEmbeddedOverlapping(i, j)
			key := uint64(i)<<32 | uint64(j)
			planted := heldPairWord(tc.planted[0], tc.planted[1], !want)
			e.pairs[key] = planted
			e.gen[i], e.gen[j] = tc.planted[0], tc.planted[1]
			require.Equal(t, !want, e.pairOK(i, j, true), `a matching word is replayed`)
			e.gen[i], e.gen[j] = tc.stamped[0], tc.stamped[1]
			if tc.aliasesPlanted {
				require.Equal(t, planted&^1, (e.gen[i]<<32|e.gen[j]<<1)&^1, `the stamps must alias the planted word`)
			}
			require.Equal(t, want, e.pairOK(i, j, true), `a stamp at the limit is decided afresh`)
			require.Equal(t, planted, e.pairs[key], `nothing is stored for it`)
			require.Len(t, e.pairs, 1)
		})
	}
}
