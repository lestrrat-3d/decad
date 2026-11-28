package meshbool

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestGateSkippingEmptyMemoMatchesLookups holds the gate that skips memo
// lookups on an untouched memo to the gate that looks up every pair: the same
// error, refusal contacts, context polls and stored memo, on prisms that
// touch, nearly touch, cross and stand apart, and when the context is
// cancelled at each of the first polls.
//
// Leg shown to fail: dropping the memo Store of a skipped gather changes the
// stored memo of every case that classifies a pair.
func TestGateSkippingEmptyMemoMatchesLookups(t *testing.T) {
	t.Parallel()
	for _, c := range skipGateCases(t) {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			want := runSkipGate(t, c, nil, false, 0, nil)
			got := runSkipGate(t, c, nil, true, 0, nil)
			requireSameSkipRun(t, want, got, "full run")
			for cancelAt := int64(1); cancelAt <= min(want.polls, 40); cancelAt++ {
				requireSameSkipRun(t,
					runSkipGate(t, c, nil, false, cancelAt, nil),
					runSkipGate(t, c, nil, true, cancelAt, nil), "cancel at a poll")
			}
			evidence := func(*topo.Face, *topo.Face) (bool, error) { return false, nil }
			requireSameSkipRun(t, runSkipGate(t, c, nil, false, 0, evidence), runSkipGate(t, c, nil, true, 0, evidence),
				"with evidence")
		})
	}
}

// TestGateKeepsLookupsOnTouchedMemo runs the gate a second time on the memo
// the first run left. The second run must not skip its lookups, and it must
// match the run that never skips: every pair is a memo hit, which the
// consumer reads without a worker measurement.
func TestGateKeepsLookupsOnTouchedMemo(t *testing.T) {
	t.Parallel()
	for _, c := range skipGateCases(t) {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			first := runSkipGate(t, c, nil, false, 0, nil)
			if first.memo.untouched() {
				t.Skip("the gate classified no pair")
			}
			size := len(first.memo.Values)
			again := runSkipGate(t, c, first.memo, true, 0, nil)
			reference := runSkipGate(t, c, runSkipGate(t, c, nil, false, 0, nil).memo, false, 0, nil)
			requireSameSkipRun(t, reference, again, "second run")
			require.Equal(t, size, len(again.memo.Values),
				"hits store nothing, so the memo does not grow")
		})
	}
}

// TestContactMemoUntouched pins which states count as untouched: only one on
// which no Store ever happened, in every storage mode.
func TestContactMemoUntouched(t *testing.T) {
	t.Parallel()
	cases := skipGateCases(t)
	c := cases[0]
	require.False(t, (*ContactMemo)(nil).untouched())
	require.True(t, NewContactMemo(c.a, c.b).untouched())
	require.True(t, (&ContactMemo{Ma: c.a, Mb: c.b, Cols: len(c.b.Tris)}).untouched(), "a replay shell")

	none := NewContactMemo(c.a, c.b)
	none.Store(0, 0, TriContact{Kind: ContactNone})
	require.False(t, none.untouched(), "a stored ContactNone")

	far := NewContactMemo(c.a, c.b)
	far.Store(-1, 0, TriContact{Kind: ContactNone})
	require.False(t, far.untouched(), "a stored key outside the product")

	value := NewContactMemo(c.a, c.b)
	value.Store(0, 0, TriContact{Kind: ContactSegment})
	require.False(t, value.untouched(), "a stored contact with a value")

	block := NewContactMemo(c.a, c.b)
	block.BlockIndex = make([]uint32, 1)
	require.False(t, block.untouched(), "block mode with an empty overflow map")

	dense := NewContactMemo(c.a, c.b)
	for i := range len(c.a.Tris) {
		for j := range len(c.b.Tris) {
			dense.Store(i, j, TriContact{Kind: ContactNone})
		}
	}
	require.NotNil(t, dense.Dense)
	require.False(t, dense.untouched(), "a dense table")
}

// TestSkippedLookupNeedsDistinctPairs shows why the skip needs each facet pair
// once: a gather over a face pair that a memo already holds stores nothing
// when it looks up, and stores every pair a second time when it does not.
// The gate never gathers a face pair twice, and it skips only on a memo it
// finds untouched.
func TestSkippedLookupNeedsDistinctPairs(t *testing.T) {
	t.Parallel()
	c := skipGateCases(t)[2] // the crossing prisms
	fis, fjs := AllFacets(c.a), AllFacets(c.b)
	gather := func(noLookup bool, memo *ContactMemo) {
		ctx := WithContactWorkers(t.Context(), 1)
		_, deferred, err := gatherNearContactsLog(ctx, c.a, fis, c.b, fjs, 0.2, memo,
			NearContactRunner(0.2), ContactBatchSize, nil, noLookup)
		require.NoError(t, err)
		require.False(t, deferred)
	}
	looked := NewContactMemo(c.a, c.b)
	gather(false, looked)
	once := len(looked.Values)
	require.Positive(t, once, "the fixture must store a contact with a value")
	gather(false, looked)
	require.Equal(t, once, len(looked.Values), "lookups hit, so nothing is stored again")

	skipped := NewContactMemo(c.a, c.b)
	gather(true, skipped)
	require.Equal(t, once, len(skipped.Values), "an untouched memo: the skip stores what the lookup path stores")
	gather(true, skipped)
	require.Equal(t, 2*once, len(skipped.Values), "a repeated pair stores twice when lookups are skipped")
}

// TestGateFacesPartitionFacets holds the premise of the skip: the facets of
// an operand fall in exactly one face group each, so two face pairs share no
// facet pair.
func TestGateFacesPartitionFacets(t *testing.T) {
	t.Parallel()
	for _, c := range skipGateCases(t) {
		for _, m := range []struct {
			mesh *topo.Mesh
			bm   *BoolMesh
		}{{c.ma, c.a}, {c.mb, c.b}} {
			groups, err := facesOfMesh(newTestBudget(t), m.mesh, m.bm.Boxes, m.bm.FaceSurface)
			require.NoError(t, err)
			seen := map[int]struct{}{}
			sources := map[*topo.Face]struct{}{}
			for _, g := range groups {
				_, dup := sources[g.source]
				require.False(t, dup, "%s: a face is grouped twice", c.name)
				sources[g.source] = struct{}{}
				for _, f := range g.facets {
					_, dup := seen[f]
					require.False(t, dup, "%s: facet %d is in two groups", c.name, f)
					seen[f] = struct{}{}
				}
			}
			require.Len(t, seen, len(m.mesh.Triangles), "%s: every facet is grouped", c.name)
		}
	}
}
