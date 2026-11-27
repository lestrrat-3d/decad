package meshbool

// CI: isolated-tests

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestSharedEdgeWedgeVerdicts(t *testing.T) {
	for _, tc := range []struct {
		name         string
		a, b         [2]float64
		zb           [2]float64
		wantOK       bool
		wantA, wantB int
	}{
		{name: "overlapping wedges", a: [2]float64{0, 90}, b: [2]float64{45, 135}, zb: [2]float64{0.5, 1.5},
			wantOK: true, wantA: 1, wantB: 1},
		{name: "disjoint wedges", a: [2]float64{0, 30}, b: [2]float64{100, 130}, zb: [2]float64{0.5, 1.5},
			wantOK: true},
		{name: "second wedge inside the first", a: [2]float64{0, 90}, b: [2]float64{20, 50}, zb: [2]float64{0.5, 1.5},
			wantOK: true, wantB: 2},
		{name: "first wedge inside the second", a: [2]float64{20, 50}, b: [2]float64{0, 90}, zb: [2]float64{0.5, 1.5},
			wantOK: true, wantA: 2},
		{name: "coincident half-plane", a: [2]float64{0, 90}, b: [2]float64{0, 45}, zb: [2]float64{0.5, 1.5}},
		{name: "opposite half-plane", a: [2]float64{0, 90}, b: [2]float64{180, 225}, zb: [2]float64{0.5, 1.5}},
		{name: "half-plane within the margin", a: [2]float64{0, 90}, b: [2]float64{0.01, 45}, zb: [2]float64{0.5, 1.5}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := wedgePrism(t, wedgeAt(tc.a[0], tc.a[1]), 0, 2)
			b := wedgePrism(t, wedgeAt(tc.b[0], tc.b[1]), tc.zb[0], tc.zb[1])
			i, k := sharedEdgeOf(t, a)
			j, l := sharedEdgeOf(t, b)
			verdict, ok := sharedEdgeWedge(a, b, i, k, j, l)
			require.Equal(t, tc.wantOK, ok)
			if !ok {
				return
			}
			require.Equal(t, tc.wantA, verdict.aInside)
			require.Equal(t, tc.wantB, verdict.bInside)
		})
	}
}

func TestSharedEdgeWedgeRefusesEdgesOffTheLine(t *testing.T) {
	a := wedgePrism(t, wedgeAt(0, 90), 0, 2)
	moved := wedgeAt(45, 135)
	for i := range moved {
		moved[i][0] += 0.25
	}
	b := wedgePrism(t, moved, 0.5, 1.5)
	i, k := sharedEdgeOf(t, a)
	// The second prism's apex edge lies at x = 0.25, off the first's line.
	for j, tri := range b.Tris {
		for l := range 3 {
			u, v := b.Verts[tri[l]], b.Verts[tri[(l+1)%3]]
			if u.X == 0.25 && u.Y == 0 && v.X == 0.25 && v.Y == 0 && u.Z != v.Z {
				_, ok := sharedEdgeWedge(a, b, i, k, j, l)
				require.False(t, ok)
				return
			}
		}
	}
	require.Fail(t, "no moved edge")
}

func TestSharedEdgeVerdictManifoldCounts(t *testing.T) {
	for _, tc := range []struct {
		name    string
		verdict sharedEdgeVerdict
		op      OperationKind
		refused bool
	}{
		{name: "crossing wedges union", verdict: sharedEdgeVerdict{1, 1}, op: OpUnion},
		{name: "crossing wedges cut", verdict: sharedEdgeVerdict{1, 1}, op: OpCut},
		{name: "crossing wedges intersect", verdict: sharedEdgeVerdict{1, 1}, op: OpIntersect},
		{name: "disjoint wedges union leaves four", verdict: sharedEdgeVerdict{}, op: OpUnion, refused: true},
		{name: "disjoint wedges cut", verdict: sharedEdgeVerdict{}, op: OpCut},
		{name: "disjoint wedges intersect", verdict: sharedEdgeVerdict{}, op: OpIntersect},
		{name: "contained tool cut leaves four", verdict: sharedEdgeVerdict{0, 2}, op: OpCut, refused: true},
		{name: "contained tool union", verdict: sharedEdgeVerdict{0, 2}, op: OpUnion},
		{name: "contained target cut", verdict: sharedEdgeVerdict{2, 0}, op: OpCut},
	} {
		t.Run(tc.name, func(t *testing.T) {
			wantA, wantB, _, err := BooleanKeep(tc.op)
			require.NoError(t, err)
			err = refuseNonManifoldSharedEdges(map[sharedEdgeKey]sharedEdgeVerdict{{}: tc.verdict}, wantA, wantB)
			if tc.refused {
				require.Error(t, err)
				return
			}
			require.NoError(t, err)
		})
	}
}

func TestPairCutsClassifiesVouchedSharedEdgeOnly(t *testing.T) {
	build := func(t *testing.T, bLo, bHi float64) (*BoolMesh, *BoolMesh) {
		return wedgePrism(t, wedgeAt(0, 90), 0, 2), wedgePrism(t, wedgeAt(bLo, bHi), 0.5, 1.5)
	}
	t.Run("vouched and strictly arranged", func(t *testing.T) {
		a, b := build(t, 45, 135)
		memo := NewContactMemo(a, b)
		memo.SharedEdges = vouch(true)
		res, err := pairCuts(t.Context(), a, b, memo, newKeyMemo())
		require.NoError(t, err)
		require.Len(t, res.SharedEdges, 1)
		for key, verdict := range res.SharedEdges {
			require.Equal(t, sharedEdgeVerdict{aInside: 1, bInside: 1}, verdict)
			require.Contains(t, res.BlockedA, key.a)
			require.Contains(t, res.BlockedB, key.b)
		}
	})
	t.Run("no certifier keeps the refusal", func(t *testing.T) {
		a, b := build(t, 45, 135)
		_, err := pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
		require.ErrorContains(t, err, touchGraze)
	})
	t.Run("a certifier that declines keeps the refusal", func(t *testing.T) {
		a, b := build(t, 45, 135)
		memo := NewContactMemo(a, b)
		memo.SharedEdges = vouch(false)
		_, err := pairCuts(t.Context(), a, b, memo, newKeyMemo())
		require.ErrorContains(t, err, touchGraze)
	})
	t.Run("a vouched edge with a coplanar half-plane keeps the refusal", func(t *testing.T) {
		a, b := build(t, 0, 45)
		memo := NewContactMemo(a, b)
		memo.SharedEdges = vouch(true)
		_, err := pairCuts(t.Context(), a, b, memo, newKeyMemo())
		require.Error(t, err)
	})
}

func TestSharedEdgeCutStitchesPartialOverlap(t *testing.T) {
	a := wedgePrism(t, [][2]float64{{0, 0}, {2, 0}, {0, 2}}, 0, 2)
	b := wedgePrism(t, [][2]float64{{0, 0}, {2, 1}, {-1, 2}}, 0.5, 1.5)
	memo := NewContactMemo(a, b)
	memo.SharedEdges = vouch(true)
	kept, stats, err := MeshBooleanStats(t.Context(), OpCut, a, b, memo)
	require.NoError(t, err)
	stitched, err := StitchFacetsWith(t.Context(), kept, StitchOptions{Coplanar: stats.Coplanar})
	require.NoError(t, err)
	volume, _ := heldVolume(stitched).Float64()
	require.InDelta(t, 2.75, volume, 1e-10)
}
