package meshbool

import (
	"math"
	"math/big"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cutRegionSnapshot copies scalar values, so later scratch reuse cannot change
// the reference and a failure reports only one bounded coordinate or flag.
func cutRegionSnapshot(regions []CutRegion) []string {
	var out []string
	point := func(p proof.Xpt) {
		for _, v := range [4]*big.Int{p.X, p.Y, p.Z, p.W} {
			out = append(out, v.Text(16))
		}
	}
	for _, r := range regions {
		out = append(out, strconv.FormatBool(r.HasAnchor), strconv.Itoa(len(r.Tris)))
		for _, p := range r.Partner {
			for _, v := range [3]float64{p.X, p.Y, p.Z} {
				out = append(out, strconv.FormatUint(math.Float64bits(v), 16))
			}
		}
		point(r.Probe)
		for _, tri := range r.Tris {
			for _, p := range tri {
				point(p)
			}
		}
	}
	return out
}

func TestCutProjectionScratchDoesNotEscapeIntoRegions(t *testing.T) {
	var scratch cutProjectionScratch
	var retained [][]CutRegion
	var snapshots [][]string
	for _, tc := range []struct {
		verts [3]r3.Vec
		half  r3.Vec
		at    r3.Vec
	}{
		{[3]r3.Vec{{}, {X: 10}, {Y: 10}}, r3.Vec{X: 0.8, Y: 0.6, Z: 1.7}, r3.Vec{X: 2.5, Y: 2.5, Z: 0.125}},
		{[3]r3.Vec{{}, {Y: 10}, {X: 10}}, r3.Vec{X: 0.6, Y: 0.8, Z: 1.7}, r3.Vec{X: 2.5, Y: 2.5, Z: -0.125}},
		{[3]r3.Vec{{}, {Y: 10}, {Z: 10}}, r3.Vec{X: 1.7, Y: 0.8, Z: 0.6}, r3.Vec{X: 0.125, Y: 2.5, Z: 2.5}},
	} {
		m, err := PrepContext(t.Context(), tc.verts[:], [][3]int{{0, 1, 2}}, []int{0})
		require.NoError(t, err)
		b := boxMesh(t, tc.half, func(v r3.Vec) r3.Vec { return v }, tc.at)
		var segs []Xseg
		for j := range b.Tris {
			contact, err := TriTriClassify(TriCorners(m, 0), TriCorners(b, j),
				XtriCorners(m, 0), XtriCorners(b, j), m.Norms[0], b.Norms[j])
			require.NoError(t, err)
			if contact.Kind != ContactSegment {
				continue
			}
			require.False(t, contact.P0OnA || contact.P1OnA, "the box cuts a closed interior loop")
			segs = append(segs, Xseg{A: contact.P0, B: contact.P1, Partner: TriCorners(b, j)})
		}
		require.GreaterOrEqual(t, len(segs), 4)
		want, err := CutTriangle(t.Context(), XtriCorners(m, 0), m.Norms[0], segs)
		require.NoError(t, err)
		snapshots = append(snapshots, cutRegionSnapshot(want))
		got, err := cutTriangle(t.Context(), XtriCorners(m, 0), m.Norms[0], segs, &scratch)
		require.NoError(t, err)
		retained = append(retained, got)
		for i, r := range scratch.rats {
			r.SetFrac64(int64(i+17), int64(i+19))
		}
		for i := range scratch.cutter.Verts {
			scratch.cutter.Verts[i] = CutVert{}
		}
		for i := range scratch.cutter.Edges {
			scratch.cutter.Edges[i] = CutEdge{}
		}
		for i := range scratch.cutter.raw.entries {
			scratch.cutter.raw.entries[i] = cutRawEntry{}
		}
		for i := range scratch.cutter.regionPoints {
			scratch.cutter.regionPoints[i] = Xp2{}
		}
		for i, regions := range retained {
			actual := cutRegionSnapshot(regions)
			require.Len(t, actual, len(snapshots[i]))
			for field, value := range actual {
				require.Equal(t, snapshots[i][field], value, "retained result %d field %d", i, field)
			}
		}
	}
}

func TestCutWorkspaceDropsOversizedStorage(t *testing.T) {
	var scratch cutProjectionScratch
	old := &scratch.cutter
	old.Verts = make([]CutVert, cutWorkspaceRetainLimit+1)
	old.Edges = make([]CutEdge, cutWorkspaceRetainLimit+1)
	old.regionPoints = make([]Xp2, cutWorkspaceRetainLimit+1)
	old.index = cutVertIndex{head: map[uint64]int{}, next: make([]int, cutWorkspaceRetainLimit+1)}
	old.index.promoted = map[uint64]struct{}{}
	old.index.exactHead = map[[2]uint64]int{}
	old.raw = cutRawIndex{head: map[uint64]int{}, entries: make([]cutRawEntry, cutWorkspaceRetainLimit+1)}
	old.seen = map[[2]int]struct{}{}
	old.anchors = map[[2]int]ChainAnchor{}
	for i := range cutWorkspaceRetainLimit + 1 {
		old.index.head[uint64(i)] = i
		old.index.promoted[uint64(i)] = struct{}{}
		old.index.exactHead[[2]uint64{uint64(i), 0}] = i
		old.raw.head[uint64(i)] = i
		old.seen[[2]int{i, i + 1}] = struct{}{}
		old.anchors[[2]int{i, i + 1}] = ChainAnchor{}
	}
	index, raw, seen, anchors := old.index.head, old.raw.head, old.seen, old.anchors
	promoted, exact := old.index.promoted, old.index.exactHead
	tc := newTriCutter(t.Context(), &scratch)
	require.Zero(t, len(tc.Verts))
	require.Zero(t, len(tc.Edges))
	require.LessOrEqual(t, cap(tc.Verts), cutWorkspaceRetainLimit)
	require.LessOrEqual(t, cap(tc.Edges), cutWorkspaceRetainLimit)
	require.Zero(t, len(tc.regionPoints))
	require.LessOrEqual(t, cap(tc.regionPoints), cutWorkspaceRetainLimit)
	require.Zero(t, len(tc.index.head))
	require.Zero(t, len(tc.index.next))
	require.Empty(t, tc.index.promoted)
	require.Empty(t, tc.index.exactHead)
	tc.index.promoted[0] = struct{}{}
	tc.index.exactHead[[2]uint64{}] = 0
	require.Len(t, promoted, cutWorkspaceRetainLimit+1, "oversized promotion map was dropped")
	require.Len(t, exact, cutWorkspaceRetainLimit+1, "oversized exact index was dropped")
	require.LessOrEqual(t, cap(tc.index.next), cutWorkspaceRetainLimit)
	require.Zero(t, len(tc.raw.head))
	require.Zero(t, len(tc.raw.entries))
	require.LessOrEqual(t, cap(tc.raw.entries), cutWorkspaceRetainLimit)
	require.Zero(t, len(tc.seen))
	require.Zero(t, len(tc.anchors))
	index[math.MaxUint64] = 1
	raw[math.MaxUint64] = 1
	seen[[2]int{-1, -1}] = struct{}{}
	anchors[[2]int{-1, -1}] = ChainAnchor{}
	_, reusedIndex := tc.index.head[math.MaxUint64]
	_, reusedRaw := tc.raw.head[math.MaxUint64]
	_, reusedSeen := tc.seen[[2]int{-1, -1}]
	_, reusedAnchors := tc.anchors[[2]int{-1, -1}]
	require.False(t, reusedIndex)
	require.False(t, reusedRaw)
	require.False(t, reusedSeen)
	require.False(t, reusedAnchors)
}

func TestCutWorkspaceResetsCollisionLinks(t *testing.T) {
	var scratch cutProjectionScratch
	points := []CutVert{
		{P2: NewXP2(big.NewRat(1, 3), big.NewRat(2, 5))},
		{P2: NewXP2(big.NewRat(3, 7), big.NewRat(4, 9))},
	}
	old := newTriCutter(t.Context(), &scratch)
	old.Verts = append(old.Verts, points...)
	old.index.add(11, 0)
	old.index.add(22, 1)
	old.raw.add(11, big.NewInt(1), big.NewInt(2), big.NewInt(3), 0)

	tc := newTriCutter(t.Context(), &scratch)
	require.Zero(t, len(tc.index.next))
	require.Zero(t, len(tc.raw.entries))
	tc.Verts = append(tc.Verts, points...)
	// The second facet deliberately puts both vertices in one hash chain.
	tc.index.add(99, 0)
	tc.index.add(99, 1)
	for want, p := range points {
		got, ok := tc.index.lookup(tc.Verts, 99, p.P2)
		require.True(t, ok, "vertex %d", want)
		require.Equal(t, want, got)
	}
}
