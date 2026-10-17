package spiralsector

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldBox is a closed box whose vertex v sits at x = bit 0, y = 1 + bit 1 and
// z = bit 2, so every vertex has Y > 0. The +y face is group 1, the -x face
// group 2 and the other four faces group 3. Faces are wound outward. With
// splitSeam, vertex 8 splits the edge 2-6 that the +y and -x faces share, so
// the group 1 / group 2 seam has three vertices instead of two.
func heldBox(splitSeam bool) *meshbool.StitchedMesh {
	verts := make([]r3.Vec, 0, 9)
	for v := range 8 {
		verts = append(verts, r3.Vec{X: float64(v & 1), Y: 1 + float64(v>>1&1), Z: float64(v >> 2)})
	}
	top := []int{2, 6, 7, 3}
	left := []int{0, 4, 6, 2}
	if splitSeam {
		verts = append(verts, r3.Vec{X: 0, Y: 2, Z: 0.5})
		top = []int{2, 8, 6, 7, 3}
		left = []int{0, 4, 6, 8, 2}
	}
	polys := []struct {
		src  int
		face []int
	}{
		{1, top}, {2, left},
		{3, []int{1, 3, 7, 5}}, {3, []int{0, 1, 5, 4}}, {3, []int{0, 2, 3, 1}}, {3, []int{4, 5, 7, 6}},
	}
	held := &meshbool.StitchedMesh{Verts: verts}
	for _, poly := range polys {
		for i := 1; i+1 < len(poly.face); i++ {
			held.Tris = append(held.Tris, [3]int{poly.face[0], poly.face[i], poly.face[i+1]})
			held.Src = append(held.Src, poly.src)
		}
	}
	held.VertexBound = make([]float64, len(verts))
	for i := range held.VertexBound {
		held.VertexBound[i] = 0.001
	}
	return held
}

// heldSphere is a closed random mesh: an octahedron around (0, 3, 0) whose
// edges were split at random, with every facet labelled 0 to 3 by the nearest
// of four random seed facets in the facet adjacency graph. The boundary
// between two labels is usually an open chain that ends at a triple junction.
func heldSphere(rng *rand.Rand, splits int) *meshbool.StitchedMesh {
	verts := []r3.Vec{{X: 1, Y: 3}, {X: -1, Y: 3}, {X: 0, Y: 4}, {X: 0, Y: 2}, {X: 0, Y: 3, Z: 1}, {X: 0, Y: 3, Z: -1}}
	tris := [][3]int{
		{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4},
		{2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5},
	}
	for range splits {
		t := rng.IntN(len(tris))
		s := rng.IntN(3)
		a, b, c := tris[t][s], tris[t][(s+1)%3], tris[t][(s+2)%3]
		other, d := -1, 0
		for i, tri := range tris {
			for k := range 3 {
				if tri[k] == b && tri[(k+1)%3] == a {
					other, d = i, tri[(k+2)%3]
				}
			}
		}
		m := len(verts)
		jitter := r3.Vec{X: rng.Float64()*0.1 - 0.05, Y: rng.Float64()*0.1 - 0.05, Z: rng.Float64()*0.1 - 0.05}
		verts = append(verts, verts[a].Add(verts[b]).Scale(0.5).Add(jitter))
		tris[t], tris[other] = [3]int{-1}, [3]int{-1}
		tris = append(tris, [3]int{a, m, c}, [3]int{m, b, c}, [3]int{b, m, d}, [3]int{m, a, d})
		tris = slices.DeleteFunc(tris, func(tri [3]int) bool { return tri[0] < 0 })
	}
	edgeFacets := map[[2]int][]int{}
	edgeOf := func(tri [3]int, s int) [2]int {
		return [2]int{min(tri[s], tri[(s+1)%3]), max(tri[s], tri[(s+1)%3])}
	}
	for i, tri := range tris {
		for s := range 3 {
			edgeFacets[edgeOf(tri, s)] = append(edgeFacets[edgeOf(tri, s)], i)
		}
	}
	label := make([]int, len(tris))
	for i := range label {
		label[i] = -1
	}
	queue := make([]int, 0, len(tris))
	for g := range 4 {
		for {
			seed := rng.IntN(len(tris))
			if label[seed] >= 0 {
				continue
			}
			label[seed] = g
			queue = append(queue, seed)
			break
		}
	}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for s := range 3 {
			for _, n := range edgeFacets[edgeOf(tris[cur], s)] {
				if label[n] < 0 {
					label[n] = label[cur]
					queue = append(queue, n)
				}
			}
		}
	}
	held := &meshbool.StitchedMesh{Verts: verts, Tris: tris, Src: label}
	held.VertexBound = make([]float64, len(verts))
	for i := range held.VertexBound {
		held.VertexBound[i] = 0.001 * (1 + rng.Float64())
	}
	return held
}

func cloneHeld(held *meshbool.StitchedMesh) *meshbool.StitchedMesh {
	out := *held
	out.Verts = slices.Clone(held.Verts)
	out.Tris = slices.Clone(held.Tris)
	out.Src = slices.Clone(held.Src)
	out.VertexBound = slices.Clone(held.VertexBound)
	return &out
}

func TestSpiralHeldEdgesRefusesBrokenMeshes(t *testing.T) {
	box := heldBox(false)
	owners, ok, err := HeldEdges(t.Context(), box)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, owners, 18)
	for _, owner := range owners {
		require.Equal(t, uint8(2), owner.count)
	}
	// The +y face is facets 0-1 and the -x face facets 2-3, so the edge they
	// share lists group 1 first (lower facet) and group 2 second.
	require.Equal(t, [2]int{1, 2}, owners[HeldEdge{2, 6}].src)

	tests := []struct {
		name   string
		mutate func(*meshbool.StitchedMesh)
	}{
		{"dropped facet", func(h *meshbool.StitchedMesh) {
			h.Tris, h.Src = h.Tris[1:], h.Src[1:]
		}},
		{"duplicated facet", func(h *meshbool.StitchedMesh) {
			h.Tris, h.Src = append(h.Tris, h.Tris[0]), append(h.Src, h.Src[0])
		}},
		{"index past the vertices", func(h *meshbool.StitchedMesh) { h.Tris[3][1] = len(h.Verts) }},
		{"negative index", func(h *meshbool.StitchedMesh) { h.Tris[3][1] = -1 }},
		{"degenerate facet", func(h *meshbool.StitchedMesh) { h.Tris[3][1] = h.Tris[3][0] }},
		{"short Src", func(h *meshbool.StitchedMesh) { h.Src = h.Src[:len(h.Src)-1] }},
		{"short VertexBound", func(h *meshbool.StitchedMesh) { h.VertexBound = h.VertexBound[1:] }},
		{"no facets", func(h *meshbool.StitchedMesh) { h.Tris, h.Src = nil, nil }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			held := cloneHeld(box)
			tt.mutate(held)
			owners, ok, err := HeldEdges(t.Context(), held)
			require.NoError(t, err)
			require.False(t, ok)
			require.Nil(t, owners)
		})
	}

	t.Run("nil mesh and nil context", func(t *testing.T) {
		_, ok, err := HeldEdges(t.Context(), nil)
		require.NoError(t, err)
		require.False(t, ok)
		//nolint:staticcheck // a nil context is the refusal under test
		_, ok, err = HeldEdges(nil, box)
		require.NoError(t, err)
		require.False(t, ok)
	})

	t.Run("cancelled context", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, ok, err := HeldEdges(ctx, box)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
	})
}

// heldSwitchPathReference is the switch search as it stood before the
// edge table was split out: one table rebuilt per call from two maps.
func heldSwitchPathReference(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, monotone bool) (HeldSwitch, bool, error) {
	if ctx == nil || held == nil || blankHeel < 0 || toothRoot < 0 || blankHeel == toothRoot ||
		len(held.Tris) == 0 || len(held.Tris) > 1<<17 ||
		len(held.Src) != len(held.Tris) || len(held.VertexBound) != len(held.Verts) {
		return HeldSwitch{}, false, nil
	}
	type edge struct{ lo, hi int }
	owners := make(map[edge][2]int, len(held.Tris)*3/2)
	counts := make(map[edge]uint8, len(held.Tris)*3/2)
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return HeldSwitch{}, false, err
			}
		}
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(held.Verts) || b >= len(held.Verts) || a == b {
				return HeldSwitch{}, false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := edge{a, b}
			if counts[key] >= 2 {
				return HeldSwitch{}, false, nil
			}
			owner := owners[key]
			owner[counts[key]] = held.Src[i]
			owners[key] = owner
			counts[key]++
		}
	}
	adj := make(map[int][]int)
	seamEdges := 0
	seen := 0
	for key, pair := range owners {
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return HeldSwitch{}, false, err
			}
		}
		seen++
		if counts[key] != 2 {
			return HeldSwitch{}, false, nil
		}
		if pair[0] != blankHeel || pair[1] != toothRoot {
			if pair[0] != toothRoot || pair[1] != blankHeel {
				continue
			}
		}
		adj[key.lo] = append(adj[key.lo], key.hi)
		adj[key.hi] = append(adj[key.hi], key.lo)
		seamEdges++
	}
	if seamEdges < 1 || seamEdges > 4096 {
		return HeldSwitch{}, false, nil
	}
	starts := make([]int, 0, 2)
	for vertex, neighbors := range adj {
		switch len(neighbors) {
		case 1:
			starts = append(starts, vertex)
		case 2:
		default:
			return HeldSwitch{}, false, nil
		}
	}
	if len(starts) != 2 || len(adj) != seamEdges+1 {
		return HeldSwitch{}, false, nil
	}
	ordered := make([]int, 0, len(adj))
	previous, current := -1, starts[0]
	sign := 0
	minBeta := math.Inf(1)
	var x loftface.Interval
	var box [3]loftface.Interval
	for len(ordered) < len(adj) {
		if len(ordered)%128 == 0 {
			if err := ctx.Err(); err != nil {
				return HeldSwitch{}, false, err
			}
		}
		p := held.Verts[current]
		beta := held.VertexBound[current]
		if !proofbound.FiniteVec(p) || p.Y <= 0 ||
			proofbound.IsNonFinite(beta) || beta < 0 {
			return HeldSwitch{}, false, nil
		}
		if len(ordered) == 0 {
			x = loftface.Interval{Lo: ratF(p.X), Hi: ratF(p.X)}
		} else {
			if ratF(p.X).Cmp(x.Lo) < 0 {
				x.Lo = ratF(p.X)
			}
			if ratF(p.X).Cmp(x.Hi) > 0 {
				x.Hi = ratF(p.X)
			}
		}
		for axis, coordinate := range [...]float64{p.X, p.Y, p.Z} {
			value := ratF(coordinate)
			if box[axis].Lo == nil || value.Cmp(box[axis].Lo) < 0 {
				box[axis].Lo = value
			}
			if box[axis].Hi == nil || value.Cmp(box[axis].Hi) > 0 {
				box[axis].Hi = value
			}
		}
		minBeta = min(minBeta, beta)
		ordered = append(ordered, current)
		next := -1
		for _, neighbor := range adj[current] {
			if neighbor != previous {
				next = neighbor
				break
			}
		}
		if next < 0 {
			break
		}
		q := held.Verts[next]
		if !proofbound.FiniteVec(q) || q.Y <= 0 {
			return HeldSwitch{}, false, nil
		}
		cross := ratSub(ratMul(ratF(p.Y), ratF(q.Z)), ratMul(ratF(p.Z), ratF(q.Y))).Sign()
		if cross == 0 || monotone && sign != 0 && cross != sign {
			return HeldSwitch{}, false, nil
		}
		sign = cross
		previous, current = current, next
	}
	if len(ordered) != len(adj) || ordered[len(ordered)-1] != starts[1] ||
		proofbound.IsNonFinite(minBeta) || minBeta <= 0 {
		return HeldSwitch{}, false, nil
	}
	first, last := held.Verts[ordered[0]], held.Verts[ordered[len(ordered)-1]]
	return HeldSwitch{Vertices: ordered, X: x, box: box,
		slope:   [2]*big.Rat{new(big.Rat).Quo(ratF(first.Z), ratF(first.Y)), new(big.Rat).Quo(ratF(last.Z), ratF(last.Y))},
		MinBeta: minBeta, Increasing: sign > 0}, true, nil
}

// requireSameHeldSwitch checks that two accepted switches describe the same
// chain. The walk starts at either end of the chain, so a switch may be the
// other's reversal, which swaps its end slopes and flips its direction.
//
// Only a monotone chain has one turning direction. For any other chain
// increasing is the sign of the last edge walked, which depends on the end the
// walk started from, so it is compared only when the walks agree.
func requireSameHeldSwitch(t *testing.T, want, got HeldSwitch, monotone bool) {
	t.Helper()
	reversed := !slices.Equal(want.Vertices, got.Vertices)
	if reversed {
		back := slices.Clone(got.Vertices)
		slices.Reverse(back)
		require.Equal(t, want.Vertices, back)
	}
	require.Equal(t, want.MinBeta, got.MinBeta)
	require.Zero(t, want.X.Lo.Cmp(got.X.Lo))
	require.Zero(t, want.X.Hi.Cmp(got.X.Hi))
	for axis := range 3 {
		require.Zero(t, want.box[axis].Lo.Cmp(got.box[axis].Lo))
		require.Zero(t, want.box[axis].Hi.Cmp(got.box[axis].Hi))
	}
	first, last := got.slope[0], got.slope[1]
	increasing := got.Increasing
	if reversed {
		first, last, increasing = last, first, !increasing
	}
	require.Zero(t, want.slope[0].Cmp(first))
	require.Zero(t, want.slope[1].Cmp(last))
	if monotone || !reversed {
		require.Equal(t, want.Increasing, increasing)
	}
}

// requireHeldSwitchesAgree runs the reference, heldSwitchPath and
// HeldSwitchFrom over one shared table and requires the same verdict,
// error and chain. It reports whether the switch was accepted.
func requireHeldSwitchesAgree(ctx context.Context, t *testing.T, held *meshbool.StitchedMesh,
	owners map[HeldEdge]HeldEdgeOwners, a, b int, monotone bool) bool {
	t.Helper()
	want, wantOK, wantErr := heldSwitchPathReference(ctx, held, a, b, monotone)
	path, pathOK, pathErr := heldSwitchPath(ctx, held, a, b, monotone)
	require.Equal(t, wantErr, pathErr)
	require.Equal(t, wantOK, pathOK)
	if owners == nil {
		require.False(t, wantOK)
	} else {
		from, fromOK, fromErr := HeldSwitchFrom(ctx, held, owners, a, b, monotone)
		require.Equal(t, wantErr, fromErr)
		require.Equal(t, wantOK, fromOK)
		if fromOK {
			requireSameHeldSwitch(t, want, from, monotone)
		}
	}
	if pathOK {
		requireSameHeldSwitch(t, want, path, monotone)
	}
	return wantOK
}

func TestSpiralHeldSwitchFromMatchesSwitchPath(t *testing.T) {
	pairs := [][2]int{{1, 2}, {2, 1}, {1, 3}, {3, 3}, {-1, 2}}
	for _, splitSeam := range []bool{false, true} {
		held := heldBox(splitSeam)
		owners, ok, err := HeldEdges(t.Context(), held)
		require.NoError(t, err)
		require.True(t, ok)
		for _, pair := range pairs {
			for _, monotone := range []bool{true, false} {
				accepted := requireHeldSwitchesAgree(t.Context(), t, held, owners, pair[0], pair[1], monotone)
				// The +y and -x faces share exactly one open seam.
				require.Equal(t, pair[0] > 0 && pair[0] != pair[1] && pair[0]+pair[1] == 3 && pair[1] > 0, accepted,
					"split %v pair %v monotone %v", splitSeam, pair, monotone)
			}
		}
		chain, accepted, err := HeldSwitchFrom(t.Context(), held, owners, 1, 2, true)
		require.NoError(t, err)
		require.True(t, accepted)
		require.Len(t, chain.Vertices, 2+btoi(splitSeam))
		require.Equal(t, math.Float64bits(0.001), math.Float64bits(chain.MinBeta))
	}
}

func btoi(b bool) int {
	if b {
		return 1
	}
	return 0
}

func TestSpiralHeldSwitchFromMatchesSwitchPathRandomMeshes(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	accepted, refused := 0, 0
	for trial := range 60 {
		held := heldSphere(rng, 5+rng.IntN(60))
		if trial%3 == 2 {
			// Break the mesh so the table build must refuse it.
			switch trial % 9 {
			case 2:
				held.Tris, held.Src = held.Tris[1:], held.Src[1:]
			case 5:
				held.Tris, held.Src = append(held.Tris, held.Tris[0]), append(held.Src, held.Src[0])
			default:
				held.Tris[len(held.Tris)/2][1] = len(held.Verts)
			}
		}
		owners, ok, err := HeldEdges(t.Context(), held)
		require.NoError(t, err)
		require.Equal(t, trial%3 != 2, ok, "trial %d", trial)
		for a := range 4 {
			for b := range 4 {
				for _, monotone := range []bool{true, false} {
					if requireHeldSwitchesAgree(t.Context(), t, held, owners, a, b, monotone) {
						accepted++
					} else {
						refused++
					}
				}
			}
		}
	}
	// Both verdicts must be exercised or the comparison proves nothing.
	require.Greater(t, accepted, 20)
	require.Greater(t, refused, 20)
}

func TestSpiralHeldSwitchFromMatchesSwitchPathCancelled(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	held := heldBox(false)
	owners, ok, err := HeldEdges(t.Context(), held)
	require.NoError(t, err)
	require.True(t, ok)
	_, _, wantErr := heldSwitchPathReference(ctx, held, 1, 2, true)
	require.ErrorIs(t, wantErr, context.Canceled)
	_, pathOK, pathErr := heldSwitchPath(ctx, held, 1, 2, true)
	require.Equal(t, wantErr, pathErr)
	require.False(t, pathOK)
	// A table built before the cancel still reaches the chain walk's own poll.
	_, fromOK, fromErr := HeldSwitchFrom(ctx, held, owners, 1, 2, true)
	require.Equal(t, wantErr, fromErr)
	require.False(t, fromOK)
}
