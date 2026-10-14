package loftmesh

import (
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// sortBuildReference is the tree build as it was before selection: every
// inner node sorts its whole range by the midpoint comparator. It is the
// oracle for treeScan.build.
func sortBuildReference(t *treeScan, start, end int) int {
	box := t.boxes[t.order[start]]
	maxID := t.order[start]
	for _, i := range t.order[start+1 : end] {
		maxID = max(maxID, i)
		b := t.boxes[i]
		box[0] = r3.Vec{X: min(box[0].X, b[0].X), Y: min(box[0].Y, b[0].Y), Z: min(box[0].Z, b[0].Z)}
		box[1] = r3.Vec{X: max(box[1].X, b[1].X), Y: max(box[1].Y, b[1].Y), Z: max(box[1].Z, b[1].Z)}
	}
	index := len(t.nodes)
	t.nodes = append(t.nodes, treeNode{box: box, start: start, end: end, left: -1, right: -1, maxID: maxID})
	if end-start <= treeLeafSize {
		return index
	}
	axis := 0
	for a := 1; a < 3; a++ {
		if vecAxis(box[1], a)-vecAxis(box[0], a) > vecAxis(box[1], axis)-vecAxis(box[0], axis) {
			axis = a
		}
	}
	slices.SortFunc(t.order[start:end], func(a, b int32) int {
		alo, ahi := boxAxis(t.boxes[a], axis)
		blo, bhi := boxAxis(t.boxes[b], axis)
		amid, bmid := alo/2+ahi/2, blo/2+bhi/2
		switch {
		case amid < bmid:
			return -1
		case amid > bmid:
			return 1
		default:
			return int(a) - int(b)
		}
	})
	mid := start + (end-start)/2
	left := sortBuildReference(t, start, mid)
	right := sortBuildReference(t, mid, end)
	t.nodes[index].left, t.nodes[index].right = left, right
	return index
}

func newSortTreeScan(boxes [][2]r3.Vec, members []int) treeScan {
	t := treeScan{boxes: boxes, order: make([]int32, len(members))}
	for k, i := range members {
		t.order[k] = int32(i)
	}
	if len(members) != 0 {
		sortBuildReference(&t, 0, len(members))
	}
	t.flat = newFlatTree(&t)
	return t
}

func sameFloats(a, b []float64) bool {
	return slices.EqualFunc(a, b, func(x, y float64) bool { return math.Float64bits(x) == math.Float64bits(y) })
}

func sameBox(a, b [2]r3.Vec) bool {
	return sameFloats([]float64{a[0].X, a[0].Y, a[0].Z, a[1].X, a[1].Y, a[1].Z},
		[]float64{b[0].X, b[0].Y, b[0].Z, b[1].X, b[1].Y, b[1].Z})
}

// requireSameTree compares the order, every node and the flat tree bit for
// bit, so NaN bounds and signed zeros are compared exactly.
func requireSameTree(t *testing.T, want, got treeScan, label string) {
	t.Helper()
	require.Equal(t, want.order, got.order, "%s: order", label)
	require.Len(t, got.nodes, len(want.nodes), "%s: node count", label)
	for k, w := range want.nodes {
		g := got.nodes[k]
		// Plain comparisons: require per node is too slow under -race.
		if !sameBox(w.box, g.box) {
			require.Failf(t, "tree", "%s: node %d box differs", label, k)
		}
		w.box, g.box = [2]r3.Vec{}, [2]r3.Vec{}
		if w != g {
			require.Failf(t, "tree", "%s: node %d differs: want %+v got %+v", label, k, w, g)
		}
	}
	want.flat.ensureSorted()
	got.flat.ensureSorted()
	wf, gf := want.flat, got.flat
	require.True(t, sameFloats(wf.nbox, gf.nbox), "%s: flat nbox", label)
	require.True(t, sameFloats(wf.box, gf.box), "%s: flat box", label)
	require.True(t, sameFloats(wf.inter, gf.inter), "%s: flat inter", label)
	require.Equal(t, wf.interOK, gf.interOK, "%s: flat interOK", label)
	for name, pair := range map[string][2][]int32{
		"left": {wf.left, gf.left}, "right": {wf.right, gf.right}, "start": {wf.start, gf.start},
		"end": {wf.end, gf.end}, "maxID": {wf.maxID, gf.maxID}, "order": {wf.order, gf.order},
		"minID": {wf.minID, gf.minID}, "subN": {wf.subN, gf.subN}, "sorted": {wf.sorted, gf.sorted},
		"sortOff": {wf.sortOff, gf.sortOff},
	} {
		require.Equal(t, pair[0], pair[1], "%s: flat %s", label, name)
	}
}

type treeFixtureKind struct {
	name string
	// box returns box i of n.
	box func(rng *rand.Rand, i, n int) [2]r3.Vec
}

func cube(lo, hi r3.Vec) [2]r3.Vec { return [2]r3.Vec{lo, hi} }

func treeFixtureKinds() []treeFixtureKind {
	quant := func(rng *rand.Rand, k int) float64 { return float64(rng.IntN(k)) }
	return []treeFixtureKind{
		{"random", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			lo := r3.Vec{X: rng.Float64() * 100, Y: rng.Float64() * 100, Z: rng.Float64() * 100}
			return cube(lo, r3.Vec{X: lo.X + rng.Float64()*5, Y: lo.Y + rng.Float64()*5, Z: lo.Z + rng.Float64()*5})
		}},
		{"ties-quantized", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			lo := r3.Vec{X: quant(rng, 4), Y: quant(rng, 4), Z: quant(rng, 4)}
			return cube(lo, r3.Vec{X: lo.X + 2, Y: lo.Y + 2, Z: lo.Z + 2})
		}},
		{"ties-one-axis", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			// Equal midpoints on x, wide on y: the split axis is y and x ties are irrelevant,
			// but z midpoints tie in pairs.
			return cube(r3.Vec{X: 1, Y: rng.Float64() * 10, Z: quant(rng, 3)},
				r3.Vec{X: 2, Y: 20 + rng.Float64()*10, Z: quant(rng, 3) + 1})
		}},
		{"all-equal", func(_ *rand.Rand, _, _ int) [2]r3.Vec {
			return cube(r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 4, Y: 5, Z: 6})
		}},
		{"sorted", func(_ *rand.Rand, i, _ int) [2]r3.Vec {
			return cube(r3.Vec{X: float64(i)}, r3.Vec{X: float64(i) + 1, Y: 1, Z: 1})
		}},
		{"reverse-sorted", func(_ *rand.Rand, i, n int) [2]r3.Vec {
			return cube(r3.Vec{X: float64(n - i)}, r3.Vec{X: float64(n-i) + 1, Y: 1, Z: 1})
		}},
		{"organ-pipe", func(_ *rand.Rand, i, n int) [2]r3.Vec {
			x := float64(min(i, n-1-i))
			return cube(r3.Vec{X: x}, r3.Vec{X: x + 1, Y: 1, Z: 1})
		}},
		{"duplicate-boxes", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			x := quant(rng, 3)
			return cube(r3.Vec{X: x, Y: x, Z: x}, r3.Vec{X: x + 1, Y: x + 1, Z: x + 1})
		}},
		{"signed-zeros", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			z := [2]float64{0, math.Copysign(0, -1)}
			return cube(r3.Vec{X: z[rng.IntN(2)], Y: z[rng.IntN(2)], Z: z[rng.IntN(2)]},
				r3.Vec{X: z[rng.IntN(2)], Y: z[rng.IntN(2)], Z: z[rng.IntN(2)]})
		}},
		{"huge", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			s := math.MaxFloat64 * (rng.Float64()*2 - 1)
			return cube(r3.Vec{X: s, Y: -s, Z: s / 3}, r3.Vec{X: math.MaxFloat64, Y: math.MaxFloat64, Z: math.MaxFloat64})
		}},
		{"infinite-half", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			// A midpoint of +Inf or -Inf is ordered, so the comparator stays total.
			x := float64(rng.IntN(5))
			return cube(r3.Vec{X: x, Y: math.Inf(-1), Z: x}, r3.Vec{X: x + 1, Y: float64(rng.IntN(3)), Z: x + 1})
		}},
		{"nan", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			lo := r3.Vec{X: rng.Float64() * 10, Y: rng.Float64() * 10, Z: rng.Float64() * 10}
			b := cube(lo, r3.Vec{X: lo.X + 1, Y: lo.Y + 1, Z: lo.Z + 1})
			if rng.IntN(7) == 0 {
				b[rng.IntN(2)].Y = math.NaN()
			}
			return b
		}},
		{"inf-extent", func(rng *rand.Rand, _, _ int) [2]r3.Vec {
			// -Inf..+Inf has a NaN midpoint.
			b := cube(r3.Vec{X: rng.Float64(), Y: rng.Float64(), Z: rng.Float64()},
				r3.Vec{X: 2 + rng.Float64(), Y: 2 + rng.Float64(), Z: 2 + rng.Float64()})
			if rng.IntN(9) == 0 {
				b[0].Z, b[1].Z = math.Inf(-1), math.Inf(1)
			}
			return b
		}},
	}
}

func TestTreeBuildSelectionMatchesSort(t *testing.T) {
	t.Parallel()
	sizes := []int{0, 1, 2, 3, 7, 8, 9, 10, 15, 16, 17, 18, 24, 25, 33, 100, 1000, 3000}
	rng := rand.New(rand.NewPCG(61, 67))
	for _, kind := range treeFixtureKinds() {
		for _, n := range sizes {
			boxes := make([][2]r3.Vec, n)
			for i := range boxes {
				boxes[i] = kind.box(rng, i, n)
			}
			ident := make([]int, n)
			for i := range ident {
				ident[i] = i
			}
			shuffled := slices.Clone(ident)
			rng.Shuffle(n, func(a, b int) { shuffled[a], shuffled[b] = shuffled[b], shuffled[a] })
			// A strict subset in reverse, as the grouped scans pass.
			var subset []int
			for i := n - 1; i >= 0; i -= 2 {
				subset = append(subset, i)
			}
			for name, members := range map[string][]int{"identity": ident, "shuffled": shuffled, "subset": subset} {
				label := fmt.Sprintf("%s/n=%d/%s", kind.name, n, name)
				want := newSortTreeScan(boxes, members)
				got := newTreeScan(boxes, members)
				requireSameTree(t, want, got, label)
			}
		}
	}
}

// TestTreeBuildNaNMidpointDetection pins which boxes send the build to the
// full-sort path: exactly those with a NaN midpoint. Every other box set,
// infinite coordinates included, takes the selection path.
func TestTreeBuildNaNMidpointDetection(t *testing.T) {
	t.Parallel()
	hasNaN := func(boxes [][2]r3.Vec) bool {
		tree := newTreeScan(boxes, []int{0})
		return tree.hasNaNMidpoint()
	}
	ok := [][2]r3.Vec{cube(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1})}
	require.False(t, hasNaN(ok))
	for _, bad := range []func(b *[2]r3.Vec){
		func(b *[2]r3.Vec) { b[0].X = math.NaN() },
		func(b *[2]r3.Vec) { b[1].Z = math.NaN() },
		func(b *[2]r3.Vec) { b[0].Y, b[1].Y = math.Inf(-1), math.Inf(1) },
	} {
		boxes := slices.Clone(ok)
		bad(&boxes[0])
		require.True(t, hasNaN(boxes))
	}
	inf := cube(r3.Vec{Y: math.Inf(-1)}, r3.Vec{X: 1, Y: 0, Z: 1})
	require.False(t, hasNaN([][2]r3.Vec{inf}))
}

func intCmp(a, b int32) int { return int(a) - int(b) }

func TestSelectNthPartitions(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(71, 73))
	check := func(label string, in []int32, k, rounds int) {
		t.Helper()
		x := slices.Clone(in)
		selectNthRounds(x, k, intCmp, rounds)
		sorted := slices.Clone(in)
		slices.Sort(sorted)
		require.Equal(t, sorted[k], x[k], "%s: k=%d", label, k)
		// Plain loops: require per element is too slow under -race.
		for i := range k {
			if x[i] >= x[k] {
				require.Failf(t, "partition", "%s: x[%d]=%d is not left of x[%d]=%d", label, i, x[i], k, x[k])
			}
		}
		for i := k + 1; i < len(x); i++ {
			if x[i] <= x[k] {
				require.Failf(t, "partition", "%s: x[%d]=%d is not right of x[%d]=%d", label, i, x[i], k, x[k])
			}
		}
		slices.Sort(x)
		require.Equal(t, sorted, x, "%s: permutation", label)
	}
	for _, rounds := range []int{selectRounds, 2, 1, 0} {
		for n := 1; n <= 40; n++ {
			in := make([]int32, n)
			for i, v := range rng.Perm(n) {
				in[i] = int32(v)
			}
			for k := range n {
				check(fmt.Sprintf("perm n=%d rounds=%d", n, rounds), in, k, rounds)
			}
		}
		for _, n := range []int{1000, 4097} {
			asc := make([]int32, n)
			pipe := make([]int32, n)
			for i := range asc {
				asc[i] = int32(i)
				pipe[i] = int32(min(i, n-1-i)*2 + b2i(i >= n/2))
			}
			desc := slices.Clone(asc)
			slices.Reverse(desc)
			for name, in := range map[string][]int32{"asc": asc, "desc": desc, "pipe": pipe} {
				for _, k := range []int{0, 1, n / 4, n / 2, n - 2, n - 1} {
					check(fmt.Sprintf("%s n=%d rounds=%d", name, n, rounds), in, k, rounds)
				}
			}
		}
	}
}

func b2i(b bool) int {
	if b {
		return 1
	}
	return 0
}

// TestSelectNthProductionCapFires builds the 100 000-member organ-pipe order
// the tree's root split sees, and requires that the production round cap
// stops the selection (the sort fallback runs) where an uncapped selection
// finishes without it, and that the tree built from it equals the sort build.
// Removing the cap, or raising it past the rounds this input needs, fails it.
func TestSelectNthProductionCapFires(t *testing.T) {
	t.Parallel()
	const n = 100000
	boxes := make([][2]r3.Vec, n)
	members := make([]int, n)
	for i := range boxes {
		// Distinct midpoints, organ-pipe in the member order.
		x := float64(min(i, n-1-i)*2 + b2i(i >= n/2))
		boxes[i] = cube(r3.Vec{X: x}, r3.Vec{X: x + 1, Y: 1, Z: 1})
		members[i] = i
	}
	cmp := func(a, b int32) int {
		alo, ahi := boxAxis(boxes[a], 0)
		blo, bhi := boxAxis(boxes[b], 0)
		amid, bmid := alo/2+ahi/2, blo/2+bhi/2
		switch {
		case amid < bmid:
			return -1
		case amid > bmid:
			return 1
		default:
			return int(a) - int(b)
		}
	}
	in := make([]int32, n)
	for i := range in {
		in[i] = int32(i)
	}
	capped := slices.Clone(in)
	require.True(t, selectNth(capped, n/2, cmp), "the production cap of %d rounds must fire", selectRounds)
	uncapped := slices.Clone(in)
	require.False(t, selectNthRounds(uncapped, n/2, cmp, math.MaxInt), "the uncapped selection needs more than the cap")
	// Both put the same members left of the median, which is the tree's invariant.
	left := func(x []int32) []int32 {
		l := slices.Clone(x[:n/2])
		slices.Sort(l)
		return l
	}
	require.Equal(t, left(uncapped), left(capped))

	requireSameTree(t, newSortTreeScan(boxes, members), newTreeScan(boxes, members), "organ-pipe 100000")
}
