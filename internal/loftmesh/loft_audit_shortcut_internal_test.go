package loftmesh

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// walkRowAppendReference is flatTree.walkRowAppend without the common-point
// shortcut: it descends to every leaf.
func walkRowAppendReference(f *flatTree, row [2]r3.Vec, i int, limit uint64) ([]int32, uint64, bool) {
	var out []int32
	if len(f.left) == 0 {
		return out, 0, false
	}
	bi := box6(row)
	stack := []int32{0}
	var scanned uint64
	i32 := int32(i)
	for len(stack) > 0 {
		node := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		scanned++
		if scanned > limit {
			return out, scanned, true
		}
		if f.maxID[node] <= i32 || !boxes6Overlap(bi[:], f.nbox[6*int(node):6*int(node)+6]) {
			continue
		}
		if l := f.left[node]; l >= 0 {
			stack = append(stack, f.right[node], l)
			continue
		}
		for k := int(f.start[node]); k < int(f.end[node]); k++ {
			j := f.order[k]
			if j <= i32 {
				continue
			}
			scanned++
			if scanned > limit {
				return out, scanned, true
			}
			if boxes6Overlap(bi[:], f.box[6*k:6*k+6]) {
				out = append(out, j)
			}
		}
	}
	return out, scanned, false
}

// pairWalkReference is pairWalker.walk without the shortcut, counting pairs
// at their lower index into counts and returning the work and exceed flag.
func pairWalkReference(t *treeScan, a, b int, limit uint64, counts []int32) (uint64, bool) {
	f := t.flat
	var scanned uint64
	stack := [][2]int32{{int32(a), int32(b)}}
	for len(stack) > 0 {
		top := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		a, b := int(top[0]), int(top[1])
		scanned++
		if scanned > limit {
			return scanned, true
		}
		if !boxes6Overlap(f.nbox[6*a:6*a+6], f.nbox[6*b:6*b+6]) {
			continue
		}
		la, lb := f.left[a], f.left[b]
		if la < 0 && lb < 0 {
			na, nb := &t.nodes[a], &t.nodes[b]
			for ia := na.start; ia < na.end; ia++ {
				begin := nb.start
				if a == b {
					begin = ia + 1
				}
				for ib := begin; ib < nb.end; ib++ {
					scanned++
					if scanned > limit {
						return scanned, true
					}
					i, j := int(t.order[ia]), int(t.order[ib])
					if boxes6Overlap(f.box[6*ia:6*ia+6], f.box[6*ib:6*ib+6]) {
						counts[min(i, j)+1]++
					}
				}
			}
			continue
		}
		ra, rb := f.right[a], f.right[b]
		switch {
		case a == b:
			stack = append(stack, [2]int32{ra, ra}, [2]int32{la, ra}, [2]int32{la, la})
		case lb < 0 || (la >= 0 && f.end[a]-f.start[a] >= f.end[b]-f.start[b]):
			stack = append(stack, [2]int32{ra, int32(b)}, [2]int32{la, int32(b)})
		default:
			stack = append(stack, [2]int32{int32(a), rb}, [2]int32{int32(a), lb})
		}
	}
	return scanned, false
}

func boxOf(pts ...r3.Vec) [2]r3.Vec {
	lo, hi := pts[0], pts[0]
	for _, p := range pts[1:] {
		lo = r3.NewVec(min(lo.X, p.X), min(lo.Y, p.Y), min(lo.Z, p.Z))
		hi = r3.NewVec(max(hi.X, p.X), max(hi.Y, p.Y), max(hi.Z, p.Z))
	}
	return [2]r3.Vec{lo, hi}
}

// fanBoxes returns n triangle boxes around centre c, with rim points on a
// circle in angular order, starting at rim index off.
func fanBoxes(c r3.Vec, n, off, rimN int, radius float64) [][2]r3.Vec {
	rim := func(k int) r3.Vec {
		a := 2 * math.Pi * float64(k%rimN) / float64(rimN)
		return r3.NewVec(c.X+radius*math.Cos(a), c.Y+radius*math.Sin(a), c.Z)
	}
	boxes := make([][2]r3.Vec, n)
	for k := range boxes {
		boxes[k] = boxOf(c, rim(off+k), rim(off+k+1))
	}
	return boxes
}

func shortcutFixtures() map[string][][2]r3.Vec {
	rng := rand.New(rand.NewPCG(61, 67))
	walls := func(n int) [][2]r3.Vec {
		out := make([][2]r3.Vec, n)
		for i := range out {
			c := r3.NewVec(rng.Float64()*4-2, rng.Float64()*4-2, rng.Float64()-0.5)
			h := r3.NewVec(0.3*rng.Float64(), 0.3*rng.Float64(), 0.3*rng.Float64())
			out[i] = [2]r3.Vec{c.Sub(h), c.Add(h)}
		}
		return out
	}
	shuffle := func(in [][2]r3.Vec) [][2]r3.Vec {
		out := slices.Clone(in)
		rng.Shuffle(len(out), func(a, b int) { out[a], out[b] = out[b], out[a] })
		return out
	}
	fan := fanBoxes(r3.NewVec(0, 0, 0), 600, 0, 600, 1)
	fixtures := map[string][][2]r3.Vec{}
	fixtures["fan+walls"] = append(slices.Clone(fan), walls(400)...)
	fixtures["fan shuffled+walls"] = shuffle(append(slices.Clone(fan), walls(400)...))
	// Two fans around different centres sharing the rim vertices.
	two := append(fanBoxes(r3.NewVec(0, 0, 0), 300, 0, 300, 1), fanBoxes(r3.NewVec(0, 0, 0.5), 300, 0, 300, 1)...)
	fixtures["two fans"] = shuffle(two)
	fixtures["random"] = walls(700)
	// Zero extent on one axis, and boxes touching exactly at a face.
	var flat [][2]r3.Vec
	for k := range 300 {
		x := float64(k % 7)
		flat = append(flat, [2]r3.Vec{r3.NewVec(x, 0, 0), r3.NewVec(x+1, 1+float64(k%3), 0)})
	}
	fixtures["degenerate"] = shuffle(flat)
	nan := slices.Clone(fixtures["fan+walls"])
	nan[17][0].Y = math.NaN()
	fixtures["nan member"] = nan
	return fixtures
}

func TestShortcutRowWalksMatchReference(t *testing.T) {
	for name, boxes := range shortcutFixtures() {
		t.Run(name, func(t *testing.T) {
			tree := newTreeScan(boxes, allMembers(len(boxes)))
			shortcuts := 0
			for i := 0; i < len(boxes); i += 3 {
				_, work, exceeded := walkRowAppendReference(tree.flat, boxes[i], i, math.MaxUint64)
				require.False(t, exceeded)
				for _, limit := range []uint64{0, 1, 17, work / 2, max(work, 1) - 1, work, math.MaxUint64} {
					want, wantWork, wantExceeded := walkRowAppendReference(tree.flat, boxes[i], i, limit)
					got, gotWork, gotExceeded := tree.walkRowAppend(i, limit, nil)
					require.Equal(t, want, got, "row %d limit %d", i, limit)
					require.Equal(t, wantWork, gotWork, "row %d limit %d", i, limit)
					require.Equal(t, wantExceeded, gotExceeded, "row %d limit %d", i, limit)

					var viaFn []int32
					fnWork, fnExceeded := tree.walkRow(i, limit, func(j int) { viaFn = append(viaFn, int32(j)) })
					require.Equal(t, want, viaFn, "row %d limit %d", i, limit)
					require.Equal(t, wantWork, fnWork)
					require.Equal(t, wantExceeded, fnExceeded)

					count, countWork, countExceeded := tree.flat.walkRowCount(boxes[i], i, limit)
					require.Equal(t, len(want), count, "row %d limit %d", i, limit)
					require.Equal(t, wantWork, countWork)
					require.Equal(t, wantExceeded, countExceeded)
				}
				for node := range tree.flat.left {
					b := box6(boxes[i])
					if tree.flat.minID[node] > int32(i) && tree.flat.interOK[node] &&
						boxes6Overlap(b[:], tree.flat.inter[6*node:6*node+6]) {
						shortcuts++
					}
				}
			}
			if name == "fan+walls" || name == "fan shuffled+walls" {
				require.Positive(t, shortcuts, "the fixture must reach the shortcut")
			}
			if name == "nan member" {
				require.False(t, tree.flat.interOK[0])
			}
		})
	}
}

func TestShortcutPairCountMatchesReference(t *testing.T) {
	filtering := func(i, j int) bool { return (i+j)%3 != 0 }
	everyPair := func(int, int) bool { return true } // not keepEveryPair: no shortcut
	require.True(t, isKeepEveryPair(keepEveryPair))
	require.False(t, isKeepEveryPair(everyPair))
	for name, boxes := range shortcutFixtures() {
		t.Run(name, func(t *testing.T) {
			tree := newTreeScan(boxes, allMembers(len(boxes)))
			f := len(boxes)
			ref := make([]int32, f+1)
			total, exceeded := pairWalkReference(&tree, 0, 0, math.MaxUint64, ref)
			require.False(t, exceeded)
			tree.flat.ensureSorted()
			for _, limit := range []uint64{math.MaxUint64, total, total - 1, total / 2, 1000, 1} {
				want := make([]int32, f+1)
				wantWork, wantExceeded := pairWalkReference(&tree, 0, 0, limit, want)
				got := make([]int32, f+1)
				w := pairWalker{t: &tree, limit: limit, counts: got, memo: map[[3]int32]uint64{},
					fn: func(i, j int) { got[i+1]++ }}
				gotExceeded := w.walk(0, 0)
				require.Equal(t, wantExceeded, gotExceeded, "limit %d", limit)
				require.Equal(t, wantWork, w.scanned, "limit %d", limit)
				if !wantExceeded {
					require.Equal(t, want, got, "limit %d", limit)
				}
			}
			for _, limit := range []uint64{math.MaxUint64, total, total - 1, 1000} {
				for _, keep := range []func(i, j int) bool{keepEveryPair, everyPair, filtering} {
					want, wantErr := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, f, keep, limit, 1)
					got, gotErr := tree.countParallel(unlimitedBudget(t), f, keep, limit, 4)
					require.Equal(t, wantErr, gotErr)
					require.Equal(t, want, got, "limit %d", limit)
				}
			}
		})
	}
}

// allOverlapWork is the unit count of walk's split rule over node a of fa and
// node b of fb (the same tree when self), as if every pair overlapped.
func allOverlapWork(fa, fb *flatTree, a, b int32, self bool) uint64 {
	n := uint64(1)
	la, lb := fa.left[a], fb.left[b]
	na, nb := uint64(fa.end[a]-fa.start[a]), uint64(fb.end[b]-fb.start[b])
	if la < 0 && lb < 0 {
		if self {
			return n + na*(na-1)/2
		}
		return n + na*nb
	}
	ra, rb := fa.right[a], fb.right[b]
	switch {
	case self:
		return n + allOverlapWork(fa, fa, la, la, true) + allOverlapWork(fa, fa, la, ra, false) +
			allOverlapWork(fa, fa, ra, ra, true)
	case lb < 0 || (la >= 0 && na >= nb):
		return n + allOverlapWork(fa, fb, la, b, false) + allOverlapWork(fa, fb, ra, b, false)
	default:
		return n + allOverlapWork(fa, fb, a, lb, false) + allOverlapWork(fa, fb, a, rb, false)
	}
}

func TestPairWorkMatchesWalk(t *testing.T) {
	same := func(n int) *flatTree {
		boxes := make([][2]r3.Vec, n)
		for i := range boxes {
			boxes[i] = [2]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1)}
		}
		return newTreeScan(boxes, allMembers(n)).flat
	}
	memo := map[[3]int32]uint64{}
	for n := 1; n <= 300; n++ {
		f := same(n)
		require.Equal(t, allOverlapWork(f, f, 0, 0, true), pairWork(memo, int32(n), 0, true), "self %d", n)
	}
	for _, q := range []int{1, 7, 8, 9, 17, 33, 64, 100, 300} {
		fq := same(q)
		for p := 1; p <= 300; p += 1 + p/16 {
			fp := same(p)
			require.Equal(t, allOverlapWork(fp, fq, 0, 0, false), pairWork(memo, int32(p), int32(q), false),
				"sizes %d %d", p, q)
		}
	}
}
