package retainedblend

import (
	"cmp"
	"math"
	"math/rand/v2"
	"slices"
)

// bridgeKeyShapes maps a shape name to a generator of the candidate key at index i of n.
func bridgeKeyShapes(r *rand.Rand) map[string]func(i, n int) float64 {
	return map[string]func(i, n int) float64{
		"few values": func(i, n int) float64 { return float64(r.IntN(4)) },
		"many ties":  func(i, n int) float64 { return float64(r.IntN(n/8 + 1)) },
		"distinct":   func(i, n int) float64 { return r.Float64() },
		"sorted":     func(i, n int) float64 { return float64(i / 3) },
		"reversed":   func(i, n int) float64 { return float64((n - i) / 3) },
		"equal":      func(i, n int) float64 { return 1 },
		"sawtooth":   func(i, n int) float64 { return float64(i % 17) },
		"organ pipe": func(i, n int) float64 { return float64(min(i, n-i)) },
		"nan":        func(i, n int) float64 { return []float64{math.NaN(), 1, 2, 2, math.Inf(1)}[r.IntN(5)] },
		"nan first":  func(i, n int) float64 { return []float64{math.NaN(), math.NaN(), 3, 3}[r.IntN(4)] },
		"min last":   func(i, n int) float64 { return float64(n - i) },
	}
}

// bridgeKeyCandidates returns n candidates whose m field records the input position, so equal keys stay told apart.
func bridgeKeyCandidates(n int, key func(i, n int) float64) []bridgeCandidate {
	data := make([]bridgeCandidate, n)
	for i := range data {
		data[i] = bridgeCandidate{d: key(i, n), m: i}
	}
	return data
}

// sortedFirstInput returns the input position of the candidate slices.SortFunc leaves at index 0.
func sortedFirstInput(data []bridgeCandidate) int {
	want := slices.Clone(data)
	slices.SortFunc(want, func(a, b bridgeCandidate) int { return cmp.Compare(a.d, b.d) })
	return want[0].m
}

// sizesAroundPdqsortThresholds lists the sizes where pdqsort changes strategy: insertion sort up to 12, the ninther
// from 50, partial insertion shifting from 50, and the larger sizes that exercise partition recursion and the
// heapsort depth limit.
func sizesAroundPdqsortThresholds() []int {
	return []int{
		0, 1, 2, 3, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 31, 32, 33, 49, 50, 51, 52, 63, 64, 65, 100,
		127, 128, 129, 257, 1000, 4099, 20000,
	}
}

// ringScene returns an n-gon outer loop of radius 10 and a concentric n-gon hole of radius 4 turned by rot, so a
// rot of 0 puts every hole vertex on a ray through an outer vertex and ties n candidates.
func ringScene(n int, rot float64) ([]pt2, []int, [][]int) {
	var ps []pt2
	outer := make([]int, n)
	hole := make([]int, n)
	for i := range n {
		a := 2 * math.Pi * float64(i) / float64(n)
		outer[i] = len(ps)
		ps = append(ps, pt2{10 * math.Cos(a), 10 * math.Sin(a)})
	}
	// The hole runs clockwise, as a hole loop does.
	for i := range n {
		a := rot - 2*math.Pi*float64(i)/float64(n)
		hole[i] = len(ps)
		ps = append(ps, pt2{4 * math.Cos(a), 4 * math.Sin(a)})
	}
	return ps, outer, [][]int{hole}
}

// bridgeRandomScene builds a 20 x 20 square with one or two holes of 3 to 5 vertices on a 2 mm grid, so many
// candidates tie. Some holes are concave or self-crossing; the sort reference sees the same scene.
func bridgeRandomScene(r *rand.Rand) ([]pt2, []int, [][]int) {
	ps := []pt2{{0, 0}, {20, 0}, {20, 20}, {0, 20}}
	outer := []int{0, 1, 2, 3}
	var holes [][]int
	for range 1 + r.IntN(2) {
		var hole []int
		for range 3 + r.IntN(3) {
			hole = append(hole, len(ps))
			ps = append(ps, pt2{float64(2 * (1 + r.IntN(9))), float64(2 * (1 + r.IntN(9)))})
		}
		holes = append(holes, hole)
	}
	return ps, outer, holes
}

// firstRejected reports whether the candidate the first-element selector picks fails the acceptance test, so
// bridgeNearest takes the full-sort fallback, and whether the minimum distance is tied.
func firstRejected(ps []pt2, merged []int, holes [][]int) (rejected, tied bool) {
	var cands []bridgeCandidate
	for hi, hole := range holes {
		for hk, hv := range hole {
			for mi, mv := range merged {
				du, dw := ps[hv].u-ps[mv].u, ps[hv].w-ps[mv].w
				cands = append(cands, bridgeCandidate{d: du*du + dw*dw, m: mi, h: hk, holeIdx: hi})
			}
		}
	}
	if len(cands) == 0 {
		return false, false
	}
	minD := slices.MinFunc(cands, func(a, b bridgeCandidate) int { return cmp.Compare(a.d, b.d) }).d
	minCount := 0
	for _, c := range cands {
		if c.d == minD {
			minCount++
		}
	}
	bridgeFirstSort(cands)
	c := cands[0]
	hole := holes[c.holeIdx]
	a, b := ps[merged[c.m]], ps[hole[c.h]]
	ok := bridgeClear(ps, merged, holes, c.holeIdx, c.m, c.h) &&
		insidePolygon(ps, merged, pt2{(a.u + b.u) / 2, (a.w + b.w) / 2})
	return !ok, minCount > 1
}

// bridgeSweepShape is a named generator of n candidate keys. The generator may use r, so repeats differ.
type bridgeSweepShape struct {
	name string
	keys func(r *rand.Rand, n int) []float64
}

func bridgeSweepFill(n int, f func(i int) float64) []float64 {
	keys := make([]float64, n)
	for i := range keys {
		keys[i] = f(i)
	}
	return keys
}

// bridgeNearlySorted returns ascending keys with ties (step 3) and swaps random pairs, some of them near the start,
// so partial insertion sort has to shift a small key left across the front of the range.
func bridgeNearlySorted(r *rand.Rand, n int, swaps int, front bool) []float64 {
	keys := bridgeSweepFill(n, func(i int) float64 { return float64(i / 3) })
	for range swaps {
		if n < 2 {
			break
		}
		i, j := r.IntN(n), r.IntN(n)
		if front {
			i = r.IntN(min(n, 4))
		}
		keys[i], keys[j] = keys[j], keys[i]
	}
	return keys
}

// bridgeSweepShapes lists the key shapes the sweep covers.
func bridgeSweepShapes() []bridgeSweepShape {
	pick := func(vals ...float64) func(r *rand.Rand, n int) []float64 {
		return func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return vals[r.IntN(len(vals))] })
		}
	}
	return []bridgeSweepShape{
		{"all equal", func(_ *rand.Rand, n int) []float64 { return bridgeSweepFill(n, func(int) float64 { return 7 }) }},
		{"two values", pick(0, 1)},
		{"three values", pick(0, 1, 2)},
		{"many ties", func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return float64(r.IntN(n/8 + 1)) })
		}},
		{"sorted ties", func(_ *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(i int) float64 { return float64(i / 3) })
		}},
		{"reversed ties", func(_ *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(i int) float64 { return float64((n - i) / 3) })
		}},
		{"sawtooth", func(_ *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(i int) float64 { return float64(i % 17) })
		}},
		{"organ pipe", func(_ *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(i int) float64 { return float64(min(i, n-i)) })
		}},
		{"inverted organ pipe", func(_ *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(i int) float64 { return float64(max(i, n-i)) })
		}},
		{"signed zeros", func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return []float64{0, math.Copysign(0, -1), 1}[r.IntN(3)] })
		}},
		{"nan mix", func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return []float64{math.NaN(), 1, 2, 2, math.Inf(1)}[r.IntN(5)] })
		}},
		{"nan minimum", func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return []float64{math.NaN(), math.NaN(), 3, 3}[r.IntN(4)] })
		}},
		{"rare minimum ties", func(r *rand.Rand, n int) []float64 {
			keys := bridgeSweepFill(n, func(int) float64 { return 1 + r.Float64() })
			for range 2 + r.IntN(3) {
				if n > 0 {
					keys[r.IntN(n)] = 0
				}
			}
			return keys
		}},
		{"distinct", func(r *rand.Rand, n int) []float64 {
			return bridgeSweepFill(n, func(int) float64 { return r.Float64() })
		}},
		{"nearly sorted", func(r *rand.Rand, n int) []float64 { return bridgeNearlySorted(r, n, 1+r.IntN(3), false) }},
		{"nearly sorted front", func(r *rand.Rand, n int) []float64 { return bridgeNearlySorted(r, n, 1+r.IntN(3), true) }},
		{"nearly reversed", func(r *rand.Rand, n int) []float64 {
			keys := bridgeNearlySorted(r, n, 1+r.IntN(3), false)
			slices.Reverse(keys)
			return keys
		}},
	}
}

// bridgeBoundaryScene builds a 20 x 20 square outer loop with a vertex every 2 units on each side (40 vertices) and
// one or two holes of 3 to 5 vertices on the 2-unit grid inside it, so many candidates tie. Some holes are concave or
// self-crossing; the sort reference sees the same scene.
func bridgeBoundaryScene(r *rand.Rand) ([]pt2, []int, [][]int) {
	var ps []pt2
	var outer []int
	add := func(u, w float64) {
		outer = append(outer, len(ps))
		ps = append(ps, pt2{u, w})
	}
	for k := range 10 {
		add(float64(2*k), 0)
	}
	for k := range 10 {
		add(20, float64(2*k))
	}
	for k := range 10 {
		add(float64(20-2*k), 20)
	}
	for k := range 10 {
		add(0, float64(20-2*k))
	}
	var holes [][]int
	for range 1 + r.IntN(2) {
		var hole []int
		for range 3 + r.IntN(3) {
			hole = append(hole, len(ps))
			ps = append(ps, pt2{float64(2 * (1 + r.IntN(9))), float64(2 * (1 + r.IntN(9)))})
		}
		holes = append(holes, hole)
	}
	return ps, outer, holes
}
