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
