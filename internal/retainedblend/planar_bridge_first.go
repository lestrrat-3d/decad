// Portions of this file are copied from the Go standard library, package slices (zsortanyfunc.go), Go 1.27.4.
//
// Copyright 2009 The Go Authors.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google LLC nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

package retainedblend

// bridgeNearest picks, among candidates at the same distance, the one slices.SortFunc leaves first. Which of several
// equal keys comes first is decided by the pdqsort steps of the Go toolchain, not by decad, and the chosen bridge
// shapes every triangulation built from it. This file copies those steps from Go 1.27.4 so the choice stays what
// current results depend on, whatever a later toolchain does to its sort. Sorting every candidate only to read the
// first costs O(n log n); this copy runs only the steps on ranges that hold index 0, which costs O(n).
//
// The copy is specialised to bridgeCandidate ordered by cmp.Less(a.d, b.d), which equals cmp.Compare(a.d, b.d) < 0
// for every pair (NaN included); pdqsort reads only that outcome. Steps on disjoint ranges touch disjoint elements,
// so skipping every range right of a pivot leaves data[0] as the full sort leaves it. Partial insertion sort's left
// shift stops at index 1, so it cannot cross a pivot either.

import (
	"cmp"
	"math/bits"
)

// bridgeCandidate is one hole-vertex to boundary-vertex segment, keyed by its squared length d.
type bridgeCandidate struct {
	d       float64
	m, h    int // position in merged, position in the hole
	holeIdx int
}

func bridgeLess(a, b bridgeCandidate) bool { return cmp.Less(a.d, b.d) }

// bridgeFirstSort reorders data so that data[0] is the element slices.SortFunc(data, by d) would leave there.
// The rest of data is left in an unspecified order.
func bridgeFirstSort(data []bridgeCandidate) {
	n := len(data)
	bridgeFirstRange(data, 0, n, bits.Len(uint(n)))
}

func bridgeFirstRange(data []bridgeCandidate, a, b, limit int) {
	const maxInsertion = 12
	wasBalanced, wasPartitioned := true, true
	for {
		length := b - a
		if length <= maxInsertion {
			bsInsertionSort(data, a, b)
			return
		}
		if limit == 0 {
			bsHeapSort(data, a, b)
			return
		}
		if !wasBalanced {
			bsBreakPatterns(data, a, b)
			limit--
		}
		pivot, hint := bsChoosePivot(data, a, b)
		if hint == decreasingHint {
			bsReverseRange(data, a, b)
			pivot = (b - 1) - (pivot - a)
			hint = increasingHint
		}
		if wasBalanced && wasPartitioned && hint == increasingHint {
			if bsPartialInsertionSort(data, a, b) {
				return
			}
		}
		// a is always 0 here, so Go's partitionEqual branch (which needs a > 0) never runs.
		mid, already := bsPartition(data, a, b, pivot)
		wasPartitioned = already
		leftLen, rightLen := mid-a, b-mid
		balanceThreshold := length / 8
		if leftLen < rightLen {
			// SortFunc recurses into [a, mid) with fresh flags, then loops on [mid+1, b).
			bridgeFirstRange(data, a, mid, limit)
			return
		}
		// SortFunc recurses into [mid+1, b), then loops on [a, mid).
		wasBalanced = rightLen >= balanceThreshold
		b = mid
	}
}

func bsInsertionSort(data []bridgeCandidate, a, b int) {
	for i := a + 1; i < b; i++ {
		for j := i; j > a && bridgeLess(data[j], data[j-1]); j-- {
			data[j], data[j-1] = data[j-1], data[j]
		}
	}
}

func bsSiftDown(data []bridgeCandidate, lo, hi, first int) {
	root := lo
	for {
		child := 2*root + 1
		if child >= hi {
			break
		}
		if child+1 < hi && bridgeLess(data[first+child], data[first+child+1]) {
			child++
		}
		if !bridgeLess(data[first+root], data[first+child]) {
			return
		}
		data[first+root], data[first+child] = data[first+child], data[first+root]
		root = child
	}
}

func bsHeapSort(data []bridgeCandidate, a, b int) {
	first := a
	lo := 0
	hi := b - a
	for i := (hi - 1) / 2; i >= 0; i-- {
		bsSiftDown(data, i, hi, first)
	}
	for i := hi - 1; i >= 0; i-- {
		data[first], data[first+i] = data[first+i], data[first]
		bsSiftDown(data, lo, i, first)
	}
}

func bsPartition(data []bridgeCandidate, a, b, pivot int) (int, bool) {
	data[a], data[pivot] = data[pivot], data[a]
	i, j := a+1, b-1
	for i <= j && bridgeLess(data[i], data[a]) {
		i++
	}
	for i <= j && !bridgeLess(data[j], data[a]) {
		j--
	}
	if i > j {
		data[j], data[a] = data[a], data[j]
		return j, true
	}
	data[i], data[j] = data[j], data[i]
	i++
	j--
	for {
		for i <= j && bridgeLess(data[i], data[a]) {
			i++
		}
		for i <= j && !bridgeLess(data[j], data[a]) {
			j--
		}
		if i > j {
			break
		}
		data[i], data[j] = data[j], data[i]
		i++
		j--
	}
	data[j], data[a] = data[a], data[j]
	return j, false
}

func bsPartialInsertionSort(data []bridgeCandidate, a, b int) bool {
	const (
		maxSteps         = 5
		shortestShifting = 50
	)
	i := a + 1
	for range maxSteps {
		for i < b && !bridgeLess(data[i], data[i-1]) {
			i++
		}
		if i == b {
			return true
		}
		if b-a < shortestShifting {
			return false
		}
		data[i], data[i-1] = data[i-1], data[i]
		if i-a >= 2 {
			for j := i - 1; j >= 1; j-- {
				if !bridgeLess(data[j], data[j-1]) {
					break
				}
				data[j], data[j-1] = data[j-1], data[j]
			}
		}
		if b-i >= 2 {
			for j := i + 1; j < b; j++ {
				if !bridgeLess(data[j], data[j-1]) {
					break
				}
				data[j], data[j-1] = data[j-1], data[j]
			}
		}
	}
	return false
}

type bsXorshift uint64

func (r *bsXorshift) Next() uint64 {
	*r ^= *r << 13
	*r ^= *r >> 7
	*r ^= *r << 17
	return uint64(*r)
}

func bsNextPowerOfTwo(length int) uint {
	return 1 << bits.Len(uint(length))
}

func bsBreakPatterns(data []bridgeCandidate, a, b int) {
	length := b - a
	if length >= 8 {
		random := bsXorshift(length)
		modulus := bsNextPowerOfTwo(length)
		for idx := a + (length/4)*2 - 1; idx <= a+(length/4)*2+1; idx++ {
			other := int(uint(random.Next()) & (modulus - 1))
			if other >= length {
				other -= length
			}
			data[idx], data[a+other] = data[a+other], data[idx]
		}
	}
}

type bridgeSortHint int

const (
	unknownHint bridgeSortHint = iota
	increasingHint
	decreasingHint
)

func bsChoosePivot(data []bridgeCandidate, a, b int) (int, bridgeSortHint) {
	const (
		shortestNinther = 50
		maxSwaps        = 4 * 3
	)
	l := b - a
	var (
		swaps int
		i     = a + l/4*1
		j     = a + l/4*2
		k     = a + l/4*3
	)
	if l >= 8 {
		if l >= shortestNinther {
			i = bsMedian(data, i-1, i, i+1, &swaps)
			j = bsMedian(data, j-1, j, j+1, &swaps)
			k = bsMedian(data, k-1, k, k+1, &swaps)
		}
		j = bsMedian(data, i, j, k, &swaps)
	}
	switch swaps {
	case 0:
		return j, increasingHint
	case maxSwaps:
		return j, decreasingHint
	default:
		return j, unknownHint
	}
}

func bsOrder2(data []bridgeCandidate, a, b int, swaps *int) (int, int) {
	if bridgeLess(data[b], data[a]) {
		*swaps++
		return b, a
	}
	return a, b
}

func bsMedian(data []bridgeCandidate, a, b, c int, swaps *int) int {
	a, b = bsOrder2(data, a, b, swaps)
	b, _ = bsOrder2(data, b, c, swaps)
	_, b = bsOrder2(data, a, b, swaps)
	return b
}

func bsReverseRange(data []bridgeCandidate, a, b int) {
	i := a
	j := b - 1
	for i < j {
		data[i], data[j] = data[j], data[i]
		i++
		j--
	}
}
