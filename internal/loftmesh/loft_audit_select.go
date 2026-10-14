package loftmesh

import "slices"

// selectRounds is the number of partition rounds selectNth runs before it
// sorts the rest.
const selectRounds = 64

// selectNth rearranges x so that x[k] is the element a full sort by cmp would
// put there, every element before k is less than it and every element after k
// is greater. cmp must be a strict total order on x's elements (no two
// distinct elements compare equal), which is what makes the two sides equal as
// sets to the sides of a sorted x. The order inside each side is unspecified.
//
// It is quickselect with a Hoare partition around a median-of-three pivot.
// After selectRounds rounds it sorts the remaining range, so the worst case
// is O(n log n). k must satisfy 0 <= k < len(x). It reports whether it fell
// back to the sort.
func selectNth(x []int32, k int, cmp func(a, b int32) int) bool {
	return selectNthRounds(x, k, cmp, selectRounds)
}

// selectNthRounds is selectNth with the round cap as a parameter, so a test
// can force the sort fallback. It reports whether the fallback ran.
func selectNthRounds(x []int32, k int, cmp func(a, b int32) int, rounds int) bool {
	lo, hi := 0, len(x)-1
	for round := 0; lo < hi; round++ {
		if round >= rounds {
			slices.SortFunc(x[lo:hi+1], cmp)
			return true
		}
		pivot := medianOfThree(x[lo], x[lo+(hi-lo)/2], x[hi], cmp)
		i, j := lo, hi
		for i <= j {
			// The pivot is an element of x[lo:hi+1], so each scan stops at it
			// at the latest on the first pass and at a swapped element after.
			for cmp(x[i], pivot) < 0 {
				i++
			}
			for cmp(x[j], pivot) > 0 {
				j--
			}
			if i <= j {
				x[i], x[j] = x[j], x[i]
				i++
				j--
			}
		}
		// Now x[lo:j+1] <= pivot <= x[i:hi+1], and anything between j and i
		// equals the pivot, so it is final.
		switch {
		case k <= j:
			hi = j
		case k >= i:
			lo = i
		default:
			return false
		}
	}
	return false
}

func medianOfThree(a, b, c int32, cmp func(a, b int32) int) int32 {
	if cmp(a, b) > 0 {
		a, b = b, a
	}
	// a <= b.
	if cmp(b, c) <= 0 {
		return b
	}
	// c < b.
	if cmp(a, c) <= 0 {
		return c
	}
	return a
}
