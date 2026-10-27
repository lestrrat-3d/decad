package loftmesh

import "slices"

// selectRounds is the number of partition rounds selectNth runs before it
// sorts the rest, and selectNintherMin the range size from which it takes
// the ninther pivot.
const (
	selectRounds     = 64
	selectNintherMin = 128
)

// treeKey is one member in the tree build's selection: its box midpoint on
// the node's split axis (treeMidpoints), its member id, and its local index
// into the build's member arrays. less orders by midpoint, ties by member id.
// With no NaN midpoint it is a strict total order, because ids are distinct.
type treeKey struct {
	mid float64
	id  int32
	l   int32
}

func (a treeKey) less(b treeKey) bool {
	return a.mid < b.mid || (a.mid == b.mid && a.id < b.id)
}

// selectNth rearranges x so that x[k] is the element a full sort by less would
// put there, every element before k is less than it and every element after k
// is greater. less must be a strict total order on x's elements (no NaN
// midpoint), which is what makes the two sides equal as sets to the sides of
// a sorted x. The order inside each side is unspecified.
//
// It is quickselect with a Hoare partition around a median-of-three pivot,
// or, on a range of selectNintherMin elements or more, Tukey's ninther (the
// median of three medians of three), which keeps periodic orders such as a
// twisted loft's from defeating the pivot. After selectRounds rounds it
// sorts the remaining range, so the worst case is O(n log n). k must satisfy
// 0 <= k < len(x). It reports whether it fell back to the sort.
func selectNth(x []treeKey, k int) bool {
	return selectNthRounds(x, k, selectRounds)
}

// selectNthRounds is selectNth with the round cap as a parameter, so a test
// can force the sort fallback. It reports whether the fallback ran.
func selectNthRounds(x []treeKey, k int, rounds int) bool {
	lo, hi := 0, len(x)-1
	for round := 0; lo < hi; round++ {
		if round >= rounds {
			sortTreeKeys(x[lo : hi+1])
			return true
		}
		m := lo + (hi-lo)/2
		var pivot treeKey
		if n := hi - lo + 1; n < selectNintherMin {
			pivot = medianOfThree(x[lo], x[m], x[hi])
		} else {
			s := n / 8
			pivot = medianOfThree(
				medianOfThree(x[lo], x[lo+s], x[lo+2*s]),
				medianOfThree(x[m-s], x[m], x[m+s]),
				medianOfThree(x[hi-2*s], x[hi-s], x[hi]))
		}
		i, j := lo, hi
		for i <= j {
			// The pivot is an element of x[lo:hi+1], so each scan stops at it
			// at the latest on the first pass and at a swapped element after.
			for x[i].less(pivot) {
				i++
			}
			for pivot.less(x[j]) {
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

func medianOfThree(a, b, c treeKey) treeKey {
	if b.less(a) {
		a, b = b, a
	}
	// a <= b.
	if !c.less(b) {
		return b
	}
	// c < b.
	if !c.less(a) {
		return c
	}
	return a
}

// sortTreeKeys sorts x by less: insertion sort for a leaf-sized range, else
// slices.SortFunc. less is a strict total order, so the result is the one
// sorted sequence either way.
func sortTreeKeys(x []treeKey) {
	if len(x) > 2*treeLeafSize {
		slices.SortFunc(x, func(a, b treeKey) int {
			switch {
			case a.less(b):
				return -1
			case b.less(a):
				return 1
			default:
				return 0
			}
		})
		return
	}
	for i := 1; i < len(x); i++ {
		for j := i; j > 0 && x[j].less(x[j-1]); j-- {
			x[j], x[j-1] = x[j-1], x[j]
		}
	}
}
