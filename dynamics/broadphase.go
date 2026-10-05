package dynamics

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad"
)

// broadPhaseEntry is one body's swept box with the outward float extent of
// its X interval, which orders the sort-and-sweep walk.
type broadPhaseEntry struct {
	body   int
	box    decad.SweptBox
	lo, hi float64
}

// broadPhaseCandidates is the sort-and-sweep of docs/multibody-dynamics-design.md
// §4.3. It returns the scheduled pairs whose swept boxes are not strictly
// disjoint, as canonical pair keys in ascending order.
//
// The walk sorts bodies by the outward float lower X extent of their swept
// box, ties by world index, and keeps an active set. An active body leaves the
// set when its outward upper X extent lies strictly below the next body's
// outward lower X extent: the float extents enclose the exact ones, so the
// exact X intervals of that body and every later one are strictly apart, which
// is StrictlyDisjoint's own certificate on that axis. Every pair the walk
// meets is decided by StrictlyDisjoint on the exact extremes. The candidate
// list is sorted into canonical pair order, so no sort tie or walk order
// reaches a published result.
func broadPhaseCandidates(boxes []decad.SweptBox, scheduled map[int]struct{}) []int {
	n := len(boxes)
	entries := make([]broadPhaseEntry, n)
	for i, box := range boxes {
		extent := box.Box()
		entries[i] = broadPhaseEntry{body: i, box: box, lo: extent.Min.X, hi: extent.Max.X}
	}
	slices.SortFunc(entries, func(a, b broadPhaseEntry) int {
		if c := cmp.Compare(a.lo, b.lo); c != 0 {
			return c
		}
		return cmp.Compare(a.body, b.body)
	})
	var candidates []int
	active := make([]broadPhaseEntry, 0, n)
	for _, entry := range entries {
		kept := active[:0]
		for _, other := range active {
			if other.hi < entry.lo {
				continue
			}
			kept = append(kept, other)
		}
		active = kept
		for _, other := range active {
			key := canonicalPairIndex(n, other.body, entry.body)
			if _, ok := scheduled[key]; !ok {
				continue
			}
			if other.box.StrictlyDisjoint(entry.box) {
				continue
			}
			candidates = append(candidates, key)
		}
		active = append(active, entry)
	}
	slices.Sort(candidates)
	return candidates
}
