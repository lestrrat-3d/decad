// Package arrangeplan packs axis-aligned rectangles onto a bed in shelves
// (agent-docs/arrange-design.md). It works on plain millimetre data and
// touches no body or document.
package arrangeplan

import (
	"sort"
)

// Item is the footprint of one body: its extent along X and along Y.
type Item struct {
	Width, Depth float64
}

// Bed is the rectangle the items go on, in millimetres, and the clear gap kept
// between any two items and between an item and the bed's edge.
type Bed struct {
	Width, Depth, Gap float64
}

// Pack returns the position of each item's minimum corner, measured from the
// bed's minimum corner, in the order of items. ok is false when some item
// does not fit.
//
// Items are placed deepest first. Each goes to the right of the previous one
// on the current shelf, and a shelf is as deep as its deepest item. An item
// that would cross the bed's right edge starts a new shelf above the last, and
// one that would cross the top edge ends the search. Items of equal depth keep
// their input order, so the result is deterministic.
func Pack(items []Item, bed Bed) (positions [][2]float64, ok bool) {
	order := make([]int, len(items))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(a, b int) bool { return items[order[a]].Depth > items[order[b]].Depth })

	positions = make([][2]float64, len(items))
	x, y, shelf := bed.Gap, bed.Gap, 0.0
	for _, i := range order {
		it := items[i]
		if x+it.Width+bed.Gap > bed.Width && x > bed.Gap {
			x = bed.Gap
			y += shelf + bed.Gap
			shelf = 0
		}
		if x+it.Width+bed.Gap > bed.Width || y+it.Depth+bed.Gap > bed.Depth {
			return nil, false
		}
		positions[i] = [2]float64{x, y}
		x += it.Width + bed.Gap
		shelf = max(shelf, it.Depth)
	}
	return positions, true
}
