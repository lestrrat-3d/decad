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

// PackTurning is Pack for items that may turn a quarter about Z, which swaps
// an item's width and depth. It returns the position of each item's minimum
// corner after any turn, and which items turned.
//
// It tries two layouts and returns the first that fits. In the first every item
// lies with its longer side along X, which keeps shelves shallow. In the second
// no item turns, so a set that Pack fits is never refused for being allowed to
// turn.
func PackTurning(items []Item, bed Bed) (positions [][2]float64, turned []bool, ok bool) {
	turned = make([]bool, len(items))
	laid := make([]Item, len(items))
	for i, it := range items {
		laid[i] = it
		if it.Depth > it.Width {
			laid[i] = Item{Width: it.Depth, Depth: it.Width}
			turned[i] = true
		}
	}
	if positions, ok = Pack(laid, bed); ok {
		return positions, turned, true
	}
	positions, ok = Pack(items, bed)
	return positions, make([]bool, len(items)), ok
}

// Layout is where PackLayers puts each item: its minimum corner on the bed, the
// height of the bed's surface it rests on, and whether it turned a quarter.
type Layout struct {
	Positions [][2]float64
	Turned    []bool
	// ZBase is each item's base above the bed's surface, in millimetres.
	ZBase []float64
	// Layers counts the layers used.
	Layers int
}

// PackLayers lays items on the bed in layers, one above another. heights are the
// items' extents along Z, in the order of items. Items are taken in order: each
// joins the current layer if that layer still packs, by Pack, or by PackTurning
// when turning is set, and otherwise waits for a later layer. A layer rests above
// the tallest item of the one below, by a clear gap of bed.Gap. ok is false when
// an item fits no empty layer, or when the layers together stand higher than
// maxHeight.
func PackLayers(items []Item, heights []float64, bed Bed, maxHeight float64, turning bool) (layout Layout, ok bool) {
	layout = Layout{
		Positions: make([][2]float64, len(items)),
		Turned:    make([]bool, len(items)),
		ZBase:     make([]float64, len(items)),
	}
	pack := func(set []int) ([][2]float64, []bool, bool) {
		sub := make([]Item, len(set))
		for k, i := range set {
			sub[k] = items[i]
		}
		if turning {
			return PackTurning(sub, bed)
		}
		pos, fits := Pack(sub, bed)
		return pos, make([]bool, len(set)), fits
	}
	remaining := make([]int, len(items))
	for i := range remaining {
		remaining[i] = i
	}
	base := 0.0
	for len(remaining) > 0 {
		var layer, waiting []int
		for _, i := range remaining {
			if _, _, fits := pack(append(append([]int(nil), layer...), i)); fits {
				layer = append(layer, i)
			} else {
				waiting = append(waiting, i)
			}
		}
		if len(layer) == 0 {
			return Layout{}, false
		}
		positions, turned, _ := pack(layer)
		tallest := 0.0
		for k, i := range layer {
			layout.Positions[i], layout.Turned[i], layout.ZBase[i] = positions[k], turned[k], base
			tallest = max(tallest, heights[i])
		}
		base += tallest + bed.Gap
		layout.Layers++
		remaining = waiting
	}
	if top := base - bed.Gap; top > maxHeight {
		return Layout{}, false
	}
	return layout, true
}
