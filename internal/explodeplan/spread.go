package explodeplan

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// Spread asks Run for the layer spread of agent-docs/parts-design.md §5.9.
type Spread struct {
	Axis, Across, Rows r3.Vec  // unit positive world axes
	Layer              []int   // per part: the derived half of L2, signed, 0 for every other part
	CosTol             float64 // cos of the axis tolerance (L1, L2)
	Gap                float64 // g of L4a, L4 and L5, mm, at least 1
	Thin               bool    // L3 (a) and (b) hold; Run adds (c)
	Stacks             [][]int // derived rigid groups L4a may stack, members ascending
}

// SpreadOutcome is what the spread step did.
type SpreadOutcome struct {
	Layer        []int // per part: its layer after the explicit half of L2; set whether or not the spread ran
	Layers       int   // the number of layers, the base included
	Stack        []int // per part: its L4a place, 0 when not stacked or when the spread did not run
	Coaxial      bool
	Ran          bool
	Columns      int
	Width, Depth float64 // the grid's extent along Across and Rows, mm, gaps included
}

// coaxialTol is how far outside a footprint a centre may lie and still count as inside (L3 c), in mm.
const coaxialTol = 1e-9

// footprint is the rectangle a layer's boxes cover across the axis, in coordinates along Across and Rows.
type footprint struct{ aLo, aHi, rLo, rHi float64 }

func (f footprint) holds(a, r float64) bool {
	return a >= f.aLo-coaxialTol && a <= f.aHi+coaxialTol && r >= f.rLo-coaxialTol && r <= f.rHi+coaxialTol
}

// explicitLayers is the explicit half of L2: each part an explicit stage moved last, along a direction, takes the
// layer of that stage. A stage lies on the + side when the sum of Dir·Axis over its parts is zero or more, else on
// the - side; on each side its layers are numbered 1, 2, ... in stage order and every derived layer there moves out
// by their count.
func (r *runner) explicitLayers() []int {
	sp := r.in.Spread
	layer := slices.Clone(sp.Layer)
	byStage := map[int][]int{}
	var stages []int
	for p := range layer {
		st := r.res[p].Stage
		if p == r.in.Ground || layer[p] != 0 || st < 0 || r.in.Stages[st].Round != 0 || r.res[p].Dir == (r3.Vec{}) {
			continue
		}
		if _, ok := byStage[st]; !ok {
			stages = append(stages, st)
		}
		byStage[st] = append(byStage[st], p)
	}
	slices.Sort(stages)
	var count [2]int // explicit layers on the + and - side
	for _, st := range stages {
		sum := 0.0
		for _, p := range byStage[st] {
			sum += r.res[p].Dir.Dot(sp.Axis)
		}
		side := 0
		if sum < 0 {
			side = 1
		}
		count[side]++
		k := count[side]
		if side == 1 {
			k = -k
		}
		for _, p := range byStage[st] {
			layer[p] = k
		}
	}
	for p, l := range sp.Layer {
		switch {
		case l > 0:
			layer[p] = l + count[0]
		case l < 0:
			layer[p] = l - count[1]
		}
	}
	return layer
}

// spread is the layer spread: the explicit half of L2, the coaxial test (L3 c), the stacks (L4a), the slabs (L4)
// and the cell grid (L5). It moves whole layers, and the members of a stacked group, so every move is added to
// the part's Slide.
func (r *runner) spread() SpreadOutcome {
	sp := r.in.Spread
	layer := r.explicitLayers()
	n := len(layer)
	out := SpreadOutcome{Layer: layer, Stack: make([]int, n)}
	lo, hi := 0, 0
	for _, l := range layer {
		lo, hi = min(lo, l), max(hi, l)
	}
	out.Layers = hi - lo + 1
	layers := make([][]int, hi-lo+1) // index q = layer - lo
	for p, l := range layer {
		layers[l-lo] = append(layers[l-lo], p)
	}
	for _, ms := range layers {
		if len(ms) == 0 {
			return out
		}
	}
	m := -lo
	fps := make([]footprint, len(layers))
	for q, ms := range layers {
		fps[q] = r.footprint(sp, ms)
	}
	out.Coaxial = true
	for _, a := range fps {
		ca, cr := (a.aLo+a.aHi)/2, (a.rLo+a.rHi)/2
		for _, b := range fps {
			if !b.holds(ca, cr) {
				out.Coaxial = false
			}
		}
	}
	if !sp.Thin || out.Coaxial {
		return out
	}
	out.Ran = true
	g := sp.Gap

	// L4a: stacks, before the slabs.
	for _, grp := range sp.Stacks {
		r.stack(sp, grp, layer, out.Stack)
	}

	// L4: slabs, outward from the base.
	_, prevTop := r.extent(sp, layers[m])
	for k := 1; k <= hi; k++ {
		ms := layers[k+m]
		bottom, _ := r.extent(sp, ms)
		if push := math.Max(0, prevTop+g-bottom); push > 0 {
			r.move(ms, sp.Axis.Scale(push))
		}
		_, prevTop = r.extent(sp, ms)
	}
	prevBottom, _ := r.extent(sp, layers[m])
	for k := -1; k >= lo; k-- {
		ms := layers[k+m]
		_, top := r.extent(sp, ms)
		if push := math.Max(0, top-prevBottom+g); push > 0 {
			r.move(ms, sp.Axis.Scale(-push))
		}
		prevBottom, _ = r.extent(sp, ms)
	}

	// L5: the grid with the least aspect.
	n = len(layers)
	bestAspect := math.Inf(1)
	for c := 1; c <= n; c++ {
		w, d := gridSizes(fps, c, g)
		if a := aspect(w, d); out.Columns == 0 || a < bestAspect-1e-9 {
			out.Columns, bestAspect = c, a
		}
	}
	c := out.Columns
	out.Width, out.Depth = gridSizes(fps, c, g)
	colW, rowD := cellSizes(fps, c)
	colStart := make([]float64, len(colW))
	rowStart := make([]float64, len(rowD))
	bj, br := m%c, m/c
	colStart[bj], rowStart[br] = fps[m].aLo, fps[m].rLo
	for j := bj + 1; j < len(colW); j++ {
		colStart[j] = colStart[j-1] + colW[j-1] + g
	}
	for j := bj - 1; j >= 0; j-- {
		colStart[j] = colStart[j+1] - g - colW[j]
	}
	for i := br + 1; i < len(rowD); i++ {
		rowStart[i] = rowStart[i-1] + rowD[i-1] + g
	}
	for i := br - 1; i >= 0; i-- {
		rowStart[i] = rowStart[i+1] - g - rowD[i]
	}
	for q, ms := range layers {
		v := sp.Across.Scale(colStart[q%c] - fps[q].aLo).Add(sp.Rows.Scale(rowStart[q/c] - fps[q].rLo))
		if v != (r3.Vec{}) {
			r.move(ms, v)
		}
	}
	return out
}

// stack is L4a for one group: the members of a layer other than 0, largest footprint first, each later one moved
// outward along the layer's sense by the least distance that takes it past every box it meets, at the places the
// earlier members and the other parts of the layer hold.
func (r *runner) stack(sp *Spread, grp, layer, place []int) {
	l := layer[grp[0]]
	if l == 0 {
		return
	}
	for _, p := range grp[1:] {
		if layer[p] != l {
			return
		}
	}
	u := sp.Axis
	if l < 0 {
		u = sp.Axis.Scale(-1)
	}
	area := func(p int) float64 {
		d := r.in.Boxes[p].Max.Sub(r.in.Boxes[p].Min)
		return d.Dot(sp.Across) * d.Dot(sp.Rows)
	}
	trailing := func(p int) float64 {
		b := shift(r.in.Boxes[p], r.offsets[p])
		return math.Min(b.Min.Dot(u), b.Max.Dot(u))
	}
	ms := slices.Clone(grp)
	slices.SortStableFunc(ms, func(a, b int) int {
		if c := cmp.Compare(area(b), area(a)); c != 0 {
			return c
		}
		if c := cmp.Compare(trailing(a), trailing(b)); c != 0 {
			return c
		}
		return cmp.Compare(a, b)
	})
	var standing []int // the earlier members and every other part of the layer
	for p, lp := range layer {
		if lp == l && !slices.Contains(ms, p) {
			standing = append(standing, p)
		}
	}
	for k, m := range ms {
		place[m] = k + 1
		if k > 0 {
			if t := beyondAll(shift(r.in.Boxes[m], r.offsets[m]), u, sp.Gap, r.in.Boxes, r.offsets, standing); t > 0 {
				r.move([]int{m}, u.Scale(t))
			}
		}
		standing = append(standing, m)
	}
}

// cellSizes returns the width of each column along Across and the depth of each row along Rows for c columns.
func cellSizes(fps []footprint, c int) ([]float64, []float64) {
	colW := make([]float64, c)
	rowD := make([]float64, (len(fps)+c-1)/c)
	for q, f := range fps {
		colW[q%c] = math.Max(colW[q%c], f.aHi-f.aLo)
		rowD[q/c] = math.Max(rowD[q/c], f.rHi-f.rLo)
	}
	return colW, rowD
}

// gridSizes returns the grid's extent along Across and Rows for c columns, cells g apart.
func gridSizes(fps []footprint, c int, g float64) (float64, float64) {
	colW, rowD := cellSizes(fps, c)
	w, d := float64(len(colW)-1)*g, float64(len(rowD)-1)*g
	for _, x := range colW {
		w += x
	}
	for _, x := range rowD {
		d += x
	}
	return w, d
}

// aspect is the longer extent over the shorter.
func aspect(w, d float64) float64 {
	if lo := math.Min(w, d); lo > 0 {
		return math.Max(w, d) / lo
	}
	return math.Inf(1)
}

// footprint reads the rectangle the members' boxes, at their offsets, cover across the axis.
func (r *runner) footprint(sp *Spread, ms []int) footprint {
	f := footprint{aLo: math.Inf(1), aHi: math.Inf(-1), rLo: math.Inf(1), rHi: math.Inf(-1)}
	for _, p := range ms {
		b := shift(r.in.Boxes[p], r.offsets[p])
		f.aLo, f.aHi = math.Min(f.aLo, b.Min.Dot(sp.Across)), math.Max(f.aHi, b.Max.Dot(sp.Across))
		f.rLo, f.rHi = math.Min(f.rLo, b.Min.Dot(sp.Rows)), math.Max(f.rHi, b.Max.Dot(sp.Rows))
	}
	return f
}

// extent reads the least and greatest coordinate along the axis of the members' boxes at their offsets.
func (r *runner) extent(sp *Spread, ms []int) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, p := range ms {
		b := shift(r.in.Boxes[p], r.offsets[p])
		lo, hi = math.Min(lo, b.Min.Dot(sp.Axis)), math.Max(hi, b.Max.Dot(sp.Axis))
	}
	return lo, hi
}

// move moves every member by v and counts it in their Slide.
func (r *runner) move(ms []int, v r3.Vec) {
	for _, p := range ms {
		r.setOffset(p, r.offsets[p].Add(v))
		r.slide[p] = r.slide[p].Add(v)
	}
}
