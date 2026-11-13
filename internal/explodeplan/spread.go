package explodeplan

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// Spread asks Run for the layer spread of agent-docs/parts-design.md §5.9.
type Spread struct {
	Axis, Across, Rows r3.Vec  // unit positive world axes
	Layer              []int   // per part: signed layer, 0 for the base
	Gap                float64 // g of L4 and L5, mm, at least 1
	Thin               bool    // L3 (a) and (b) hold; Run adds (c)
}

// SpreadOutcome is what the spread step did.
type SpreadOutcome struct {
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

// spread is the layer spread: the coaxial test (L3 c), the slabs (L4) and the cell grid (L5). It moves whole
// layers, so every move is appended to its members' stops and added to their Slide.
func (r *runner) spread() SpreadOutcome {
	sp := r.in.Spread
	lo, hi := 0, 0
	for _, l := range sp.Layer {
		lo, hi = min(lo, l), max(hi, l)
	}
	layers := make([][]int, hi-lo+1) // index q = layer - lo
	for p, l := range sp.Layer {
		layers[l-lo] = append(layers[l-lo], p)
	}
	for _, ms := range layers {
		if len(ms) == 0 {
			return SpreadOutcome{}
		}
	}
	m := -lo
	fps := make([]footprint, len(layers))
	for q, ms := range layers {
		fps[q] = r.footprint(sp, ms)
	}
	var out SpreadOutcome
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
	n := len(layers)
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
