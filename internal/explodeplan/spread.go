package explodeplan

import (
	"cmp"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// Spread asks Run for the layer spread of agent-docs/parts-design.md §5.9.
type Spread struct {
	Axis, Shear, Rows r3.Vec  // unit positive world axes: the explode axis, the step axis, the third axis
	Layer             []int   // per part: the derived half of L2, signed, 0 for every other part
	CosTol            float64 // cos of the axis tolerance (L1, L2)
	Gap               float64 // g of L4a, L4 and L5, mm, at least 1
	Thin              bool    // L3 (a) and (b) hold; Run adds (c)
	Stacks            [][]int // derived rigid groups L4a may stack, members ascending
	Mates             []Mate  // L4's mates, one per joint between parts with bodies, in Join order
}

// Mate is one joint of L4: parts A and B, and whether the joint is a pin whose axis lies within the axis tolerance
// of the explode axis.
type Mate struct {
	A, B    int
	Coaxial bool
}

// SpreadOutcome is what the spread step did.
type SpreadOutcome struct {
	Layer   []int // per part: its layer after the explicit half of L2; set whether or not the spread ran
	Layers  int   // the number of layers, the base included
	Stack   []int // per part: its L4a place, 0 when not stacked or when the spread did not run
	Coaxial bool
	Ran     bool
}

// coaxialTol is how far outside a footprint a centre may lie and still count as inside (L3 c), in mm.
const coaxialTol = 1e-9

// footprint is the rectangle a layer's boxes cover across the axis, in coordinates along Shear and Rows.
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

// spread is the layer spread: the explicit half of L2, the coaxial test (L3 c), the stacks (L4a), the steps (L4)
// and the settle (L5). It moves whole layers, and the members of a stacked group, so every move is added to
// the part's Slide.
func (r *runner) spread() SpreadOutcome {
	sp := r.in.Spread
	layer := r.explicitLayers()
	out := SpreadOutcome{Layer: layer, Stack: make([]int, len(layer))}
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

	// L4a: stacks, before the steps.
	for _, grp := range sp.Stacks {
		r.stack(sp, grp, layer, out.Stack)
	}

	// L4: steps, then L5: settle.
	byLayer := make(map[int][]int, len(layers))
	for q, ms := range layers {
		byLayer[q-m] = ms
	}
	r.steps(sp, byLayer, layer)
	r.settle(sp, byLayer, layer)
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
		return d.Dot(sp.Shear) * d.Dot(sp.Rows)
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

// footprint reads the rectangle the members' boxes, at their offsets, cover across the axis.
func (r *runner) footprint(sp *Spread, ms []int) footprint {
	f := footprint{aLo: math.Inf(1), aHi: math.Inf(-1), rLo: math.Inf(1), rHi: math.Inf(-1)}
	for _, p := range ms {
		b := shift(r.in.Boxes[p], r.offsets[p])
		f.aLo, f.aHi = math.Min(f.aLo, b.Min.Dot(sp.Shear)), math.Max(f.aHi, b.Max.Dot(sp.Shear))
		f.rLo, f.rHi = math.Min(f.rLo, b.Min.Dot(sp.Rows)), math.Max(f.rHi, b.Max.Dot(sp.Rows))
	}
	return f
}

// move moves every member by v and counts it in their Slide.
func (r *runner) move(ms []int, v r3.Vec) {
	for _, p := range ms {
		r.setOffset(p, r.offsets[p].Add(v))
		r.slide[p] = r.slide[p].Add(v)
	}
}

// sides are the outward senses of the layers on the +Axis side and the -Axis side of the base.
var sides = [2]float64{1, -1}

// stepEpsilon is the tolerance of the coaxial skip, in mm.
const stepEpsilon = 1e-6

// steps is L4: per side, outward, the least step along the side's sense of Shear that leaves each covered mate's
// inner part a band of m toward the opposite sense, every layer starting from the step of the one before.
func (r *runner) steps(sp *Spread, layers map[int][]int, layer []int) {
	hi, lo := 0, 0
	for l := range layers {
		hi, lo = max(hi, l), min(lo, l)
	}
	for _, s := range sides {
		top := hi
		if s < 0 {
			top = -lo
		}
		e := sp.Shear.Scale(s)
		step := 0.0 // S_(k-1); the base has none
		for k := 1; k <= top; k++ {
			q := int(s) * k
			for pass := 0; pass <= len(sp.Mates)+1; pass++ {
				changed := false
				for _, mt := range sp.Mates {
					p, qq := mt.A, mt.B
					if layer[qq] != q {
						p, qq = mt.B, mt.A
					}
					if layer[qq] != q {
						continue
					}
					lp := int(s) * layer[p]
					if layer[p] != 0 && (lp < 1 || lp >= k) {
						continue
					}
					oq := r.offsets[qq].Add(e.Scale(step))
					if mt.Coaxial {
						d := r.offsets[p].Sub(oq)
						if d.Sub(sp.Axis.Scale(d.Dot(sp.Axis))).Len() <= stepEpsilon {
							continue
						}
					}
					if lack, ok := covered(shift(r.in.Boxes[p], r.offsets[p]), shift(r.in.Boxes[qq], oq), e, sp.Rows,
						sp.Gap); ok {
						step += lack
						changed = true
					}
				}
				if !changed {
					break
				}
			}
			if step != 0 {
				r.move(layers[q], e.Scale(step))
			}
		}
	}
}

// covered reports whether p, the inner part's box, is covered by q, the outer part's, with e the outer layer's step
// sense: their footprints overlap and p reaches beyond q by less than m toward -e and toward both senses of rows.
// It returns what p lacks toward -e.
func covered(p, q Box, e, rows r3.Vec, g float64) (float64, bool) {
	read := func(b Box, d r3.Vec) (float64, float64) {
		a, c := b.Min.Dot(d), b.Max.Dot(d)
		return math.Min(a, c), math.Max(a, c)
	}
	pE0, pE1 := read(p, e)
	qE0, qE1 := read(q, e)
	pR0, pR1 := read(p, rows)
	qR0, qR1 := read(q, rows)
	if math.Min(pE1, qE1)-math.Max(pE0, qE0) <= 1e-9 || math.Min(pR1, qR1)-math.Max(pR0, qR0) <= 1e-9 {
		return 0, false
	}
	m := math.Min(g, math.Min(pE1-pE0, pR1-pR0)/2)
	if pE0 <= qE0-m+1e-9 || pR0 <= qR0-m+1e-9 || pR1 >= qR1+m-1e-9 {
		return 0, false
	}
	return pE0 - (qE0 - m), true
}

// settle is L5: per side, outward, each layer moved along its outward sense as one by the largest beyondAll of its
// members against every part not in it or beyond it on that side.
func (r *runner) settle(sp *Spread, layers map[int][]int, layer []int) {
	hi, lo := 0, 0
	for l := range layers {
		hi, lo = max(hi, l), min(lo, l)
	}
	for _, s := range sides {
		top := hi
		if s < 0 {
			top = -lo
		}
		u := sp.Axis.Scale(s)
		for k := 1; k <= top; k++ {
			var standing []int
			for p, l := range layer {
				if ls := int(s) * l; ls <= 0 || ls < k {
					standing = append(standing, p)
				}
			}
			ms := layers[int(s)*k]
			t := 0.0
			for _, p := range ms {
				t = math.Max(t, beyondAll(shift(r.in.Boxes[p], r.offsets[p]), u, sp.Gap, r.in.Boxes, r.offsets, standing))
			}
			if t > 0 {
				r.move(ms, u.Scale(t))
			}
		}
	}
}
