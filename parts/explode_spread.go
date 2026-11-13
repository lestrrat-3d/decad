package parts

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// WithoutExplodeSpread keeps a derived explode's layers stacked along their
// axis: the layer spread of agent-docs/parts-design.md §5.9 does not run, and
// Exploded.Spread still reports its gate. It needs WithDerivedExplode.
func WithoutExplodeSpread() ExplodeOption {
	return option{func(c *config) error { c.noSpread = true; return nil }}
}

// Spread reports the layer spread of a derived explode (§5.9). Exploded.Spread
// is nil without WithDerivedExplode, and when no derived part leaves along a
// world axis.
type Spread struct {
	// Ran is true when the layers were moved. Across, Rows, Columns, Width,
	// Depth, Aspect and View are zero when it is false.
	Ran bool
	// Axis is the positive world axis the derived layers leave along.
	Axis r3.Vec
	// Thinness is the assembled extent along Axis over the larger assembled
	// extent across it, Dimensionless. The spread needs it below 1/3.
	Thinness units.Value
	// Coaxial is true when the centre of every layer's footprint lies inside
	// every layer's footprint. The spread needs it false.
	Coaxial bool
	// Layers counts the layers, the base included.
	Layers int
	// Across is the positive world axis a row of cells runs along, and Rows
	// the positive world axis the rows advance along.
	Across, Rows r3.Vec
	// Columns is the number of cells in a row.
	Columns int
	// Width and Depth are the grid's extent along Across and along Rows, gaps
	// included, as Length values; Aspect is the longer over the shorter,
	// Dimensionless.
	Width, Depth, Aspect units.Value
	// View is a unit view direction, pointing from the camera into the scene,
	// from which no part hides a part of another layer: -Axis + Across + Rows,
	// normalised. Every view v with (v·Axis)(v·Across) <= 0 and
	// (v·Axis)(v·Rows) <= 0 has the same property.
	View r3.Vec
}

// thinBelow is the thinness under which an assembly counts as flat (§5.9 L3).
const thinBelow = 1.0 / 3.0

// spreadPlan is the request the spread makes of explodeplan.Run and what the
// public Spread reports besides the runner's outcome.
type spreadPlan struct {
	req      explodeplan.Spread
	thinness float64
	layers   int
}

// planSpread reads L1, L2, L3 (a) and (b) and the choice of Across and Rows
// from the assembled boxes and the derived report. It returns nil when no
// derived part leaves along a world axis.
func planSpread(c config, boxes []explodeplan.Box, rep *derivedReport) *spreadPlan {
	world := [3]r3.Vec{{X: 1}, {Y: 1}, {Z: 1}}
	asm := boxes[0]
	for _, b := range boxes[1:] {
		asm.Min = r3.Vec{X: math.Min(asm.Min.X, b.Min.X), Y: math.Min(asm.Min.Y, b.Min.Y), Z: math.Min(asm.Min.Z, b.Min.Z)}
		asm.Max = r3.Vec{X: math.Max(asm.Max.X, b.Max.X), Y: math.Max(asm.Max.Y, b.Max.Y), Z: math.Max(asm.Max.Z, b.Max.Z)}
	}
	var extent [3]float64
	for i, w := range world {
		extent[i] = asm.Max.Dot(w) - asm.Min.Dot(w)
	}
	cosTol := math.Cos(c.axisTol)

	// L1: the world axis most derived parts leave along, either sense.
	var count [3]int
	for k, o := range rep.order {
		if o == 0 {
			continue
		}
		for i, w := range world {
			if math.Abs(rep.dir[k].Dot(w)) >= cosTol {
				count[i]++
			}
		}
	}
	axis := -1
	for _, i := range []int{2, 1, 0} { // Z, Y, X
		if count[i] == 0 {
			continue
		}
		if axis < 0 || count[i] > count[axis] || (count[i] == count[axis] && extent[i] < extent[axis]) {
			axis = i
		}
	}
	if axis < 0 {
		return nil
	}
	var across, rows int // L5: the other axes, the shorter extent first, X, Y, Z on a tie
	var others []int
	for i := range world {
		if i != axis {
			others = append(others, i)
		}
	}
	across, rows = others[0], others[1]
	if extent[others[1]] < extent[others[0]] {
		across, rows = others[1], others[0]
	}

	sp := &spreadPlan{req: explodeplan.Spread{Axis: world[axis], Across: world[across], Rows: world[rows],
		Layer: make([]int, len(boxes)), Gap: math.Max(c.gap, 1)}}
	if w := math.Max(extent[across], extent[rows]); w > 0 {
		sp.thinness = extent[axis] / w
	} else {
		sp.thinness = math.Inf(1)
	}
	sp.req.Thin = !c.noSpread && sp.thinness < thinBelow

	// L2: the k-th round counting from the one removed last, each way along the axis.
	ranks := [2]map[int]int{{}, {}} // round -> k, for +Axis and -Axis
	for k, o := range rep.order {
		if o == 0 {
			continue
		}
		switch d := rep.dir[k].Dot(world[axis]); {
		case d >= cosTol:
			ranks[0][rep.round[k]] = 0
		case d <= -cosTol:
			ranks[1][rep.round[k]] = 0
		}
	}
	for _, m := range ranks {
		rounds := make([]int, 0, len(m))
		for r := range m {
			rounds = append(rounds, r)
		}
		slices.SortFunc(rounds, func(a, b int) int { return b - a }) // the last removed first
		for k, r := range rounds {
			m[r] = k + 1
		}
	}
	sp.layers = 1 + len(ranks[0]) + len(ranks[1])
	for k, o := range rep.order {
		if o == 0 {
			continue
		}
		switch d := rep.dir[k].Dot(world[axis]); {
		case d >= cosTol:
			sp.req.Layer[k] = ranks[0][rep.round[k]]
		case d <= -cosTol:
			sp.req.Layer[k] = -ranks[1][rep.round[k]]
		}
	}
	return sp
}

// report builds the public Spread from the plan and the runner's outcome.
func (sp *spreadPlan) report(out explodeplan.SpreadOutcome) *Spread {
	r := &Spread{Ran: out.Ran, Axis: sp.req.Axis, Thinness: units.Scalar(sp.thinness), Coaxial: out.Coaxial, Layers: sp.layers}
	if !out.Ran {
		return r
	}
	r.Across, r.Rows, r.Columns = sp.req.Across, sp.req.Rows, out.Columns
	r.Width, r.Depth = units.Millimeters(out.Width), units.Millimeters(out.Depth)
	r.Aspect = units.Scalar(math.Max(out.Width, out.Depth) / math.Min(out.Width, out.Depth))
	r.View, _ = sp.req.Across.Add(sp.req.Rows).Sub(sp.req.Axis).Normalize()
	return r
}
