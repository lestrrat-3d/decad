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
}

// planSpread reads L1, the derived half of L2, L3 (a) and (b), the choice of
// Across and Rows and L4a's candidate groups from the assembled boxes, the
// derived report, the stages and the joints (axes[n] is joint n's axis, nil when
// it has none). It returns nil when no derived part leaves along a world axis.
func planSpread(c config, boxes []explodeplan.Box, rep *derivedReport, stages []explodeplan.Stage,
	joints []explodeplan.Joint, axes []*Axis) *spreadPlan {
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
		Layer: make([]int, len(boxes)), CosTol: cosTol, Gap: math.Max(c.gap, 1)}}
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
	sp.req.Stacks = stackGroups(sp.req, rep, stages, joints, axes)
	return sp
}

// stackGroups returns the derived rigid groups L4a may stack: a Together stage of a round, none of whose members
// has a WithRemoval Group, whose members are connected through joints between members with an axis along the
// spread's axis. Members are ascending.
func stackGroups(req explodeplan.Spread, rep *derivedReport, stages []explodeplan.Stage, joints []explodeplan.Joint,
	axes []*Axis) [][]int {
	var out [][]int
	for _, st := range stages {
		if st.Round == 0 || !st.Together || len(st.Members) < 2 {
			continue
		}
		members := slices.Clone(st.Members)
		slices.Sort(members)
		if slices.ContainsFunc(members, func(m int) bool { return rep.grouped[m] }) {
			continue
		}
		seen := map[int]struct{}{members[0]: {}}
		queue := []int{members[0]}
		for len(queue) > 0 {
			p := queue[0]
			queue = queue[1:]
			for n, j := range joints {
				if axes[n] == nil || math.Abs(axes[n].Dir.Dot(req.Axis)) < req.CosTol {
					continue
				}
				q := j.A
				switch p {
				case j.A:
					q = j.B
				case j.B:
				default:
					continue
				}
				_, in := slices.BinarySearch(members, q)
				if _, ok := seen[q]; in && !ok {
					seen[q] = struct{}{}
					queue = append(queue, q)
				}
			}
		}
		if len(seen) == len(members) {
			out = append(out, members)
		}
	}
	return out
}

// report builds the public Spread from the plan and the runner's outcome.
func (sp *spreadPlan) report(out explodeplan.SpreadOutcome) *Spread {
	r := &Spread{Ran: out.Ran, Axis: sp.req.Axis, Thinness: units.Scalar(sp.thinness), Coaxial: out.Coaxial, Layers: out.Layers}
	if !out.Ran {
		return r
	}
	r.Across, r.Rows, r.Columns = sp.req.Across, sp.req.Rows, out.Columns
	r.Width, r.Depth = units.Millimeters(out.Width), units.Millimeters(out.Depth)
	r.Aspect = units.Scalar(math.Max(out.Width, out.Depth) / math.Min(out.Width, out.Depth))
	r.View, _ = sp.req.Across.Add(sp.req.Rows).Sub(sp.req.Axis).Normalize()
	return r
}
