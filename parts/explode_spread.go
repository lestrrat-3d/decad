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
	// Ran is true when the layers were laid out. Shear, Width, Depth, Height
	// and View are zero when it is false.
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
	// Shear is the positive world axis across Axis along which a layer on the
	// +Axis side steps where a part joined to it would be covered; a layer on
	// the -Axis side steps along -Shear.
	Shear r3.Vec
	// Width, Depth and Height are the extents of the exploded layout, every
	// part's box at its final Offset, along Shear, along the third world axis
	// and along Axis, as Length values after WithExplodeScale.
	Width, Depth, Height units.Value
	// View is a unit view direction, pointing from the camera into the scene,
	// from which no part wholly hides a part of a nearer layer it is joined to,
	// unless the two share an axis (§5.9 L7): Shear - Axis, normalised. Every
	// view v at right angles to the third world axis with
	// (v·Axis)(v·Shear) <= 0 has the same property for the layers on the side
	// of the base the camera looks from.
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
// Shear and Rows, L4's mates and L4a's candidate groups from the assembled boxes, the
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
	var across, rows int // L4: the other axes, the shorter extent first, X, Y, Z on a tie
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

	sp := &spreadPlan{req: explodeplan.Spread{Axis: world[axis], Shear: world[across], Rows: world[rows],
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
	for n, j := range joints {
		sp.req.Mates = append(sp.req.Mates, explodeplan.Mate{A: j.A, B: j.B,
			Coaxial: axes[n] != nil && math.Abs(axes[n].Dir.Dot(world[axis])) >= cosTol})
	}
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

// report builds the public Spread from the plan and the runner's outcome; moved holds the exploded boxes.
func (sp *spreadPlan) report(out explodeplan.SpreadOutcome, moved []explodeplan.Box) *Spread {
	r := &Spread{Ran: out.Ran, Axis: sp.req.Axis, Thinness: units.Scalar(sp.thinness), Coaxial: out.Coaxial, Layers: out.Layers}
	if !out.Ran {
		return r
	}
	extent := func(d r3.Vec) units.Value {
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, b := range moved {
			lo, hi = math.Min(lo, b.Min.Dot(d)), math.Max(hi, b.Max.Dot(d))
		}
		return units.Millimeters(hi - lo)
	}
	r.Shear = sp.req.Shear
	r.Width, r.Depth, r.Height = extent(sp.req.Shear), extent(sp.req.Rows), extent(sp.req.Axis)
	r.View, _ = sp.req.Shear.Sub(sp.req.Axis).Normalize()
	return r
}
