package explodeplan

import (
	"cmp"
	"context"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// Clearing says what a stage's members must clear.
type Clearing int

const (
	// ClearJoined clears the parent and the joined parts already moved.
	ClearJoined Clearing = iota
	// ClearAll clears every part that stays put, by the gap.
	ClearAll
)

// PathState says what a move's straight path met.
type PathState int

const (
	PathNone     PathState = iota // the part did not move
	PathClear                     // the swept box meets no part it is not joined to
	PathBlocked                   // the start is clear and the sweep meets a part
	PathEnclosed                  // the start box already meets a part's box
	PathStuck                     // no distance clears
)

// Blocker answers the path question of agent-docs/parts-design.md §5.3 S7: a
// part swept from its assembled box moved by from to that box moved by to, against
// every other part's box at its offset except those in ignore. It reports the
// state and the index of the first part met, -1 when none.
type Blocker interface {
	Path(ctx context.Context, part int, from, to r3.Vec, offsets []r3.Vec, ignore []int) (PathState, int, error)
}

// Node is one part's place in the joint walk.
type Node struct {
	Depth  int // -1 when no joint reaches the part
	Joint  int // the parent joint's index, -1 for the ground and unreached parts
	Parent int // the parent part, -1 for the ground and unreached parts
}

// Walk walks the joints breadth-first from ground, joints in input order, so each
// part has one parent joint and a part reached twice keeps its first parent. It
// returns the parts other than the ground in the order they were reached.
func Walk(n int, joints []Joint, ground int) ([]int, []Node) {
	nodes := make([]Node, n)
	for i := range nodes {
		nodes[i] = Node{Depth: -1, Joint: -1, Parent: -1}
	}
	nodes[ground].Depth = 0
	queue := []int{ground}
	var order []int
	for len(queue) > 0 {
		p := queue[0]
		queue = queue[1:]
		for ji, j := range joints {
			var child int
			switch p {
			case j.A:
				child = j.B
			case j.B:
				child = j.A
			default:
				continue
			}
			if child == p || nodes[child].Depth >= 0 {
				continue
			}
			nodes[child] = Node{Depth: nodes[p].Depth + 1, Joint: ji, Parent: p}
			order = append(order, child)
			queue = append(queue, child)
		}
	}
	return order, nodes
}

// Stage is one step of a staged explosion.
type Stage struct {
	Members  []int // part indices
	Gap      float64
	Clear    Clearing
	Dir      r3.Vec // not zero: every member moves along it, in its one sense
	Together bool
	// Round, when not zero, marks a ClearAll stage as one derived removal round
	// (agent-docs/parts-design.md §5.8 D7). Its members then clear only the parts
	// still present when they leave: not the parts a later stage moves, and not
	// the other members of stages with the same Round, except the members of
	// their own stage already moved whose boxes overlap theirs seen along the
	// direction.
	Round int
}

// Input is everything Run reads.
type Input struct {
	Boxes      []Box
	Joints     []Joint
	Ground     int
	Order      []int // the walk order of Walk
	Nodes      []Node
	Stages     []Stage
	DefaultGap float64
	PartGap    []float64 // per part; NaN when unset
	Scale      float64
	Blocker    Blocker
	// Spread, when not nil, asks for the layer spread of agent-docs/parts-design.md
	// §5.9 after the last stage.
	Spread *Spread
}

// StageDirection is the Source a result carries when its stage gave the
// direction.
const StageDirection = -2

// Result is one part's place in the explosion.
type Result struct {
	Offset    r3.Vec // final, scaled
	Dir       r3.Vec // unit direction of its last move, zero when it did not move
	Stage     int    // index of the last stage that moved it, -1 for none
	Source    int    // the parent joint's Source, StageDirection, or -1
	Travel    float64
	Gap       float64
	Moved     float64 // length of its last move, scaled
	Path      PathState
	Blocker   int     // the part that blocked or enclosed the path, else the nearest part; -1 for none
	Clearance float64 // least box distance to a part it is not joined to; 0 when boxes meet
	Short     bool
	// Slide is what the layer spread added to Offset, scaled; zero when it did not run.
	Slide r3.Vec
	// Stops are the offsets the part passed through, scaled: zero first, the last equal to Offset
	// (agent-docs/parts-design.md §5.10 T2). Consecutive stops differ.
	Stops []r3.Vec
}

// Outcome is what Run returns.
type Outcome struct {
	Parts  []Result
	Spread SpreadOutcome // zero when Input.Spread is nil
}

// stuckAt is the distance past which a clearing is given up (S5).
const stuckAt = 1e6

// Run runs the stages in order, then the layer spread when asked, and returns one Result per part.
func Run(ctx context.Context, in Input) (Outcome, error) {
	n := len(in.Boxes)
	r := &runner{in: in, offsets: make([]r3.Vec, n), done: make([]bool, n), res: make([]Result, n),
		stops: make([][]r3.Vec, n), slide: make([]r3.Vec, n)}
	for i := range r.res {
		r.res[i] = Result{Stage: -1, Source: -1, Blocker: -1}
		r.stops[i] = []r3.Vec{{}}
	}
	r.done[in.Ground] = true
	r.pos = make([]int, n)
	for i := range r.pos {
		r.pos[i] = -1
	}
	for k, p := range in.Order {
		r.pos[p] = k
	}
	r.partners = make([][]int, n)
	for _, j := range in.Joints {
		if j.A != j.B {
			r.partners[j.A] = append(r.partners[j.A], j.B)
			r.partners[j.B] = append(r.partners[j.B], j.A)
		}
	}
	r.stageOf = make([][]int, n)
	for si, st := range in.Stages {
		for _, m := range st.Members {
			r.stageOf[m] = append(r.stageOf[m], si)
		}
	}
	for si := range in.Stages {
		if err := r.runStage(ctx, si); err != nil {
			return Outcome{}, err
		}
	}
	var out SpreadOutcome
	if in.Spread != nil {
		out = r.spread()
	}
	r.finish()
	return Outcome{Parts: r.res, Spread: out}, nil
}

type runner struct {
	in       Input
	offsets  []r3.Vec
	done     []bool
	pos      []int // walk position, -1 for the ground and unreached parts
	partners [][]int
	stageOf  [][]int
	res      []Result
	stops    [][]r3.Vec // per part: the offsets it passed through, before the scale
	slide    []r3.Vec   // per part: what the spread added, before the scale
}

// setOffset moves part m to offset v and keeps the stop.
func (r *runner) setOffset(m int, v r3.Vec) {
	r.offsets[m] = v
	r.addStop(m, v)
}

// addStop keeps v as a stop of m unless it equals the last one (§5.10 T2).
func (r *runner) addStop(m int, v r3.Vec) {
	if s := r.stops[m]; v != s[len(s)-1] {
		r.stops[m] = append(s, v)
	}
}

func (r *runner) gapOf(si, m int) float64 {
	if g := r.in.PartGap[m]; !math.IsNaN(g) {
		return g
	}
	if g := r.in.Stages[si].Gap; !math.IsNaN(g) {
		return g
	}
	return r.in.DefaultGap
}

// members returns the stage's members in walk order; the ground and unreached
// parts, which have no walk position, come first in part order.
func (r *runner) members(si int) []int {
	ms := slices.Clone(r.in.Stages[si].Members)
	slices.SortStableFunc(ms, func(a, b int) int {
		if c := cmp.Compare(r.pos[a], r.pos[b]); c != 0 {
			return c
		}
		return cmp.Compare(a, b)
	})
	return ms
}

func (r *runner) runStage(ctx context.Context, si int) error {
	st := r.in.Stages[si]
	ms := r.members(si)
	if st.Together {
		return r.moveTogether(ctx, si, ms)
	}
	inStage := map[int]int{}
	for k, m := range ms {
		inStage[m] = k
	}
	for k, m := range ms {
		if err := r.moveOne(ctx, si, m, k, ms, inStage); err != nil {
			return err
		}
	}
	return nil
}

// senses returns the unit directions a member may move along, and whether they
// came from the stage.
func (r *runner) senses(st Stage, m int) ([]r3.Vec, bool) {
	if u, ok := st.Dir.Normalize(); ok {
		return []r3.Vec{u}, true
	}
	node := r.in.Nodes[m]
	if node.Joint < 0 {
		return nil, false
	}
	dir, both := direction(r.in.Boxes, r.in.Joints[node.Joint], node.Parent, m)
	if dir == (r3.Vec{}) {
		return nil, false
	}
	if both {
		return []r3.Vec{dir, dir.Scale(-1)}, false
	}
	return []r3.Vec{dir}, false
}

type choice struct {
	move   r3.Vec // unit direction
	t      float64
	travel float64
	path   PathState
	block  int
}

func (c choice) free() bool { return c.path == PathClear }

func (r *runner) moveOne(ctx context.Context, si, m, k int, ms []int, inStage map[int]int) error {
	st := r.in.Stages[si]
	node := r.in.Nodes[m]
	senses, fromStage := r.senses(st, m)
	r.done[m] = r.done[m] || node.Joint >= 0 || fromStage
	if len(senses) == 0 {
		r.record(m, si, r3.Vec{}, 0, 0, 0, PathNone, -1, fromStage, node)
		return nil
	}
	start := r.offsets[m]
	if pk, ok := inStage[node.Parent]; ok && node.Parent >= 0 && pk < k {
		start = r.offsets[node.Parent] // carried with its parent
	}
	r.addStop(m, start)
	gap := r.gapOf(si, m)
	partners := slices.Clone(r.partners[m])
	var best *choice
	for _, s := range senses {
		var c choice
		var standing []int
		switch st.Clear {
		case ClearAll:
			if st.Round > 0 {
				standing = r.present(si, m, ms[:k], s)
				c.t = beyondAll(shift(r.in.Boxes[m], start), s, gap, r.in.Boxes, r.offsets, standing)
			} else {
				standing = r.standing(si, m, ms[k+1:])
				c.t = leastClear(shift(r.in.Boxes[m], start), s, gap, r.in.Boxes, r.offsets, standing)
			}
			c.travel = math.Max(0, c.t-gap)
			c.path, c.block = PathNone, -1
			if c.t > stuckAt {
				c.path = PathStuck
			}
		default:
			c.t = 0
			for _, cl := range r.cleared(m, node) {
				c.t = math.Max(c.t, travel(shift(r.in.Boxes[m], start), shift(r.in.Boxes[cl], r.offsets[cl]), s))
			}
			c.travel = c.t
			if math.IsInf(c.t, 1) {
				c.path = PathStuck
			}
		}
		c.move = s
		if c.path != PathStuck {
			ignore := partners
			switch {
			case st.Clear == ClearAll && st.Round > 0:
				ignore = append(slices.Clone(partners), r.absent(m, standing)...)
			case st.Clear == ClearAll:
				ignore = append(slices.Clone(partners), ms[k+1:]...)
			}
			var err error
			c.path, c.block, err = r.in.Blocker.Path(ctx, m, start, start.Add(s.Scale(c.t)), r.offsets, ignore)
			if err != nil {
				return err
			}
		}
		if best == nil || better(c.free(), c.t, c.move, best.free(), best.t, best.move) {
			cc := c
			best = &cc
		}
	}
	if best.path == PathStuck {
		r.setOffset(m, start)
		r.record(m, si, r3.Vec{}, 0, 0, 0, PathStuck, -1, fromStage, node)
		return nil
	}
	dist := best.t
	if st.Clear == ClearJoined {
		dist += gap
	}
	r.setOffset(m, start.Add(best.move.Scale(dist)))
	r.record(m, si, best.move, best.travel, dist, gap, best.path, best.block, fromStage, node)
	return nil
}

func (r *runner) record(m, si int, dir r3.Vec, travelOut, moved, gap float64, path PathState, block int, fromStage bool,
	node Node) {
	src := -1
	switch {
	case fromStage:
		src = StageDirection
	case node.Joint >= 0:
		src = r.in.Joints[node.Joint].Source
	}
	r.res[m].Stage, r.res[m].Dir, r.res[m].Travel, r.res[m].Gap, r.res[m].Moved = si, dir, travelOut, gap, moved
	r.res[m].Path, r.res[m].Blocker, r.res[m].Source = path, block, src
}

// cleared is E3's set: the parent and the joined parts already placed.
func (r *runner) cleared(m int, node Node) []int {
	var out []int
	if node.Parent >= 0 {
		out = append(out, node.Parent)
	}
	for _, o := range r.partners[m] {
		if o != m && r.done[o] && o != node.Parent {
			out = append(out, o)
		}
	}
	return out
}

// standing is S5's set for member m: every part except m, the stage members not
// yet moved, and the joined parts a later stage moves.
func (r *runner) standing(si, m int, notYet []int) []int {
	var out []int
	for p := range r.in.Boxes {
		if p == m || slices.Contains(notYet, p) {
			continue
		}
		if slices.Contains(r.partners[m], p) && r.movesLater(p, si) {
			continue
		}
		out = append(out, p)
	}
	return out
}

// present is the standing set of member m of a derived round (Stage.Round): the
// parts still there when it leaves, so the round removed last sits nearest the
// assembly. A part a later stage moves left before m and ends beyond it. The
// other members of m's round leave independently (§5.8 D3), except the members
// of m's own stage already moved whose boxes overlap m's seen along dir: m would
// pass through them, so it ends beyond them.
func (r *runner) present(si, m int, moved []int, dir r3.Vec) []int {
	round := r.in.Stages[si].Round
	var out []int
	for p := range r.in.Boxes {
		if p == m || r.movesLater(p, si) {
			continue
		}
		if r.inRound(p, round) && (!slices.Contains(moved, p) || !r.shadowed(m, p, dir)) {
			continue
		}
		out = append(out, p)
	}
	return out
}

// absent is every part other than m that is not in standing.
func (r *runner) absent(m int, standing []int) []int {
	var out []int
	for p := range r.in.Boxes {
		if p != m && !slices.Contains(standing, p) {
			out = append(out, p)
		}
	}
	return out
}

// inRound reports whether a stage with the given derived round moves p.
func (r *runner) inRound(p, round int) bool {
	for _, s := range r.stageOf[p] {
		if r.in.Stages[s].Round == round {
			return true
		}
	}
	return false
}

// shadowed reports whether the boxes of a and b, at their offsets, overlap with
// positive area seen along dir.
func (r *runner) shadowed(a, b int, dir r3.Vec) bool {
	eu, ev := planeBasis(dir)
	ra := project(shift(r.in.Boxes[a], r.offsets[a]), eu, ev)
	rb := project(shift(r.in.Boxes[b], r.offsets[b]), eu, ev)
	return math.Min(ra[1], rb[1])-math.Max(ra[0], rb[0]) > eps && math.Min(ra[3], rb[3])-math.Max(ra[2], rb[2]) > eps
}

func (r *runner) movesLater(p, si int) bool {
	for _, s := range r.stageOf[p] {
		if s > si {
			return true
		}
	}
	return false
}

// leastClear returns the least t >= 0 at which the box, moved by t along dir, is
// outside every standing box grown by gap on each side.
func leastClear(moving Box, dir r3.Vec, gap float64, boxes []Box, offsets []r3.Vec, standing []int) float64 {
	spans := clearSpans(moving, dir, gap, boxes, offsets, standing)
	slices.SortFunc(spans, func(a, b span) int { return cmp.Compare(a.lo, b.lo) })
	t := 0.0
	for _, s := range spans {
		if s.lo < t && t < s.hi {
			t = s.hi
		}
	}
	return t
}

// beyondAll returns the least t >= 0 at which the box, moved by t along dir, is
// past every standing box grown by gap that it meets on the way: a derived
// round's member never settles in a space between standing parts, so it ends
// beyond each part still present in its way (agent-docs/parts-design.md §5.8
// D7).
func beyondAll(moving Box, dir r3.Vec, gap float64, boxes []Box, offsets []r3.Vec, standing []int) float64 {
	t := 0.0
	for _, s := range clearSpans(moving, dir, gap, boxes, offsets, standing) {
		t = math.Max(t, s.hi)
	}
	return t
}

type span struct{ lo, hi float64 }

// clearSpans returns, per standing box grown by gap, the open interval of t over
// which the box moved by t along dir overlaps it.
func clearSpans(moving Box, dir r3.Vec, gap float64, boxes []Box, offsets []r3.Vec, standing []int) []span {
	var spans []span
	for _, p := range standing {
		a := shift(boxes[p], offsets[p])
		lo, hi := math.Inf(-1), math.Inf(1)
		ok := true
		for _, ax := range []struct{ bl, bh, al, ah, d float64 }{
			{moving.Min.X, moving.Max.X, a.Min.X, a.Max.X, dir.X},
			{moving.Min.Y, moving.Max.Y, a.Min.Y, a.Max.Y, dir.Y},
			{moving.Min.Z, moving.Max.Z, a.Min.Z, a.Max.Z, dir.Z},
		} {
			al, ah := ax.al-gap, ax.ah+gap
			if math.Abs(ax.d) <= eps {
				if !(ax.bl < ah && ax.bh > al) {
					ok = false
					break
				}
				continue
			}
			t0, t1 := (al-ax.bh)/ax.d, (ah-ax.bl)/ax.d
			if t0 > t1 {
				t0, t1 = t1, t0
			}
			lo, hi = math.Max(lo, t0), math.Min(hi, t1)
		}
		if ok && lo < hi {
			spans = append(spans, span{lo, hi})
		}
	}
	return spans
}

// moveTogether is S6: the members move by one offset.
func (r *runner) moveTogether(ctx context.Context, si int, ms []int) error {
	st := r.in.Stages[si]
	in := map[int]bool{}
	for _, m := range ms {
		in[m] = true
	}
	// The direction: the stage's, else the first member whose parent is not a member.
	lead := -1
	for _, m := range ms {
		if !in[r.in.Nodes[m].Parent] {
			lead = m
			break
		}
	}
	if lead < 0 {
		lead = ms[0]
	}
	senses, fromStage := r.senses(st, lead)
	gap := r.gapOf(si, lead)
	if len(senses) == 0 {
		for _, m := range ms {
			r.record(m, si, r3.Vec{}, 0, 0, 0, PathNone, -1, fromStage, r.in.Nodes[m])
			r.done[m] = r.done[m] || r.in.Nodes[m].Joint >= 0 || fromStage
		}
		return nil
	}
	union := shift(r.in.Boxes[ms[0]], r.offsets[ms[0]])
	for _, m := range ms[1:] {
		b := shift(r.in.Boxes[m], r.offsets[m])
		union = Box{Min: r3.Vec{X: math.Min(union.Min.X, b.Min.X), Y: math.Min(union.Min.Y, b.Min.Y), Z: math.Min(union.Min.Z, b.Min.Z)},
			Max: r3.Vec{X: math.Max(union.Max.X, b.Max.X), Y: math.Max(union.Max.Y, b.Max.Y), Z: math.Max(union.Max.Z, b.Max.Z)}}
	}
	var absent []int // the parts a derived round's group does not clear (Stage.Round)
	if st.Clear == ClearAll && st.Round > 0 {
		for p := range r.in.Boxes {
			if !in[p] && (r.movesLater(p, si) || r.inRound(p, st.Round)) {
				absent = append(absent, p)
			}
		}
	}
	var best *choice
	for _, s := range senses {
		var c choice
		c.move = s
		switch st.Clear {
		case ClearAll:
			var standing []int
			for p := range r.in.Boxes {
				if in[p] || slices.Contains(absent, p) {
					continue
				}
				joinedLater := false
				for _, m := range ms {
					if slices.Contains(r.partners[m], p) && r.movesLater(p, si) {
						joinedLater = true
					}
				}
				if !joinedLater {
					standing = append(standing, p)
				}
			}
			if st.Round > 0 {
				c.t = beyondAll(union, s, gap, r.in.Boxes, r.offsets, standing)
			} else {
				c.t = leastClear(union, s, gap, r.in.Boxes, r.offsets, standing)
			}
			c.travel = math.Max(0, c.t-gap)
		default:
			for _, m := range ms {
				for _, cl := range r.cleared(m, r.in.Nodes[m]) {
					if !in[cl] {
						c.t = math.Max(c.t, travel(union, shift(r.in.Boxes[cl], r.offsets[cl]), s))
					}
				}
			}
			c.travel = c.t
		}
		if c.t > stuckAt || math.IsInf(c.t, 1) {
			c.path = PathStuck
		} else {
			add := c.t
			c.path, c.block = PathClear, -1
			for _, m := range ms {
				ignore := append(append(slices.Clone(r.partners[m]), ms...), absent...)
				p, b, err := r.in.Blocker.Path(ctx, m, r.offsets[m], r.offsets[m].Add(s.Scale(add)), r.offsets, ignore)
				if err != nil {
					return err
				}
				if p > c.path || (p == c.path && c.block < 0) {
					c.path, c.block = p, b
				}
			}
		}
		if best == nil || better(c.free(), c.t, c.move, best.free(), best.t, best.move) {
			cc := c
			best = &cc
		}
	}
	for _, m := range ms {
		node := r.in.Nodes[m]
		if best.path == PathStuck {
			r.record(m, si, r3.Vec{}, 0, 0, 0, PathStuck, -1, fromStage, node)
			r.done[m] = true
			continue
		}
		dist := best.t
		travelOut := best.travel
		if st.Clear == ClearJoined {
			dist += gap
		}
		r.setOffset(m, r.offsets[m].Add(best.move.Scale(dist)))
		r.record(m, si, best.move, travelOut, dist, r.gapOf(si, m), best.path, best.block, fromStage, node)
		r.done[m] = true
	}
	return nil
}

// finish scales the offsets, then reads each part's clearance and short flag (S8, S9).
func (r *runner) finish() {
	n := len(r.in.Boxes)
	for i := range r.res {
		r.res[i].Offset = r.offsets[i].Scale(r.in.Scale)
		r.res[i].Moved *= r.in.Scale
		r.res[i].Slide = r.slide[i].Scale(r.in.Scale)
		r.res[i].Stops = make([]r3.Vec, len(r.stops[i]))
		for j, st := range r.stops[i] {
			r.res[i].Stops[j] = st.Scale(r.in.Scale)
		}
	}
	final := make([]Box, n)
	for i := range final {
		final[i] = shift(r.in.Boxes[i], r.res[i].Offset)
	}
	for i := range final {
		best, bestJ := math.Inf(1), -1
		for j := range final {
			if j == i || slices.Contains(r.partners[i], j) {
				continue
			}
			if d := BoxDistance(final[i], final[j]); d < best {
				best, bestJ = d, j
			}
		}
		if bestJ < 0 {
			continue
		}
		r.res[i].Clearance = best
		if r.res[i].Blocker < 0 {
			r.res[i].Blocker = bestJ
		}
		if g := r.res[i].Gap; best < g-1e-9*g {
			r.res[i].Short = true
		}
	}
}

// BoxDistance is the distance between two boxes, zero when they meet.
func BoxDistance(a, b Box) float64 {
	d := func(alo, ahi, blo, bhi float64) float64 { return math.Max(0, math.Max(alo-bhi, blo-ahi)) }
	return math.Sqrt(math.Pow(d(a.Min.X, a.Max.X, b.Min.X, b.Max.X), 2) + math.Pow(d(a.Min.Y, a.Max.Y, b.Min.Y, b.Max.Y), 2) +
		math.Pow(d(a.Min.Z, a.Max.Z, b.Min.Z, b.Max.Z), 2))
}

// ProjectedCover returns, for two boxes seen along view, which is nearer (front is
// 0 for a, 1 for b) and the share of the farther box's projected rectangle the
// nearer box's rectangle covers; zero unless the boxes' exact projections overlap
// with positive area.
func ProjectedCover(a, b Box, view r3.Vec) (int, float64) {
	u, ok := view.Normalize()
	if !ok {
		return 0, 0
	}
	// The projections overlap with positive area exactly when a moved along the view meets b with
	// positive volume (agent-docs/parts-design.md §5.10 H1).
	if len(clearSpans(a, u, 0, []Box{b}, []r3.Vec{{}}, []int{0})) == 0 {
		return 0, 0
	}
	eu, ev := planeBasis(u)
	ra, rb := project(a, eu, ev), project(b, eu, ev)
	w := math.Min(ra[1], rb[1]) - math.Max(ra[0], rb[0])
	h := math.Min(ra[3], rb[3]) - math.Max(ra[2], rb[2])
	if w <= 0 || h <= 0 {
		return 0, 0
	}
	front, back := 0, rb
	if a.Centre().Dot(u) > b.Centre().Dot(u) {
		front, back = 1, ra
	}
	area := (back[1] - back[0]) * (back[3] - back[2])
	if area <= 0 {
		return front, 0
	}
	return front, math.Min(1, w*h/area)
}

func planeBasis(n r3.Vec) (r3.Vec, r3.Vec) {
	ref := r3.Vec{X: 1}
	if math.Abs(n.X) > 0.9 {
		ref = r3.Vec{Y: 1}
	}
	eu, _ := n.Cross(ref).Normalize()
	return eu, n.Cross(eu)
}

// project returns the rectangle [minU, maxU, minV, maxV] of the box's corners.
func project(b Box, eu, ev r3.Vec) [4]float64 {
	r := [4]float64{math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)}
	for i := range 8 {
		p := r3.Vec{X: pick(i&1, b.Min.X, b.Max.X), Y: pick(i&2, b.Min.Y, b.Max.Y), Z: pick(i&4, b.Min.Z, b.Max.Z)}
		u, v := p.Dot(eu), p.Dot(ev)
		r[0], r[1], r[2], r[3] = math.Min(r[0], u), math.Max(r[1], u), math.Min(r[2], v), math.Max(r[3], v)
	}
	return r
}

func pick(bit int, lo, hi float64) float64 {
	if bit == 0 {
		return lo
	}
	return hi
}

// LeastClear is the least t >= 0 at which a box moved by t along unit dir is
// outside every standing box, each grown by gap. It is the distance ClearAll
// stages use.
func LeastClear(moving Box, dir r3.Vec, gap float64, boxes []Box, offsets []r3.Vec, standing []int) float64 {
	return leastClear(moving, dir, gap, boxes, offsets, standing)
}
