package linkage

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/motionengine"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyJointBox (agent-docs/linkage-check-design.md §14):
// the joint-box vocabulary, Linkage.Configuration, the cell certificate, and
// the subdivision that proves a box of joint values clear cell by cell. It
// runs internal/motionengine's engine — the same groups, pairs, per-pose kernel and
// pair walk VerifyLinkage runs — at each cell's centre, and reads the chain
// bounds linkage/bound.go proves over the whole box. A closed loop is varied
// at its driver alone (§16): linkage/loop.go answers each cell's asks for its
// dependent joints, and this file charges them into the cell's certificate.

// JointBox is a box of joint values (agent-docs/linkage-check-design.md §14.1).
type JointBox[B comparable] []JointRange[B]

// JointRange is one link's range of joint values.
type JointRange[B comparable] struct {
	Link     *Link[B]
	Min, Max units.Value
}

// JointConfiguration records every link's joint value and world pose.
type JointConfiguration struct {
	Values []units.Value
	Bounds []units.Value
	Poses  []r3.Transform
}

// Configuration builds every link's world pose at the stated joint values,
// one per link in Links() order (agent-docs/linkage-check-design.md §14.1). It
// composes exactly as PoseAt composes — PoseAt is Configuration of a drive's
// values at s — so a renderer drawing a cell and VerifyJointBox evaluating its
// centre read the same transform. Every Bounds entry is zero.
//
// It refuses a nil linkage or a value count other than len(Links())
// (decaderr.ErrDegenerate); a value of the wrong Kind for its joint (decaderr.ErrUnitKind); a
// non-finite value (decaderr.ErrNotFinite); a value outside its joint's declared
// limits (decaderr.ErrDegenerate, naming the link); and a pose r3 cannot represent
// (decaderr.ErrNotFinite). A linkage with a loop is decaderr.ErrUnsupported: a dependent
// joint's value is not the caller's to state. A looped linkage's
// configuration is a drive whose every sweep holds, posed by Linkage.PoseAt
// or Schedule.PoseAt (agent-docs/linkage-check-design.md §16.1).
func (l *Linkage[B]) Configuration(values []units.Value) (JointConfiguration, error) {
	if l == nil {
		return JointConfiguration{}, fmt.Errorf(`%w: a nil linkage has no link to pose`, decaderr.ErrDegenerate)
	}
	if len(l.loops) > 0 {
		return JointConfiguration{}, errLoopNotStated()
	}
	if len(values) != len(l.links) {
		return JointConfiguration{}, fmt.Errorf(`%w: a configuration names one value per link, %d here, got %d`, decaderr.ErrDegenerate, len(l.links), len(values))
	}
	spec := l.restSpec()
	for k, v := range values {
		jt := spec.joints[k]
		if err := motionbound.MotionValueValid(v, jt.kind, "a joint value"); err != nil {
			return JointConfiguration{}, err
		}
		if _, ok := motionbound.ExactMotionParam(v); !ok {
			return JointConfiguration{}, fmt.Errorf(`%w: link %d's joint value is not representable`, decaderr.ErrNotFinite, k)
		}
		if jt.limits != nil && !jt.limits.within(v) {
			return JointConfiguration{}, fmt.Errorf(`%w: link %d's joint value %s lies outside its limits [%s, %s]`,
				decaderr.ErrDegenerate, k, v, jt.limits.Min, jt.limits.Max)
		}
	}
	poses, err := spec.posesOf(values)
	if err != nil {
		return JointConfiguration{}, err
	}
	return JointConfiguration{Values: slices.Clone(values), Bounds: zeroBounds(values), Poses: poses}, nil
}

// JointBoxReport records the cell subdivision and verdict.
type JointBoxReport[B comparable] struct {
	Request           JointBoxRequest
	ReadingResolution units.Value
	Linkage           *Linkage[B]
	Box               JointBox[B]
	Links             []*Link[B]
	Against           []B
	JointContacts     []vocab.DiagnosticPairOf[B]
	Cells             []JointCellResult[B]
	CellsEvaluated    int
	Collisions        []JointBoxCollision[B]
	Clearance         *vocab.ScalarReading
	Assessment        vocab.Assessment
	Diagnostics       []vocab.DiagnosticOf[B, vocab.JointCell]
	Status            vocab.Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *JointBoxReport[B]) Passed() bool { return r != nil && r.Status == vocab.Sound }

// JointBoxRequest records the effective settings of a VerifyJointBox call.
type JointBoxRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
	CellBudget        int
}

// JointCellResult records one leaf cell and its centre's findings.
type JointCellResult[B comparable] struct {
	Cell          vocab.JointCell
	Outcome       CellOutcome
	Center        JointConfiguration
	Interferences []vocab.InterferenceOf[B]
	Clearances    []vocab.ClearanceOf[B]
	Diagnostics   []vocab.DiagnosticOf[B, vocab.JointCell]
	Clearance     *vocab.Measurement
}

// CellOutcome states what a JointCellResult proves.
type CellOutcome int

const (
	CellNotEvaluated CellOutcome = iota
	CellClear
	CellBlocked
	CellColliding
	CellUndecided
)

// String renders the stable lower-snake token, including unknown values.
func (o CellOutcome) String() string {
	switch o {
	case CellNotEvaluated:
		return "not_evaluated"
	case CellClear:
		return "clear"
	case CellBlocked:
		return "blocked"
	case CellColliding:
		return "colliding"
	case CellUndecided:
		return "undecided"
	default:
		return fmt.Sprintf("cell_outcome(%d)", int(o))
	}
}

// JointBoxCollision is a proven overlap at one evaluated centre.
type JointBoxCollision[B comparable] struct {
	Configuration JointConfiguration
	A, B          B
	Volume        vocab.Measurement
}

// VerifyBox is the engine of Document.VerifyJointBox over a linkage the
// caller has already admitted. options resolves the caller's options and cell
// budget once the box is known good.
func VerifyBox[B comparable](ctx context.Context, h Hooks[B], l *Linkage[B], box JointBox[B],
	options func() (motionoption.Config, int, error)) (*JointBoxReport[B], error) {
	spec, axes, err := l.resolveBox(box)
	if err != nil {
		return nil, err
	}
	if len(axes) == 0 {
		return nil, fmt.Errorf(`%w: a box with no varying joint names one configuration`, decaderr.ErrDegenerate)
	}
	for _, corner := range []*big.Rat{new(big.Rat), big.NewRat(1, 1)} {
		if _, _, err := spec.posesAt(corner); err != nil {
			return nil, err
		}
	}
	frames, ok := linkageFrames(spec)
	if !ok {
		return nil, linkageBoundsError()
	}
	cfg, budget, err := options()
	if err != nil {
		return nil, err
	}
	if !cfg.Stated {
		// Unstated, the reading refines past the verdict floor to its own,
		// per axis, as a drive's does (agent-docs/linkage-check-design.md §3, §14.1).
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.ReadingP = &reading
	}
	if len(spec.loops) > 0 {
		// agent-docs/linkage-check-design.md §16.2: each loop axis is the drive
		// Min → Max; its scenes, its zero pose and its certifiable set, down
		// to the verdict floor, before any cell, so a dependent's reach enters
		// the bounds.
		if err := spec.prepareLoops(ctx, cfg.ResolutionP.Base); err != nil {
			return nil, err
		}
	}
	bounds, ok := linkagebound.ReadReachBounds(reachSource[B]{spec: spec, frames: frames, node: h.Node})
	if !ok {
		return nil, linkageBoundsError()
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	run := newLinkageRun(ctx, h, spec, frames, bounds, cfg)
	run.PoseBodies = make([]motionengine.PoseBodyCache, len(run.Movers))
	dr, _ := run.Drive.(*linkageDriver[B]) // newLinkageRun always drives by a linkageDriver
	br := &boxRun[B]{h: h, run: run, dr: dr, axes: axes, budget: budget}
	if err := br.subdivide(); err != nil {
		return nil, err
	}
	return br.publish(l, box), nil
}

// resolveBox validates box against l (agent-docs/linkage-check-design.md §14.1) and
// reads it into one joint per link: a listed joint's schedule is its range,
// Min to Max, so every bound linkage/bound.go reads over a drive's waypoints
// reads over the range's two ends; an unlisted joint holds 0. axes lists the
// varying joints, Min < Max, in Links() order.
func (l *Linkage[B]) resolveBox(box JointBox[B]) (*linkageSpec[B], []int, error) {
	spec := l.restSpec()
	varying := make([]bool, len(spec.joints))
	for _, rg := range box {
		link := rg.Link
		if link == nil || link.linkage != l || link.joint == nil {
			return nil, nil, fmt.Errorf(`%w: a box names a link that is not a jointed link of this linkage`, decaderr.ErrDegenerate)
		}
		jt := &spec.joints[link.index]
		if jt.listed {
			return nil, nil, fmt.Errorf(`%w: a box names link %d twice`, decaderr.ErrDegenerate, link.index)
		}
		for _, v := range []units.Value{rg.Min, rg.Max} {
			if v.Kind() != jt.kind {
				return nil, nil, fmt.Errorf(`%w: a joint range's ends must be a %s, got %s`, decaderr.ErrUnitKind, jt.kind, v.Kind())
			}
		}
		if err := motionbound.MotionFinite(rg.Min, rg.Max); err != nil {
			return nil, nil, err
		}
		pMin, okMin := motionbound.ExactMotionParam(rg.Min)
		pMax, okMax := motionbound.ExactMotionParam(rg.Max)
		if !okMin || !okMax {
			return nil, nil, fmt.Errorf(`%w: a joint range's end is not representable`, decaderr.ErrNotFinite)
		}
		c, ok := motionbound.ParamCompare(rg.Min, rg.Max)
		if !ok || c > 0 {
			return nil, nil, fmt.Errorf(`%w: link %d's range needs Min <= Max, got %s and %s`, decaderr.ErrDegenerate, link.index, rg.Min, rg.Max)
		}
		jt.listed, jt.values, jt.points = true, []units.Value{rg.Min, rg.Max}, []motionbound.MotionParam{pMin, pMax}
		varying[link.index] = c < 0
	}
	// A loop is varied at its driver alone, its range the one-segment drive
	// Min → Max (agent-docs/linkage-check-design.md §16.1, §16.2).
	if err := l.resolveLoops(spec, "box"); err != nil {
		return nil, nil, err
	}
	// Both ends inside a joint's limits put the whole range inside them; an
	// unlisted joint holds 0. A loop's dependent is held to its certified hull
	// instead, once the decomposition has read it.
	for k, jt := range spec.joints {
		if jt.limits == nil || jt.dep != nil {
			continue
		}
		for _, v := range jt.values {
			if !jt.limits.within(v) {
				return nil, nil, fmt.Errorf(`%w: the box takes link %d's joint over [%s, %s], outside its limits [%s, %s]`,
					decaderr.ErrDegenerate, k, jt.values[0], jt.values[1], jt.limits.Min, jt.limits.Max)
			}
		}
	}
	var axes []int
	for k, v := range varying {
		if v {
			axes = append(axes, k)
		}
	}
	return spec, axes, nil
}

// boxRun is one VerifyJointBox call's subdivision over the engine's run. It
// lives in the call alone.
type boxRun[B comparable] struct {
	h      Hooks[B]
	run    *motionengine.Run
	dr     *linkageDriver[B]
	axes   []int // the varying joints, in Links() order
	budget int
	// evaluated is every cell whose centre was evaluated, in evaluation
	// order; leaves are the cells not since split, in cell order.
	evaluated []*boxCell[B]
	leaves    []*boxCell[B]
	// exhausted: the budget stopped a split the floor would have allowed;
	// unsplit counts the cells it held.
	exhausted bool
	unsplit   int
	// upper is the centre holding the smallest gap upper end, upperHi that
	// end; anyViolated: some centre disproves the margin.
	upper       *motionengine.Pose
	upperHi     *float64
	anyViolated bool
	// statics holds each static body's point reading for the cell form's
	// projection bounds, read once per call.
	statics map[cellReadingKey]cornerBounds
}

// boxCell is one cell of the subdivision: per link, the fractions lo and hi
// of its joint's range the cell spans — both 0 for a joint that does not
// vary — its depth in the subdivision tree, and what its centre proved.
type boxCell[B comparable] struct {
	lo, hi    []*big.Rat
	depth     int
	cell      vocab.JointCell
	pose      *motionengine.Pose
	outcome   CellOutcome
	clearance *vocab.Measurement
	// held lists the pairs that held the cell back — the uncertified pairs
	// of an undecided cell, the colliding pairs of a colliding one — as
	// (mover, pair) indices; stuck marks an undecided cell some pair that is
	// never evaluated holds back, which no split can change.
	held  [][2]int
	stuck bool
	split bool
	// low is the evaluated pair whose bound is the clear cell's vocab.ClearanceOf[B],
	// the pair the reading and the margin split for; nil when that bound is
	// an excluded pair's, which no split changes.
	low *[2]int
	// A looped linkage's cell (agent-docs/linkage-check-design.md §16.3): delta is,
	// per dependent joint, δ_j, the distance from the centre's enclosure to
	// the far end of the dependent's hull over the cell, zero for a held
	// loop's. gate is the refusal that left the cell's centre or a hull
	// unread, empty otherwise, and gateLoop the loop that refused: such a
	// cell has no δ_j, so no pair of it has a τ_half.
	delta map[int]*big.Rat
	// depReach is, per dependent joint, its centre and half-span for the
	// projection bound's expansion over the cell (linkagebound.DependentCellReach).
	depReach map[int]linkagebound.CellReach
	gate     string
	gateLoop *loopDrive[B]
	// projShares holds, for each evaluated pair whose bound over the cell is
	// the projection bound (agent-docs/linkage-check-design.md §5.8), every axis's
	// share of that bound's defect; a pair absent here takes the travel
	// bound's shares, linkagebound.SumTerms over boxRun.pairTerms.
	projShares map[[2]int]map[int]*big.Rat
}

// centre is the fraction of joint k's range at the cell's centre.
func (c *boxCell[B]) centre(k int) *big.Rat {
	mid := new(big.Rat).Add(c.lo[k], c.hi[k])
	return mid.Quo(mid, big.NewRat(2, 1))
}

// subdivide evaluates the whole box's centre, then splits for the verdict and
// for the readings (agent-docs/linkage-check-design.md §14.4 steps 3-7): the verdict
// first, and once no cell is splittable for it, one split at a time for the
// whole-box reading or the margin, the verdict resuming on the halves, until
// neither asks for a split or the budget stops it.
func (b *boxRun[B]) subdivide() error {
	n := len(b.dr.spec.joints)
	root := &boxCell[B]{lo: make([]*big.Rat, n), hi: make([]*big.Rat, n)}
	for k := range n {
		root.lo[k], root.hi[k] = new(big.Rat), new(big.Rat)
	}
	for _, k := range b.axes {
		root.hi[k] = big.NewRat(1, 1)
	}
	schedule := linkagebound.CellSubdivision[*boxCell[B]]{
		Budget:           b.budget,
		Evaluated:        func() int { return len(b.evaluated) },
		Evaluate:         b.evaluate,
		Split:            b.split,
		Depth:            func(c *boxCell[B]) int { return c.depth },
		VerdictAxis:      b.splitAxis,
		NextReadingSplit: b.nextReadingSplit,
	}
	if err := schedule.Run(root); err != nil {
		return err
	}
	b.leaves, b.exhausted, b.unsplit = schedule.Leaves, schedule.Exhausted, schedule.Unsplit
	return b.run.Ctx.Err()
}

// nextReadingSplit is §14.4 step 6: the leaf to split for the readings and
// the axis to split it along, or −1. The leaf is the clear cell holding the
// smallest lower bound, ties in cell order. It splits while every leaf is
// clear and the whole-box reading fails the tolerance gate, along an axis
// wider than the reading floor; and while a requested margin is neither proven
// by that bound nor disproven by some centre, along an axis wider than the
// verdict floor. The axis is the one with the largest share of the defect of
// the bound the pair that attained it holds (readingAxis).
func (b *boxRun[B]) nextReadingSplit(leaves []*boxCell[B]) (int, int) {
	r := b.run
	allClear, smallest := true, -1
	for n, c := range leaves {
		if c.outcome != CellClear {
			allClear = false
			continue
		}
		if c.clearance != nil && (smallest < 0 || c.clearance.Value.Base() < leaves[smallest].clearance.Value.Base()) {
			smallest = n
		}
	}
	if smallest < 0 || leaves[smallest].low == nil {
		return -1, -1
	}
	c := leaves[smallest]
	if allClear {
		if axis := b.readingAxis(c, b.wideForReading); axis >= 0 {
			if reading, _ := r.PathClearance(b.upperPoses(), c.clearance, "whole-box"); reading != nil && reading.Tolerance.State != vocab.ToleranceSatisfied {
				return smallest, axis
			}
		}
	}
	if r.Cfg.MinimumMM != nil && !b.anyViolated && !r.MeetsMinimum(c.clearance) {
		if axis := b.readingAxis(c, b.wide); axis >= 0 {
			return smallest, axis
		}
	}
	return -1, -1
}

// readingAxis is the varying joint, among those wide reports wider than its
// floor, with the largest share of the defect of the bound the pair that
// attained a clear cell's bound holds (boundShares); ties go to the earliest
// link, −1 when none qualifies.
func (b *boxRun[B]) readingAxis(c *boxCell[B], wide func(*boxCell[B], int) bool) int {
	share, projected := b.boundShares(c, c.low[0], c.low[1])
	return linkagebound.RankAxes(b.axes, share, projected, func(k int) bool { return wide(c, k) })
}

// boundShares is each axis's share of the defect of pair k of mover i's
// bound over cell c: the projection bound's own shares where that bound is
// the pair's (projected true), else the travel bound's, w_i·span_i.
func (b *boxRun[B]) boundShares(c *boxCell[B], i, k int) (map[int]*big.Rat, bool) {
	if shares, ok := c.projShares[[2]int{i, k}]; ok {
		return shares, true
	}
	return linkagebound.SumTerms(b.pairTerms(c, i, k)), false
}

// upperPoses is the evaluated centre holding the smallest proven upper end
// of any pair's gap, the first in evaluation order on a tie, or none: all the
// whole-box reading takes from the centres besides the leaves' bounds, so the
// reading over it is the reading over every centre.
func (b *boxRun[B]) upperPoses() []*motionengine.Pose {
	if b.upper == nil {
		return nil
	}
	return []*motionengine.Pose{b.upper}
}

// note keeps the running facts the readings consult about every evaluated
// centre: the smallest gap upper end and whether some margin is disproven.
func (b *boxRun[B]) note(mp *motionengine.Pose) {
	b.anyViolated = b.anyViolated || mp.Violated
	for _, row := range mp.Pairs {
		for _, pp := range row {
			if pp.HasGap && (b.upperHi == nil || pp.Hi < *b.upperHi) {
				hi := pp.Hi
				b.upper, b.upperHi = mp, &hi
			}
		}
	}
}

// split halves cell c along joint axis, evaluating both halves' centres.
func (b *boxRun[B]) split(c *boxCell[B], axis int) (*boxCell[B], *boxCell[B], error) {
	mid := c.centre(axis)
	half := func(lo, hi *big.Rat) *boxCell[B] {
		child := &boxCell[B]{lo: slices.Clone(c.lo), hi: slices.Clone(c.hi), depth: c.depth + 1}
		child.lo[axis], child.hi[axis] = lo, hi
		return child
	}
	lower, upper := half(c.lo[axis], mid), half(mid, c.hi[axis])
	c.split = true
	for _, child := range []*boxCell[B]{lower, upper} {
		if err := b.evaluate(child); err != nil {
			return nil, nil, err
		}
	}
	return lower, upper, nil
}
