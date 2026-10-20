package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is Document.VerifyJointBox (agent-docs/linkage-check-design.md §14):
// the joint-box vocabulary, Linkage.Configuration, the cell certificate, and
// the subdivision that proves a box of joint values clear cell by cell. It
// runs motion_verify.go's engine — the same groups, pairs, per-pose kernel and
// pair walk VerifyLinkage runs — at each cell's centre, and reads the chain
// bounds linkage_bound.go proves over the whole box. A closed loop is varied
// at its driver alone (§16): linkage_loop.go answers each cell's asks for its
// dependent joints, and this file charges them into the cell's certificate.

// JointBox is a box of joint values (agent-docs/linkage-check-design.md §14.1).
type JointBox []JointRange

// JointRange is one link's range of joint values.
type JointRange struct {
	Link     *Link
	Min, Max units.Value
}

// defaultCellBudget is WithCellBudget's default (agent-docs/linkage-check-design.md
// §14.1).
const defaultCellBudget = 16384

// WithCellBudget caps the number of cell centres VerifyJointBox evaluates
// (agent-docs/linkage-check-design.md §14.4); the default is 16384. Every split
// evaluates two centres, so once fewer than two evaluations remain no cell is
// split and the report raises DiagJointBoxBudgetExhausted. A budget of 1
// evaluates the whole box's centre alone; cells < 1 is ErrDegenerate. It is a
// JointBoxOption only, so VerifyMotion and VerifyLinkage do not accept it.
func WithCellBudget(cells int) JointBoxOption {
	return jointBoxOptionValue{motionoption.WithCellBudget(cells)}
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
// (ErrDegenerate); a value of the wrong Kind for its joint (ErrUnitKind); a
// non-finite value (ErrNotFinite); a value outside its joint's declared
// limits (ErrDegenerate, naming the link); and a pose r3 cannot represent
// (ErrNotFinite). A linkage with a loop is ErrUnsupported: a dependent
// joint's value is not the caller's to state. A looped linkage's
// configuration is a drive whose every sweep holds, posed by Linkage.PoseAt
// or Schedule.PoseAt (agent-docs/linkage-check-design.md §16.1).
func (l *Linkage) Configuration(values []units.Value) (JointConfiguration, error) {
	if l == nil {
		return JointConfiguration{}, fmt.Errorf(`%w: a nil linkage has no link to pose`, ErrDegenerate)
	}
	if len(l.loops) > 0 {
		return JointConfiguration{}, errLoopNotStated()
	}
	if len(values) != len(l.links) {
		return JointConfiguration{}, fmt.Errorf(`%w: a configuration names one value per link, %d here, got %d`, ErrDegenerate, len(l.links), len(values))
	}
	spec := l.restSpec()
	for k, v := range values {
		jt := spec.joints[k]
		if err := motionbound.MotionValueValid(v, jt.kind, "a joint value"); err != nil {
			return JointConfiguration{}, err
		}
		if _, ok := motionbound.ExactMotionParam(v); !ok {
			return JointConfiguration{}, fmt.Errorf(`%w: link %d's joint value is not representable`, ErrNotFinite, k)
		}
		if jt.limits != nil && !jt.limits.within(v) {
			return JointConfiguration{}, fmt.Errorf(`%w: link %d's joint value %s lies outside its limits [%s, %s]`,
				ErrDegenerate, k, v, jt.limits.Min, jt.limits.Max)
		}
	}
	poses, err := spec.posesOf(values)
	if err != nil {
		return JointConfiguration{}, err
	}
	return JointConfiguration{Values: slices.Clone(values), Bounds: zeroBounds(values), Poses: poses}, nil
}

// JointBoxReport records the cell subdivision and verdict.
type JointBoxReport struct {
	Request           JointBoxRequest
	ReadingResolution units.Value
	Linkage           *Linkage
	Box               JointBox
	Links             []*Link
	Against           []*Body
	JointContacts     []DiagnosticPair
	Cells             []JointCellResult
	CellsEvaluated    int
	Collisions        []JointBoxCollision
	Clearance         *ScalarReading
	Assessment        Assessment
	Diagnostics       []Diagnostic
	Status            Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *JointBoxReport) Passed() bool { return r != nil && r.Status == Sound }

// JointBoxRequest records the effective settings of a VerifyJointBox call.
type JointBoxRequest struct {
	RelativeTolerance units.Value
	Resolution        units.Value
	MinClearance      *units.Value
	CellBudget        int
}

// JointCell records one closed cell of joint values.
type JointCell struct {
	Min, Max []units.Value
}

func cloneJointCell(c JointCell) JointCell {
	return JointCell{Min: slices.Clone(c.Min), Max: slices.Clone(c.Max)}
}

// JointCellResult records one leaf cell and its centre's findings.
type JointCellResult struct {
	Cell          JointCell
	Outcome       CellOutcome
	Center        JointConfiguration
	Interferences []Interference
	Clearances    []Clearance
	Diagnostics   []Diagnostic
	Clearance     *Measurement
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
type JointBoxCollision struct {
	Configuration JointConfiguration
	A, B          *Body
	Volume        Measurement
}

// VerifyJointBox checks whether every configuration in box is clear: whether
// no link of l, at any joint values the box holds, meets a static body or a
// body of another link (agent-docs/linkage-check-design.md §14). It never changes
// the document: each centre re-evaluates every link body's payload under its
// link's pose as a transient body that is never committed.
//
// The box is cut into cells. A cell is CellClear only when every pair's
// proven gap at the cell's centre exceeds τ_half, the chain travel bound of
// §5.2 over half the cell's weighted span, so no configuration of the cell
// can close it; a cell its centre neither certifies nor proves colliding is
// halved along the joint that most lowers τ_half for the pair that held it
// back, shallowest cells first, until the resolution — a Dimensionless
// fraction of each varying joint's own range — or WithCellBudget stops it. A
// cell left uncertified reads CellUndecided and makes the report Suspect,
// never Sound.
//
// A closed loop (Linkage.Close) is varied at its driver alone: the box lists
// at most one joint of each loop, whose parent is the loop's common link, and
// every other joint of the loop follows it (agent-docs/linkage-check-design.md
// §16). Each cell reads the loop's dependent joints from sketch's certified
// enclosures, as VerifyLinkage reads a drive's intervals: their enclosure at
// the centre is charged into the pose deviation, and the distance from it to
// the far end of their hull over the cell into τ_half. A cell sketch cannot
// enclose is never CellClear or CellBlocked.
//
// The options are VerifyLinkage's (WithMotionTolerance, WithResolution,
// WithMinClearance) plus WithCellBudget. Every body of every link, and every
// body a declared joint contact names, MUST be a live body of d. A nil
// context, document or linkage, a linkage with no link, a box with no varying
// joint, a range with Min > Max, a link named twice, two joints of one loop,
// a range end outside a joint's declared limits, a dependent joint whose
// certified hull is not proven inside its limits, and a budget below 1 are
// ErrDegenerate; a loop joint whose parent is not the loop's common link, and
// a loop whose zero pose sketch cannot enclose, are ErrUnsupported.
// Validation precedes cancellation; after it a canceled context returns
// ctx.Err() and no report.
func (d *Document) VerifyJointBox(ctx context.Context, l *Linkage, box JointBox, opts ...JointBoxOption) (*JointBoxReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control joint-box verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := d.requireLinkage(l); err != nil {
		return nil, err
	}
	spec, axes, err := l.resolveBox(box)
	if err != nil {
		return nil, err
	}
	if len(axes) == 0 {
		return nil, fmt.Errorf(`%w: a box with no varying joint names one configuration`, ErrDegenerate)
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
	cfg, budget, err := resolveJointBoxOptions(opts)
	if err != nil {
		return nil, err
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
	bounds, ok := linkagebound.ReadReachBounds(reachSource{spec: spec, frames: frames})
	if !ok {
		return nil, linkageBoundsError()
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	run := newLinkageRun(ctx, d, spec, frames, bounds, cfg)
	run.poseBodies = make([]poseBodyCache, len(run.movers))
	dr, _ := run.drive.(*linkageDriver) // newLinkageRun always drives by a linkageDriver
	br := &boxRun{run: run, dr: dr, axes: axes, budget: budget}
	if err := br.subdivide(); err != nil {
		return nil, err
	}
	return br.publish(l, box), nil
}

// requireLinkage applies the linkage refusals VerifyLinkage and
// VerifyJointBox share (agent-docs/linkage-check-design.md §3): a nil linkage or
// one with no link, and a link body or declared contact body that is not a
// live body of d this evaluator built.
func (d *Document) requireLinkage(l *Linkage) error {
	if l == nil {
		return fmt.Errorf(`%w: a nil linkage has no link to move`, ErrDegenerate)
	}
	if len(l.links) == 0 {
		return fmt.Errorf(`%w: a linkage with no link moves nothing`, ErrDegenerate)
	}
	for _, link := range l.links {
		for _, b := range link.bodies {
			if err := d.requireLive(b); err != nil {
				return err
			}
			if b.n.Payload == nil {
				return fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
			}
		}
	}
	for _, c := range l.contacts {
		for _, b := range []*Body{c.A, c.B} {
			if err := d.requireLive(b); err != nil {
				return err
			}
		}
	}
	return nil
}

// resolveBox validates box against l (agent-docs/linkage-check-design.md §14.1) and
// reads it into one joint per link: a listed joint's schedule is its range,
// Min to Max, so every bound linkage_bound.go reads over a drive's waypoints
// reads over the range's two ends; an unlisted joint holds 0. axes lists the
// varying joints, Min < Max, in Links() order.
func (l *Linkage) resolveBox(box JointBox) (*linkageSpec, []int, error) {
	spec := l.restSpec()
	varying := make([]bool, len(spec.joints))
	for _, rg := range box {
		link := rg.Link
		if link == nil || link.linkage != l || link.joint == nil {
			return nil, nil, fmt.Errorf(`%w: a box names a link that is not a jointed link of this linkage`, ErrDegenerate)
		}
		jt := &spec.joints[link.index]
		if jt.listed {
			return nil, nil, fmt.Errorf(`%w: a box names link %d twice`, ErrDegenerate, link.index)
		}
		for _, v := range []units.Value{rg.Min, rg.Max} {
			if v.Kind() != jt.kind {
				return nil, nil, fmt.Errorf(`%w: a joint range's ends must be a %s, got %s`, ErrUnitKind, jt.kind, v.Kind())
			}
		}
		if err := motionbound.MotionFinite(rg.Min, rg.Max); err != nil {
			return nil, nil, err
		}
		pMin, okMin := motionbound.ExactMotionParam(rg.Min)
		pMax, okMax := motionbound.ExactMotionParam(rg.Max)
		if !okMin || !okMax {
			return nil, nil, fmt.Errorf(`%w: a joint range's end is not representable`, ErrNotFinite)
		}
		c, ok := motionbound.ParamCompare(rg.Min, rg.Max)
		if !ok || c > 0 {
			return nil, nil, fmt.Errorf(`%w: link %d's range needs Min <= Max, got %s and %s`, ErrDegenerate, link.index, rg.Min, rg.Max)
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
					ErrDegenerate, k, jt.values[0], jt.values[1], jt.limits.Min, jt.limits.Max)
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

// resolveJointBoxOptions folds a joint box's options: the cell budget is the
// box's own, and every other option is a MotionOption folded as
// VerifyLinkage folds it, over the Dimensionless fraction of each range.
func resolveJointBoxOptions(opts []JointBoxOption) (motionConfig, int, error) {
	budget := defaultCellBudget
	motion := make([]MotionOption, 0, len(opts))
	for _, o := range opts {
		if o == nil {
			return motionConfig{}, 0, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := o.(jointBoxOptionValue); ok {
			cells, ok := option.Get[int](o)
			if !ok {
				return motionConfig{}, 0, fmt.Errorf(`%w: a joint-box option carries no value`, ErrDegenerate)
			}
			if cells < 1 {
				return motionConfig{}, 0, fmt.Errorf(`%w: a cell budget must allow at least one centre, got %d`, ErrDegenerate, cells)
			}
			budget = cells
			continue
		}
		mo, ok := o.(MotionOption)
		if !ok {
			return motionConfig{}, 0, fmt.Errorf(`%w: a joint-box option of type %T is not one this evaluator reads`, ErrDegenerate, o)
		}
		motion = append(motion, mo)
	}
	encodedMotion, err := decodeMotionOptions(motion)
	if err != nil {
		return motionConfig{}, 0, err
	}
	cfg, err := motionoption.Resolve(encodedMotion, motionbound.FractionDomain())
	if err != nil {
		return motionConfig{}, 0, err
	}
	if !cfg.Stated {
		// Unstated, the reading refines past the verdict floor to its own,
		// per axis, as a drive's does (agent-docs/linkage-check-design.md §3, §14.1).
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.ReadingP = &reading
	}
	return cfg, budget, nil
}

// boxRun is one VerifyJointBox call's subdivision over the engine's run. It
// lives in the call alone.
type boxRun struct {
	run    *motionRun
	dr     *linkageDriver
	axes   []int // the varying joints, in Links() order
	budget int
	// evaluated is every cell whose centre was evaluated, in evaluation
	// order; leaves are the cells not since split, in cell order.
	evaluated []*boxCell
	leaves    []*boxCell
	// exhausted: the budget stopped a split the floor would have allowed;
	// unsplit counts the cells it held.
	exhausted bool
	unsplit   int
	// upper is the centre holding the smallest gap upper end, upperHi that
	// end; anyViolated: some centre disproves the margin.
	upper       *motionPose
	upperHi     *float64
	anyViolated bool
	// statics holds each static body's point reading for the cell form's
	// projection bounds, read once per call.
	statics map[cellReadingKey]cornerBounds
}

// boxCell is one cell of the subdivision: per link, the fractions lo and hi
// of its joint's range the cell spans — both 0 for a joint that does not
// vary — its depth in the subdivision tree, and what its centre proved.
type boxCell struct {
	lo, hi    []*big.Rat
	depth     int
	cell      JointCell
	pose      *motionPose
	outcome   CellOutcome
	clearance *Measurement
	// held lists the pairs that held the cell back — the uncertified pairs
	// of an undecided cell, the colliding pairs of a colliding one — as
	// (mover, pair) indices; stuck marks an undecided cell some pair that is
	// never evaluated holds back, which no split can change.
	held  [][2]int
	stuck bool
	split bool
	// low is the evaluated pair whose bound is the clear cell's Clearance,
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
	gateLoop *loopDrive
	// projShares holds, for each evaluated pair whose bound over the cell is
	// the projection bound (agent-docs/linkage-check-design.md §5.8), every axis's
	// share of that bound's defect; a pair absent here takes the travel
	// bound's shares, linkagebound.SumTerms over boxRun.pairTerms.
	projShares map[[2]int]map[int]*big.Rat
}

// centre is the fraction of joint k's range at the cell's centre.
func (c *boxCell) centre(k int) *big.Rat {
	mid := new(big.Rat).Add(c.lo[k], c.hi[k])
	return mid.Quo(mid, big.NewRat(2, 1))
}

// subdivide evaluates the whole box's centre, then splits for the verdict and
// for the readings (agent-docs/linkage-check-design.md §14.4 steps 3-7): the verdict
// first, and once no cell is splittable for it, one split at a time for the
// whole-box reading or the margin, the verdict resuming on the halves, until
// neither asks for a split or the budget stops it.
func (b *boxRun) subdivide() error {
	n := len(b.dr.spec.joints)
	root := &boxCell{lo: make([]*big.Rat, n), hi: make([]*big.Rat, n)}
	for k := range n {
		root.lo[k], root.hi[k] = new(big.Rat), new(big.Rat)
	}
	for _, k := range b.axes {
		root.hi[k] = big.NewRat(1, 1)
	}
	schedule := linkagebound.CellSubdivision[*boxCell]{
		Budget:           b.budget,
		Evaluated:        func() int { return len(b.evaluated) },
		Evaluate:         b.evaluate,
		Split:            b.split,
		Depth:            func(c *boxCell) int { return c.depth },
		VerdictAxis:      b.splitAxis,
		NextReadingSplit: b.nextReadingSplit,
	}
	if err := schedule.Run(root); err != nil {
		return err
	}
	b.leaves, b.exhausted, b.unsplit = schedule.Leaves, schedule.Exhausted, schedule.Unsplit
	return b.run.ctx.Err()
}

// nextReadingSplit is §14.4 step 6: the leaf to split for the readings and
// the axis to split it along, or −1. The leaf is the clear cell holding the
// smallest lower bound, ties in cell order. It splits while every leaf is
// clear and the whole-box reading fails the tolerance gate, along an axis
// wider than the reading floor; and while a requested margin is neither proven
// by that bound nor disproven by some centre, along an axis wider than the
// verdict floor. The axis is the one with the largest share of the defect of
// the bound the pair that attained it holds (readingAxis).
func (b *boxRun) nextReadingSplit(leaves []*boxCell) (int, int) {
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
			if reading, _ := r.pathClearance(b.upperPoses(), c.clearance, "whole-box"); reading != nil && reading.Tolerance.State != ToleranceSatisfied {
				return smallest, axis
			}
		}
	}
	if r.cfg.MinimumMM != nil && !b.anyViolated && !r.meetsMinimum(c.clearance) {
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
func (b *boxRun) readingAxis(c *boxCell, wide func(*boxCell, int) bool) int {
	share, projected := b.boundShares(c, c.low[0], c.low[1])
	return linkagebound.RankAxes(b.axes, share, projected, func(k int) bool { return wide(c, k) })
}

// boundShares is each axis's share of the defect of pair k of mover i's
// bound over cell c: the projection bound's own shares where that bound is
// the pair's (projected true), else the travel bound's, w_i·span_i.
func (b *boxRun) boundShares(c *boxCell, i, k int) (map[int]*big.Rat, bool) {
	if shares, ok := c.projShares[[2]int{i, k}]; ok {
		return shares, true
	}
	return linkagebound.SumTerms(b.pairTerms(c, i, k)), false
}

// upperPoses is the evaluated centre holding the smallest proven upper end
// of any pair's gap, the first in evaluation order on a tie, or none: all the
// whole-box reading takes from the centres besides the leaves' bounds, so the
// reading over it is the reading over every centre.
func (b *boxRun) upperPoses() []*motionPose {
	if b.upper == nil {
		return nil
	}
	return []*motionPose{b.upper}
}

// note keeps the running facts the readings consult about every evaluated
// centre: the smallest gap upper end and whether some margin is disproven.
func (b *boxRun) note(mp *motionPose) {
	b.anyViolated = b.anyViolated || mp.violated
	for _, row := range mp.pairs {
		for _, pp := range row {
			if pp.hasGap && (b.upperHi == nil || pp.hi < *b.upperHi) {
				hi := pp.hi
				b.upper, b.upperHi = mp, &hi
			}
		}
	}
}

// split halves cell c along joint axis, evaluating both halves' centres.
func (b *boxRun) split(c *boxCell, axis int) (*boxCell, *boxCell, error) {
	mid := c.centre(axis)
	half := func(lo, hi *big.Rat) *boxCell {
		child := &boxCell{lo: slices.Clone(c.lo), hi: slices.Clone(c.hi), depth: c.depth + 1}
		child.lo[axis], child.hi[axis] = lo, hi
		return child
	}
	lower, upper := half(c.lo[axis], mid), half(mid, c.hi[axis])
	c.split = true
	for _, child := range []*boxCell{lower, upper} {
		if err := b.evaluate(child); err != nil {
			return nil, nil, err
		}
	}
	return lower, upper, nil
}
