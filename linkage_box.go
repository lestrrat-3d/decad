package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is Document.VerifyJointBox (docs/linkage-check-design.md §14):
// the joint-box vocabulary, Linkage.Configuration, the cell certificate, and
// the subdivision that proves a box of joint values clear cell by cell. It
// runs motion_verify.go's engine — the same groups, pairs, per-pose kernel and
// pair walk VerifyLinkage runs — at each cell's centre, and reads the chain
// bounds linkage_bound.go proves over the whole box. A closed loop is varied
// at its driver alone (§16): linkage_loop.go answers each cell's asks for its
// dependent joints, and this file charges them into the cell's certificate.

// JointBox is a box of joint values (docs/linkage-check-design.md §14.1).
type JointBox []JointRange

// JointRange is one link's range of joint values.
type JointRange struct {
	Link     *Link
	Min, Max units.Value
}

// defaultCellBudget is WithCellBudget's default (docs/linkage-check-design.md
// §14.1).
const defaultCellBudget = 16384

// WithCellBudget caps the number of cell centres VerifyJointBox evaluates
// (docs/linkage-check-design.md §14.4); the default is 16384. Every split
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
// one per link in Links() order (docs/linkage-check-design.md §14.1). It
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
// or Schedule.PoseAt (docs/linkage-check-design.md §16.1).
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
// body of another link (docs/linkage-check-design.md §14). It never changes
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
// every other joint of the loop follows it (docs/linkage-check-design.md
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
		// docs/linkage-check-design.md §16.2: each loop axis is the drive
		// Min → Max; its scenes, its zero pose and its certifiable set, down
		// to the verdict floor, before any cell, so a dependent's reach enters
		// the bounds.
		if err := spec.prepareLoops(ctx, cfg.ResolutionP.Base); err != nil {
			return nil, err
		}
	}
	bounds, ok := readLinkBounds(spec, frames)
	if !ok {
		return nil, linkageBoundsError()
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	run := newLinkageRun(ctx, d, spec, frames, bounds, cfg)
	dr, _ := run.drive.(*linkageDriver) // newLinkageRun always drives by a linkageDriver
	br := &boxRun{run: run, dr: dr, axes: axes, budget: budget}
	if err := br.subdivide(); err != nil {
		return nil, err
	}
	return br.publish(l, box), nil
}

// requireLinkage applies the linkage refusals VerifyLinkage and
// VerifyJointBox share (docs/linkage-check-design.md §3): a nil linkage or
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
			if b.payload == nil {
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

// resolveBox validates box against l (docs/linkage-check-design.md §14.1) and
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
	// Min → Max (docs/linkage-check-design.md §16.1, §16.2).
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
		// per axis, as a drive's does (docs/linkage-check-design.md §3, §14.1).
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
	// A looped linkage's cell (docs/linkage-check-design.md §16.3): delta is,
	// per dependent joint, δ_j, the distance from the centre's enclosure to
	// the far end of the dependent's hull over the cell, zero for a held
	// loop's. gate is the refusal that left the cell's centre or a hull
	// unread, empty otherwise, and gateLoop the loop that refused: such a
	// cell has no δ_j, so no pair of it has a τ_half.
	delta map[int]*big.Rat
	// depReach is, per dependent joint, its centre and half-span for the
	// projection bound's expansion over the cell (dependentReach).
	depReach map[int]linkagebound.CellReach
	gate     string
	gateLoop *loopDrive
	// projShares holds, for each evaluated pair whose bound over the cell is
	// the projection bound (docs/linkage-check-design.md §5.8), every axis's
	// share of that bound's defect; a pair absent here takes the travel
	// bound's shares, pairShares.
	projShares map[[2]int]map[int]*big.Rat
}

// centre is the fraction of joint k's range at the cell's centre.
func (c *boxCell) centre(k int) *big.Rat {
	mid := new(big.Rat).Add(c.lo[k], c.hi[k])
	return mid.Quo(mid, big.NewRat(2, 1))
}

// subdivide evaluates the whole box's centre, then splits for the verdict and
// for the readings (docs/linkage-check-design.md §14.4 steps 3-7): the verdict
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

// evaluate runs the pair procedure at cell c's centre and classifies the
// cell from it (docs/linkage-check-design.md §14.3, §14.4 step 4): blocked
// when a collision there survives every configuration of the cell, else
// colliding when some pair collides there, clear when every pair certifies
// the cell, undecided otherwise.
func (b *boxRun) evaluate(c *boxCell) error {
	r := b.run
	if err := r.ctx.Err(); err != nil {
		return err
	}
	spec := b.dr.spec
	c.cell = JointCell{Min: make([]units.Value, len(spec.joints)), Max: make([]units.Value, len(spec.joints))}
	for k, jt := range spec.joints {
		c.cell.Min[k], c.cell.Max[k] = jt.label(c.lo[k]), jt.label(c.hi[k])
	}
	var values, bounds []units.Value
	var poses []r3.Transform
	var ideals []motionbound.IdealPose
	if len(spec.loops) == 0 {
		values = make([]units.Value, len(spec.joints))
		params := make([]motionbound.MotionParam, len(spec.joints))
		for k, jt := range spec.joints {
			f := c.centre(k)
			values[k], params[k] = jt.label(f), jointParam(jt, f)
		}
		var err error
		if poses, err = spec.posesOf(values); err != nil {
			return err
		}
		bounds, ideals = zeroBounds(values), idealPosesOf(spec, b.dr.frames, params)
	} else {
		lp, err := b.loopCentre(c)
		if err != nil {
			return err
		}
		values, bounds, poses, ideals = lp.values, lp.bounds, lp.poses, lp.ideals
	}
	var mp *motionPose
	if c.gate != "" && poses == nil {
		// docs/linkage-check-design.md §16.3: an unbuildable centre has no
		// pose; it evaluates no pair and carries no row.
		mp = &motionPose{
			cell: &c.cell, unbuildable: errors.New(c.gate), pairs: make([][]motionPairPose, len(r.movers)),
			result: PoseResult{Interferences: []Interference{}, Clearances: []Clearance{}, Diagnostics: []Diagnostic{}},
		}
		for i := range r.movers {
			mp.pairs[i] = make([]motionPairPose, len(r.pairs[i]))
		}
	} else {
		groups := make([]motionGroupPose, len(poses))
		for k := range poses {
			groups[k] = motionGroupPose{
				pose: poses[k], ideals: []motionbound.IdealPose{ideals[k]}, stretch: 1, value: values[k], bound: bounds[k],
				fixed: b.dr.standing[k] == linkFixed, constant: b.dr.standing[k] == linkConstant,
			}
		}
		mp = r.newPose(groups, units.Value{}, "the configuration "+formatValues(values))
		mp.cell = &c.cell
		if err := r.runPairs(mp); err != nil {
			return err
		}
	}
	c.pose = mp
	b.evaluated = append(b.evaluated, c)
	b.note(mp)
	b.classify(c)
	return nil
}

// loopCentre reads a looped linkage's cell (docs/linkage-check-design.md
// §16.2, §16.3): the centre's pose, each dependent at its enclosure's float
// midpoint and its ideal pose over the whole enclosure M_j, then per loop the
// hull H_j of each dependent over the cell's loop-axis range, δ_j =
// max(h_hi − m_lo, m_hi − h_lo) and the cell's published range for the
// dependent, H_j's ends rounded outward. A held loop's dependents stand
// still across the cell: δ_j is zero and the range is M_j. A refused centre
// returns no pose and gates the cell; a refused hull gates it and publishes
// the dependent's range as zero values.
func (b *boxRun) loopCentre(c *boxCell) (loopPose, error) {
	spec := b.dr.spec
	fracs := make([]*big.Rat, len(spec.joints))
	for k := range fracs {
		fracs[k] = c.centre(k)
	}
	lp, err := spec.loopPosesOf(b.run.ctx, b.dr.frames, fracs)
	var ub *unbuildableError
	if errors.As(err, &ub) {
		c.gate, c.gateLoop = ub.Error(), ub.loop
		for _, ld := range spec.loops {
			for _, d := range ld.deps {
				c.cell.Min[d], c.cell.Max[d] = units.Value{}, units.Value{}
			}
		}
		return loopPose{}, nil
	}
	if err != nil {
		return loopPose{}, err
	}
	c.delta = make(map[int]*big.Rat)
	c.depReach = make(map[int]linkagebound.CellReach)
	for _, ld := range spec.loops {
		var hulls []proofbound.RatInterval
		if !ld.held {
			hulls, err = ld.cellHulls(b.run.ctx, c.lo[ld.driver], c.hi[ld.driver])
			if errors.As(err, &ub) {
				if c.gate == "" {
					c.gate, c.gateLoop = ub.Error(), ld
				}
				for _, d := range ld.deps {
					c.cell.Min[d], c.cell.Max[d] = units.Value{}, units.Value{}
				}
				continue
			}
			if err != nil {
				return loopPose{}, err
			}
		}
		for j, d := range ld.deps {
			m := linkagebound.ValueInterval(lp.lo[d], lp.hi[d])
			h, delta := m, new(big.Rat)
			if hulls != nil {
				h, delta = hulls[j], linkagebound.DependentCellDelta(m, hulls[j])
			}
			c.delta[d] = delta
			c.depReach[d] = linkagebound.DependentCellReach(m, h)
			unit := lp.values[d].Unit()
			c.cell.Min[d], c.cell.Max[d] = units.New(proofbound.RatFloatDown(h.Lo), unit), units.New(proofbound.RatFloatUp(h.Hi), unit)
		}
	}
	return lp, nil
}

// classify sets a cell's outcome from its evaluated centre. A gated cell
// (docs/linkage-check-design.md §16.3) is never CellClear or CellBlocked: it
// is CellColliding when its centre collides and CellUndecided otherwise.
func (b *boxRun) classify(c *boxCell) {
	r := b.run
	mp := c.pose
	for i := range r.movers {
		for k := range r.pairs[i] {
			if mp.pairs[i][k].collision {
				c.held = append(c.held, [2]int{i, k})
			}
		}
	}
	if c.gate != "" {
		c.outcome = CellUndecided
		if len(c.held) > 0 {
			c.outcome = CellColliding
		}
		return
	}
	if len(c.held) > 0 {
		c.outcome = CellColliding
		for _, hit := range mp.collisions {
			for _, pair := range c.held {
				i, k := pair[0], pair[1]
				if hit.Moving == r.movers[i].body && hit.Static == r.partner(i, k) && b.blocks(c, i, k, hit.Volume) {
					c.outcome = CellBlocked
					return
				}
			}
		}
		return
	}
	var lowBound *big.Rat
	var low *[2]int
	readings := make(map[cellReadingKey]cornerBounds)
	lowest, ok := r.certifyPairs(func(i, k int) *big.Rat {
		pp := mp.pairs[i][k]
		if !pp.hasGap {
			return nil
		}
		// docs/linkage-check-design.md §14.3: the pair's bound over the cell is
		// the larger of the travel bound lo_m − τ_half and the projection
		// bound L_n(C), two proven lower bounds on one gap.
		lo := proofarith.FloatRat(pp.lo)
		bound := lo.Sub(lo, b.halfTravel(c, i, k))
		if proj, shares := b.cellProjection(c, i, k, readings); proj != nil && proj.Cmp(bound) > 0 {
			bound = proj
			if c.projShares == nil {
				c.projShares = make(map[[2]int]map[int]*big.Rat)
			}
			c.projShares[[2]int{i, k}] = shares
		}
		if bound.Sign() <= 0 {
			return nil
		}
		if lowBound == nil || bound.Cmp(lowBound) < 0 {
			lowBound, low = bound, &[2]int{i, k}
		}
		return bound
	}, func(i, k int) {
		if !r.pairs[i][k].evaluated() {
			c.stuck = true
		}
		c.held = append(c.held, [2]int{i, k})
	})
	if !ok {
		c.outcome = CellUndecided
		return
	}
	c.outcome, c.clearance = CellClear, motionbound.LowerBoundMeasurement(lowest)
	if lowBound != nil && lowBound.Cmp(lowest) == 0 {
		c.low = low
	}
}

// cellProjection is docs/linkage-check-design.md §5.8's cell form for pair k
// of mover i over cell c (§14.3): each body's inflated box read at the
// centre's ideal poses under the joints on its relative path — the joints
// strictly below a link-link pair's lowest common ancestor, less a symmetric
// body's own joint (linkageDriver.pathOf, §5.2) — and expanded to
// second order in h_i, half of each joint's span across the cell rounded up
// to a float; the largest separation along the six coordinate directions. A
// loop's dependent joint is read at its centre, the midpoint of its
// enclosure at the cell's centre, with h_i the farthest that centre sits
// from the hull of the cell's hull and that enclosure (dependentReach), and
// its share is charged to its driver. It returns the bound and each axis's
// share of its defect, or nil when a dependent's readings are missing, as on
// a gated cell, or a box cannot be read exactly. readings
// keeps each mover's corner reading at the centre for the cell's other
// pairs.
func (b *boxRun) cellProjection(c *boxCell, i, k int, readings map[cellReadingKey]cornerBounds) (*big.Rat, map[int]*big.Rat) {
	r, dr := b.run, b.dr
	other := r.pairs[i][k].other
	below := dr.pairDepth(i, other)
	mine := dr.pathOf(i, below)
	var theirs linkBound
	if other >= 0 {
		theirs = dr.pathOf(other, below)
	}
	side := func(m int, bound linkBound, hull bool) (projectionSide, bool) {
		h := make([]*big.Rat, 0, len(bound.Path)-below)
		for _, j := range bound.Path[below:] {
			jt := dr.spec.joints[j]
			var span *big.Rat
			if jt.dep == nil {
				span = jointParam(jt, c.lo[j]).SpanUpper(jointParam(jt, c.hi[j]))
				span.Quo(span, big.NewRat(2, 1))
			} else {
				reach, ok := c.depReach[j]
				if !ok {
					return projectionSide{}, false
				}
				span = new(big.Rat).Set(reach.Half)
			}
			half := proofarith.FloatRat(proofbound.RatFloatUp(span))
			if half == nil {
				return projectionSide{}, false
			}
			h = append(h, half)
		}
		key := cellReadingKey{mover: m, below: below, hull: hull}
		corners, ok := readings[key]
		if !ok {
			points, okPoints := bodyPoints(r.movers[m].body, hull)
			if !okPoints {
				return projectionSide{}, false
			}
			params := make([]motionbound.MotionParam, len(dr.spec.joints))
			for _, j := range bound.Path[below:] {
				if dr.spec.joints[j].dep != nil {
					// A dependent is read at its centre, the midpoint of its
					// enclosure at the cell's centre (§5.8).
					params[j] = motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Set(c.depReach[j].Centre)}
					continue
				}
				params[j] = jointParam(dr.spec.joints[j], c.centre(j))
			}
			if corners, ok = linkagebound.RoundCorners(readPoints(dr.spec, dr.frames, params, bound, below, points)); !ok {
				return projectionSide{}, false
			}
			readings[key] = corners
		}
		return projectionSide{Corners: corners, H: h, Rem: linkagebound.Remainder(bound.Rho[below:], h)}, true
	}
	partner := func(hull bool) (projectionSide, bool) {
		if other >= 0 {
			return side(other, theirs, hull)
		}
		corners, ok := b.staticReading(k, hull)
		return projectionSide{Corners: corners}, ok
	}
	axisOf := func(joint int) int {
		if ld := dr.spec.joints[joint].dep; ld != nil {
			return ld.driver
		}
		return joint
	}
	a, ok := side(i, mine, false)
	if !ok {
		return nil, nil
	}
	p, ok := partner(false)
	if !ok {
		return nil, nil
	}
	bound := linkagebound.Lower(a, p)
	shares := make(map[int]*big.Rat)
	axis, sense := linkagebound.AttainedDirection(a, p, bound)
	linkagebound.AddAxisShares(shares, a, mine.Rho[below:], linkagebound.ProjectionAxes(mine.Path, below, axisOf), axis, sense)
	if other >= 0 {
		linkagebound.AddAxisShares(shares, p, theirs.Rho[below:], linkagebound.ProjectionAxes(theirs.Path, below, axisOf), axis, -sense)
	}
	// The hull bound (§5.8): the same expansion over each body's hull points
	// along every candidate direction, the larger of the two serving.
	ah, okA := side(i, mine, true)
	ph, okP := partner(true)
	if !okA || !okP {
		return bound, shares
	}
	witness := linkagebound.LowerHullWithWitness(ah, ph)
	hull := witness.Bound
	if hull == nil || hull.Cmp(bound) <= 0 {
		return bound, shares
	}
	shares = make(map[int]*big.Rat)
	linkagebound.AddHullShares(shares, ah, mine.Rho[below:], linkagebound.ProjectionAxes(mine.Path, below, axisOf),
		witness.Axis, witness.Norm, witness.Sense)
	if other >= 0 {
		linkagebound.AddHullShares(shares, ph, theirs.Rho[below:], linkagebound.ProjectionAxes(theirs.Path, below, axisOf),
			witness.Axis, witness.Norm, -witness.Sense)
	}
	return hull, shares
}

// cellReadingKey names one corner reading at a cell's centre: a mover's
// body under the joints on its path from position below on, read at its box
// corners or at its hull points (bodyHullPoints).
type cellReadingKey struct {
	mover, below int
	hull         bool
}

// staticReading is static body k's point reading — its hull points when hull
// is set and it has them, else its box corners — rounded outward, read once
// per call and kept.
func (b *boxRun) staticReading(k int, hull bool) (cornerBounds, bool) {
	key := cellReadingKey{mover: k, below: -1, hull: hull}
	if got, ok := b.statics[key]; ok {
		return got, true
	}
	points, ok := bodyPoints(b.run.statics[k].body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.RoundCorners(points)
	if !ok {
		return cornerBounds{}, false
	}
	if b.statics == nil {
		b.statics = make(map[cellReadingKey]cornerBounds)
	}
	b.statics[key] = reading
	return reading, true
}

// pairTerms is every joint's share of pair k of mover i's travel across cell
// c: against a static body, every joint on the mover's path; against a body
// of another link, the joints strictly below the two links' lowest common
// ancestor on each branch, each with its own body's ρ. A joint's span is
// motionbound.MotionParam.SpanUpper of its exact values at the cell's two
// ends; a joint that does not vary spans 0.
func (b *boxRun) pairTerms(c *boxCell, i, k int) []linkagebound.JointTerm {
	mine, theirs := b.branchTerms(c, i, k)
	return append(mine, theirs...)
}

// branchTerms is pairTerms split by body: the terms that move mover i's own
// body, and those that move its partner's — none for a static partner. A
// body symmetric about its own joint's axis takes its path without that
// joint (linkageDriver.pathOf, docs/linkage-check-design.md §5.2).
//
// A loop's dependent joint j (docs/linkage-check-design.md §16.3) moves at
// most δ_j from its exact value at the centre to its value anywhere in the
// cell, the whole one-sided distance and not half the hull, so its share is
// 2·w_j·δ_j — twice its part of τ_half, as a stated joint's w_i·span_i is —
// charged to its loop's driver, whose halving shrinks the hull. Only a cell
// with every δ_j read reaches here: a gated cell has no pair travel.
func (b *boxRun) branchTerms(c *boxCell, i, k int) (mine, theirs []linkagebound.JointTerm) {
	r, dr := b.run, b.dr
	if r.pairs[i][k].once != nil {
		// A constant pair (docs/linkage-check-design.md §5.9) moves nothing
		// relative to its partner: no joint carries a term.
		return nil, nil
	}
	span := func(joint int) (int, *big.Rat) {
		jt := dr.spec.joints[joint]
		if jt.dep != nil {
			return jt.dep.driver, new(big.Rat).Mul(c.delta[joint], big.NewRat(2, 1))
		}
		return joint, jointParam(jt, c.lo[joint]).SpanUpper(jointParam(jt, c.hi[joint]))
	}
	other := r.pairs[i][k].other
	below := dr.pairDepth(i, other)
	if other < 0 {
		return linkagebound.TermsOnPath(dr.pathOf(i, below), below, span), nil
	}
	return linkagebound.TermsOnPath(dr.pathOf(i, below), below, span),
		linkagebound.TermsOnPath(dr.pathOf(other, below), below, span)
}

// halfTravel is τ_half(C) of docs/linkage-check-design.md §14.3 for pair k of
// mover i: half the sum of its joint terms, a proven upper bound on how far
// the pair's distance changes from the cell's centre to any configuration of
// the cell, since no joint lies farther than half its span from the centre.
func (b *boxRun) halfTravel(c *boxCell, i, k int) *big.Rat {
	return linkagebound.HalfSum(b.pairTerms(c, i, k))
}

// blocks reports whether pair k of mover i, which collides at cell c's centre
// with the published volume, overlaps at every configuration of the cell
// (docs/linkage-check-design.md §14.3, the blocked certificate). Along the
// straight joint-space segment from the centre to any configuration of the
// cell a body moves at most its own τ_half, and the overlap volume changes at
// a rate no larger than each body's surface area times its speed, so it loses
// at most SweptVolumeAllow(τ_half, A) per moving body. The cell is blocked
// when the volume's proven lower end, Value − Bound rounded down, strictly
// exceeds that sum, compared over exact rationals. A is the mover's proven
// upper bound on its area at rest (motionMover.area), which a rigid motion
// preserves; a static partner moves nothing and is charged nothing.
func (b *boxRun) blocks(c *boxCell, i, k int, volume Measurement) bool {
	r := b.run
	value, bound := proofarith.FloatRat(volume.Value.Base()), proofarith.FloatRat(volume.Bound.Base())
	if value == nil || bound == nil {
		return false
	}
	lower := proofbound.RatFloatDown(new(big.Rat).Sub(value, bound))
	mine, theirs := b.branchTerms(c, i, k)
	allow := proofbound.SweptVolumeAllow(proofbound.RatFloatUp(linkagebound.HalfSum(mine)), r.movers[i].area)
	if other := r.pairs[i][k].other; other >= 0 {
		allow = proofbound.AbsSumUpper(allow, proofbound.SweptVolumeAllow(proofbound.RatFloatUp(linkagebound.HalfSum(theirs)), r.movers[other].area))
	}
	if proofbound.IsNonFinite(allow) || proofbound.IsNonFinite(lower) {
		return false
	}
	return proofarith.FloatRat(lower).Cmp(proofarith.FloatRat(allow)) > 0
}

// splitAxis is the joint a cell splits along (docs/linkage-check-design.md
// §14.4 step 5), or −1 when the cell is not splittable: a cell splits only
// when it is undecided by pairs a split can help, or colliding, and some
// varying joint's span is wider than the resolution. The axis is the varying
// joint, among those wider than the resolution, with the largest share of
// the defect of the bound of any pair that held the cell back — w_i·span_i
// of its travel bound, or its projection bound's own share where that bound
// is the larger (boundShares) — ties to the earliest link.
//
// A gated cell (docs/linkage-check-design.md §16.4) has no pair travel to
// rank: it splits along the refusing loop's driver, the one axis a split
// changes, while that axis is wider than the resolution and its range meets
// some stretch the decomposition certified; otherwise it is stuck.
func (b *boxRun) splitAxis(c *boxCell) int {
	switch {
	case c.outcome == CellColliding:
	case c.outcome == CellUndecided && !c.stuck:
	default:
		return -1
	}
	if c.gate != "" {
		ld := c.gateLoop
		if ld == nil || ld.held || !b.wide(c, ld.driver) || !ld.meetsCertified(c.lo[ld.driver], c.hi[ld.driver]) {
			return -1
		}
		return ld.driver
	}
	best := make(map[int]*big.Rat, len(b.axes))
	projected := false
	for _, pair := range c.held {
		shares, proj := b.boundShares(c, pair[0], pair[1])
		projected = projected || proj
		for joint, v := range shares {
			if cur, ok := best[joint]; !ok || v.Cmp(cur) > 0 {
				best[joint] = v
			}
		}
	}
	return linkagebound.RankAxes(b.axes, best, projected, func(k int) bool { return b.wide(c, k) })
}

// wide reports whether cell c's span along joint k, as a fraction of the
// joint's range, is wider than the resolution.
func (b *boxRun) wide(c *boxCell, k int) bool {
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.cfg.ResolutionP.Base) > 0
}

// wideForReading reports whether cell c's span along joint k is wider than
// the reading floor: its own when unstated, else the resolution.
func (b *boxRun) wideForReading(c *boxCell, k int) bool {
	if b.run.cfg.ReadingP == nil {
		return b.wide(c, k)
	}
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.cfg.ReadingP.Base) > 0
}

// configuration is the joint configuration a cell's centre was evaluated at:
// the zero JointConfiguration for an unbuildable centre.
func (c *boxCell) configuration() JointConfiguration {
	if c.pose.unbuildable != nil {
		return JointConfiguration{}
	}
	n := len(c.pose.groups)
	conf := JointConfiguration{Values: make([]units.Value, n), Bounds: make([]units.Value, n), Poses: make([]r3.Transform, n)}
	for k, g := range c.pose.groups {
		conf.Values[k], conf.Bounds[k], conf.Poses[k] = g.value, g.bound, g.pose
	}
	return conf
}

// publish assembles VerifyJointBox's report (docs/linkage-check-design.md
// §14.2, §14.4 step 8).
func (b *boxRun) publish(l *Linkage, box JointBox) *JointBoxReport {
	r := b.run
	report := &JointBoxReport{
		Request: JointBoxRequest{
			RelativeTolerance: units.Scalar(r.cfg.Rel),
			Resolution:        r.cfg.Resolution,
			MinClearance:      r.cfg.Minimum,
			CellBudget:        b.budget,
		},
		ReadingResolution: readingResolution(r.cfg),
		Linkage:           l,
		Box:               slices.Clone(box),
		Links:             l.Links(),
		Against:           []*Body{},
		JointContacts:     l.JointContacts(),
		Cells:             make([]JointCellResult, 0, len(b.leaves)),
		CellsEvaluated:    len(b.evaluated),
		Collisions:        []JointBoxCollision{},
		Diagnostics:       []Diagnostic{},
	}
	for _, st := range r.statics {
		report.Against = append(report.Against, st.body)
	}
	poses := make([]*motionPose, len(b.evaluated))
	violated := false
	for n, c := range b.evaluated {
		poses[n] = c.pose
		violated = violated || c.pose.violated
		for _, hit := range c.pose.collisions {
			report.Collisions = append(report.Collisions, JointBoxCollision{Configuration: c.configuration(), A: hit.Moving, B: hit.Static, Volume: hit.Volume})
		}
	}
	allClear, met := true, true
	var lowest *Measurement
	for _, c := range b.leaves {
		res := JointCellResult{
			Cell:          cloneJointCell(c.cell),
			Outcome:       c.outcome,
			Center:        c.configuration(),
			Interferences: c.pose.result.Interferences,
			Clearances:    c.pose.result.Clearances,
			Diagnostics:   slices.Clone(c.pose.result.Diagnostics),
			Clearance:     c.clearance,
		}
		var own []Diagnostic
		switch {
		case c.outcome != CellClear:
			allClear, met = false, false
			if c.outcome == CellUndecided {
				msg := fmt.Sprintf("the joint cell %s is neither certified clear nor bounded by a proven collision", formatCell(c.cell))
				if c.gate != "" {
					msg += ": " + c.gate
				}
				own = append(own, b.cellFinding(c, Diagnostic{
					Code:    DiagMotionUndecidedInterval,
					Status:  Suspect,
					Reading: ReadingNone,
					Message: msg,
				}))
			}
		case r.cfg.MinimumMM != nil && !r.meetsMinimum(c.clearance):
			met = false
			if violated {
				break
			}
			obs := *c.clearance
			own = append(own, b.cellFinding(c, Diagnostic{
				Code:     DiagMotionUndecidedClearance,
				Status:   Suspect,
				Reading:  ReadingGap,
				Observed: &obs,
				Required: r.cfg.Minimum,
				Message:  fmt.Sprintf("the joint cell %s is certified clear, but its proven lower bound does not reach the required minimum", formatCell(c.cell)),
			}))
		}
		if c.clearance != nil && (lowest == nil || c.clearance.Value.Base() < lowest.Value.Base()) {
			lowest = c.clearance
		}
		res.Diagnostics = append(res.Diagnostics, own...)
		report.Cells = append(report.Cells, res)
		report.Diagnostics = append(append(report.Diagnostics, c.pose.findings...), own...)
	}
	// A split cell's centre is gone from Cells, but a collision or a margin
	// violation proven there is a fact the caller asked for.
	for _, c := range b.evaluated {
		if !c.split {
			continue
		}
		for _, diag := range c.pose.findings {
			if diag.Code == DiagMotionCollision || diag.Code == DiagMotionClearanceViolated {
				report.Diagnostics = append(report.Diagnostics, diag)
			}
		}
	}
	if b.exhausted {
		report.Diagnostics = append(report.Diagnostics, Diagnostic{
			Code:    DiagJointBoxBudgetExhausted,
			Status:  Suspect,
			Reading: ReadingNone,
			Message: fmt.Sprintf("the cell budget of %d centres stopped the subdivision with %d cells left unsplit that its floors would still have split", b.budget, b.unsplit),
		})
	}
	switch {
	case r.cfg.MinimumMM == nil:
		report.Assessment = AssessmentNotEvaluated
	case violated:
		report.Assessment = AssessmentViolated
	case met:
		report.Assessment = AssessmentMet
	default:
		report.Assessment = AssessmentUndecided
	}
	if allClear && lowest != nil {
		if reading, diag := r.pathClearance(poses, lowest, "whole-box"); reading != nil {
			report.Clearance = reading
			if diag != nil {
				report.Diagnostics = append(report.Diagnostics, *diag)
			}
		}
	}
	report.Status = reportvocab.WorstStatus(diagnosticsToInternal(report.Diagnostics))
	return report
}

// cellFinding marks a cell's own finding with the cell.
func (b *boxRun) cellFinding(c *boxCell, diag Diagnostic) Diagnostic {
	cell := cloneJointCell(c.cell)
	diag.Cell = &cell
	return diag
}

// formatValues renders joint values for a finding's message.
func formatValues(values []units.Value) string {
	parts := make([]string, len(values))
	for k, v := range values {
		parts[k] = v.String()
	}
	return "(" + strings.Join(parts, ", ") + ")"
}

// formatCell renders a cell for a finding's message, one range per link.
func formatCell(c JointCell) string {
	parts := make([]string, len(c.Min))
	for k := range c.Min {
		parts[k] = fmt.Sprintf("[%s, %s]", c.Min[k], c.Max[k])
	}
	return strings.Join(parts, " × ")
}

// errLoopNotStated is the refusal of a looped linkage where every joint's
// value is the caller's to state (docs/linkage-check-design.md §16.1).
func errLoopNotStated() error {
	return fmt.Errorf(`%w: a linkage with a closed loop has dependent joints whose values follow from its driver, so it cannot be posed joint by joint; pose a drive whose every sweep holds instead`, ErrUnsupported)
}
