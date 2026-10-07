package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/motionbound"

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
// bounds linkage_bound.go proves over the whole box.

// JointBox is a box of joint values (docs/linkage-check-design.md §14.1). A
// listed joint ranges over [Min, Max] when Min < Max and holds at Min when
// Min == Max; an unlisted joint holds 0.
type JointBox []JointRange

// JointRange is one link's joint range in a JointBox.
type JointRange struct {
	Link     *Link       // the link whose joint this ranges
	Min, Max units.Value // the joint's Kind: Angle for a revolute, Length for a prismatic
}

type jointBoxOption struct{ option.Interface }

func (jointBoxOption) jointBoxOption() {}

type identCellBudget struct{}

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
	return jointBoxOption{option.New(identCellBudget{}, cells)}
}

// JointConfiguration is one point of joint space: every link's joint value
// and world pose, in Linkage.Links() order.
type JointConfiguration struct {
	Values []units.Value
	Poses  []r3.Transform
}

// Configuration builds every link's world pose at the stated joint values,
// one per link in Links() order (docs/linkage-check-design.md §14.1). It
// composes exactly as PoseAt composes — PoseAt is Configuration of a drive's
// values at s — so a renderer drawing a cell and VerifyJointBox evaluating its
// centre read the same transform.
//
// It refuses a nil linkage or a value count other than len(Links())
// (ErrDegenerate); a value of the wrong Kind for its joint (ErrUnitKind); a
// non-finite value (ErrNotFinite); a value outside its joint's declared
// limits (ErrDegenerate, naming the link); and a pose r3 cannot represent
// (ErrNotFinite).
func (l *Linkage) Configuration(values []units.Value) (JointConfiguration, error) {
	if l == nil {
		return JointConfiguration{}, fmt.Errorf(`%w: a nil linkage has no link to pose`, ErrDegenerate)
	}
	if len(values) != len(l.links) {
		return JointConfiguration{}, fmt.Errorf(`%w: a configuration names one value per link, %d here, got %d`, ErrDegenerate, len(l.links), len(values))
	}
	spec := l.restSpec()
	for k, v := range values {
		jt := spec.joints[k]
		if err := motionValueValid(v, jt.kind, "a joint value"); err != nil {
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
	return JointConfiguration{Values: slices.Clone(values), Poses: poses}, nil
}

// JointBoxReport is what VerifyJointBox returns
// (docs/linkage-check-design.md §14.2). Diagnostics lists, per leaf cell in
// cell order, its centre's findings then its own; then the collision and
// margin-violation findings of the centres of cells since split; then the
// budget's finding; then the whole-box reading's. It is empty exactly when
// Status is Sound, and Status is the worst Diagnostic.Status in it.
type JointBoxReport struct {
	Request JointBoxRequest // the validated effective settings, including defaults
	// ReadingResolution is the floor the whole-box Clearance reading refines
	// to, per axis: Request.Resolution when WithResolution was stated, and
	// units.Scalar(1.0/16384) otherwise, while the verdict stops at
	// Request.Resolution (docs/linkage-check-design.md §14.1).
	ReadingResolution units.Value
	Linkage           *Linkage            // the linkage as given
	Box               JointBox            // the box as stated
	Links             []*Link             // Linkage.Links() order
	Against           []*Body             // every static body, in Document.Bodies() order
	JointContacts     []DiagnosticPair    // every declared joint contact, in declaration order
	Cells             []JointCellResult   // the leaves of the subdivision, in cell order; they tile the box
	CellsEvaluated    int                 // every centre evaluated, split cells included
	Collisions        []JointBoxCollision // every proven collision at every evaluated centre, in evaluation order then pair order
	Clearance         *ScalarReading      // the minimum gap over the whole box; nil unless every cell is CellClear
	Assessment        Assessment          // against WithMinClearance; AssessmentNotEvaluated when not requested
	Diagnostics       []Diagnostic        // per leaf its centre's findings then its own; then split centres'; then the budget's and the reading's
	Status            Status              // Unverified on a zero value; VerifyJointBox always returns a decided status
}

// Passed reports whether the report is Sound. It returns false for a nil
// report and for any other Status.
func (r *JointBoxReport) Passed() bool {
	return r != nil && r.Status == Sound
}

// JointBoxRequest is one VerifyJointBox call's effective settings, each value
// as the caller stated it or as the default was formed.
type JointBoxRequest struct {
	RelativeTolerance units.Value  // always present
	Resolution        units.Value  // Dimensionless: the finest cell, as a fraction of each varying joint's range
	MinClearance      *units.Value // non-nil exactly when WithMinClearance was requested
	CellBudget        int          // the most centres the check evaluates
}

// JointCell is a box of joint values inside a stated JointBox: per link, in
// Links() order, the least and greatest value the cell holds. A held or
// unlisted joint has Min == Max.
type JointCell struct {
	Min, Max []units.Value
}

func (c JointCell) clone() JointCell {
	return JointCell{Min: slices.Clone(c.Min), Max: slices.Clone(c.Max)}
}

// JointCellResult is one leaf cell of the subdivision and what its centre
// proves. In every row and diagnostic, A is a link body and B a static body
// or a body of a later link (docs/linkage-check-design.md §4's pair order).
// A Clearance row is a measurement at the centre only; the claim over the
// whole cell is Outcome and Clearance.
type JointCellResult struct {
	Cell          JointCell
	Outcome       CellOutcome
	Center        JointConfiguration // the evaluated centre
	Interferences []Interference     // at the centre; proven overlap, bounded volume
	Clearances    []Clearance        // at the centre; every pair proven disjoint or touching
	Diagnostics   []Diagnostic       // the centre's undecided or unsupported pairs and invalid bodies, then the cell's own finding
	// Clearance is a PROVEN LOWER BOUND on every undeclared evaluated pair's
	// gap over the whole closed cell, set only on a CellClear cell that holds
	// at least one such pair. It reads Approximate with a zero Bound: the
	// number is the claim itself.
	Clearance *Measurement
}

// CellOutcome is what a JointCellResult proves (docs/linkage-check-design.md
// §14.2).
type CellOutcome int

const (
	// CellNotEvaluated is the reserved zero value: VerifyJointBox never
	// returns it.
	CellNotEvaluated CellOutcome = iota
	// CellClear — at EVERY configuration of the cell, every undeclared
	// evaluated pair has disjoint interiors at a proven positive gap, and
	// every declared pair is free of transferred overlap at the centre.
	CellClear
	// CellBlocked — at EVERY configuration of the cell some pair overlaps:
	// a collision at the centre whose proven volume exceeds what the bodies'
	// travel across the cell can sweep away (docs/linkage-check-design.md
	// §14.3). The centre's collisions are its witnesses.
	CellBlocked
	// CellColliding — a proven collision sits at the centre; nothing is
	// claimed about the rest of the cell.
	CellColliding
	// CellUndecided — none of the above. It claims nothing.
	CellUndecided
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "cell_outcome(<n>)", never a panic.
func (o CellOutcome) String() string {
	switch o {
	case CellNotEvaluated:
		return tokenNotEvaluated
	case CellClear:
		return "clear"
	case CellBlocked:
		return "blocked"
	case CellColliding:
		return "colliding"
	case CellUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("cell_outcome(%d)", int(o))
	}
}

// JointBoxCollision is a proven overlap at an evaluated centre, about the
// ideal poses (docs/linkage-check-design.md §5.1), with the configuration as
// its witness: Volume.Value − Volume.Bound is a proven lower bound on the
// ideal overlap there. A belongs to a link; B is a static body or a body of a
// later link. Nothing is claimed about the cell around it.
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
// The options are VerifyLinkage's (WithMotionTolerance, WithResolution,
// WithMinClearance) plus WithCellBudget. Every body of every link, and every
// body a declared joint contact names, MUST be a live body of d. A nil
// context, document or linkage, a linkage with no link, a box with no varying
// joint, a range with Min > Max, a link named twice, a range end outside a
// joint's declared limits and a budget below 1 are ErrDegenerate. Validation
// precedes cancellation; after it a canceled context returns ctx.Err() and no
// report.
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
		if err := motionFinite(rg.Min, rg.Max); err != nil {
			return nil, nil, err
		}
		pMin, okMin := motionbound.ExactMotionParam(rg.Min)
		pMax, okMax := motionbound.ExactMotionParam(rg.Max)
		if !okMin || !okMax {
			return nil, nil, fmt.Errorf(`%w: a joint range's end is not representable`, ErrNotFinite)
		}
		c, ok := paramCompare(rg.Min, rg.Max)
		if !ok || c > 0 {
			return nil, nil, fmt.Errorf(`%w: link %d's range needs Min <= Max, got %s and %s`, ErrDegenerate, link.index, rg.Min, rg.Max)
		}
		jt.listed, jt.values, jt.points = true, []units.Value{rg.Min, rg.Max}, []motionbound.MotionParam{pMin, pMax}
		varying[link.index] = c < 0
	}
	// Both ends inside a joint's limits put the whole range inside them; an
	// unlisted joint holds 0.
	for k, jt := range spec.joints {
		if jt.limits == nil {
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
		if _, ok := o.Ident().(identCellBudget); ok {
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
	cfg, err := resolveMotionOptions(motion, motionSpec{motionDomain: fractionDomain()})
	if err != nil {
		return motionConfig{}, 0, err
	}
	if !cfg.stated {
		// Unstated, the reading refines past the verdict floor to its own,
		// per axis, as a drive's does (docs/linkage-check-design.md §3, §14.1).
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.readingP = &reading
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
	if err := b.evaluate(root); err != nil {
		return err
	}
	b.leaves = []*boxCell{root}
	for {
		if err := b.splitForVerdict(); err != nil {
			return err
		}
		if b.exhausted {
			break
		}
		n, axis := b.nextReadingSplit()
		if n < 0 {
			break
		}
		if len(b.evaluated)+2 > b.budget {
			b.exhausted = true
			b.unsplit++
			break
		}
		lower, upper, err := b.split(b.leaves[n], axis)
		if err != nil {
			return err
		}
		b.leaves = slices.Insert(slices.Delete(b.leaves, n, n+1), n, lower, upper)
	}
	return b.run.ctx.Err()
}

// splitForVerdict is §14.4 step 5: level by level from the shallowest
// splittable cell, each splittable cell of the level in cell order replaced in
// place by its two halves, lower half first, until no cell is splittable or
// the budget stops the split.
func (b *boxRun) splitForVerdict() error {
	for {
		level := -1
		for _, c := range b.leaves {
			if b.splitAxis(c) >= 0 && (level < 0 || c.depth < level) {
				level = c.depth
			}
		}
		if level < 0 {
			return nil
		}
		next := make([]*boxCell, 0, len(b.leaves))
		for _, c := range b.leaves {
			axis := b.splitAxis(c)
			if c.depth != level || axis < 0 {
				next = append(next, c)
				continue
			}
			if b.exhausted || len(b.evaluated)+2 > b.budget {
				b.exhausted = true
				b.unsplit++
				next = append(next, c)
				continue
			}
			lower, upper, err := b.split(c, axis)
			if err != nil {
				return err
			}
			next = append(next, lower, upper)
		}
		b.leaves = next
		if b.exhausted {
			return nil
		}
	}
}

// nextReadingSplit is §14.4 step 6: the leaf to split for the readings and
// the axis to split it along, or −1. The leaf is the clear cell holding the
// smallest lower bound, ties in cell order. It splits while every leaf is
// clear and the whole-box reading fails the tolerance gate, along an axis
// wider than the reading floor; and while a requested margin is neither proven
// by that bound nor disproven by some centre, along an axis wider than the
// verdict floor. The axis is the one with the largest share of the travel of
// the pair that attained the bound.
func (b *boxRun) nextReadingSplit() (int, int) {
	r := b.run
	allClear, smallest := true, -1
	for n, c := range b.leaves {
		if c.outcome != CellClear {
			allClear = false
			continue
		}
		if c.clearance != nil && (smallest < 0 || c.clearance.Value.Base() < b.leaves[smallest].clearance.Value.Base()) {
			smallest = n
		}
	}
	if smallest < 0 || b.leaves[smallest].low == nil {
		return -1, -1
	}
	c := b.leaves[smallest]
	if allClear {
		if axis := b.readingAxis(c, b.wideForReading); axis >= 0 {
			if reading, _ := r.pathClearance(b.upperPoses(), c.clearance, "whole-box"); reading != nil && reading.Tolerance.State != ToleranceSatisfied {
				return smallest, axis
			}
		}
	}
	if r.cfg.minimumMM != nil && !b.anyViolated && !r.meetsMinimum(c.clearance) {
		if axis := b.readingAxis(c, b.wide); axis >= 0 {
			return smallest, axis
		}
	}
	return -1, -1
}

// readingAxis is the varying joint, among those wide reports wider than its
// floor, with the largest share of the travel of the pair that attained a
// clear cell's bound; ties go to the earliest link, −1 when none is wide.
func (b *boxRun) readingAxis(c *boxCell, wide func(*boxCell, int) bool) int {
	share := make(map[int]*big.Rat, len(b.axes))
	for _, t := range b.pairTerms(c, c.low[0], c.low[1]) {
		share[t.joint] = t.value
	}
	axis := -1
	var top *big.Rat
	for _, k := range b.axes {
		if !wide(c, k) {
			continue
		}
		v := share[k]
		if v == nil {
			v = new(big.Rat)
		}
		if axis < 0 || v.Cmp(top) > 0 {
			axis, top = k, v
		}
	}
	return axis
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
	values := make([]units.Value, len(spec.joints))
	params := make([]motionbound.MotionParam, len(spec.joints))
	c.cell = JointCell{Min: make([]units.Value, len(spec.joints)), Max: make([]units.Value, len(spec.joints))}
	for k, jt := range spec.joints {
		f := c.centre(k)
		values[k], params[k] = jt.label(f), jointParam(jt, f)
		c.cell.Min[k], c.cell.Max[k] = jt.label(c.lo[k]), jt.label(c.hi[k])
	}
	poses, err := spec.posesOf(values)
	if err != nil {
		return err
	}
	ideals := idealPosesOf(spec, b.dr.frames, params)
	groups := make([]motionGroupPose, len(poses))
	for k := range poses {
		groups[k] = motionGroupPose{
			pose: poses[k], ideals: []motionbound.IdealPose{ideals[k]}, stretch: 1, value: values[k],
			fixed: b.dr.standing[k] == linkFixed, constant: b.dr.standing[k] == linkConstant,
		}
	}
	mp := r.newPose(groups, units.Value{}, "the configuration "+formatValues(values))
	mp.cell = &c.cell
	if err := r.runPairs(mp); err != nil {
		return err
	}
	c.pose = mp
	b.evaluated = append(b.evaluated, c)
	b.note(mp)
	b.classify(c)
	return nil
}

// classify sets a cell's outcome from its evaluated centre.
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
	lowest, ok := r.certifyPairs(func(i, k int) *big.Rat {
		pp := mp.pairs[i][k]
		if !pp.hasGap {
			return nil
		}
		lo, tau := proofarith.FloatRat(pp.lo), b.halfTravel(c, i, k)
		if lo.Cmp(tau) <= 0 {
			return nil
		}
		bound := lo.Sub(lo, tau)
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
	c.outcome, c.clearance = CellClear, lowerBoundMeasurement(lowest)
	if lowBound != nil && lowBound.Cmp(lowest) == 0 {
		c.low = low
	}
}

// jointTerm is one joint's share w_i·span_i(C) of a pair's travel over a
// cell (docs/linkage-check-design.md §14.3).
type jointTerm struct {
	joint int
	value *big.Rat
}

// pairTerms is every joint's share of pair k of mover i's travel across cell
// c: against a static body, every joint on the mover's path; against a body
// of another link, the joints strictly below the two links' lowest common
// ancestor on each branch, each with its own body's ρ. A joint's span is
// motionbound.MotionParam.SpanUpper of its exact values at the cell's two
// ends; a joint that does not vary spans 0.
func (b *boxRun) pairTerms(c *boxCell, i, k int) []jointTerm {
	mine, theirs := b.branchTerms(c, i, k)
	return append(mine, theirs...)
}

// branchTerms is pairTerms split by body: the terms that move mover i's own
// body, and those that move its partner's — none for a static partner.
func (b *boxRun) branchTerms(c *boxCell, i, k int) (mine, theirs []jointTerm) {
	r, dr := b.run, b.dr
	span := func(joint int) *big.Rat {
		jt := dr.spec.joints[joint]
		return jointParam(jt, c.lo[joint]).SpanUpper(jointParam(jt, c.hi[joint]))
	}
	terms := func(bound linkBound, below int, out []jointTerm) []jointTerm {
		for n := below; n < len(bound.path); n++ {
			term := span(bound.path[n])
			if bound.rho[n] != nil {
				term.Mul(term, bound.rho[n])
			}
			out = append(out, jointTerm{joint: bound.path[n], value: term})
		}
		return out
	}
	own := dr.bounds[r.movers[i].group]
	other := r.pairs[i][k].other
	if other < 0 {
		return terms(own, 0, nil), nil
	}
	partner := dr.bounds[r.movers[other].group]
	below := commonDepth(own.path, partner.path)
	return terms(own, below, nil), terms(partner, below, nil)
}

// halfSum is half the sum of a body's joint terms: its τ_half.
func halfSum(terms []jointTerm) *big.Rat {
	sum := new(big.Rat)
	for _, t := range terms {
		sum.Add(sum, t.value)
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}

// halfTravel is τ_half(C) of docs/linkage-check-design.md §14.3 for pair k of
// mover i: half the sum of its joint terms, a proven upper bound on how far
// the pair's distance changes from the cell's centre to any configuration of
// the cell, since no joint lies farther than half its span from the centre.
func (b *boxRun) halfTravel(c *boxCell, i, k int) *big.Rat {
	return halfSum(b.pairTerms(c, i, k))
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
	allow := proofbound.SweptVolumeAllow(proofbound.RatFloatUp(halfSum(mine)), r.movers[i].area)
	if other := r.pairs[i][k].other; other >= 0 {
		allow = proofbound.AbsSumUpper(allow, proofbound.SweptVolumeAllow(proofbound.RatFloatUp(halfSum(theirs)), r.movers[other].area))
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
// joint, among those wider than the resolution, with the largest share
// w_i·span_i of the travel of any pair that held the cell back; ties go to
// the earliest link.
func (b *boxRun) splitAxis(c *boxCell) int {
	switch {
	case c.outcome == CellColliding:
	case c.outcome == CellUndecided && !c.stuck:
	default:
		return -1
	}
	best := make(map[int]*big.Rat, len(b.axes))
	for _, pair := range c.held {
		for _, t := range b.pairTerms(c, pair[0], pair[1]) {
			if cur, ok := best[t.joint]; !ok || t.value.Cmp(cur) > 0 {
				best[t.joint] = t.value
			}
		}
	}
	axis := -1
	var top *big.Rat
	for _, k := range b.axes {
		if !b.wide(c, k) {
			continue
		}
		v := best[k]
		if v == nil {
			v = new(big.Rat)
		}
		if axis < 0 || v.Cmp(top) > 0 {
			axis, top = k, v
		}
	}
	return axis
}

// wide reports whether cell c's span along joint k, as a fraction of the
// joint's range, is wider than the resolution.
func (b *boxRun) wide(c *boxCell, k int) bool {
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.cfg.resolutionP.Base) > 0
}

// wideForReading reports whether cell c's span along joint k is wider than
// the reading floor: its own when unstated, else the resolution.
func (b *boxRun) wideForReading(c *boxCell, k int) bool {
	if b.run.cfg.readingP == nil {
		return b.wide(c, k)
	}
	width := new(big.Rat).Sub(c.hi[k], c.lo[k])
	return width.Cmp(b.run.cfg.readingP.Base) > 0
}

// configuration is the joint configuration a cell's centre was evaluated at.
func (c *boxCell) configuration() JointConfiguration {
	conf := JointConfiguration{Values: make([]units.Value, len(c.pose.groups)), Poses: make([]r3.Transform, len(c.pose.groups))}
	for k, g := range c.pose.groups {
		conf.Values[k], conf.Poses[k] = g.value, g.pose
	}
	return conf
}

// publish assembles VerifyJointBox's report (docs/linkage-check-design.md
// §14.2, §14.4 step 8).
func (b *boxRun) publish(l *Linkage, box JointBox) *JointBoxReport {
	r := b.run
	report := &JointBoxReport{
		Request: JointBoxRequest{
			RelativeTolerance: units.Scalar(r.cfg.rel),
			Resolution:        r.cfg.resolution,
			MinClearance:      r.cfg.minimum,
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
			Cell:          c.cell.clone(),
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
				own = append(own, b.cellFinding(c, Diagnostic{
					Code:    DiagMotionUndecidedInterval,
					Status:  Suspect,
					Reading: ReadingNone,
					Message: fmt.Sprintf("the joint cell %s is neither certified clear nor bounded by a proven collision", formatCell(c.cell)),
				}))
			}
		case r.cfg.minimumMM != nil && !r.meetsMinimum(c.clearance):
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
				Required: r.cfg.minimum,
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
	case r.cfg.minimumMM == nil:
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
	report.Status = worstStatus(report.Diagnostics)
	return report
}

// cellFinding marks a cell's own finding with the cell.
func (b *boxRun) cellFinding(c *boxCell, diag Diagnostic) Diagnostic {
	cell := c.cell.clone()
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
