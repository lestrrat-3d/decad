package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/linkage"
	"github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the root face of docs/linkage-check-design.md: the linkage
// model is the generic `linkage` package instantiated with *Body, and the
// two checks gate their document-bound refusals here before the engine runs.

type (
	// Linkage is a tree of links joined by joints, rooted at the ground link.
	Linkage = linkage.Linkage[*Body]
	// Link is one rigid set of bodies and the joint that attaches it to its parent.
	Link = linkage.Link[*Body]
	// Drive is a one-parameter motion of a linkage.
	Drive = linkage.Drive[*Body]
	// JointSweep moves one link's joint.
	JointSweep = linkage.JointSweep[*Body]
	// LinkageReport is VerifyLinkage's report.
	LinkageReport = linkage.LinkageReport[*Body]
	// LinkagePoseResult records one evaluated pose and its pair findings.
	LinkagePoseResult = linkage.LinkagePoseResult[*Body]
	// LinkCollision is a proven overlap at one ideal linkage pose.
	LinkCollision = linkage.LinkCollision[*Body]
	// LinkageLoop is a closed kinematic loop of a linkage.
	LinkageLoop = linkage.LinkageLoop[*Body]
	// LoopBar is one bar of a linkage loop.
	LoopBar = linkage.LoopBar[*Body]
	// Schedule is a drive whose loops are enclosed for posing.
	Schedule = linkage.Schedule[*Body]
	// JointBox is a box of joint values.
	JointBox = linkage.JointBox[*Body]
	// JointRange is one link's range of joint values.
	JointRange = linkage.JointRange[*Body]
	// JointBoxReport records the cell subdivision and verdict.
	JointBoxReport = linkage.JointBoxReport[*Body]
	// JointCellResult records one leaf cell and its centre's findings.
	JointCellResult = linkage.JointCellResult[*Body]
	// JointBoxCollision is a proven overlap at one evaluated centre.
	JointBoxCollision = linkage.JointBoxCollision[*Body]
)

type (
	// Joint is how a link moves against its parent.
	Joint = linkage.Joint
	// RevoluteJoint rotates its link about an axis.
	RevoluteJoint = linkage.RevoluteJoint
	// PrismaticJoint slides its link along a direction.
	PrismaticJoint = linkage.PrismaticJoint
	// JointLimits is a joint's declared working range.
	JointLimits = linkage.JointLimits
	// JointOption configures a joint built by Link.Revolute or Link.Prismatic.
	JointOption = linkage.JointOption
	// LinkagePose is every link's pose at one parameter value.
	LinkagePose = linkage.LinkagePose
	// JointConfiguration records every link's joint value and world pose.
	JointConfiguration = linkage.JointConfiguration
	// JointBoxRequest records the effective settings of a VerifyJointBox call.
	JointBoxRequest = linkage.JointBoxRequest
	// CellOutcome states what a JointCellResult proves.
	CellOutcome = linkage.CellOutcome
	// JointCell records one closed cell of joint values.
	JointCell = report.JointCell
)

const (
	CellNotEvaluated = linkage.CellNotEvaluated
	CellClear        = linkage.CellClear
	CellBlocked      = linkage.CellBlocked
	CellColliding    = linkage.CellColliding
	CellUndecided    = linkage.CellUndecided
)

// NewLinkage returns a linkage holding the ground link alone.
func NewLinkage() *Linkage { return linkage.NewLinkage[*Body]() }

// WithJointLimits declares the joint's working range [minimum, maximum], in
// the joint's Kind: an Angle for a revolute, a Length for a prismatic. A
// wrong Kind is ErrUnitKind, a non-finite bound ErrNotFinite, and
// minimum >= maximum ErrDegenerate.
func WithJointLimits(minimum, maximum units.Value) JointOption {
	return linkage.WithJointLimits(minimum, maximum)
}

// linkageHooks are the services a linkage check takes from d.
func (d *Document) linkageHooks() linkage.Hooks[*Body] {
	return linkage.Hooks[*Body]{
		Engine: motionHooks(), Doc: &d.n, Live: d.n.Live(), Node: bodyNode, Handle: bodyHandle,
	}
}

// VerifyLinkage checks whether any link of l, moved along drive, meets a
// static body or a body of another link anywhere on the drive
// (docs/linkage-check-design.md). It never changes the document: each pose
// re-evaluates every link body's payload under its link's pose as a
// transient body that is never committed, so Document.Bodies(), every body's
// live state and the next producer identity are what they were before the
// call.
//
// The options are VerifyMotion's own (WithMotionTolerance, WithResolution,
// WithMinClearance), with the resolution a Dimensionless fraction of the
// drive, default units.Scalar(1.0/1024). Every pair result at a pose is
// Verify's own row or diagnostic. Between two adjacent poses the interval
// certificate proves the drive clear only when every pair's two proven lower
// bounds together exceed the chain travel bound τ (§5.2), summed over the
// joints below the pair's lowest common ancestor, each joint's travel taken on
// both sides of every waypoint inside the interval; an interval it cannot
// certify down to the resolution is IntervalUndecided and the report reads
// Suspect, never Sound. A pose publishes a LinkCollision only for an overlap
// that survives the pose's own deviation from the ideal poses (§5.1).
//
// Declared joint contacts (Linkage.DeclareJointContact) are checked for
// proven overlap at every pose but enter no interval certificate; the report
// lists them in JointContacts.
//
// Every body of every link, and every body a declared joint contact names,
// MUST be a live body of d. A nil context, document or linkage, a linkage
// with no link, a drive in which every sweep holds, a drive whose sweeps carry
// different numbers of Via values, and a drive with a waypoint outside a
// joint's declared limits are decaderr.ErrDegenerate. Validation precedes
// cancellation; after it a canceled context returns ctx.Err() and no report.
func (d *Document) VerifyLinkage(ctx context.Context, l *Linkage, drive Drive, opts ...MotionOption) (*LinkageReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control linkage verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := d.requireLinkage(l); err != nil {
		return nil, err
	}
	return linkage.Verify(ctx, d.linkageHooks(), l, drive, func(dom motionbound.Domain) (motionoption.Config, error) {
		encodedOpts, err := decodeMotionOptions(opts)
		if err != nil {
			return motionoption.Config{}, err
		}
		return motionoption.Resolve(encodedOpts, dom)
	})
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
// decaderr.ErrDegenerate; a loop joint whose parent is not the loop's common link, and
// a loop whose zero pose sketch cannot enclose, are decaderr.ErrUnsupported.
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
	return linkage.VerifyBox(ctx, d.linkageHooks(), l, box, func() (motionoption.Config, int, error) {
		return resolveJointBoxOptions(opts)
	})
}

// requireLinkage applies the linkage refusals VerifyLinkage and
// VerifyJointBox share (docs/linkage-check-design.md §3): a nil linkage or
// one with no link, and a link body or declared contact body that is not a
// live body of d this evaluator built.
func (d *Document) requireLinkage(l *Linkage) error {
	if l == nil {
		return fmt.Errorf(`%w: a nil linkage has no link to move`, ErrDegenerate)
	}
	links := l.Links()
	if len(links) == 0 {
		return fmt.Errorf(`%w: a linkage with no link moves nothing`, ErrDegenerate)
	}
	for _, link := range links {
		for _, b := range link.Bodies() {
			if err := d.requireLive(b); err != nil {
				return err
			}
			if b.n.Payload == nil {
				return fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
			}
		}
	}
	for _, c := range l.JointContacts() {
		for _, b := range []*Body{c.A, c.B} {
			if err := d.requireLive(b); err != nil {
				return err
			}
		}
	}
	return nil
}

// defaultCellBudget is WithCellBudget's default (docs/linkage-check-design.md
// §14.1).
const defaultCellBudget = 16384

// WithCellBudget caps the number of cell centres VerifyJointBox evaluates
// (docs/linkage-check-design.md §14.4); the default is 16384. Every split
// evaluates two centres, so once fewer than two evaluations remain no cell is
// split and the report raises DiagJointBoxBudgetExhausted. A budget of 1
// evaluates the whole box's centre alone; cells < 1 is decaderr.ErrDegenerate. It is a
// JointBoxOption only, so VerifyMotion and VerifyLinkage do not accept it.
func WithCellBudget(cells int) JointBoxOption {
	return jointBoxOptionValue{motionoption.WithCellBudget(cells)}
}

// resolveJointBoxOptions folds a joint box's options: the cell budget is the
// box's own, and every other option is a MotionOption folded as
// VerifyLinkage folds it, over the Dimensionless fraction of each range.
func resolveJointBoxOptions(opts []JointBoxOption) (motionoption.Config, int, error) {
	budget := defaultCellBudget
	motion := make([]MotionOption, 0, len(opts))
	for _, o := range opts {
		if o == nil {
			return motionoption.Config{}, 0, fmt.Errorf(`%w: a nil option names nothing to apply`, decaderr.ErrDegenerate)
		}
		if _, ok := o.(jointBoxOptionValue); ok {
			cells, ok := option.Get[int](o)
			if !ok {
				return motionoption.Config{}, 0, fmt.Errorf(`%w: a joint-box option carries no value`, decaderr.ErrDegenerate)
			}
			if cells < 1 {
				return motionoption.Config{}, 0, fmt.Errorf(`%w: a cell budget must allow at least one centre, got %d`, decaderr.ErrDegenerate, cells)
			}
			budget = cells
			continue
		}
		mo, ok := o.(MotionOption)
		if !ok {
			return motionoption.Config{}, 0, fmt.Errorf(`%w: a joint-box option of type %T is not one this evaluator reads`, decaderr.ErrDegenerate, o)
		}
		motion = append(motion, mo)
	}
	encodedMotion, err := decodeMotionOptions(motion)
	if err != nil {
		return motionoption.Config{}, 0, err
	}
	cfg, err := motionoption.Resolve(encodedMotion, motionbound.FractionDomain())
	if err != nil {
		return motionoption.Config{}, 0, err
	}
	return cfg, budget, nil
}
