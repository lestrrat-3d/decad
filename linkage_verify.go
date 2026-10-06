package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyLinkage (docs/linkage-check-design.md §3, §6,
// §8): validation, the driver that hands motion_verify.go's engine one
// moving group per link, and the report assembly. linkage.go owns the
// vocabulary and linkage_bound.go the bounds.

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
// joints below the pair's lowest common ancestor; an interval it cannot
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
// with no link, a drive in which every sweep holds, and a drive that takes a
// joint outside its declared limits are ErrDegenerate. Validation precedes
// cancellation; after it a canceled context returns ctx.Err() and no report.
func (d *Document) VerifyLinkage(ctx context.Context, l *Linkage, drive Drive, opts ...MotionOption) (*LinkageReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control linkage verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if l == nil {
		return nil, fmt.Errorf(`%w: a nil linkage has no link to move`, ErrDegenerate)
	}
	if len(l.links) == 0 {
		return nil, fmt.Errorf(`%w: a linkage with no link moves nothing`, ErrDegenerate)
	}
	for _, link := range l.links {
		for _, b := range link.bodies {
			if err := d.requireLive(b); err != nil {
				return nil, err
			}
			if b.payload == nil {
				return nil, fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
			}
		}
	}
	for _, c := range l.contacts {
		for _, b := range []*Body{c.A, c.B} {
			if err := d.requireLive(b); err != nil {
				return nil, err
			}
		}
	}
	spec, err := l.resolveDrive(drive)
	if err != nil {
		return nil, err
	}
	if spec.holds() {
		return nil, fmt.Errorf(`%w: a drive in which every sweep holds names no motion`, ErrDegenerate)
	}
	for _, end := range []*big.Rat{new(big.Rat), big.NewRat(1, 1)} {
		if _, _, err := spec.posesAt(end); err != nil {
			return nil, err
		}
	}
	frames, ok := linkageFrames(spec)
	if !ok {
		return nil, linkageBoundsError()
	}
	cfg, err := resolveMotionOptions(opts, motionSpec{motionDomain: fractionDomain()})
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
	poses, spans, err := run.refine()
	if err != nil {
		return nil, err
	}
	return publishLinkage(run, l, drive, poses, spans), nil
}

// newLinkageRun is VerifyLinkage's working state over a validated drive: one
// moving group per link, in Linkage.Links() order, every other live body
// static, and each link body's swept box for the exclusion against them.
func newLinkageRun(ctx context.Context, d *Document, spec *linkageSpec, frames []motionbound.MotionFrame, bounds []linkBound, cfg motionConfig) *motionRun {
	run := &motionRun{ctx: ctx, d: d, dom: fractionDomain(), cfg: cfg, cache: &bodyGeomCache{}}
	run.drive = &linkageDriver{run: run, spec: spec, frames: frames, bounds: bounds}
	run.declared = make(map[[2]*Body]struct{}, 2*len(spec.linkage.contacts))
	for _, c := range spec.linkage.contacts {
		run.declared[[2]*Body{c.A, c.B}] = struct{}{}
		run.declared[[2]*Body{c.B, c.A}] = struct{}{}
	}
	groups := make([][]*Body, len(spec.joints))
	for k, jt := range spec.joints {
		groups[k] = jt.link.bodies
	}
	run.readBodies(groups)
	swept := make([]motionSweptBox, len(run.movers))
	for i, mv := range run.movers {
		// A link body's box at the zero pose, inflated by its own Bound and by
		// the farthest any point of its link moves from the zero pose over the
		// drive (§6 step 4): a joint whose From is 80° has moved before the
		// drive begins.
		lo, hi, ok := boxCornersExact(mv.body.bounds, bounds[mv.group].reach)
		swept[i] = motionSweptBox{lo: lo, hi: hi, ok: ok}
	}
	run.formPairs(swept)
	return run
}

// linkageDriver drives the engine's groups, one per link, along a drive.
type linkageDriver struct {
	run    *motionRun
	spec   *linkageSpec
	frames []motionbound.MotionFrame
	bounds []linkBound
}

// posesAt builds every link's float pose by linkageSpec.posesAt — the
// function Linkage.PoseAt calls — and its ideal pose over exact rationals.
// Every ideal linear part is a product of exactly orthogonal rotations, so
// the stretch base is 1.
func (dr *linkageDriver) posesAt(f *big.Rat, _ units.Value, param motionbound.MotionParam) ([]motionGroupPose, error) {
	values, poses, err := dr.spec.posesAt(f)
	if err != nil {
		return nil, err
	}
	ideals := idealPosesAt(dr.spec, dr.frames, param.Base)
	out := make([]motionGroupPose, len(poses))
	for k := range poses {
		out[k] = motionGroupPose{pose: poses[k], ideals: []motionbound.IdealPose{ideals[k]}, stretch: 1, value: values[k]}
	}
	return out, nil
}

// travel is the chain travel bound of docs/linkage-check-design.md §5.2 for
// pair k of mover i: against a static body, every joint on the mover's path;
// against a body of another link, the joints strictly below the two links'
// lowest common ancestor on each branch, since the joints at and above it
// move both bodies by one rigid motion.
func (dr *linkageDriver) travel(i, k int, a, b motionbound.MotionParam) *big.Rat {
	r := dr.run
	mine := dr.bounds[r.movers[i].group]
	other := r.pairs[i][k].other
	if other < 0 {
		return chainTravel(dr.spec, mine, 0, a.Base, b.Base)
	}
	theirs := dr.bounds[r.movers[other].group]
	below := commonDepth(mine.path, theirs.path)
	tau := chainTravel(dr.spec, mine, below, a.Base, b.Base)
	return tau.Add(tau, chainTravel(dr.spec, theirs, below, a.Base, b.Base))
}

// publishLinkage assembles VerifyLinkage's report
// (docs/linkage-check-design.md §4).
func publishLinkage(r *motionRun, l *Linkage, drive Drive, poses []*motionPose, spans []motionSpan) *LinkageReport {
	c := r.conclude(poses, spans)
	report := &LinkageReport{
		Request:       c.request,
		Linkage:       l,
		Drive:         slices.Clone(drive),
		Links:         l.Links(),
		JointContacts: l.JointContacts(),
		Against:       c.against,
		Intervals:     c.intervals,
		Collisions:    []LinkCollision{},
		Clearance:     c.clearance,
		Assessment:    c.assessment,
		Diagnostics:   c.diagnostics,
		Status:        c.status,
	}
	for _, pose := range poses {
		lp := LinkagePose{At: pose.result.At, Values: make([]units.Value, len(pose.groups)), Poses: make([]r3.Transform, len(pose.groups))}
		for g, gp := range pose.groups {
			lp.Values[g], lp.Poses[g] = gp.value, gp.pose
		}
		report.Poses = append(report.Poses, LinkagePoseResult{
			Pose:          lp,
			Interferences: pose.result.Interferences,
			Clearances:    pose.result.Clearances,
			Diagnostics:   pose.result.Diagnostics,
		})
		for _, hit := range pose.collisions {
			report.Collisions = append(report.Collisions, LinkCollision{At: hit.At, A: hit.Moving, B: hit.Static, Volume: hit.Volume})
		}
	}
	return report
}
