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
// joint's declared limits are ErrDegenerate. Validation precedes
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
	if !cfg.stated {
		// docs/linkage-check-design.md §3: unstated, the reading refines
		// past the verdict floor to its own.
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.readingP = &reading
	}
	var bounds []linkBound
	if len(spec.loops) == 0 {
		if bounds, ok = readLinkBounds(spec, frames); !ok {
			return nil, linkageBoundsError()
		}
	} else if err := spec.prepareLoops(ctx, cfg.resolutionP.Base); err != nil {
		// docs/linkage-check-design.md §15.7: a driven loop's scene, its zero
		// pose and its certifiable set, down to the verdict floor, before
		// any pose; a dependent's reach enters the bounds.
		return nil, err
	} else if bounds, ok = readLinkBounds(spec, frames); !ok {
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

// linkageReadingFloor is the denominator of the default reading floor,
// units.Scalar(1.0/16384) (docs/linkage-check-design.md §3).
const linkageReadingFloor = 16384

// readingResolution is the reading floor a run used: the stated resolution,
// or the default reading floor.
func readingResolution(cfg motionConfig) units.Value {
	if cfg.readingP == nil {
		return cfg.resolution
	}
	return units.Scalar(1.0 / linkageReadingFloor)
}

// newLinkageRun is VerifyLinkage's working state over a validated drive: one
// moving group per link, in Linkage.Links() order, every other live body
// static, and each link body's swept box for the exclusion against them.
func newLinkageRun(ctx context.Context, d *Document, spec *linkageSpec, frames []motionbound.MotionFrame, bounds []linkBound, cfg motionConfig) *motionRun {
	run := &motionRun{ctx: ctx, d: d, dom: fractionDomain(), cfg: cfg, cache: &bodyGeomCache{}}
	dr := &linkageDriver{run: run, spec: spec, frames: frames, bounds: bounds, standing: linkStandings(spec, bounds)}
	run.drive = dr
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
	dr.settlePairs(swept)
	return run
}

// settlePairs applies the linkage's own pair standings on top of the
// engine's (docs/linkage-check-design.md §6 steps 2 and 4): a held link's
// pair against a static body, or two held links' pair, is not formed; a
// link-link pair whose swept boxes separate is excluded; and any pair the
// layer exclusion (§5.7) settles is excluded. A pair an operand's validity
// already decided is left as it stands.
func (dr *linkageDriver) settlePairs(swept []motionSweptBox) {
	r := dr.run
	for i, mv := range r.movers {
		for k := range r.pairs[i] {
			pair := &r.pairs[i][k]
			other := pair.other
			fixedHere := dr.standing[mv.group] == linkFixed
			if fixedHere && (other < 0 || dr.standing[r.movers[other].group] == linkFixed) {
				*pair = motionPair{other: other, unformed: true, declared: pair.declared}
				continue
			}
			if pair.invalid || pair.excluded {
				continue
			}
			lower, ok := dr.settled(i, other, k, swept)
			if !ok {
				continue
			}
			pair.excluded, pair.lower, pair.sheet = true, lower, false
		}
	}
}

// settled tries the link-link swept-box exclusion, then the layer exclusion,
// on pair k of mover i.
func (dr *linkageDriver) settled(i, other, k int, swept []motionSweptBox) (float64, bool) {
	r := dr.run
	mine := dr.bounds[r.movers[i].group]
	if other < 0 {
		return layerLower(dr.spec, dr.frames, mine.path, r.movers[i].body, r.statics[k].body)
	}
	if swept[i].ok && swept[other].ok {
		if lower, ok := sweptBoxesLower(swept[i].lo, swept[i].hi, swept[other].lo, swept[other].hi); ok {
			return lower, true
		}
	}
	theirs := dr.bounds[r.movers[other].group]
	below := commonDepth(mine.path, theirs.path)
	path := append(slices.Clone(mine.path[below:]), theirs.path[below:]...)
	return layerLower(dr.spec, dr.frames, path, r.movers[i].body, r.movers[other].body)
}

// linkageDriver drives the engine's groups, one per link, along a drive.
type linkageDriver struct {
	run      *motionRun
	spec     *linkageSpec
	standing []linkStanding // per link, §6 step 2
	frames   []motionbound.MotionFrame
	bounds   []linkBound
}

// posesAt builds every link's float pose by linkageSpec.posesAt — the
// function Linkage.PoseAt calls — and its ideal pose over exact rationals.
// Every ideal linear part is a product of exactly orthogonal rotations, so
// the stretch base is 1.
//
// A drive that moves a loop builds each dependent at its certified
// enclosure's midpoint and its ideal pose over the whole enclosure
// (docs/linkage-check-design.md §15.4); a pose its loop cannot be enclosed at
// is unbuildable.
func (dr *linkageDriver) posesAt(f *big.Rat, _ units.Value, param motionbound.MotionParam) ([]motionGroupPose, error) {
	var (
		values, bounds []units.Value
		poses          []r3.Transform
		ideals         []motionbound.IdealPose
		err            error
	)
	if len(dr.spec.loops) == 0 {
		if values, poses, err = dr.spec.posesAt(f); err != nil {
			return nil, err
		}
		bounds = zeroBounds(values)
		ideals = idealPosesAt(dr.spec, dr.frames, param.Base)
	} else if values, bounds, poses, ideals, err = dr.spec.loopPosesAt(dr.run.ctx, dr.frames, f); err != nil {
		return nil, err
	}
	out := make([]motionGroupPose, len(poses))
	for k := range poses {
		out[k] = motionGroupPose{
			pose: poses[k], ideals: []motionbound.IdealPose{ideals[k]}, stretch: 1, value: values[k], bound: bounds[k],
			fixed: dr.standing[k] == linkFixed, constant: dr.standing[k] == linkConstant,
		}
	}
	return out, nil
}

// intervalGate reads every driven loop's dependents over the interval
// between two poses, for travel to consume (docs/linkage-check-design.md
// §15.5); an interval a loop cannot be enclosed over is refused with sketch's
// cause.
func (dr *linkageDriver) intervalGate(a, b *motionPose) string {
	for _, ld := range dr.spec.loops {
		if _, err := ld.intervalSpans(dr.run.ctx, a.f, b.f); err != nil {
			return err.Error()
		}
	}
	return ""
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
		Request:           c.request,
		ReadingResolution: readingResolution(r.cfg),
		Linkage:           l,
		Drive:             slices.Clone(drive),
		Links:             l.Links(),
		JointContacts:     l.JointContacts(),
		Against:           c.against,
		Intervals:         c.intervals,
		Collisions:        []LinkCollision{},
		Clearance:         c.clearance,
		Assessment:        c.assessment,
		Diagnostics:       c.diagnostics,
		Status:            c.status,
	}
	for _, pose := range poses {
		if pose.unbuildable != nil {
			// docs/linkage-check-design.md §15.6: a pose the loop could not
			// be enclosed at is not evaluated and not published.
			continue
		}
		n := len(pose.groups)
		lp := LinkagePose{At: pose.result.At, Values: make([]units.Value, n), Bounds: make([]units.Value, n), Poses: make([]r3.Transform, n)}
		for g, gp := range pose.groups {
			lp.Values[g], lp.Bounds[g], lp.Poses[g] = gp.value, gp.bound, gp.pose
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
