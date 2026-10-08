package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	if !cfg.Stated {
		// docs/linkage-check-design.md §3: unstated, the reading refines
		// past the verdict floor to its own.
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.ReadingP = &reading
	}
	var bounds []linkBound
	if len(spec.loops) == 0 {
		if bounds, ok = readLinkBounds(spec, frames); !ok {
			return nil, linkageBoundsError()
		}
	} else if err := spec.prepareLoops(ctx, cfg.ResolutionP.Base); err != nil {
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
	if cfg.ReadingP == nil {
		return cfg.Resolution
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
		lo, hi, ok := motionbound.BoxCornersExact(mv.body.bounds, bounds[mv.group].reach)
		swept[i] = motionSweptBox{lo: lo, hi: hi, ok: ok}
	}
	run.formPairs(swept)
	dr.settlePairs(swept)
	dr.symmetric = make([]bool, len(run.movers))
	for i, mv := range run.movers {
		dr.symmetric[i] = symmetricAboutJoint(mv.body, spec.joints[mv.group], frames[mv.group])
	}
	dr.markConstantPairs()
	return run
}

// markConstantPairs marks every evaluated pair whose relation is the same at
// every configuration (docs/linkage-check-design.md §5.9): its relative path —
// every joint on the mover's path against a static body, the joints strictly
// below the two links' lowest common ancestor against another link's body —
// holds exactly one joint that does not hold 0, a revolute, and one of the
// two bodies is symmetric about that joint's axis line (symmetricAboutJoint,
// the exact test of §5.2). The engine evaluates such a pair once, on the
// bodies as they stand.
func (dr *linkageDriver) markConstantPairs() {
	r := dr.run
	for i, mv := range r.movers {
		for k := range r.pairs[i] {
			pair := &r.pairs[i][k]
			if !pair.evaluated() {
				continue
			}
			mine := dr.bounds[mv.group].path
			var theirs []int
			below := 0
			if pair.other >= 0 {
				theirs = dr.bounds[r.movers[pair.other].group].path
				below = commonDepth(mine, theirs)
			}
			joint := -1
			moving := 0
			for _, path := range [][]int{mine[below:], theirs[min(below, len(theirs)):]} {
				for _, j := range path {
					if heldAtZeroJoint(dr.spec.joints[j]) {
						continue
					}
					joint = j
					moving++
				}
			}
			if moving != 1 || !dr.spec.joints[joint].revolute {
				continue
			}
			jt, frame := dr.spec.joints[joint], dr.frames[joint]
			pair.constant = symmetricAboutJoint(mv.body, jt, frame) || symmetricAboutJoint(r.partner(i, k), jt, frame)
		}
	}
}

// settlePairs applies the linkage's own pair standings on top of the
// engine's (docs/linkage-check-design.md §6 steps 2 and 4): a held link's
// pair against a static body, or two held links' pair, is not formed; a
// link-link pair whose swept boxes separate is excluded; and any pair the
// layer exclusion (§5.7) settles is excluded. A pair settled both ways keeps
// the larger of the two proven lower bounds, including a pair the engine's
// swept box against a static body settled already. A pair an operand's
// validity already decided is left as it stands.
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
			if pair.invalid {
				continue
			}
			if pair.excluded {
				// The swept-box exclusion against a static body settled the
				// pair already; the layer exclusion's bound is a second proven
				// lower bound on the same distance, and the larger serves.
				if lower, ok := dr.settled(i, other, k, swept); ok && lower > pair.lower {
					pair.lower = lower
				}
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

// settled tries the layer exclusion and, for a link-link pair, the swept-box
// exclusion on pair k of mover i: each a proven lower bound on the pair's
// distance over the whole drive or box, so the larger serves.
func (dr *linkageDriver) settled(i, other, k int, swept []motionSweptBox) (float64, bool) {
	r := dr.run
	mine := dr.bounds[r.movers[i].group]
	if other < 0 {
		return layerLower(dr.spec, dr.frames, mine.path, r.movers[i].body, r.statics[k].body)
	}
	theirs := dr.bounds[r.movers[other].group]
	below := commonDepth(mine.path, theirs.path)
	path := append(slices.Clone(mine.path[below:]), theirs.path[below:]...)
	best, settled := layerLower(dr.spec, dr.frames, path, r.movers[i].body, r.movers[other].body)
	if swept[i].ok && swept[other].ok {
		if lower, ok := sweptBoxesLower(swept[i].lo, swept[i].hi, swept[other].lo, swept[other].hi); ok && lower > best {
			best, settled = lower, true
		}
	}
	return best, settled
}

// linkageDriver drives the engine's groups, one per link, along a drive.
type linkageDriver struct {
	run      *motionRun
	spec     *linkageSpec
	standing []linkStanding // per link, §6 step 2
	frames   []motionbound.MotionFrame
	bounds   []linkBound
	// symmetric marks, per mover, a body its own link's joint does not move
	// (docs/linkage-check-design.md §5.2): that joint leaves the body's
	// relative path in travel and projection.
	symmetric []bool
	// corners holds every corner reading the projection bound has read
	// (docs/linkage-check-design.md §5.8), once per pose, body and relative
	// path, so a pose's reading serves both intervals it ends.
	corners map[cornerKey]cornerBounds
	// statics holds each static body's hull point reading, by its position.
	statics map[int]cornerBounds
}

// cornerKey names one corner reading: a mover's own box under the joints on
// its link's path from position below on, at one pose.
type cornerKey struct {
	pose         *motionPose
	mover, below int
	// hull marks the reading of the body's hull points (bodyHullPoints)
	// rather than its box corners.
	hull bool
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
	other := r.pairs[i][k].other
	below := dr.pairDepth(i, other)
	tau := chainTravel(dr.spec, dr.pathOf(i, below), below, a.Base, b.Base)
	if other < 0 || tau == nil {
		return tau
	}
	theirs := chainTravel(dr.spec, dr.pathOf(other, below), below, a.Base, b.Base)
	if theirs == nil {
		return nil
	}
	return tau.Add(tau, theirs)
}

// pairDepth is the position on each path of the first joint below the
// lowest common ancestor of mover i's link and its partner's: 0 against a
// static body (other < 0).
func (dr *linkageDriver) pairDepth(i, other int) int {
	if other < 0 {
		return 0
	}
	r := dr.run
	return commonDepth(dr.bounds[r.movers[i].group].path, dr.bounds[r.movers[other].group].path)
}

// pathOf is mover m's link reading for a pair whose relative path starts at
// position below: the link's own, or, for a body its own joint does not move
// (docs/linkage-check-design.md §5.2), the same without that joint when the
// joint lies on the relative path.
func (dr *linkageDriver) pathOf(m, below int) linkBound {
	b := dr.bounds[dr.run.movers[m].group]
	if dr.symmetric[m] && below < len(b.path) {
		return withoutOwnJoint(b)
	}
	return b
}

// projection is the projection bound of docs/linkage-check-design.md §5.8
// for pair k of mover i over the interval between poses a and b: each body's
// inflated box expanded from each end to second order in its joints' travel
// over the interval — along the drive's own segment when no waypoint lies
// inside it, over the box of the joints' travel otherwise — the largest
// separation along the six coordinate directions, the larger of the two
// ends' readings. A link-link pair drops
// the joints at and above the two links' lowest common ancestor, as travel
// does. It is nil when either body's relative path holds a loop's dependent
// joint, whose value is an enclosure the expansion does not consume, or a box
// cannot be read exactly.
func (dr *linkageDriver) projection(i, k int, a, b *motionPose) *big.Rat {
	r := dr.run
	other := r.pairs[i][k].other
	below := dr.pairDepth(i, other)
	mine := dr.pathOf(i, below)
	var theirs linkBound
	if other >= 0 {
		theirs = dr.pathOf(other, below)
	}
	segMine := dr.projectionSteps(mine, below, a.f, b.f)
	var segTheirs []proofbound.RatInterval
	if other >= 0 {
		segTheirs = dr.projectionSteps(theirs, below, a.f, b.f)
	}
	if segMine == nil || (other >= 0 && segTheirs == nil) {
		segMine, segTheirs = nil, nil
	}
	var best *big.Rat
	for n, end := range []*motionPose{a, b} {
		hMine, ok := dr.projectionSpans(mine, below, a.f, b.f, end)
		if !ok {
			return nil
		}
		remMine := projectionRemainder(mine, below, hMine)
		var hTheirs []*big.Rat
		var remTheirs *big.Rat
		if other >= 0 {
			if hTheirs, ok = dr.projectionSpans(theirs, below, a.f, b.f, end); !ok {
				return nil
			}
			remTheirs = projectionRemainder(theirs, below, hTheirs)
		}
		cm, ok := dr.cornersAt(end, i, mine, below, false)
		if !ok {
			return nil
		}
		// From end b the segment runs backward.
		backward := n == 1
		side := projectionSide{corners: cm, h: hMine, seg: stepsFrom(segMine, backward), rem: remMine}
		var partner projectionSide
		if other < 0 {
			lo, hi, ok := motionbound.BoxCornersExact(r.statics[k].body.bounds, new(big.Rat))
			if !ok {
				return nil
			}
			corners, ok := roundCorners(staticCorners(lo, hi))
			if !ok {
				return nil
			}
			partner = projectionSide{corners: corners}
		} else {
			ct, ok := dr.cornersAt(end, other, theirs, below, false)
			if !ok {
				return nil
			}
			partner = projectionSide{corners: ct, h: hTheirs, seg: stepsFrom(segTheirs, backward), rem: remTheirs}
		}
		if l := projectionLower(side, partner); best == nil || l.Cmp(best) > 0 {
			best = l
		}
		// The hull bound (§5.8): the same expansion over each body's hull
		// points, along every candidate direction; the larger serves.
		if side.corners, ok = dr.cornersAt(end, i, mine, below, true); !ok {
			continue
		}
		if other < 0 {
			if partner.corners, ok = dr.staticHull(k); !ok {
				continue
			}
		} else if partner.corners, ok = dr.cornersAt(end, other, theirs, below, true); !ok {
			continue
		}
		if l := projectionLowerHull(side, partner); l != nil && l.Cmp(best) > 0 {
			best = l
		}
	}
	return best
}

// projectionSpans is h of docs/linkage-check-design.md §5.8's interval form
// for each joint on b's path from position below on, read from the pose end:
// a stated joint's jointSpan, the total variation over [sa, sb], which bounds
// its change from either end to any parameter of the interval; a loop's
// dependent, the larger distance from its centre at end, the midpoint of its
// enclosure there, to an end of the hull of its interval hull and that
// enclosure, which holds every value it takes over the interval. Each is
// rounded up to a float. ok is false when a dependent's readings are missing or a span
// overflows.
func (dr *linkageDriver) projectionSpans(b linkBound, below int, sa, sb *big.Rat, end *motionPose) ([]*big.Rat, bool) {
	h := make([]*big.Rat, 0, len(b.path)-below)
	for _, i := range b.path[below:] {
		jt := dr.spec.joints[i]
		var span *big.Rat
		if jt.dep == nil {
			span = jointSpan(jt, sa, sb)
		} else {
			hull, ok := jt.dep.dependentHull(i, sa, sb)
			if !ok {
				return nil, false
			}
			at, ok := jt.dep.dependentAt(dr.run.ctx, i, end.f)
			if !ok {
				return nil, false
			}
			hull = proofbound.IntervalOwned(minRat(hull.Lo, at.Lo), maxRat(hull.Hi, at.Hi))
			centre := intervalMidpoint(at)
			span = new(big.Rat).Sub(hull.Hi, centre)
			if low := new(big.Rat).Sub(centre, hull.Lo); low.Cmp(span) > 0 {
				span = low
			}
		}
		rounded := proofarith.FloatRat(proofbound.RatFloatUp(span))
		if rounded == nil {
			return nil, false
		}
		h = append(h, rounded)
	}
	return h, true
}

// intervalMidpoint is an interval's exact midpoint.
func intervalMidpoint(iv proofbound.RatInterval) *big.Rat {
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	return mid.Quo(mid, big.NewRat(2, 1))
}

// projectionSteps is each joint's segment step Δq_i over [sa, sb] on b's path
// from position below on (jointStep), for docs/linkage-check-design.md §5.8's
// segment term; nil when a waypoint lies inside the interval, or a step
// cannot be read, and the box form serves.
func (dr *linkageDriver) projectionSteps(b linkBound, below int, sa, sb *big.Rat) []proofbound.RatInterval {
	steps := make([]proofbound.RatInterval, 0, len(b.path)-below)
	for _, i := range b.path[below:] {
		if dr.spec.joints[i].dep != nil {
			// A dependent is not affine in s (§5.8): the box form serves.
			return nil
		}
		step, ok := jointStep(dr.spec.joints[i], sa, sb)
		if !ok {
			return nil
		}
		steps = append(steps, step)
	}
	return steps
}

// stepsFrom is the segment steps read from one end: as they are from the
// interval's first end, negated from its second, where the segment runs
// backward. nil stays nil.
func stepsFrom(steps []proofbound.RatInterval, backward bool) []proofbound.RatInterval {
	if steps == nil || !backward {
		return steps
	}
	out := make([]proofbound.RatInterval, len(steps))
	for n, step := range steps {
		out[n] = proofbound.IntervalNeg(step)
	}
	return out
}

// cornersAt is mover m's corner reading at a pose under the joints on its
// link's path from position below on, rounded outward, read once and kept.
func (dr *linkageDriver) cornersAt(pose *motionPose, m int, b linkBound, below int, hull bool) (cornerBounds, bool) {
	key := cornerKey{pose: pose, mover: m, below: below, hull: hull}
	if got, ok := dr.corners[key]; ok {
		return got, true
	}
	points, ok := bodyPoints(dr.run.movers[m].body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	params := make([]motionbound.MotionParam, len(dr.spec.joints))
	for _, i := range b.path[below:] {
		jt := dr.spec.joints[i]
		if jt.dep == nil {
			params[i] = jointParam(jt, pose.f)
			continue
		}
		// A dependent is read at its centre, the midpoint of its enclosure
		// at the pose (docs/linkage-check-design.md §5.8).
		at, ok := jt.dep.dependentAt(dr.run.ctx, i, pose.f)
		if !ok {
			return cornerBounds{}, false
		}
		params[i] = motionbound.MotionParam{Turn: new(big.Rat), Base: intervalMidpoint(at)}
	}
	reading, ok := roundCorners(readPoints(dr.spec, dr.frames, params, b, below, points))
	if !ok {
		return cornerBounds{}, false
	}
	if dr.corners == nil {
		dr.corners = make(map[cornerKey]cornerBounds)
	}
	dr.corners[key] = reading
	return reading, true
}

// staticHull is static body k's hull point reading, read once and kept.
func (dr *linkageDriver) staticHull(k int) (cornerBounds, bool) {
	if got, ok := dr.statics[k]; ok {
		return got, true
	}
	points, ok := bodyPoints(dr.run.statics[k].body, true)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := roundCorners(points)
	if !ok {
		return cornerBounds{}, false
	}
	if dr.statics == nil {
		dr.statics = make(map[int]cornerBounds)
	}
	dr.statics[k] = reading
	return reading, true
}

// bodyPoints is a body's static point reading: its hull points when hull is
// set and the body has them (bodyHullPoints), and otherwise the eight corners
// of its Bounds box inflated by its own Bound.
func bodyPoints(b *Body, hull bool) (cornerReading, bool) {
	if hull {
		if points, pad, k, ok := bodyHullPoints(b); ok {
			reading := staticPoints(points)
			reading.pad, reading.prismK = pad, k
			return reading, true
		}
	}
	lo, hi, ok := motionbound.BoxCornersExact(b.bounds, new(big.Rat))
	if !ok {
		return cornerReading{}, false
	}
	return staticCorners(lo, hi), true
}

// publishLinkage assembles VerifyLinkage's report
// (docs/linkage-check-design.md §4).
func publishLinkage(r *motionRun, l *Linkage, drive Drive, poses []*motionPose, spans []motionSpan) *LinkageReport {
	c := r.conclude(poses, spans)
	report := &LinkageReport{
		Request:           c.Request,
		ReadingResolution: readingResolution(r.cfg),
		Linkage:           l,
		Drive:             slices.Clone(drive),
		Links:             l.Links(),
		JointContacts:     l.JointContacts(),
		Against:           c.Against,
		Intervals:         c.Intervals,
		Collisions:        []LinkCollision{},
		Clearance:         c.Clearance,
		Assessment:        c.Assessment,
		Diagnostics:       c.Diagnostics,
		Status:            c.Status,
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
