package linkage

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/motionengine"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyLinkage (agent-docs/linkage-check-design.md §3, §6,
// §8): validation, the driver that hands internal/motionengine's engine one
// moving group per link, and the report assembly. linkage.go owns the
// vocabulary and linkage/bound.go the bounds.

// Verify is the engine of Document.VerifyLinkage over a linkage the caller
// has already admitted (non-nil, with links, every body live and built):
// it resolves the drive, proves the chain bounds and runs the motion engine.
// options resolves the caller's options once the drive is known good, so the
// drive's refusals precede the options'.
func Verify[B comparable](ctx context.Context, h Hooks[B], l *Linkage[B], drive Drive[B],
	options func(motionbound.Domain) (motionoption.Config, error)) (*LinkageReport[B], error) {
	spec, err := l.resolveDrive(drive)
	if err != nil {
		return nil, err
	}
	if spec.holds() {
		return nil, fmt.Errorf(`%w: a drive in which every sweep holds names no motion`, decaderr.ErrDegenerate)
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
	cfg, err := options(motionbound.FractionDomain())
	if err != nil {
		return nil, err
	}
	if !cfg.Stated {
		// agent-docs/linkage-check-design.md §3: unstated, the reading refines
		// past the verdict floor to its own.
		reading := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).SetFrac64(1, linkageReadingFloor)}
		cfg.ReadingP = &reading
	}
	var bounds []linkBound
	if len(spec.loops) == 0 {
		if bounds, ok = linkagebound.ReadReachBounds(reachSource[B]{spec: spec, frames: frames, node: h.Node}); !ok {
			return nil, linkageBoundsError()
		}
	} else if err := spec.prepareLoops(ctx, cfg.ResolutionP.Base); err != nil {
		// agent-docs/linkage-check-design.md §15.7: a driven loop's scene, its zero
		// pose and its certifiable set, down to the verdict floor, before
		// any pose; a dependent's reach enters the bounds.
		return nil, err
	} else if bounds, ok = linkagebound.ReadReachBounds(reachSource[B]{spec: spec, frames: frames, node: h.Node}); !ok {
		return nil, linkageBoundsError()
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	run := newLinkageRun(ctx, h, spec, frames, bounds, cfg)
	poses, spans, err := run.Refine()
	if err != nil {
		return nil, err
	}
	return publishLinkage(run, h, l, drive, poses, spans), nil
}

// linkageReadingFloor is the denominator of the default reading floor,
// units.Scalar(1.0/16384) (agent-docs/linkage-check-design.md §3).
const linkageReadingFloor = 16384

// readingResolution is the reading floor a run used: the stated resolution,
// or the default reading floor.
func readingResolution(cfg motionoption.Config) units.Value {
	if cfg.ReadingP == nil {
		return cfg.Resolution
	}
	return units.Scalar(1.0 / linkageReadingFloor)
}

// newLinkageRun is VerifyLinkage's working state over a validated drive: one
// moving group per link, in Linkage.Links() order, every other live body
// static, and each link body's swept box for the exclusion against them.
func newLinkageRun[B comparable](ctx context.Context, h Hooks[B], spec *linkageSpec[B], frames []motionbound.MotionFrame, bounds []linkBound, cfg motionoption.Config) *motionengine.Run {
	run := motionengine.NewRun(ctx, h.Engine, h.Doc, h.Live, cfg)
	run.Dom = motionbound.FractionDomain()
	dr := &linkageDriver[B]{run: run, spec: spec, frames: frames, bounds: bounds, standing: linkStandings(spec, bounds)}
	run.Drive = dr
	run.Declared = make(map[[2]*topo.Body]struct{}, 2*len(spec.linkage.contacts))
	for _, c := range spec.linkage.contacts {
		run.Declared[[2]*topo.Body{h.Node(c.A), h.Node(c.B)}] = struct{}{}
		run.Declared[[2]*topo.Body{h.Node(c.B), h.Node(c.A)}] = struct{}{}
	}
	groups := make([][]*topo.Body, len(spec.joints))
	for k, jt := range spec.joints {
		groups[k] = nodesOf(h, jt.link.bodies)
	}
	run.ReadBodies(groups)
	swept := make([]motionengine.SweptBox, len(run.Movers))
	for i, mv := range run.Movers {
		// A link body's box at the zero pose, inflated by its own Bound and by
		// the farthest any point of its link moves from the zero pose over the
		// drive (§6 step 4): a joint whose From is 80° has moved before the
		// drive begins.
		lo, hi, ok := motionbound.BoxCornersExact(mv.Body.Bounds, bounds[mv.Group].Reach)
		swept[i] = motionengine.SweptBox{Lo: lo, Hi: hi, Ok: ok}
	}
	run.FormPairs(swept)
	dr.settlePairs(swept)
	dr.symmetric = make([]bool, len(run.Movers))
	for i, mv := range run.Movers {
		dr.symmetric[i] = symmetricAboutJoint(mv.Body, spec.joints[mv.Group], frames[mv.Group])
	}
	dr.markConstantPairs()
	return run
}

// markConstantPairs marks every evaluated pair whose relation is the same at
// every configuration (agent-docs/linkage-check-design.md §5.9): its relative path —
// every joint on the mover's path against a static body, the joints strictly
// below the two links' lowest common ancestor against another link's body —
// holds exactly one joint that does not hold 0, a revolute, and one of the
// two bodies is symmetric about that joint's axis line (symmetricAboutJoint,
// the exact test of §5.2). The engine evaluates such a pair once, on the
// bodies as they stand.
func (dr *linkageDriver[B]) markConstantPairs() {
	r := dr.run
	for i, mv := range r.Movers {
		for k := range r.Pairs[i] {
			pair := &r.Pairs[i][k]
			if !pair.Evaluated() {
				continue
			}
			mine := dr.bounds[mv.Group].Path
			var theirs []int
			below := 0
			if pair.Other >= 0 {
				theirs = dr.bounds[r.Movers[pair.Other].Group].Path
				below = linkagebound.CommonDepth(mine, theirs)
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
			pair.Constant = symmetricAboutJoint(mv.Body, jt, frame) || symmetricAboutJoint(r.Partner(i, k), jt, frame)
		}
	}
}

// settlePairs applies the linkage's own pair standings on top of the
// engine's (agent-docs/linkage-check-design.md §6 steps 2 and 4): a held link's
// pair against a static body, or two held links' pair, is not formed; a
// link-link pair whose swept boxes separate is excluded; and any pair the
// layer exclusion (§5.7) settles is excluded. A pair settled both ways keeps
// the larger of the two proven lower bounds, including a pair the engine's
// swept box against a static body settled already. A pair an operand's
// validity already decided is left as it stands.
func (dr *linkageDriver[B]) settlePairs(swept []motionengine.SweptBox) {
	r := dr.run
	for i, mv := range r.Movers {
		for k := range r.Pairs[i] {
			pair := &r.Pairs[i][k]
			other := pair.Other
			fixedHere := dr.standing[mv.Group] == linkFixed
			if fixedHere && (other < 0 || dr.standing[r.Movers[other].Group] == linkFixed) {
				*pair = motionengine.Pair{Other: other, Unformed: true, Declared: pair.Declared}
				continue
			}
			if pair.Invalid {
				continue
			}
			if pair.Excluded {
				// The swept-box exclusion against a static body settled the
				// pair already; the layer exclusion's bound is a second proven
				// lower bound on the same distance, and the larger serves.
				if lower, ok := dr.settled(i, other, k, swept); ok && lower > pair.Lower {
					pair.Lower = lower
				}
				continue
			}
			lower, ok := dr.settled(i, other, k, swept)
			if !ok {
				continue
			}
			pair.Excluded, pair.Lower, pair.Sheet = true, lower, false
		}
	}
}

// settled tries the layer exclusion and, for a link-link pair, the swept-box
// exclusion on pair k of mover i: each a proven lower bound on the pair's
// distance over the whole drive or box, so the larger serves.
func (dr *linkageDriver[B]) settled(i, other, k int, swept []motionengine.SweptBox) (float64, bool) {
	r := dr.run
	mine := dr.bounds[r.Movers[i].Group]
	if other < 0 {
		return layerLower(dr.spec, dr.frames, mine.Path, r.Movers[i].Body, r.Statics[k].Body)
	}
	theirs := dr.bounds[r.Movers[other].Group]
	below := linkagebound.CommonDepth(mine.Path, theirs.Path)
	path := append(slices.Clone(mine.Path[below:]), theirs.Path[below:]...)
	best, settled := layerLower(dr.spec, dr.frames, path, r.Movers[i].Body, r.Movers[other].Body)
	if swept[i].Ok && swept[other].Ok {
		if lower, ok := linkagebound.SweptBoxesLower(swept[i].Lo, swept[i].Hi, swept[other].Lo, swept[other].Hi); ok && lower > best {
			best, settled = lower, true
		}
	}
	return best, settled
}

// linkageDriver drives the engine's groups, one per link, along a drive.
type linkageDriver[B comparable] struct {
	run      *motionengine.Run
	spec     *linkageSpec[B]
	standing []linkStanding // per link, §6 step 2
	frames   []motionbound.MotionFrame
	bounds   []linkBound
	// symmetric marks, per mover, a body its own link's joint does not move
	// (agent-docs/linkage-check-design.md §5.2): that joint leaves the body's
	// relative path in travel and projection.
	symmetric []bool
	// corners holds every corner reading the projection bound has read
	// (agent-docs/linkage-check-design.md §5.8), once per pose, body and relative
	// path, so a pose's reading serves both intervals it ends.
	corners map[cornerKey]cornerBounds
	// statics holds each static body's hull point reading, by its position.
	statics map[int]cornerBounds
}

// cornerKey names one corner reading: a mover's own box under the joints on
// its link's path from position below on, at one pose.
type cornerKey struct {
	pose         *motionengine.Pose
	mover, below int
	// hull marks the reading of the body's hull points (linkagebound.PrismHullPoints)
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
// (agent-docs/linkage-check-design.md §15.4); a pose its loop cannot be enclosed at
// is unbuildable.
func (dr *linkageDriver[B]) PosesAt(f *big.Rat, _ units.Value, param motionbound.MotionParam) ([]motionengine.GroupPose, error) {
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
	} else if values, bounds, poses, ideals, err = dr.spec.loopPosesAt(dr.run.Ctx, dr.frames, f); err != nil {
		return nil, err
	}
	out := make([]motionengine.GroupPose, len(poses))
	for k := range poses {
		out[k] = motionengine.GroupPose{
			Pose: poses[k], Ideals: []motionbound.IdealPose{ideals[k]}, Stretch: 1, Value: values[k], Bound: bounds[k],
			Fixed: dr.standing[k] == linkFixed, Constant: dr.standing[k] == linkConstant,
		}
	}
	return out, nil
}

// intervalGate reads every driven loop's dependents over the interval
// between two poses, for travel to consume (agent-docs/linkage-check-design.md
// §15.5); an interval a loop cannot be enclosed over is refused with sketch's
// cause.
func (dr *linkageDriver[B]) IntervalGate(a, b *motionengine.Pose) string {
	for _, ld := range dr.spec.loops {
		if _, err := ld.intervalSpans(dr.run.Ctx, a.F, b.F); err != nil {
			return err.Error()
		}
	}
	return ""
}

// travel is the chain travel bound of agent-docs/linkage-check-design.md §5.2 for
// pair k of mover i: against a static body, every joint on the mover's path;
// against a body of another link, the joints strictly below the two links'
// lowest common ancestor on each branch, since the joints at and above it
// move both bodies by one rigid motion.
func (dr *linkageDriver[B]) Travel(i, k int, a, b motionbound.MotionParam) *big.Rat {
	r := dr.run
	other := r.Pairs[i][k].Other
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
func (dr *linkageDriver[B]) pairDepth(i, other int) int {
	if other < 0 {
		return 0
	}
	r := dr.run
	return linkagebound.CommonDepth(dr.bounds[r.Movers[i].Group].Path, dr.bounds[r.Movers[other].Group].Path)
}

// pathOf is mover m's link reading for a pair whose relative path starts at
// position below: the link's own, or, for a body its own joint does not move
// (agent-docs/linkage-check-design.md §5.2), the same without that joint when the
// joint lies on the relative path.
func (dr *linkageDriver[B]) pathOf(m, below int) linkBound {
	b := dr.bounds[dr.run.Movers[m].Group]
	if dr.symmetric[m] && below < len(b.Path) {
		n := len(b.Path) - 1
		return linkBound{Path: b.Path[:n], Rho: b.Rho[:n], Reach: b.Reach}
	}
	return b
}

// projection is the projection bound of agent-docs/linkage-check-design.md §5.8
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
func (dr *linkageDriver[B]) Projection(i, k int, a, b *motionengine.Pose) *big.Rat {
	r := dr.run
	other := r.Pairs[i][k].Other
	below := dr.pairDepth(i, other)
	mine := dr.pathOf(i, below)
	var theirs linkBound
	if other >= 0 {
		theirs = dr.pathOf(other, below)
	}
	segMine := dr.projectionSteps(mine, below, a.F, b.F)
	var segTheirs []proofbound.RatInterval
	if other >= 0 {
		segTheirs = dr.projectionSteps(theirs, below, a.F, b.F)
	}
	if segMine == nil || (other >= 0 && segTheirs == nil) {
		segMine, segTheirs = nil, nil
	}
	var best *big.Rat
	for n, end := range []*motionengine.Pose{a, b} {
		hMine, ok := dr.projectionSpans(mine, below, a.F, b.F, end)
		if !ok {
			return nil
		}
		remMine := linkagebound.Remainder(mine.Rho[below:], hMine)
		var hTheirs []*big.Rat
		var remTheirs *big.Rat
		if other >= 0 {
			if hTheirs, ok = dr.projectionSpans(theirs, below, a.F, b.F, end); !ok {
				return nil
			}
			remTheirs = linkagebound.Remainder(theirs.Rho[below:], hTheirs)
		}
		cm, ok := dr.cornersAt(end, i, mine, below, false)
		if !ok {
			return nil
		}
		// From end b the segment runs backward.
		backward := n == 1
		side := projectionSide{Corners: cm, H: hMine, Seg: linkagebound.StepsFrom(segMine, backward), Rem: remMine}
		var partner projectionSide
		if other < 0 {
			lo, hi, ok := motionbound.BoxCornersExact(r.Statics[k].Body.Bounds, new(big.Rat))
			if !ok {
				return nil
			}
			corners, ok := linkagebound.RoundCorners(linkagebound.StaticBox(lo, hi))
			if !ok {
				return nil
			}
			partner = projectionSide{Corners: corners}
		} else {
			ct, ok := dr.cornersAt(end, other, theirs, below, false)
			if !ok {
				return nil
			}
			partner = projectionSide{Corners: ct, H: hTheirs, Seg: linkagebound.StepsFrom(segTheirs, backward), Rem: remTheirs}
		}
		if l := linkagebound.Lower(side, partner); best == nil || l.Cmp(best) > 0 {
			best = l
		}
		// The hull bound (§5.8): the same expansion over each body's hull
		// points, along every candidate direction; the larger serves.
		if side.Corners, ok = dr.cornersAt(end, i, mine, below, true); !ok {
			continue
		}
		if other < 0 {
			if partner.Corners, ok = dr.staticHull(k); !ok {
				continue
			}
		} else if partner.Corners, ok = dr.cornersAt(end, other, theirs, below, true); !ok {
			continue
		}
		if l := linkagebound.LowerHullAbove(side, partner, best); l != nil && l.Cmp(best) > 0 {
			best = l
		}
	}
	return best
}

// projectionSpans is h of agent-docs/linkage-check-design.md §5.8's interval form
// for each joint on b's path from position below on, read from the pose end:
// a stated joint's jointSpan, the total variation over [sa, sb], which bounds
// its change from either end to any parameter of the interval; a loop's
// dependent, the larger distance from its centre at end, the midpoint of its
// enclosure there, to an end of the hull of its interval hull and that
// enclosure, which holds every value it takes over the interval. Each is
// rounded up to a float. ok is false when a dependent's readings are missing or a span
// overflows.
func (dr *linkageDriver[B]) projectionSpans(b linkBound, below int, sa, sb *big.Rat, end *motionengine.Pose) ([]*big.Rat, bool) {
	h := make([]*big.Rat, 0, len(b.Path)-below)
	for _, i := range b.Path[below:] {
		jt := dr.spec.joints[i]
		var span *big.Rat
		if jt.dep == nil {
			span = jointSpan(jt, sa, sb)
		} else {
			hull, ok := jt.dep.dependentHull(i, sa, sb)
			if !ok {
				return nil, false
			}
			at, ok := jt.dep.dependentAt(dr.run.Ctx, i, end.F)
			if !ok {
				return nil, false
			}
			hull = proofbound.IntervalOwned(proofbound.RatMin(hull.Lo, at.Lo), motionengine.MaxRat(hull.Hi, at.Hi))
			centre := linkagebound.Midpoint(at)
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

// projectionSteps is each joint's segment step Δq_i over [sa, sb] on b's path
// from position below on (jointStep), for agent-docs/linkage-check-design.md §5.8's
// segment term; nil when a waypoint lies inside the interval, or a step
// cannot be read, and the box form serves.
func (dr *linkageDriver[B]) projectionSteps(b linkBound, below int, sa, sb *big.Rat) []proofbound.RatInterval {
	steps := make([]proofbound.RatInterval, 0, len(b.Path)-below)
	for _, i := range b.Path[below:] {
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

// cornersAt is mover m's corner reading at a pose under the joints on its
// link's path from position below on, rounded outward, read once and kept.
func (dr *linkageDriver[B]) cornersAt(pose *motionengine.Pose, m int, b linkBound, below int, hull bool) (cornerBounds, bool) {
	key := cornerKey{pose: pose, mover: m, below: below, hull: hull}
	if got, ok := dr.corners[key]; ok {
		return got, true
	}
	points, ok := motionengine.BodyPoints(dr.run.Movers[m].Body, hull)
	if !ok {
		return cornerBounds{}, false
	}
	params := make([]motionbound.MotionParam, len(dr.spec.joints))
	for _, i := range b.Path[below:] {
		jt := dr.spec.joints[i]
		if jt.dep == nil {
			params[i] = jointParam(jt, pose.F)
			continue
		}
		// A dependent is read at its centre, the midpoint of its enclosure
		// at the pose (agent-docs/linkage-check-design.md §5.8).
		at, ok := jt.dep.dependentAt(dr.run.Ctx, i, pose.F)
		if !ok {
			return cornerBounds{}, false
		}
		params[i] = motionbound.MotionParam{Turn: new(big.Rat), Base: linkagebound.Midpoint(at)}
	}
	reading, ok := readPoints(dr.spec, dr.frames, params, b, below, points)
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
func (dr *linkageDriver[B]) staticHull(k int) (cornerBounds, bool) {
	if got, ok := dr.statics[k]; ok {
		return got, true
	}
	points, ok := motionengine.BodyPoints(dr.run.Statics[k].Body, true)
	if !ok {
		return cornerBounds{}, false
	}
	reading, ok := linkagebound.RoundCorners(points)
	if !ok {
		return cornerBounds{}, false
	}
	if dr.statics == nil {
		dr.statics = make(map[int]cornerBounds)
	}
	dr.statics[k] = reading
	return reading, true
}

// publishLinkage assembles VerifyLinkage's report
// (agent-docs/linkage-check-design.md §4).
func publishLinkage[B comparable](r *motionengine.Run, h Hooks[B], l *Linkage[B], drive Drive[B], poses []*motionengine.Pose, spans []motionengine.Span) *LinkageReport[B] {
	c := r.Conclude(poses, spans)
	report := &LinkageReport[B]{
		Request:           motionengine.MotionRequest(r.Cfg),
		ReadingResolution: readingResolution(r.Cfg),
		Linkage:           l,
		Drive:             slices.Clone(drive),
		Links:             l.Links(),
		JointContacts:     l.JointContacts(),
		Against:           h.lift().Bodies(c.Against),
		Intervals:         motionengine.Intervals(c.Intervals),
		Collisions:        []LinkCollision[B]{},
		Clearance:         c.Clearance,
		Assessment:        c.Assessment,
		Diagnostics:       h.lift().Diagnostics(c.Diagnostics),
		Status:            c.Status,
	}
	for _, pose := range poses {
		if pose.Unbuildable != nil {
			// agent-docs/linkage-check-design.md §15.6: a pose the loop could not
			// be enclosed at is not evaluated and not published.
			continue
		}
		n := len(pose.Groups)
		lp := LinkagePose{At: pose.Result.At, Values: make([]units.Value, n), Bounds: make([]units.Value, n), Poses: make([]r3.Transform, n)}
		for g, gp := range pose.Groups {
			lp.Values[g], lp.Bounds[g], lp.Poses[g] = gp.Value, gp.Bound, gp.Pose
		}
		report.Poses = append(report.Poses, LinkagePoseResult[B]{
			Pose:          lp,
			Interferences: h.lift().Interferences(pose.Result.Interferences),
			Clearances:    h.lift().Clearances(pose.Result.Clearances),
			Diagnostics:   h.lift().Diagnostics(pose.Result.Diagnostics),
		})
		for _, hit := range pose.Collisions {
			report.Collisions = append(report.Collisions, LinkCollision[B]{At: hit.At, A: h.Handle(hit.Moving), B: h.Handle(hit.Static), Volume: hit.Volume})
		}
	}
	return report
}

// The engine finds these capabilities by interface at run time, so a method
// that drifted in name or signature would silently disable them.
var (
	_ motionengine.Driver          = (*linkageDriver[struct{}])(nil)
	_ motionengine.IntervalGate    = (*linkageDriver[struct{}])(nil)
	_ motionengine.PoseUnbuildable = (*unbuildableError[struct{}])(nil)
)

// Hooks are the services a linkage check takes from its document: the motion
// engine's own, the document's node and live bodies, and the maps between a
// body reference and its topology node.
type Hooks[B comparable] struct {
	Engine motionengine.Hooks
	Doc    *topo.Document
	Live   []*topo.Body
	Node   func(B) *topo.Body
	Handle func(*topo.Body) B
}

// lift maps node lists to body references.
func (h Hooks[B]) lift() *vocab.RefLift[*topo.Body, struct{}, B, struct{}] {
	return &vocab.RefLift[*topo.Body, struct{}, B, struct{}]{Body: h.Handle}
}

// nodesOf maps body references to their topology nodes.
func nodesOf[B comparable](h Hooks[B], bodies []B) []*topo.Body {
	out := make([]*topo.Body, len(bodies))
	for i, b := range bodies {
		out[i] = h.Node(b)
	}
	return out
}
