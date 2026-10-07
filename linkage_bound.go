package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file proves the bounds docs/linkage-check-design.md §5 builds its
// interval certificate from, every one over exact rationals so that no float
// rounding sits between a bound and the claim it supports:
//
//   - each joint's exact frame and each link's ideal pose, the exact
//     composition of its path joints' ideal poses (§5.1);
//   - ρ_{ik}, a proven upper bound on the distance from joint i's axis of
//     every point of link k once the joints strictly below i on its path have
//     moved to any value the drive reaches, read from the link's rest box
//     through balls carried down the path (§5.2);
//   - the chain travel bound τ, summed joint by joint below a pair's lowest
//     common ancestor (§5.2), and the reach the swept-box exclusion inflates a
//     link body's box by (§6 step 4);
//   - the projection bound L_n (§5.8): each body's inflated box corners and
//     their velocities under each joint at a pose, the second-derivative
//     bounds B_ij and the remainder Rem they give over an interval, and the
//     separation of two bodies along a coordinate direction that follows.
//
// Every square root is proofbound.RatSqrtUp, an up-rounded float read back as
// an exact rational; every sum and product after it is big.Rat arithmetic.

// linkBound is one link's §5.2 reading under a drive.
type linkBound struct {
	// path is the links whose joints carry this link, from the ground's child
	// down to the link itself.
	path []int
	// rho is ρ_{ik} for each joint on path, in path order; nil for a
	// prismatic joint, whose travel term is its displacement alone.
	rho []*big.Rat
	// reach is a proven upper bound on how far any point of the link moves
	// from the zero pose at any parameter of the drive: Σ ρ_{ik}·m_i over the
	// revolute joints on path, plus m_i over the prismatic ones.
	reach *big.Rat
}

// linkageFrames reads every joint's exact frame (motionbound.MotionFrame);
// ok is false when an axis, direction or centre is not representable.
func linkageFrames(spec *linkageSpec) ([]motionbound.MotionFrame, bool) {
	frames := make([]motionbound.MotionFrame, len(spec.joints))
	for k, jt := range spec.joints {
		ms := motionSpec{kind: motionbound.MotionPrismatic, dir: jt.axis}
		if jt.revolute {
			ms = motionSpec{kind: motionbound.MotionRevolute, center: jt.center, axis: jt.axis}
		}
		frame, ok := newMotionFrame(ms)
		if !ok {
			return nil, false
		}
		frames[k] = frame
	}
	return frames, true
}

// jointReach is m_i of docs/linkage-check-design.md §5.2: the farthest joint
// i's value reaches from 0 over the drive, the largest |w| over its
// waypoints — radians with π at its upper enclosure for an angle, millimetres
// exactly for a length, and 0 for an unlisted joint. q(s) is linear within
// each segment, so every value the drive visits lies within it.
func jointReach(jt linkJoint) *big.Rat {
	if jt.dep != nil {
		return new(big.Rat).Set(jt.depReach)
	}
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	var out *big.Rat
	for _, p := range jt.points {
		if m := zero.SpanUpper(p); out == nil || m.Cmp(out) > 0 {
			out = m
		}
	}
	return out
}

// jointParam is joint jt's exact value at the fraction s: its segment's
// exact interpolation at the local fraction.
func jointParam(jt linkJoint, s *big.Rat) motionbound.MotionParam {
	seg, t := jt.segment(s)
	return seg.fromP.Lerp(seg.toP, t)
}

// jointSpan is |Δq_i| of docs/linkage-check-design.md §5.2 while the
// fraction runs between sa and sb: a proven upper bound on the joint's total
// variation, the sum of motionbound.MotionParam.SpanUpper over the pieces
// that every waypoint strictly inside the interval cuts it into. A joint that
// turns back at a waypoint travels farther than its two ends differ, and the
// sum is additive over any split of the interval, which is what the interval
// certificate's two one-sided bounds need. With no waypoint inside it is the
// span of the two ends.
func jointSpan(jt linkJoint, sa, sb *big.Rat) *big.Rat {
	lo, hi := sa, sb
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	n := len(jt.points) - 1
	sum := new(big.Rat)
	prev := jointParam(jt, lo)
	for j := 1; j < n; j++ {
		w := big.NewRat(int64(j), int64(n))
		if w.Cmp(lo) <= 0 || w.Cmp(hi) >= 0 {
			continue
		}
		sum.Add(sum, prev.SpanUpper(jt.points[j]))
		prev = jt.points[j]
	}
	return sum.Add(sum, prev.SpanUpper(jointParam(jt, hi)))
}

// sqrtUpRat is proofbound.RatSqrtUp read back as an exact rational; nil when
// the root overflows.
func sqrtUpRat(q *big.Rat) *big.Rat {
	return proofarith.FloatRat(proofbound.RatSqrtUp(q))
}

func axisSq(a motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
}

// linkRestBox is the per-axis union of a link's bodies' Bounds boxes, each
// inflated outward by its own Bound, as exact rational extremes.
func linkRestBox(link *Link) (lo, hi motionbound.RatVec, ok bool) {
	for n, b := range link.bodies {
		bLo, bHi, okB := boxCornersExact(b.bounds, new(big.Rat))
		if !okB {
			return motionbound.RatVec{}, motionbound.RatVec{}, false
		}
		if n == 0 {
			lo, hi = bLo, bHi
			continue
		}
		for i := range 3 {
			if bLo[i].Cmp(lo[i]) < 0 {
				lo[i] = bLo[i]
			}
			if bHi[i].Cmp(hi[i]) > 0 {
				hi[i] = bHi[i]
			}
		}
	}
	return lo, hi, true
}

// reachSource adapts the root linkage's bodies and joints to the exact
// ball and cylinder calculation in internal/linkagebound.
type reachSource struct {
	spec   *linkageSpec
	frames []motionbound.MotionFrame
}

func (s reachSource) JointCount() int                     { return len(s.spec.joints) }
func (s reachSource) Parent(i int) int                    { return s.spec.joints[i].parent }
func (s reachSource) Revolute(i int) bool                 { return s.spec.joints[i].revolute }
func (s reachSource) Frame(i int) motionbound.MotionFrame { return s.frames[i] }
func (s reachSource) RestBox(i int) (motionbound.RatVec, motionbound.RatVec, bool) {
	return linkRestBox(s.spec.joints[i].link)
}
func (s reachSource) Reach(i int) *big.Rat { return jointReach(s.spec.joints[i]) }

func readLinkBounds(spec *linkageSpec, frames []motionbound.MotionFrame) ([]linkBound, bool) {
	read, ok := linkagebound.ReadReachBounds(reachSource{spec: spec, frames: frames})
	if !ok {
		return nil, false
	}
	bounds := make([]linkBound, len(read))
	for i, b := range read {
		bounds[i] = linkBound{path: b.Path, rho: b.Rho, reach: b.Reach}
	}
	return bounds, true
}

// chainTravel is τ^(L)_k of docs/linkage-check-design.md §5.2: a proven upper
// bound on how far any point of link k moves, relative to the links at and
// above its ancestor L, while the fraction runs from a to b — the telescoping
// sum over the joints on its path strictly below L of ρ_{ik}·|Δq_i| for a
// revolute and |Δq_i| for a prismatic, |Δq_i| taken by jointSpan, or for a
// loop's dependent by dependentSpan over the readings intervalGate took; nil
// when a dependent's readings are missing. below is the position on the path
// of the first joint below L: 0 when L is the ground.
func chainTravel(spec *linkageSpec, b linkBound, below int, sa, sb *big.Rat) *big.Rat {
	return pathTravel(b, below, func(joint int) *big.Rat {
		jt := spec.joints[joint]
		if jt.dep == nil {
			return jointSpan(jt, sa, sb)
		}
		return jt.dep.span(joint, sa, sb)
	})
}

// pathTravel is the telescoping sum of docs/linkage-check-design.md §5.2 over
// the joints on b's path from position below on: ρ_{ik}·span(i) for a
// revolute joint i and span(i) for a prismatic one, span(i) a proven upper
// bound on joint i's travel. span MUST return a fresh rational, or nil when
// it has no bound, and the sum is then nil.
func pathTravel(b linkBound, below int, span func(joint int) *big.Rat) *big.Rat {
	sum := new(big.Rat)
	for n := below; n < len(b.path); n++ {
		term := span(b.path[n])
		if term == nil {
			return nil
		}
		if b.rho[n] != nil {
			term.Mul(term, b.rho[n])
		}
		sum.Add(sum, term)
	}
	return sum
}

// commonDepth is the number of leading path entries two links share: the
// position on each path of the first joint below their lowest common
// ancestor, which is the ground when it is 0.
func commonDepth(a, b []int) int {
	n := 0
	for n < len(a) && n < len(b) && a[n] == b[n] {
		n++
	}
	return n
}

// idealPosesAt is every link's ideal pose at the exact fraction s
// (docs/linkage-check-design.md §5.1): its joint's ideal motion at the exact
// joint value, then its parent's ideal pose, composed over rational
// intervals; a link under the ground takes its joint's ideal motion alone.
func idealPosesAt(spec *linkageSpec, frames []motionbound.MotionFrame, s *big.Rat) []motionbound.IdealPose {
	params := make([]motionbound.MotionParam, len(spec.joints))
	for k, jt := range spec.joints {
		params[k] = jointParam(jt, s)
	}
	return idealPosesOf(spec, frames, params)
}

// idealPosesOf is every link's ideal pose at the exact joint values params,
// one per link in Linkage.Links() order: its joint's ideal motion, then its
// parent's ideal pose, composed over rational intervals.
func idealPosesOf(spec *linkageSpec, frames []motionbound.MotionFrame, params []motionbound.MotionParam) []motionbound.IdealPose {
	ideals := make([]motionbound.IdealPose, len(spec.joints))
	for k, jt := range spec.joints {
		ideal := frames[k].At(params[k])
		if jt.parent >= 0 {
			ideal = ideal.Then(ideals[jt.parent])
		}
		ideals[k] = ideal
	}
	return ideals
}

// linkageBoundsError is the refusal for a box or axis the exact reader cannot
// form (docs/linkage-check-design.md §5.3).
func linkageBoundsError() error {
	return fmt.Errorf(`%w: a link's bounds or a joint's axis cannot be read exactly`, ErrNotFinite)
}

// linkStanding is a link's standing over a drive
// (docs/linkage-check-design.md §6 step 2).
type linkStanding int

const (
	// linkMoving: some joint on the link's path moves.
	linkMoving linkStanding = iota
	// linkFixed: every joint on the link's path holds 0, so its ideal pose is
	// the identity at every s and its bodies stand where they are.
	linkFixed
	// linkConstant: every joint on the link's path holds, one at a nonzero
	// value, so its pose is the same at every s.
	linkConstant
)

// heldAtZeroJoint reports whether a joint's value is exactly 0 at every s:
// every waypoint is 0.
func heldAtZeroJoint(jt linkJoint) bool {
	if jt.dep != nil {
		return false
	}
	for _, p := range jt.points {
		if p.Turn.Sign() != 0 || p.Base.Sign() != 0 {
			return false
		}
	}
	return true
}

// linkStandings reads every link's standing from the joints on its path.
func linkStandings(spec *linkageSpec, bounds []linkBound) []linkStanding {
	out := make([]linkStanding, len(spec.joints))
	for k, b := range bounds {
		zero, held := true, true
		for _, i := range b.path {
			jt := spec.joints[i]
			if !heldAtZeroJoint(jt) {
				zero = false
			}
			if (jt.listed || jt.dep != nil) && jt.moves() {
				held = false
			}
		}
		switch {
		case zero:
			out[k] = linkFixed
		case held:
			out[k] = linkConstant
		}
	}
	return out
}

func ratDot(a, b motionbound.RatVec) *big.Rat { return linkagebound.Dot(a, b) }

func ratCross(a, b motionbound.RatVec) motionbound.RatVec { return linkagebound.Cross(a, b) }

func ratZero(v motionbound.RatVec) bool {
	return v[0].Sign() == 0 && v[1].Sign() == 0 && v[2].Sign() == 0
}

// layerKeeps reports whether joint jt, at any value, preserves a·x for every
// point x (docs/linkage-check-design.md §5.7): a revolute whose axis is
// exactly parallel to a, or a prismatic whose direction is exactly
// perpendicular to it.
func layerKeeps(jt linkJoint, f motionbound.MotionFrame, a motionbound.RatVec) bool {
	if jt.revolute {
		return ratZero(ratCross(a, f.Axis))
	}
	return ratDot(a, f.Axis).Sign() == 0
}

// layerAxes is §5.7's candidate directions for a relative path: the axis of
// its first moving revolute when it has one, and otherwise the coordinate
// axes and the cross product of its first two non-parallel slides. Each is
// admitted only when every joint on the path keeps it. A joint holding 0 is
// passed over; a path of such joints admits every coordinate axis.
func layerAxes(spec *linkageSpec, frames []motionbound.MotionFrame, path []int) []motionbound.RatVec {
	var moving []int
	for _, i := range path {
		if !heldAtZeroJoint(spec.joints[i]) {
			moving = append(moving, i)
		}
	}
	var candidates []motionbound.RatVec
	for _, i := range moving {
		if spec.joints[i].revolute {
			candidates = []motionbound.RatVec{frames[i].Axis}
			break
		}
	}
	if candidates == nil {
		one, zero := big.NewRat(1, 1), new(big.Rat)
		candidates = []motionbound.RatVec{{one, zero, zero}, {zero, one, zero}, {zero, zero, one}}
		for n, i := range moving {
			for _, j := range moving[n+1:] {
				if c := ratCross(frames[i].Axis, frames[j].Axis); !ratZero(c) {
					candidates = append(candidates, c)
					break
				}
			}
		}
	}
	var out []motionbound.RatVec
	for _, a := range candidates {
		keeps := true
		for _, i := range moving {
			if !layerKeeps(spec.joints[i], frames[i], a) {
				keeps = false
				break
			}
		}
		if keeps {
			out = append(out, a)
		}
	}
	return out
}

// layerExtent is a body's a-extent at the zero pose: the least and greatest
// exact a·x over the eight corners of its Bounds box inflated by its Bound.
func layerExtent(b *Body, a motionbound.RatVec) (lo, hi *big.Rat, ok bool) {
	boxLo, boxHi, ok := boxCornersExact(b.bounds, new(big.Rat))
	if !ok {
		return nil, nil, false
	}
	for n, x := range linkagebound.BoxCorners(boxLo, boxHi) {
		v := ratDot(a, x)
		if n == 0 || v.Cmp(lo) < 0 {
			lo = v
		}
		if n == 0 || v.Cmp(hi) > 0 {
			hi = v
		}
	}
	return lo, hi, true
}

// layerLower is §5.7's exclusion for a pair whose relative path is path: the
// largest proven lower bound w/|a| over the admitted directions a whose
// extents separate the two bodies by w > 0, rounded down; ok is false when no
// direction separates them.
func layerLower(spec *linkageSpec, frames []motionbound.MotionFrame, path []int, x, y *Body) (float64, bool) {
	best := 0.0
	for _, a := range layerAxes(spec, frames, path) {
		xLo, xHi, okX := layerExtent(x, a)
		yLo, yHi, okY := layerExtent(y, a)
		if !okX || !okY {
			continue
		}
		w := new(big.Rat).Sub(yLo, xHi)
		if alt := new(big.Rat).Sub(xLo, yHi); alt.Cmp(w) > 0 {
			w = alt
		}
		if w.Sign() <= 0 {
			continue
		}
		norm := sqrtUpRat(axisSq(a))
		if norm == nil {
			continue
		}
		if lower := proofbound.RatFloatDown(w.Quo(w, norm)); lower > best {
			best = lower
		}
	}
	return best, best > 0
}

// sweptBoxesLower is the swept-box exclusion between two exact boxes: the
// largest strictly positive per-axis gap, rounded down; ok is false when the
// boxes do not separate.
func sweptBoxesLower(aLo, aHi, bLo, bHi motionbound.RatVec) (float64, bool) {
	var best *big.Rat
	for i := range 3 {
		for _, gap := range []*big.Rat{new(big.Rat).Sub(bLo[i], aHi[i]), new(big.Rat).Sub(aLo[i], bHi[i])} {
			if gap.Sign() > 0 && (best == nil || gap.Cmp(best) > 0) {
				best = gap
			}
		}
	}
	if best == nil {
		return 0, false
	}
	lower := proofbound.RatFloatDown(best)
	return lower, lower > 0
}

// cornerReading is one body's docs/linkage-check-design.md §5.8 reading at
// one configuration: a finite set of points whose convex hull holds the body
// — the eight corners of its inflated rest box, or its hull points
// (bodyHullPoints) — under the ideal poses of the joints on its relative
// path, and each point's velocity under each of those joints alone. Every
// entry is a rational interval that encloses the exact value for every member
// of the ideal poses' enclosures.
type cornerReading struct {
	pos []motionbound.IvVec
	// vel holds, per point, its velocity under each joint on the relative
	// path, shallowest first: ω × (x − o) for a revolute, the unit direction
	// for a prismatic, per radian or per millimetre of the joint's value.
	vel [][]motionbound.IvVec
	// pad is the radius of the ball around every point the body may reach
	// beyond the points' hull; nil for none.
	pad *big.Rat
	// prismK is the vertex count of a prism's outer loop when the points are
	// its hull points, the k bottom vertices then the k top ones; 0 for box
	// corners.
	prismK int
}

// staticCorners is the corner reading of a box no joint moves: its eight
// corners as points, with no velocity.
func staticCorners(lo, hi motionbound.RatVec) cornerReading {
	corners := linkagebound.BoxCorners(lo, hi)
	return staticPoints(corners[:])
}

// staticPoints is the reading of points no joint moves, with no velocity.
func staticPoints(points []motionbound.RatVec) cornerReading {
	r := linkagebound.StaticPoints(points)
	return cornerReading{pos: r.Pos, vel: r.Vel, pad: r.Pad, prismK: r.PrismK}
}

func applyIdeal(p motionbound.IdealPose, x motionbound.IvVec) motionbound.IvVec {
	return linkagebound.ApplyIdeal(p, x)
}

func ivCross(a, b motionbound.IvVec) motionbound.IvVec {
	return linkagebound.IvCross(a, b)
}

// readCorners is docs/linkage-check-design.md §5.8's reading of one body of
// link b at the exact joint values params, under the joints on b's path from
// position below on — the joints strictly below a pair's lowest common
// ancestor, whose rigid motion above that ancestor changes no distance. The
// relative pose composes those joints' ideal motions exactly as
// idealPosesOf composes a link's, starting from the identity; joint i's axis
// at the configuration runs through o_i, the image of its Center under the
// relative pose of the joints above it, along ω_i, its unit Axis turned by
// that pose's rotation, and a prismatic's direction is turned the same way.
// lo and hi are the body's Bounds box inflated by its own Bound.
func readCorners(spec *linkageSpec, frames []motionbound.MotionFrame, params []motionbound.MotionParam, b linkBound, below int, lo, hi motionbound.RatVec) cornerReading {
	corners := linkagebound.BoxCorners(lo, hi)
	return readPoints(spec, frames, params, b, below, staticPoints(corners[:]))
}

// readPoints maps a static point reading through the link's relative joint path.
func readPoints(spec *linkageSpec, frames []motionbound.MotionFrame, params []motionbound.MotionParam, b linkBound, below int, out cornerReading) cornerReading {
	revolute := make([]bool, len(spec.joints))
	for i := range revolute {
		revolute[i] = spec.joints[i].revolute
	}
	r := linkagebound.ReadPoints(frames, params, b.path, revolute, below, linkagebound.Reading{
		Pos: out.pos, Vel: out.vel, Pad: out.pad, PrismK: out.prismK,
	})
	return cornerReading{pos: r.Pos, vel: r.Vel, pad: r.Pad, prismK: r.PrismK}
}

func secondDerivativeBound(b linkBound, m, n int) *big.Rat {
	return linkagebound.DerivativeBound(b.rho, m, n)
}

func projectionRemainder(b linkBound, below int, h []*big.Rat) *big.Rat {
	return linkagebound.Remainder(b.rho[below:], h)
}

// cornerBounds is a corner reading rounded outward to floats and read back
// as exact rationals, so the per-interval sums of docs/linkage-check-design.md
// §5.8 run over short dyadics rather than the long rationals of composed
// rotations: each corner coordinate's enclosure and each velocity
// component's enclosure widened to the floats around it. Rounding outward
// only weakens the bound the values enter.
type cornerBounds struct {
	lo, hi [][3]*big.Rat
	// vel holds, per point and per joint on the relative path, each
	// velocity component's enclosure.
	vel [][][3]proofbound.RatInterval
	// pad and prismK are the reading's own (cornerReading).
	pad    *big.Rat
	prismK int
}

// roundOut widens an enclosure to the floats around it, read back as exact
// rationals; ok is false when an end overflows a float.
func roundOut(iv proofbound.RatInterval) (proofbound.RatInterval, bool) {
	return linkagebound.RoundOut(iv)
}

// roundCorners rounds a corner reading outward (cornerBounds); ok is false
// when a value overflows a float.
func roundCorners(r cornerReading) (cornerBounds, bool) {
	b, ok := linkagebound.RoundCorners(linkagebound.Reading{
		Pos: r.pos, Vel: r.vel, Pad: r.pad, PrismK: r.prismK,
	})
	return cornerBounds{lo: b.Lo, hi: b.Hi, vel: b.Vel, pad: b.Pad, prismK: b.PrismK}, ok
}

// projectionSide is one body of a pair as docs/linkage-check-design.md §5.8
// expands it over an interval: its rounded corner reading at one end, the
// travel bound h of each joint on its relative path over the interval, and
// the remainder those give. A static body has no joint and no remainder.
//
// seg, when set, is the segment term's step from this end: the enclosure of
// each joint's signed change Δq_i toward the other end of an interval that
// holds no waypoint, along which every joint moves together on one straight
// joint-space segment. The first-order term is then max(0, Σ_i v_i[axis]·Δq_i)
// rather than the box form's Σ_i |v_i[axis]|·h_i.
type projectionSide struct {
	corners cornerBounds
	h       []*big.Rat
	seg     []proofbound.RatInterval
	rem     *big.Rat
}

func (s projectionSide) extents() (up, down [3]*big.Rat) {
	return s.boundSide().Extents()
}

// jointStep is Δq_i of docs/linkage-check-design.md §5.8's segment term for
// joint jt between the fractions sa < sb: the enclosure, widened to floats,
// of q(sb) − q(sa) in the base unit, 2π·Δturn + Δbase with π over its
// enclosure. ok is false when a waypoint lies strictly inside (sa, sb) at
// which the joint's schedule bends — its two neighbouring segments differ in
// turn or in base, so q is not one affine function of s across it — or when
// an end overflows. A waypoint on the straight line through its neighbours at
// equal shares bends nothing (§2.3).
func jointStep(jt linkJoint, sa, sb *big.Rat) (proofbound.RatInterval, bool) {
	n := len(jt.points) - 1
	for j := 1; j < n; j++ {
		w := big.NewRat(int64(j), int64(n))
		if w.Cmp(sa) <= 0 || w.Cmp(sb) >= 0 {
			continue
		}
		prev, at, next := jt.points[j-1], jt.points[j], jt.points[j+1]
		bent := new(big.Rat).Sub(at.Turn, prev.Turn).Cmp(new(big.Rat).Sub(next.Turn, at.Turn)) != 0 ||
			new(big.Rat).Sub(at.Base, prev.Base).Cmp(new(big.Rat).Sub(next.Base, at.Base)) != 0
		if bent {
			return proofbound.RatInterval{}, false
		}
	}
	a, b := jointParam(jt, sa), jointParam(jt, sb)
	turn := new(big.Rat).Sub(b.Turn, a.Turn)
	base := new(big.Rat).Sub(b.Base, a.Base)
	step := proofbound.IntervalAdd(proofbound.IntervalScale(proofbound.TwoPiInterval(), turn), proofbound.PointInterval(base))
	return roundOut(step)
}

func projectionLower(a, b projectionSide) *big.Rat {
	return linkagebound.Lower(a.boundSide(), b.boundSide())
}

// bodySymmetryAxis is the exact axis line of a body that every rotation
// about that line carries onto itself (docs/linkage-check-design.md §5.2):
// a point on it and its direction, as exact rationals. It admits only bodies
// whose geometry is exact under a cardinal frame and a placement that
// permutes and signs the coordinate axes — a solid prism whose profile is one
// whole circle (sourceCylinderAtPose) and a solid full revolve whose axis the
// record states exactly — and answers ok false for anything else.
func bodySymmetryAxis(b *Body) (point, dir motionbound.RatVec, ok bool) {
	toRat := func(v proofarith.DyV3) motionbound.RatVec {
		return motionbound.RatVec{v[0].Rat(), v[1].Rat(), v[2].Rat()}
	}
	if rp, isRevolve := b.payload.(revolvePayload); isRevolve {
		if !b.solid || b.kind != BodySolid || rp.surfaceResult || !rp.full || rp.sectionDelta != 0 ||
			!cardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) || !signedAxisTransform(rp.xform) ||
			!proofbound.FiniteVec(rp.frame.Origin()) || !finiteMeasurementValues(rp.ax.aU, rp.ax.aV, rp.ax.dU, rp.ax.dV) ||
			rp.ax.aUBound != 0 || rp.ax.aVBound != 0 || rp.ax.dUBound != 0 || rp.ax.dVBound != 0 ||
			(rp.ax.dU == 0 && rp.ax.dV == 0) {
			return motionbound.RatVec{}, motionbound.RatVec{}, false
		}
		anchor := proofarith.DvAdd(proofarith.DyVec(rp.frame.Origin()), proofarith.DvAdd(
			dyScaleVec(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.aU)),
			dyScaleVec(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.aV))))
		w := proofarith.DvAdd(dyScaleVec(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.dU)),
			dyScaleVec(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.dV)))
		far := exactContactTransform(rp.xform, proofarith.DvAdd(anchor, w))
		anchor = exactContactTransform(rp.xform, anchor)
		var d motionbound.RatVec
		for i := range 3 {
			d[i] = new(big.Rat).Sub(far[i].Rat(), anchor[i].Rat())
		}
		return toRat(anchor), d, true
	}
	cylinder, isCylinder := sourceCylinderAtPose(b, r3.Identity())
	if !isCylinder {
		return motionbound.RatVec{}, motionbound.RatVec{}, false
	}
	half := big.NewRat(1, 2)
	for i := range 3 {
		point[i] = new(big.Rat).Add(cylinder.box.lo[i].Rat(), cylinder.box.hi[i].Rat())
		point[i].Mul(point[i], half)
		dir[i] = new(big.Rat)
	}
	dir[cylinder.axis].SetInt64(1)
	return point, dir, true
}

// symmetricAboutJoint reports whether body b, a body of link jt, does not
// move under its own joint (docs/linkage-check-design.md §5.2): jt is a
// revolute whose axis is exactly parallel to b's symmetry axis and whose
// centre lies exactly on that axis line.
func symmetricAboutJoint(b *Body, jt linkJoint, f motionbound.MotionFrame) bool {
	if !jt.revolute {
		return false
	}
	point, dir, ok := bodySymmetryAxis(b)
	if !ok || !ratZero(ratCross(dir, f.Axis)) {
		return false
	}
	var offset motionbound.RatVec
	for i := range 3 {
		offset[i] = new(big.Rat).Sub(f.Center[i], point[i])
	}
	return ratZero(ratCross(offset, dir))
}

// withoutOwnJoint is b's reading with its own joint, the last on its path,
// dropped: the reading of a body that joint does not move. ρ_{ik} of every
// joint above stays an upper bound, read over the link's whole rest box.
func withoutOwnJoint(b linkBound) linkBound {
	n := len(b.path) - 1
	return linkBound{path: b.path[:n], rho: b.rho[:n], reach: b.reach}
}

// bodyHullPoints is docs/linkage-check-design.md §5.8's hull point reading of
// a body: points whose convex hull, padded by a ball of radius pad, holds the
// body the payload denotes. A straight prism whose outer loop is all line
// segments answers its outer vertices at its two levels, k bottom then k top,
// each the exact rational image of its recorded floats through the frame and
// the placement read exactly, and pad = 4·√3 times the largest of its section
// and level displacements, rounded up — the charge prismPointBound makes
// through the two near-orthonormal maps. ok is false for any other payload.
func bodyHullPoints(b *Body) (points []motionbound.RatVec, pad *big.Rat, k int, ok bool) {
	pp, isPrism := b.payload.(prismPayload)
	if !isPrism || len(pp.profile.Outer.Segments) < 3 {
		return nil, nil, 0, false
	}
	ratOf := func(v r3.Vec) (motionbound.RatVec, bool) { return motionbound.RatVecOf(v) }
	origin, okO := ratOf(pp.frame.Origin())
	fu, okU := ratOf(pp.frame.U())
	fv, okV := ratOf(pp.frame.V())
	fn, okN := ratOf(pp.frame.N())
	basis := pp.xform.Basis()
	ex, okX := ratOf(basis.EX)
	ey, okY := ratOf(basis.EY)
	ez, okZ := ratOf(basis.EZ)
	shift, okT := ratOf(pp.xform.Translation())
	if !okO || !okU || !okV || !okN || !okX || !okY || !okZ || !okT {
		return nil, nil, 0, false
	}
	lift := func(u, v, z *big.Rat) motionbound.RatVec {
		var local, out motionbound.RatVec
		for i := range 3 {
			local[i] = proofbound.RatAdd(origin[i], proofbound.RatMul(fu[i], u), proofbound.RatMul(fv[i], v), proofbound.RatMul(fn[i], z))
		}
		for i := range 3 {
			out[i] = proofbound.RatAdd(proofbound.RatMul(ex[i], local[0]), proofbound.RatMul(ey[i], local[1]),
				proofbound.RatMul(ez[i], local[2]), shift[i])
		}
		return out
	}
	z0, z1 := proofarith.FloatRat(pp.z0), proofarith.FloatRat(pp.z1)
	if z0 == nil || z1 == nil {
		return nil, nil, 0, false
	}
	k = len(pp.profile.Outer.Segments)
	points = make([]motionbound.RatVec, 2*k)
	for n, seg := range pp.profile.Outer.Segments {
		line, isLine := seg.(LineSeg)
		if !isLine {
			return nil, nil, 0, false
		}
		u, v := proofarith.FloatRat(line.Start.U), proofarith.FloatRat(line.Start.V)
		if u == nil || v == nil {
			return nil, nil, 0, false
		}
		points[n], points[k+n] = lift(u, v, z0), lift(u, v, z1)
	}
	padF := proofbound.ProductUpper(4, proofbound.Radius3D(max(pp.sectionDelta, pp.z0Delta, pp.z1Delta)))
	if pad = proofarith.FloatRat(padF); pad == nil {
		return nil, nil, 0, false
	}
	return points, pad, k, true
}

func (s projectionSide) boundSide() linkagebound.Side {
	return linkagebound.Side{
		Corners: linkagebound.Bounds{
			Lo: s.corners.lo, Hi: s.corners.hi, Vel: s.corners.vel,
			Pad: s.corners.pad, PrismK: s.corners.prismK,
		},
		H: s.h, Seg: s.seg, Rem: s.rem,
	}
}

func projectionLowerHull(a, b projectionSide) *big.Rat {
	return linkagebound.LowerHull(a.boundSide(), b.boundSide())
}
