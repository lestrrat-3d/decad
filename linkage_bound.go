package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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

// ratBall is a ball of exact rational centre and radius.
type ratBall struct {
	c motionbound.RatVec
	r *big.Rat
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

// distanceSq is |x − c|², exactly.
func distanceSq(x, c motionbound.RatVec) *big.Rat {
	sum := new(big.Rat)
	for i := range 3 {
		d := new(big.Rat).Sub(x[i], c[i])
		sum.Add(sum, d.Mul(d, d))
	}
	return sum
}

// lineDistanceSq is the squared distance of x from the line through c along
// a, |(x − c) × a|²/|a|², exactly; aSq is |a|².
func lineDistanceSq(x, c, a motionbound.RatVec, aSq *big.Rat) *big.Rat {
	var w motionbound.RatVec
	for i := range 3 {
		w[i] = new(big.Rat).Sub(x[i], c[i])
	}
	cx := new(big.Rat).Sub(proofbound.RatMul(w[1], a[2]), proofbound.RatMul(w[2], a[1]))
	cy := new(big.Rat).Sub(proofbound.RatMul(w[2], a[0]), proofbound.RatMul(w[0], a[2]))
	cz := new(big.Rat).Sub(proofbound.RatMul(w[0], a[1]), proofbound.RatMul(w[1], a[0]))
	d := proofbound.RatAdd(proofbound.RatMul(cx, cx), proofbound.RatMul(cy, cy), proofbound.RatMul(cz, cz))
	return d.Quo(d, aSq)
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

// boxCorners is the eight corners of an exact box.
func boxCorners(lo, hi motionbound.RatVec) [8]motionbound.RatVec {
	var out [8]motionbound.RatVec
	for corner := range 8 {
		for i := range 3 {
			out[corner][i] = lo[i]
			if corner&(1<<i) != 0 {
				out[corner][i] = hi[i]
			}
		}
	}
	return out
}

// readLinkBounds is §5.2's reading for every link: its path, ρ_{ik} for each
// revolute joint on the path, and its reach. It walks DOWN each link's path,
// from its own joint toward the ground, carrying a ball that encloses the
// link under the joints walked so far:
//
//   - ρ_{kk} for the link's own revolute joint is the largest distance of a
//     rest-box corner from the joint's axis; distance from a line is convex,
//     so the maximum over the box sits at a corner;
//   - the ball under the link's own joint is centred on a revolute's Center
//     with radius the largest corner distance from it (a rotation about any
//     axis through the centre keeps every distance to it), or on the box
//     centre with radius the half-diagonal plus m_k for a prismatic;
//   - ρ_{ik} for a revolute joint i above is dist(c, axis_i) + R for the ball
//     (c, R) under the joints below i;
//   - the ball under joint i is (c_i, |c − c_i| + R) for a revolute — every
//     point within R of c stays within |c − c_i| + R of c_i under any
//     rotation about an axis through c_i — and (c, R + m_i) for a prismatic.
//
// ok is false when a box or an axis cannot be read exactly or a root
// overflows.
func readLinkBounds(spec *linkageSpec, frames []motionbound.MotionFrame) ([]linkBound, bool) {
	bounds := make([]linkBound, len(spec.joints))
	for k, jt := range spec.joints {
		var path []int
		for at := k; at >= 0; at = spec.joints[at].parent {
			path = append([]int{at}, path...)
		}
		lo, hi, ok := linkRestBox(jt.link)
		if !ok {
			return nil, false
		}
		rho := make([]*big.Rat, len(path))
		ball, own, ok := ownJointBall(jt, frames[k], lo, hi)
		if !ok {
			return nil, false
		}
		rho[len(path)-1] = own
		for n := len(path) - 2; n >= 0; n-- {
			i := path[n]
			above := spec.joints[i]
			if !above.revolute {
				ball = ratBall{c: ball.c, r: new(big.Rat).Add(ball.r, jointReach(above))}
				continue
			}
			f := frames[i]
			d := sqrtUpRat(lineDistanceSq(ball.c, f.Center, f.Axis, axisSq(f.Axis)))
			toCenter := sqrtUpRat(distanceSq(ball.c, f.Center))
			if d == nil || toCenter == nil {
				return nil, false
			}
			rho[n] = new(big.Rat).Add(d, ball.r)
			ball = ratBall{c: f.Center, r: toCenter.Add(toCenter, ball.r)}
		}
		reach := new(big.Rat)
		for n, i := range path {
			m := jointReach(spec.joints[i])
			if rho[n] != nil {
				m.Mul(m, rho[n])
			}
			reach.Add(reach, m)
		}
		bounds[k] = linkBound{path: path, rho: rho, reach: reach}
	}
	return bounds, true
}

// ownJointBall reads a link's own joint: ρ_{kk} for a revolute (nil for a
// prismatic) and the ball enclosing the link under that joint at any value.
func ownJointBall(jt linkJoint, f motionbound.MotionFrame, lo, hi motionbound.RatVec) (ratBall, *big.Rat, bool) {
	corners := boxCorners(lo, hi)
	if !jt.revolute {
		var c, half motionbound.RatVec
		for i := range 3 {
			c[i] = new(big.Rat).Add(lo[i], hi[i])
			c[i].Quo(c[i], big.NewRat(2, 1))
			half[i] = new(big.Rat).Sub(hi[i], c[i])
		}
		r := sqrtUpRat(distanceSq(half, motionbound.RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}))
		if r == nil {
			return ratBall{}, nil, false
		}
		return ratBall{c: c, r: r.Add(r, jointReach(jt))}, nil, true
	}
	aSq := axisSq(f.Axis)
	axisFar, centreFar := new(big.Rat), new(big.Rat)
	for _, x := range corners {
		if d := lineDistanceSq(x, f.Center, f.Axis, aSq); d.Cmp(axisFar) > 0 {
			axisFar = d
		}
		if d := distanceSq(x, f.Center); d.Cmp(centreFar) > 0 {
			centreFar = d
		}
	}
	rho, r := sqrtUpRat(axisFar), sqrtUpRat(centreFar)
	if rho == nil || r == nil {
		return ratBall{}, nil, false
	}
	return ratBall{c: f.Center, r: r}, rho, true
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

func ratDot(a, b motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

func ratCross(a, b motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{
		new(big.Rat).Sub(proofbound.RatMul(a[1], b[2]), proofbound.RatMul(a[2], b[1])),
		new(big.Rat).Sub(proofbound.RatMul(a[2], b[0]), proofbound.RatMul(a[0], b[2])),
		new(big.Rat).Sub(proofbound.RatMul(a[0], b[1]), proofbound.RatMul(a[1], b[0])),
	}
}

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
	for n, x := range boxCorners(boxLo, boxHi) {
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
// one configuration: the eight corners of its inflated rest box under the
// ideal poses of the joints on its relative path, and each corner's velocity
// under each of those joints alone. Every entry is a rational interval that
// encloses the exact value for every member of the ideal poses' enclosures.
type cornerReading struct {
	pos [8]motionbound.IvVec
	// vel holds, per corner, its velocity under each joint on the relative
	// path, shallowest first: ω × (x − o) for a revolute, the unit direction
	// for a prismatic, per radian or per millimetre of the joint's value.
	vel [8][]motionbound.IvVec
}

// staticCorners is the corner reading of a box no joint moves: its eight
// corners as points, with no velocity.
func staticCorners(lo, hi motionbound.RatVec) cornerReading {
	var out cornerReading
	for c, x := range boxCorners(lo, hi) {
		out.pos[c] = motionbound.PointVec(x)
	}
	return out
}

// applyIdeal maps an enclosed point through an ideal pose,
// x ↦ rot·(x − pivot) + pivot + shift.
func applyIdeal(p motionbound.IdealPose, x motionbound.IvVec) motionbound.IvVec {
	return motionbound.IvVecAdd(motionbound.IvVecAdd(p.Rot.Apply(motionbound.IvVecSub(x, p.Pivot)), p.Pivot), p.Shift)
}

// ivCross is the cross product a × b over rational intervals.
func ivCross(a, b motionbound.IvVec) motionbound.IvVec {
	term := func(i, j int) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a[i], b[j]), proofbound.IntervalMul(a[j], b[i]))
	}
	return motionbound.IvVec{term(1, 2), term(2, 0), term(0, 1)}
}

// ivAbsUpper is the largest magnitude an enclosure allows.
func ivAbsUpper(iv proofbound.RatInterval) *big.Rat {
	out := new(big.Rat).Abs(iv.Lo)
	if hi := new(big.Rat).Abs(iv.Hi); hi.Cmp(out) > 0 {
		out = hi
	}
	return out
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
	type jointAt struct {
		revolute    bool
		unit, pivot motionbound.IvVec
	}
	joints := make([]jointAt, 0, len(b.path)-below)
	var pose *motionbound.IdealPose
	for _, i := range b.path[below:] {
		f := frames[i]
		var unit motionbound.IvVec
		for d := range 3 {
			unit[d] = proofbound.IntervalScale(f.Unit, f.Axis[d])
		}
		pivot := motionbound.PointVec(f.Center)
		ideal := f.At(params[i])
		if pose != nil {
			unit = pose.Rot.Apply(unit)
			pivot = applyIdeal(*pose, pivot)
			ideal = ideal.Then(*pose)
		}
		joints = append(joints, jointAt{revolute: spec.joints[i].revolute, unit: unit, pivot: pivot})
		pose = &ideal
	}
	out := staticCorners(lo, hi)
	for c := range out.pos {
		if pose != nil {
			out.pos[c] = applyIdeal(*pose, out.pos[c])
		}
		out.vel[c] = make([]motionbound.IvVec, len(joints))
		for n, j := range joints {
			if !j.revolute {
				out.vel[c][n] = j.unit
				continue
			}
			out.vel[c][n] = ivCross(j.unit, motionbound.IvVecSub(out.pos[c], j.pivot))
		}
	}
	return out
}

// secondDerivativeBound is B_ij of docs/linkage-check-design.md §5.8 for
// the joints at positions m ≤ n of b's path, m the shallower: a proven upper
// bound on |∂²x_c/∂q_i∂q_j| for every corner c of the link at every
// configuration the drive reaches. The shallower joint turns the deeper
// one's velocity, whose length is at most w_j — ρ_jk for a revolute joint j,
// 1 for a prismatic one — so B_ij = w_j when the shallower joint is a
// revolute; a prismatic shallower joint turns nothing, and B_ij is 0, nil
// here.
func secondDerivativeBound(b linkBound, m, n int) *big.Rat {
	if b.rho[m] == nil {
		return nil
	}
	if w := b.rho[n]; w != nil {
		return w
	}
	return big.NewRat(1, 1)
}

// projectionRemainder is Rem(h) = ½·Σ_{i,j} B_ij·h_i·h_j of
// docs/linkage-check-design.md §5.8 over the joints on b's path from
// position below on, h holding each one's travel bound in path order: by
// Taylor's theorem along the straight segment in joint space, it bounds how
// far a corner's position departs from its first-order expansion.
func projectionRemainder(b linkBound, below int, h []*big.Rat) *big.Rat {
	sum := new(big.Rat)
	half := big.NewRat(1, 2)
	for m := range h {
		for n := m; n < len(h); n++ {
			w := secondDerivativeBound(b, below+m, below+n)
			if w == nil {
				continue
			}
			term := new(big.Rat).Mul(h[m], h[n])
			term.Mul(term, w)
			if n == m {
				term.Mul(term, half)
			}
			sum.Add(sum, term)
		}
	}
	return sum
}

// cornerBounds is a corner reading rounded outward to floats and read back
// as exact rationals, so the per-interval sums of docs/linkage-check-design.md
// §5.8 run over short dyadics rather than the long rationals of composed
// rotations: each corner coordinate's enclosure and each velocity
// component's enclosure widened to the floats around it. Rounding outward
// only weakens the bound the values enter.
type cornerBounds struct {
	lo, hi [8][3]*big.Rat
	// vel holds, per corner and per joint on the relative path, each
	// velocity component's enclosure.
	vel [8][][3]proofbound.RatInterval
}

// roundOut widens an enclosure to the floats around it, read back as exact
// rationals; ok is false when an end overflows a float.
func roundOut(iv proofbound.RatInterval) (proofbound.RatInterval, bool) {
	lo := proofarith.FloatRat(proofbound.RatFloatDown(iv.Lo))
	hi := proofarith.FloatRat(proofbound.RatFloatUp(iv.Hi))
	if lo == nil || hi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalOwned(lo, hi), true
}

// roundCorners rounds a corner reading outward (cornerBounds); ok is false
// when a value overflows a float.
func roundCorners(r cornerReading) (cornerBounds, bool) {
	var out cornerBounds
	for c := range r.pos {
		for d := range 3 {
			iv, ok := roundOut(r.pos[c][d])
			if !ok {
				return cornerBounds{}, false
			}
			out.lo[c][d], out.hi[c][d] = iv.Lo, iv.Hi
		}
		out.vel[c] = make([][3]proofbound.RatInterval, len(r.vel[c]))
		for n, v := range r.vel[c] {
			for d := range 3 {
				iv, ok := roundOut(v[d])
				if !ok {
					return cornerBounds{}, false
				}
				out.vel[c][n][d] = iv
			}
		}
	}
	return out, true
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

// firstOrder is the side's first-order terms at corner c along axis d: a
// proven upper bound on how far the expansion moves x[d] (up) and −x[d]
// (down) over the interval. The segment term's sum is linear in the
// fraction t ∈ [0, 1] of the step, so its largest value is at t = 0 or 1:
// max(0, Σ) along x[d] and max(0, −Σ) along −x[d], each over the enclosure
// of Σ. The box form charges |v[d]|·h along both.
func (s projectionSide) firstOrder(c, d int) (up, down *big.Rat) {
	if s.seg != nil {
		sum := proofbound.PointInterval(new(big.Rat))
		for n, v := range s.corners.vel[c] {
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(v[d], s.seg[n]))
		}
		up, down = new(big.Rat), new(big.Rat)
		if sum.Hi.Sign() > 0 {
			up.Set(sum.Hi)
		}
		if sum.Lo.Sign() < 0 {
			down.Neg(sum.Lo)
		}
		return up, down
	}
	lin := new(big.Rat)
	for n, v := range s.corners.vel[c] {
		if s.h[n].Sign() == 0 {
			continue
		}
		term := ivAbsUpper(v[d])
		lin.Add(lin, term.Mul(term, s.h[n]))
	}
	return lin, new(big.Rat).Set(lin)
}

// extents are proven upper bounds on x[axis] (up) and on −x[axis] (down)
// over the body at every parameter of the interval, per axis: the largest
// over its corners of the corner's coordinate end plus its first-order term,
// then the remainder.
func (s projectionSide) extents() (up, down [3]*big.Rat) {
	for c := range s.corners.hi {
		for d := range 3 {
			linUp, linDown := s.firstOrder(c, d)
			hi := linUp.Add(linUp, s.corners.hi[c][d])
			lo := linDown.Sub(linDown, s.corners.lo[c][d])
			if up[d] == nil || hi.Cmp(up[d]) > 0 {
				up[d] = hi
			}
			if down[d] == nil || lo.Cmp(down[d]) > 0 {
				down[d] = lo
			}
		}
	}
	if s.rem != nil {
		for d := range 3 {
			up[d].Add(up[d], s.rem)
			down[d].Add(down[d], s.rem)
		}
	}
	return up, down
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

// projectionLower is the largest L_n of docs/linkage-check-design.md §5.8
// over the six coordinate directions n = ±e_axis: the partner's least extent
// along n less the body's greatest. Along +e_axis that is −b.down − a.up,
// along −e_axis −b.up − a.down. The distance between two sets is at least
// the separation of their projections onto any unit vector, so each L_n is
// a proven lower bound on the pair's gap at every parameter of the interval.
func projectionLower(a, b projectionSide) *big.Rat {
	aUp, aDown := a.extents()
	bUp, bDown := b.extents()
	var best *big.Rat
	for d := range 3 {
		for _, l := range []*big.Rat{
			new(big.Rat).Neg(new(big.Rat).Add(bDown[d], aUp[d])),
			new(big.Rat).Neg(new(big.Rat).Add(bUp[d], aDown[d])),
		} {
			if best == nil || l.Cmp(best) > 0 {
				best = l
			}
		}
	}
	return best
}
