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
//     link body's box by (§6 step 4).
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
// revolute and |Δq_i| for a prismatic, |Δq_i| taken by jointSpan. below is the position on the path of the
// first joint below L: 0 when L is the ground.
func chainTravel(spec *linkageSpec, b linkBound, below int, sa, sb *big.Rat) *big.Rat {
	return pathTravel(b, below, func(joint int) *big.Rat { return jointSpan(spec.joints[joint], sa, sb) })
}

// pathTravel is the telescoping sum of docs/linkage-check-design.md §5.2 over
// the joints on b's path from position below on: ρ_{ik}·span(i) for a
// revolute joint i and span(i) for a prismatic one, span(i) a proven upper
// bound on joint i's travel. span MUST return a fresh rational.
func pathTravel(b linkBound, below int, span func(joint int) *big.Rat) *big.Rat {
	sum := new(big.Rat)
	for n := below; n < len(b.path); n++ {
		term := span(b.path[n])
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
			if jt.listed && jt.moves() {
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
