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
// i's value reaches from 0 over the drive, max(|From|, |To|) — radians with π
// at its upper enclosure for an angle, millimetres exactly for a length, and
// 0 for an unlisted joint. q(s) is linear in s, so every value the drive
// visits lies within it.
func jointReach(jt linkJoint) *big.Rat {
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	from, to := zero.SpanUpper(jt.dom.fromP), zero.SpanUpper(jt.dom.toP)
	if from.Cmp(to) >= 0 {
		return from
	}
	return to
}

// jointParam is joint jt's exact value at the fraction s.
func jointParam(jt linkJoint, s *big.Rat) motionbound.MotionParam {
	return jt.dom.fromP.Lerp(jt.dom.toP, s)
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
// revolute and |Δq_i| for a prismatic, |Δq_i| taken by
// motionbound.MotionParam.SpanUpper. below is the position on the path of the
// first joint below L: 0 when L is the ground.
func chainTravel(spec *linkageSpec, b linkBound, below int, sa, sb *big.Rat) *big.Rat {
	sum := new(big.Rat)
	for n := below; n < len(b.path); n++ {
		jt := spec.joints[b.path[n]]
		span := jointParam(jt, sa).SpanUpper(jointParam(jt, sb))
		if b.rho[n] != nil {
			span.Mul(span, b.rho[n])
		}
		sum.Add(sum, span)
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
	ideals := make([]motionbound.IdealPose, len(spec.joints))
	for k, jt := range spec.joints {
		ideal := frames[k].At(jointParam(jt, s))
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
