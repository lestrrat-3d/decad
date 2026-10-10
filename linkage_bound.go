package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"

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

type linkBound = linkagebound.ReachBound
type cornerReading = linkagebound.Reading
type cornerBounds = linkagebound.Bounds
type projectionSide = linkagebound.Side

// linkageFrames reads every joint's exact frame (motionbound.MotionFrame);
// ok is false when an axis, direction or centre is not representable.
func linkageFrames(spec *linkageSpec) ([]motionbound.MotionFrame, bool) {
	frames := make([]motionbound.MotionFrame, len(spec.joints))
	for k, jt := range spec.joints {
		ms := motionbound.Spec{Kind: motionbound.MotionPrismatic, Dir: jt.axis}
		if jt.revolute {
			ms = motionbound.Spec{Kind: motionbound.MotionRevolute, Center: jt.center, Axis: jt.axis}
		}
		frame, ok := motionbound.NewMotionFrame(ms)
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
	return linkagebound.JointReach(jt.points)
}

// jointParam is joint jt's exact value at the fraction s: its segment's
// exact interpolation at the local fraction.
func jointParam(jt linkJoint, s *big.Rat) motionbound.MotionParam {
	seg, t := jt.segment(s)
	return seg.FromP.Lerp(seg.ToP, t)
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
	return linkagebound.JointSpan(jt.points, sa, sb, func(s *big.Rat) motionbound.MotionParam {
		return jointParam(jt, s)
	})
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
	bodies := s.spec.joints[i].link.bodies
	boxes := make([]measurement.Box, len(bodies))
	for n, body := range bodies {
		boxes[n] = body.bounds
	}
	return linkagebound.RestBox(boxes)
}
func (s reachSource) Reach(i int) *big.Rat { return jointReach(s.spec.joints[i]) }

func readLinkBounds(spec *linkageSpec, frames []motionbound.MotionFrame) ([]linkBound, bool) {
	return linkagebound.ReadReachBounds(reachSource{spec: spec, frames: frames})
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
	return linkagebound.PathTravel(b.Path, b.Rho, below, func(joint int) *big.Rat {
		jt := spec.joints[joint]
		if jt.dep == nil {
			return jointSpan(jt, sa, sb)
		}
		return jt.dep.span(joint, sa, sb)
	})
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
		for _, i := range b.Path {
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

// layerExtent reads a body's inflated rest box and delegates its exact a-extents.
func layerExtent(b *Body, a motionbound.RatVec) (lo, hi *big.Rat, ok bool) {
	boxLo, boxHi, ok := motionbound.BoxCornersExact(b.bounds, new(big.Rat))
	if !ok {
		return nil, nil, false
	}
	lo, hi = linkagebound.LayerExtent(boxLo, boxHi, a)
	return lo, hi, true
}

// layerLower applies the exact layer exclusion to the root linkage bodies.
func layerLower(spec *linkageSpec, frames []motionbound.MotionFrame, path []int, x, y *Body) (float64, bool) {
	joints := make([]linkagebound.LayerJoint, len(spec.joints))
	for _, i := range path {
		jt := spec.joints[i]
		joints[i] = linkagebound.LayerJoint{
			Axis: frames[i].Axis, Revolute: jt.revolute, HeldAtZero: heldAtZeroJoint(jt),
		}
	}
	return linkagebound.LayerLower(joints, path, func(a motionbound.RatVec) (xLo, xHi, yLo, yHi *big.Rat, ok bool) {
		xLo, xHi, okX := layerExtent(x, a)
		yLo, yHi, okY := layerExtent(y, a)
		return xLo, xHi, yLo, yHi, okX && okY
	})
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
	return readPoints(spec, frames, params, b, below, linkagebound.StaticPoints(corners[:]))
}

// readPoints maps a static point reading through the link's relative joint path.
func readPoints(spec *linkageSpec, frames []motionbound.MotionFrame, params []motionbound.MotionParam, b linkBound, below int, out cornerReading) cornerReading {
	revolute := make([]bool, len(spec.joints))
	for i := range revolute {
		revolute[i] = spec.joints[i].revolute
	}
	return linkagebound.ReadPoints(frames, params, b.Path, revolute, below, out)
}

// jointStep reads a joint's signed interval step when no interior waypoint bends it.
func jointStep(jt linkJoint, sa, sb *big.Rat) (proofbound.RatInterval, bool) {
	return linkagebound.JointStep(jt.points, sa, sb, func(s *big.Rat) motionbound.MotionParam {
		return jointParam(jt, s)
	})
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
			!pairbox.CardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) || !signedAxisTransform(rp.xform) ||
			!proofbound.FiniteVec(rp.frame.Origin()) || !finiteMeasurementValues(rp.ax.AU, rp.ax.AV, rp.ax.DU, rp.ax.DV) ||
			rp.ax.AUBound != 0 || rp.ax.AVBound != 0 || rp.ax.DUBound != 0 || rp.ax.DVBound != 0 ||
			(rp.ax.DU == 0 && rp.ax.DV == 0) {
			return motionbound.RatVec{}, motionbound.RatVec{}, false
		}
		anchor := proofarith.DvAdd(proofarith.DyVec(rp.frame.Origin()), proofarith.DvAdd(
			proofarith.DvScale(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.AU)),
			proofarith.DvScale(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.AV))))
		w := proofarith.DvAdd(proofarith.DvScale(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.DU)),
			proofarith.DvScale(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.DV)))
		far := proofarith.DvTransform(rp.xform, proofarith.DvAdd(anchor, w))
		anchor = proofarith.DvTransform(rp.xform, anchor)
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
	if !ok || !linkagebound.ZeroVec(linkagebound.Cross(dir, f.Axis)) {
		return false
	}
	var offset motionbound.RatVec
	for i := range 3 {
		offset[i] = new(big.Rat).Sub(f.Center[i], point[i])
	}
	return linkagebound.ZeroVec(linkagebound.Cross(offset, dir))
}
