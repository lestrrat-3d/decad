package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file proves the bounds docs/motion-check-design.md §5 builds its
// interval certificate from, every one of them over exact rationals so that no
// float rounding sits between a bound and the claim it supports:
//
//   - the exact denotation of a motion parameter (motionbound.MotionParam) and of the
//     ideal pose it names (motionbound.IdealPose), with the ideal rotation's sine and
//     cosine enclosed by internal/proofbound/moments_trig.go's proofbound.TurnSinCosInterval; for a Between,
//     the exact screw of the parameters r3 read, composed onto the exact
//     From, and the stated To read exactly (motionbound.MotionFrame);
//   - η, the proven distance between the float pose the kernel measured and
//     the ideal pose the claim is about (motionbound.PoseDeviation, §5.1);
//   - R0, the record-coordinate radius η is charged at (moverRecordRadius);
//   - ρ_max, the largest distance from the rotation or screw axis of any
//     point of a mover (moverAxisRadius, §5.2);
//   - the travel bound τ and the swept-box exclusion (§5.2, §6);
//   - the area the collision transfer charges its swept-volume allowance at
//     (motionbound.PathAreaUpper, motionbound.BasisSigmaLower, motionbound.BasisSigmaUpper, §5.1);
//   - the resolution floor's width comparison (motionbound.ExceedsResolution, §6).

// The common-denominator form below (internal/proof's CommonDenom) evaluates
// an interval expression over many exact points with integer arithmetic.
// big.Rat reduces to lowest terms after every operation, a Lehmer GCD over
// operands whose denominators carry π's enclosure and a rotation axis's 1/|a|,
// hundreds of bits wide. Written over one shared denominator, the same
// expression needs only integer multiply-adds, and a result converted back
// (SetFrac) reduces to the same lowest terms: each one is the exact rational,
// bit for bit, the big.Rat evaluation produces.

func newMotionFrame(spec motionSpec) (motionbound.MotionFrame, bool) {
	dirVec := spec.dir
	center := r3.Vec{}
	switch spec.kind {
	case motionbound.MotionRevolute:
		dirVec, center = spec.axis, spec.center
	case motionbound.MotionBetween:
		dirVec, center = spec.screw.Axis, spec.screw.Point
	}
	axis, okA := motionbound.RatVecOf(dirVec)
	pivot, okC := motionbound.RatVecOf(center)
	if !okA || !okC {
		return motionbound.MotionFrame{}, false
	}
	unit, ok := motionbound.UnitScaleInterval(axis)
	if !ok {
		return motionbound.MotionFrame{}, false
	}
	mf := motionbound.MotionFrame{Kind: spec.kind, Axis: axis, Unit: unit, Center: pivot}
	if spec.kind != motionbound.MotionBetween {
		return mf, true
	}
	theta, okT := motionbound.ExactMotionParam(spec.screw.Angle)
	slide := proofarith.FloatRat(spec.screw.Slide)
	fromRot, fromT, okF := motionbound.ExactTransform(spec.between.From)
	toRot, toT, okTo := motionbound.ExactTransform(spec.between.To)
	if !okT || slide == nil || !okF || !okTo {
		return motionbound.MotionFrame{}, false
	}
	mf.Theta, mf.Slide = theta, slide
	mf.FromRot, mf.FromT, mf.ToRot, mf.ToT = fromRot, fromT, toRot, toT
	return mf, true
}

// moverRecordRadius is R0 of docs/motion-check-design.md §5.1: a proven upper
// bound on |p| for every point p of the mover's record before its own
// placement applies, read off the payload's own envelopes. It is an L1 bound,
// which dominates the Euclidean one. A prism reads its profile coordinate
// envelope (prism_payload.go) widened by its section displacement, and its
// sweep levels widened by their axial displacement; a revolve reads the
// generator envelope and axis anchor revolveCentroidGeometryBound already
// bounds a rotated material point with. Any other payload states no record
// radius and answers +Inf, which motionbound.PoseDeviation charges only where the pose's
// linear part departs from the ideal one.
func moverRecordRadius(ctx context.Context, b *Body) float64 {
	if ctx.Err() != nil {
		return math.Inf(1)
	}
	switch pl := b.payload.(type) {
	case prismPayload:
		coordUpper, err := profileCoordinateEnvelope(pl.profile, freeform.NewFreeformWork(), pl.walks)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		zUpper := proofbound.AbsSumUpper(math.Max(math.Abs(pl.z0), math.Abs(pl.z1)), pl.axialDelta())
		return proofbound.AbsSumUpper(
			vecL1(pl.frame.Origin()),
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.N()), zUpper),
		)
	case revolvePayload:
		coordUpper, err := profileCoordinateUpper(pl.profile, freeform.NewFreeformWork(), nil)
		if err != nil {
			return math.Inf(1)
		}
		coordUpper = proofbound.AbsSumUpper(coordUpper, pl.sectionDelta)
		originUpper := vecL1(pl.frame.Origin())
		profileUpper := proofbound.AbsSumUpper(
			originUpper,
			proofbound.ProductUpper(vecL1(pl.frame.U()), coordUpper),
			proofbound.ProductUpper(vecL1(pl.frame.V()), coordUpper),
		)
		axisUpper := proofbound.AbsSumUpper(
			originUpper,
			proofbound.ProductUpper(vecL1(pl.frame.U()), proofbound.AbsSumUpper(pl.ax.aU, pl.ax.aUBound)),
			proofbound.ProductUpper(vecL1(pl.frame.V()), proofbound.AbsSumUpper(pl.ax.aV, pl.ax.aVBound)),
		)
		return proofbound.AbsSumUpper(proofbound.ProductUpper(3, profileUpper), proofbound.ProductUpper(4, axisUpper))
	default:
		return math.Inf(1)
	}
}

// boxCornersExact is the mover's Bounds box inflated outward by its own Bound
// and by an extra margin, as exact rational extremes per axis. The true body
// lies inside the box inflated by its Bound (core §5.3), so the inflated box
// encloses every point the claim speaks for.
func boxCornersExact(box Box, extra *big.Rat) (lo, hi motionbound.RatVec, ok bool) {
	minV, okMin := motionbound.RatVecOf(box.Min)
	maxV, okMax := motionbound.RatVecOf(box.Max)
	bound := proofarith.FloatRat(box.Bound.Base())
	if !okMin || !okMax || bound == nil {
		return motionbound.RatVec{}, motionbound.RatVec{}, false
	}
	pad := new(big.Rat).Add(bound, extra)
	for i := range 3 {
		lo[i] = new(big.Rat).Sub(minV[i], pad)
		hi[i] = new(big.Rat).Add(maxV[i], pad)
	}
	return lo, hi, true
}

// startCorners is the eight corners of the mover's Bounds box inflated by its
// own Bound, as exact rationals, each mapped to where the path STARTS:
// unchanged for a Revolute and a Prismatic, whose parameter 0 is the mover at
// rest, and through the between's From exactly for a Between (placeFrom). The
// true body at the start lies inside the convex hull of these eight points.
func startCorners(box Box, mf motionbound.MotionFrame) ([8]motionbound.RatVec, bool) {
	lo, hi, ok := boxCornersExact(box, new(big.Rat))
	if !ok {
		return [8]motionbound.RatVec{}, false
	}
	var out [8]motionbound.RatVec
	for corner := range 8 {
		var x motionbound.RatVec
		for i := range 3 {
			x[i] = lo[i]
			if corner&(1<<i) != 0 {
				x[i] = hi[i]
			}
		}
		out[corner] = mf.PlaceFrom(x)
	}
	return out, true
}

// moverAxisRadius is ρ_max of docs/motion-check-design.md §5.2: a proven
// upper bound on the distance from the rotation axis of every point of the
// mover, read ONCE off its Bounds box at its current placement. The box is
// inflated by its own Bound, its eight corners are mapped to the path's start
// (startCorners: through From exactly for a Between, whose screw axis passes
// nowhere near the rest box in general), and each image's squared distance
// from the axis line, |(x − c) × a|²/|a|², is taken exactly over rationals;
// distance from a line is convex, so its maximum over the hull of the images
// sits at one of them. proofbound.RatSqrtUp roots the largest. A rotation about the axis
// and a slide along it both preserve every point's distance from it, so this
// one reading covers every pose.
func moverAxisRadius(b *Body, mf motionbound.MotionFrame) float64 {
	corners, ok := startCorners(b.bounds, mf)
	if !ok {
		return math.Inf(1)
	}
	a := mf.Axis
	sq := proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
	best := new(big.Rat)
	for _, x := range corners {
		var w motionbound.RatVec
		for i := range 3 {
			w[i] = new(big.Rat).Sub(x[i], mf.Center[i])
		}
		cx := new(big.Rat).Sub(proofbound.RatMul(w[1], a[2]), proofbound.RatMul(w[2], a[1]))
		cy := new(big.Rat).Sub(proofbound.RatMul(w[2], a[0]), proofbound.RatMul(w[0], a[2]))
		cz := new(big.Rat).Sub(proofbound.RatMul(w[0], a[1]), proofbound.RatMul(w[1], a[0]))
		d := proofbound.RatAdd(proofbound.RatMul(cx, cx), proofbound.RatMul(cy, cy), proofbound.RatMul(cz, cz))
		d.Quo(d, sq)
		if d.Cmp(best) > 0 {
			best = d
		}
	}
	return proofbound.RatSqrtUp(best)
}

// moverSweptBox is §6 step 3's swept box: every point the mover occupies over
// the whole path, as exact rational extremes per axis. A Revolute or a
// Prismatic reads its Bounds box at rest, inflated by its own Bound plus
// travel — the farthest any of its points moves from where it sits now. A
// Between's path starts at From, which the rest box has not undergone, so it
// reads the From-placed box: the axis-aligned hull of startCorners, which
// already carry the box's Bound, inflated by travel and by nothing else.
func moverSweptBox(box Box, mf motionbound.MotionFrame, travel *big.Rat) (motionbound.RatVec, motionbound.RatVec, bool) {
	if travel == nil {
		return motionbound.RatVec{}, motionbound.RatVec{}, false
	}
	if mf.Kind != motionbound.MotionBetween {
		return boxCornersExact(box, travel)
	}
	corners, ok := startCorners(box, mf)
	if !ok {
		return motionbound.RatVec{}, motionbound.RatVec{}, false
	}
	var lo, hi motionbound.RatVec
	for i := range 3 {
		lo[i], hi[i] = corners[0][i], corners[0][i]
		for _, c := range corners[1:] {
			if c[i].Cmp(lo[i]) < 0 {
				lo[i] = c[i]
			}
			if c[i].Cmp(hi[i]) > 0 {
				hi[i] = c[i]
			}
		}
		lo[i] = new(big.Rat).Sub(lo[i], travel)
		hi[i] = new(big.Rat).Add(hi[i], travel)
	}
	return lo, hi, true
}

// sweptBoxLower is §6 step 3's swept-box exclusion for one (mover, static)
// pair, decided over exact rationals: the mover's swept box (moverSweptBox)
// against the static body's box inflated by its own Bound. Boxes separated by
// a strictly positive gap along some axis prove the pair apart at every
// parameter, and the largest such axis gap is a proven lower bound on the
// pair's distance over the whole path. ok is false when the boxes do not
// separate.
func sweptBoxLower(mLo, mHi motionbound.RatVec, static Box) (float64, bool) {
	sLo, sHi, okS := boxCornersExact(static, new(big.Rat))
	if !okS {
		return 0, false
	}
	var best *big.Rat
	for i := range 3 {
		for _, gap := range []*big.Rat{new(big.Rat).Sub(sLo[i], mHi[i]), new(big.Rat).Sub(mLo[i], sHi[i])} {
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
