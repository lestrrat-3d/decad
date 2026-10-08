package motionbound

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// BoxCornersExact is the mover's Bounds box inflated outward by its own Bound
// and by an extra margin, as exact rational extremes per axis. The true body
// lies inside the box inflated by its Bound (core §5.3), so the inflated box
// encloses every point the claim speaks for. extra must not be nil.
func BoxCornersExact(box measurement.Box, extra *big.Rat) (RatVec, RatVec, bool) {
	minV, okMin := RatVecOf(box.Min)
	maxV, okMax := RatVecOf(box.Max)
	bound := proofarith.FloatRat(box.Bound.Base())
	if !okMin || !okMax || bound == nil {
		return RatVec{}, RatVec{}, false
	}
	pad := new(big.Rat).Add(bound, extra)
	var lo, hi RatVec
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
func startCorners(box measurement.Box, mf MotionFrame) ([8]RatVec, bool) {
	lo, hi, ok := BoxCornersExact(box, new(big.Rat))
	if !ok {
		return [8]RatVec{}, false
	}
	var out [8]RatVec
	for corner := range 8 {
		var x RatVec
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

// MoverAxisRadius is ρ_max of docs/motion-check-design.md §5.2: a proven
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
func MoverAxisRadius(box measurement.Box, mf MotionFrame) float64 {
	corners, ok := startCorners(box, mf)
	if !ok {
		return math.Inf(1)
	}
	a := mf.Axis
	sq := proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
	best := new(big.Rat)
	for _, x := range corners {
		var w RatVec
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

// MoverSweptBox is §6 step 3's swept box: every point the mover occupies over
// the whole path, as exact rational extremes per axis. A Revolute or a
// Prismatic reads its Bounds box at rest, inflated by its own Bound plus
// travel — the farthest any of its points moves from where it sits now. A
// Between's path starts at From, which the rest box has not undergone, so it
// reads the From-placed box: the axis-aligned hull of startCorners, which
// already carry the box's Bound, inflated by travel and by nothing else.
func MoverSweptBox(box measurement.Box, mf MotionFrame, travel *big.Rat) (RatVec, RatVec, bool) {
	if travel == nil {
		return RatVec{}, RatVec{}, false
	}
	if mf.Kind != MotionBetween {
		return BoxCornersExact(box, travel)
	}
	corners, ok := startCorners(box, mf)
	if !ok {
		return RatVec{}, RatVec{}, false
	}
	var lo, hi RatVec
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

// SweptBoxLower is §6 step 3's swept-box exclusion for one (mover, static)
// pair, decided over exact rationals: the mover's swept box (MoverSweptBox)
// against the static body's box inflated by its own Bound. Boxes separated by
// a strictly positive gap along some axis prove the pair apart at every
// parameter, and the largest such axis gap is a proven lower bound on the
// pair's distance over the whole path. ok is false when the boxes do not
// separate.
func SweptBoxLower(mLo, mHi RatVec, static measurement.Box) (float64, bool) {
	sLo, sHi, okS := BoxCornersExact(static, new(big.Rat))
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
