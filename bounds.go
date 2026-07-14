package decad

import (
	"fmt"
	"math"
)

// This file is the single owner of every proven error bound a faceted
// (boolean-built) measurement reports. NO measurement site computes a bound
// inline: each error mechanism the mesh boolean is subject to has exactly one
// helper here, and every site routes through it.
//
// The mechanisms, and the helper that owns each:
//
//   - the chord displacement δ of a point on an OPERAND's surface → carried in
//     the payload, composed by rimDelta;
//   - the TRIM AMPLIFICATION of a point on the boolean's OWN rim: a rim vertex
//     is the crossing of two chord PLANES, so it is displaced not by δ but by
//     (δA + δB)/sin θ, θ the crossing angle — unbounded as the surfaces
//     approach tangency → rimDelta, which refuses when the inflated bound
//     stops meaning anything;
//   - float SUMMATION of the reported value itself → sumSlop, a proven bound
//     for a NAIVE loop (never zero for a float-computed value, which is what
//     keeps exactnessOf honest);
//   - ACCUMULATION over the N elements of a chain → chainLengthBound;
//   - RIGID-MOTION rounding → rigidRoundAllow, charged at the INPUT and
//     translation magnitudes, which is where the rounding is actually
//     committed — never at the output's;
//   - a per-coordinate maximum read as a 3D DISTANCE → radius3D.

const (
	// unitRoundoff is float64's u = 2⁻⁵³: the relative error a single
	// round-to-nearest operation can commit.
	unitRoundoff = 1.1102230246251565e-16
	// sqrt3Up is √3 rounded UP, so a per-coordinate bound scaled by it stays
	// an upper bound on the 3D corner distance.
	sqrt3Up = 1.7320508075688774
)

// upRound nudges a positive bound to the next representable float64, so the
// bound's own rounding can never land it below the quantity it bounds.
func upRound(x float64) float64 {
	if x > 0 {
		return math.Nextafter(x, math.Inf(1))
	}
	return x
}

// radius3D turns a per-coordinate bound into the 3D distance bound its
// consumers read: all three coordinates can be off at once, so the corner sits
// up to √3 times the per-coordinate bound away (core §5.2 — a coordinate's
// error bound is a radius, not an axis extent).
func radius3D(perCoord float64) float64 {
	if perCoord <= 0 {
		return 0
	}
	return upRound(perCoord * sqrt3Up)
}

// sumSlop is a PROVEN bound on the rounding a NAIVE float64 summation of n
// terms commits, given absSum = Σ|term|. The classic bound is
// (n−1)·u/(1 − (n−1)·u) · Σ|term|, which 2·(n−1)·u·Σ|term| dominates for
// (n−1)·u ≤ ½ — true for any mesh a machine can hold. On top of it each term
// is itself a float evaluation (a cross product, a norm, a square root): a
// handful of ulps, charged as 4·u per term.
//
// It is NEVER zero for a positive absSum. That is the point: a float-computed
// value is not exactly representable, so it may never reach Exact.
func sumSlop(n int, absSum float64) float64 {
	if n <= 0 || absSum <= 0 || isNonFinite(absSum) {
		return 0
	}
	loop := 2 * float64(n-1) * unitRoundoff * absSum
	terms := 4 * unitRoundoff * absSum
	return upRound(loop + terms)
}

// chainLengthBound is the proven bound on a boolean rim's length: the chain
// holds nSegs chords whose two endpoints EACH move by up to delta, so the
// held length can be off by 2·nSegs·delta — plus the float slop of summing
// nSegs square roots. Never zero for a float-computed length, even when
// delta is (an all-planar boolean's rim is a float sum of sqrts, and the last
// ulp is not free).
func chainLengthBound(nSegs int, delta, heldLen float64) float64 {
	if nSegs <= 0 {
		return 0
	}
	return upRound(2*float64(nSegs)*delta + sumSlop(nSegs, heldLen))
}

// rigidRoundAllow bounds the rounding a rigid motion commits on one point.
// The rounding happens INSIDE the products and sums — at the magnitude of the
// INPUT coordinate and of the translation — not at the magnitude of the
// result: a body built far from the origin and moved back rounds at the far
// magnitude and would be charged at the near one. Every intermediate stays
// under 2·maxInputAbs + maxTransAbs (an orthonormal row's dot product is at
// most √3·|input|), 16 ulps there cover the products, the two sums and the
// translation, and the consumers read a 3D distance, so ×√3.
func rigidRoundAllow(maxInputAbs, maxTransAbs float64) float64 {
	m := 2*math.Abs(maxInputAbs) + math.Abs(maxTransAbs)
	return radius3D(16 * ulpOf(m))
}

// rimDelta is the trim-amplified displacement bound of a vertex the boolean
// itself creates. A rim vertex is not a point of either operand's surface: it
// is the exact crossing of operand A's chord PLANE with operand B's, and the
// true intersection curve lies anywhere within deltaA of the one and deltaB
// of the other. That region is a tube of half-width (deltaA + deltaB)/sin θ
// about the crossing line — so the displacement grows without limit as the
// two surfaces approach tangency, and δ itself is NOT a bound on it.
//
// sinMin is the smallest sine of a crossing angle any contact of this pair
// takes, computed exactly from the facet normals. When the inflated bound
// reaches the pair's own diameter it has stopped bounding anything, and the
// operation is refused (ErrUnsupported) rather than reported with a bound
// nobody can use — decad never understates a bound, and never fakes one.
func rimDelta(deltaA, deltaB, sinMin, dPair float64) (float64, error) {
	d := deltaA + deltaB
	if d <= 0 {
		// Both operands are held exactly (all-planar analytic faces, or a
		// faceted body whose polygons ARE its boundary): there is no chord
		// error to amplify, at any crossing angle.
		return 0, nil
	}
	if sinMin <= 0 || isNonFinite(sinMin) {
		return 0, fmt.Errorf(`%w: the operands' facets meet at an angle this evaluator cannot bound`, ErrUnsupported)
	}
	rim := upRound(d / sinMin)
	if isNonFinite(rim) || rim >= dPair {
		return 0, fmt.Errorf(`%w: the operands cross too shallowly — the rim's proven displacement bound reaches the pair's own diameter, so no measurement of the result would be trustworthy`, ErrUnsupported)
	}
	return rim, nil
}
