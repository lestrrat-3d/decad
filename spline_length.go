package decad

import (
	"fmt"
	"math"
	"math/big"
)

// This file is docs/spline-design.md §6.1: the arc length of a free-form curve
// is never exact in any tier, because it integrates a square root that has no
// polynomial antiderivative. What IS available is a PROVEN two-sided bracket:
//
//   - the CHORD is below the arc — the straight line between two points on a
//     curve is no longer than the curve between them;
//   - the CONTROL POLYGON is above it — a Bézier is variation diminishing, so
//     its arc length never exceeds its control polygon's length;
//   - de Casteljau subdivision closes the gap by roughly 4x per level.
//
// Both bounds are proofs, not samples, and the subdivision runs over exact
// rationals so no level introduces error of its own. Only the final square
// roots are floats, and each is rounded OUTWARD against an exact rational
// comparison — so the reported interval encloses the true length whatever the
// platform's sqrt does.

// freeformLengthDepth is how far the bracket subdivides. Each level quarters
// the gap: 10 levels bring a hand-sized curve's bracket to a RELATIVE width
// near 1e-7 (about 1.5e-6 mm on an 11 mm curve), which is far below any
// tolerance a caller sets, while keeping both the work and the rational
// denominators the subdivision introduces (they double per level) modest.
const freeformLengthDepth = 10

// freeformArcLength brackets the converted chain's arc length. It returns the
// interval midpoint and its half width, so the caller reports a value with a
// proven bound and NEVER an Exact zero — §6.1 forbids one here.
//
// A zero half width is that forbidden Exact, so the bracket is the gate: a
// curve whose control net has collapsed to a single point is the one shape
// whose control-polygon upper bound is zero, and it refuses as ErrDegenerate
// (Table R row R14) rather than report a length at all. That is the same answer
// the moments path already gives the identical record (freeformDegenerate in
// moments_validate.go). Every curve that is not a point brackets strictly wide:
// each subdivision level rounds the lower sum down and the upper sum up, so a
// positive length can never close its own interval.
//
// The other refusal is the interval float64 cannot hold: a curve long enough
// that its proven upper bound runs past MaxFloat64 has no representable
// interval to report, and that is Table R row R15, ErrUnsupported. The test is
// on the ENCLOSURE, so a length just under the top of the range whose upper
// bound is not representable refuses too — refusing an answer decad cannot
// state is right, and the enclosure is the only length in hand. Scale alone
// never refuses otherwise: the square-root seeds work at every scale a finite
// coordinate can reach (ratSqrtSeed), so a valid curve is never turned away for
// being merely small or large.
func freeformArcLength(spans []bezierSpan, work *freeformWork) (float64, float64, error) {
	lo, hi := 0.0, 0.0
	for _, span := range spans {
		if err := work.step(freeformBracketCost(len(span))); err != nil {
			return 0, 0, err
		}
		spanLo, spanHi := spanLengthBracket(span, freeformLengthDepth)
		lo = downRound(lo + spanLo)
		hi = upRound(hi + spanHi)
	}
	if isNonFinite(lo) || isNonFinite(hi) || hi < lo {
		return 0, 0, errFreeformLengthUnrepresentable
	}
	mid := lo + (hi-lo)/2
	bound := upRound(math.Max(mid-lo, hi-mid))
	if bound <= 0 {
		return 0, 0, errFreeformLengthDegenerate
	}
	return mid, bound, nil
}

// errFreeformLengthUnrepresentable is Table R row R15: the enclosure is still
// proven, but no float64 interval holds it, so there is no Measurement to
// publish. The sentinel is ErrUnsupported — the curve EXISTS and this evaluator
// cannot report its length — never ErrNotFinite, whose subject is a non-finite
// INPUT while every coordinate reaching here is finite. The guard also catches
// an inverted interval, which the outward rounding makes unreachable and which
// no reading could use either.
var errFreeformLengthUnrepresentable = fmt.Errorf(
	`%w: a free-form segment's arc length runs past the representable float64 range`, ErrUnsupported,
)

var errFreeformLengthDegenerate = fmt.Errorf(
	`%w: a free-form segment whose control points all coincide bounds no arc length`, ErrDegenerate,
)

// freeformBracketCost is the conservative preflight of ONE span's bracket,
// charged before the first subdivision allocates anything.
//
// The cost is driven by the subdivision depth and the span DEGREE together.
// Depth d makes 2ᵈ−1 exact de Casteljau splits and 2ᵈ leaves; one split blends
// all n(n−1)/2 de Casteljau pairs over two coordinates, and one leaf takes its
// chord and its control polygon — n exact square-root brackets between them.
// A charge read off the depth alone counts the leaves and nothing else, which
// lets an arbitrarily wide span through: a single degree-1000 span is 1024
// leaves and over five hundred million rational midpoints.
//
// Saturating arithmetic keeps the estimate an UPPER bound at every size, so an
// oversized span refuses at freeformWorkLimit instead of wrapping to a small
// charge (spline_bezier.go).
func freeformBracketCost(controls int) uint64 {
	if controls < 2 {
		return 0
	}
	n := uint64(controls)
	leaves := uint64(1) << freeformLengthDepth
	perSplit := costMul(n, n-1)
	perLeaf := n
	return costAdd(costMul(leaves-1, perSplit), costMul(leaves, perLeaf))
}

// spanLengthBracket brackets one Bézier span's arc length, subdividing to the
// given depth and summing each piece's chord (below) and control polygon
// (above).
func spanLengthBracket(span bezierSpan, depth int) (float64, float64) {
	if depth == 0 || len(span) < 2 {
		return chordLower(span[0], span[len(span)-1]), polygonUpper(span)
	}
	left, right := splitBezier(span)
	leftLo, leftHi := spanLengthBracket(left, depth-1)
	rightLo, rightHi := spanLengthBracket(right, depth-1)
	return downRound(leftLo + rightLo), upRound(leftHi + rightHi)
}

// splitBezier halves a span by de Casteljau at t = 1/2, exactly: every blend is
// a midpoint, so the arithmetic is a rational bisection.
func splitBezier(span bezierSpan) (bezierSpan, bezierSpan) {
	n := len(span)
	work := make(bezierSpan, n)
	copy(work, span)
	left := make(bezierSpan, 0, n)
	right := make(bezierSpan, n)
	left = append(left, work[0])
	right[n-1] = work[n-1]
	for round := n - 1; round > 0; round-- {
		for i := range round {
			work[i] = ratPoint{
				u: ratMidpoint(work[i].u, work[i+1].u),
				v: ratMidpoint(work[i].v, work[i+1].v),
			}
		}
		left = append(left, work[0])
		right[round-1] = work[round-1]
	}
	return left, right
}

func ratMidpoint(a, b *big.Rat) *big.Rat {
	out := new(big.Rat).Add(a, b)
	return out.Quo(out, big.NewRat(2, 1))
}

// chordLower is a proven lower bound on the distance between two exact points:
// the largest float whose square does not exceed the exact squared distance.
func chordLower(a, b ratPoint) float64 {
	return ratSqrtDown(ratSquaredDistance(a, b))
}

// polygonUpper is a proven upper bound on a control polygon's length.
func polygonUpper(span bezierSpan) float64 {
	total := 0.0
	for i := 0; i+1 < len(span); i++ {
		total = upRound(total + ratSqrtUp(ratSquaredDistance(span[i], span[i+1])))
	}
	return total
}

func ratSquaredDistance(a, b ratPoint) *big.Rat {
	du := new(big.Rat).Sub(b.u, a.u)
	dv := new(big.Rat).Sub(b.v, a.v)
	du.Mul(du, du)
	dv.Mul(dv, dv)
	return du.Add(du, dv)
}

// ratSqrtSeed approximates sqrt(q) for a positive rational at EVERY scale a
// recorded coordinate can reach. It is only a seed: the exact rational
// comparisons below decide each bound, so a better seed can only reduce false
// refusals and can never widen or invert the interval.
//
// Rounding q to a float64 first is what a seed must not do. A leg distance
// small enough to make q subnormal loses most of q's significand, and one large
// enough to make q overflow loses q entirely — either way the seed lands far
// more than sqrtAdjustLimit ulps from the true root, the walk exhausts, and a
// perfectly valid curve is refused. Scaling instead is exact: split q into
// mantissa and binary exponent, force the exponent EVEN so half of it is an
// integer, take the square root of a mantissa that always sits in [0.5, 2), and
// scale the result back by a power of two, which moves no significand bit.
func ratSqrtSeed(q *big.Rat) float64 {
	mant := new(big.Float).SetPrec(64)
	exp := new(big.Float).SetPrec(64).SetRat(q).MantExp(mant)
	if exp%2 != 0 {
		// Halving an odd exponent is not an integer, so shift one power of two
		// into the mantissa: [0.5, 1) becomes [1, 2), which still roots cleanly.
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// ratSqrtDown returns a float f with f*f <= q, proven by exact comparison. The
// float sqrt seeds the answer; the exact test decides it, so no platform's
// sqrt accuracy can widen or invert the bracket.
func ratSqrtDown(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := ratSqrtSeed(q)
	if isNonFinite(f) {
		// sqrt(q) is at or beyond the top of the range, so the largest float
		// there is starts the walk; the exact test still decides it.
		f = math.MaxFloat64
	}
	for range sqrtAdjustLimit {
		if ratSquareAtMost(f, q) {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

// ratSqrtUp returns a float f with f*f >= q, proven by exact comparison. It
// returns +Inf only where sqrt(q) genuinely exceeds MaxFloat64, which
// freeformArcLength refuses as R15 rather than report.
func ratSqrtUp(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := ratSqrtSeed(q)
	if isNonFinite(f) {
		f = math.MaxFloat64
	}
	for range sqrtAdjustLimit {
		if !ratSquareAtMost(f, q) || ratSquareEquals(f, q) {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

// sqrtAdjustLimit bounds the directed-rounding walk. A correctly-rounded sqrt
// lands within one ulp, so a handful of steps always suffices; the limit exists
// so a pathological implementation cannot spin, and reaching it returns the
// safe outward extreme rather than a value that might not bound.
const sqrtAdjustLimit = 8

func ratSquareAtMost(f float64, q *big.Rat) bool {
	square := floatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) <= 0
}

func ratSquareEquals(f float64, q *big.Rat) bool {
	square := floatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) == 0
}

// downRound is upRound's mirror: the next float toward zero, so a sum of lower
// bounds stays a lower bound.
func downRound(x float64) float64 {
	if x <= 0 || isNonFinite(x) {
		return x
	}
	return math.Nextafter(x, 0)
}
