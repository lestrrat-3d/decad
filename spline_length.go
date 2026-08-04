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
//   - de Casteljau subdivision narrows the gap, at NO guaranteed per-level rate:
//     the familiar 4x is asymptotic, and an early level can do far less — the
//     degree-31 span the tests pin narrows by 1.37x at one of its levels.
//
// Both bounds are proofs, not samples, and the subdivision runs over exact
// rationals so no level introduces error of its own. Only the final square
// roots are floats, and each is rounded OUTWARD against an exact rational
// comparison — so the reported interval encloses the true length whatever the
// platform's sqrt does.

// freeformLengthDepth is how far the bracket subdivides, and the depth is
// FIXED. This bracket has no target of its own — nothing downstream compares an
// arc length against a caller tolerance — so there is no threshold for a
// measured-target loop to stop on, and the depth is sized to bound both the work
// and the rational denominators the subdivision introduces (they double per
// level) instead. Where a target does exist, docs/spline-design.md §6.1 puts the
// loop there: §6.4's build-time gate and §6.1.1's product enclosure.
//
// A fixed depth promises NO relative width, and nothing here claims one:
// freeformArcLength sums the ACTUAL leaf brackets at this depth and reports the
// half width of the enclosure it just measured. What that width comes to varies
// with the span, and the spread is wide — an ordinary cubic measures a relative
// half width of 1.9e-7, while the widest span this bracket's own preflight
// admits (32 controls, freeformBracketCost) measures above 2e-5.
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
//
// The span is re-expressed once, here, into the split form below; every level
// under it works in that form and only the leaves come back out as rationals.
func spanLengthBracket(span bezierSpan, depth int) (float64, float64) {
	return dyadicSpanOf(span).lengthBracket(depth)
}

// dyadicPoint is one split value: the plane-local coordinate
// (u, v) / (den · 2ᵉˣᵖ), over its span's own shared den.
//
// SUBDIVIDING INTRODUCES NOTHING BUT POWERS OF TWO. Every blend below is a
// MIDPOINT, so a child value is the sum of two values of the same denominator
// halved — the denominator never gains a new odd factor at any depth, which is
// what makes an integer numerator beside a binary exponent the exact form of the
// arithmetic rather than an approximation of it. A midpoint is then one integer
// addition and one exponent increment, with no common factor to strip, where a
// normalising rational pays two GCDs to re-derive a denominator it already knows.
//
// The SPAN's own denominator is factored out because it is NOT a power of two.
// A recorded control coordinate is a float64, hence dyadic, but a converted one
// need not be: Boehm knot insertion divides by a knot difference and the
// closed-spline identity divides by 3 and by 6 (spline_bezier.go), so a
// converted control point carries whatever odd denominator that left it. That
// denominator is a CONSTANT of the whole subdivision, so it is lifted out once
// per span and the exponent carries everything the splitting adds.
type dyadicPoint struct {
	u, v *big.Int
	exp  uint
}

// dyadicSpan is one Bézier span in that form: its split values beside the
// denominator every one of them shares.
type dyadicSpan struct {
	points []dyadicPoint
	den    *big.Int
}

// dyadicSpanOf re-expresses a converted span over one shared denominator. The
// denominator is the least common multiple of the span's own, so every
// coordinate lifts to an EXACT integer numerator and the span it describes is
// the span it was given, to the last bit.
func dyadicSpanOf(span bezierSpan) dyadicSpan {
	den := big.NewInt(1)
	for _, point := range span {
		den = ratLCM(den, point.u.Denom())
		den = ratLCM(den, point.v.Denom())
	}
	points := make([]dyadicPoint, len(span))
	for i, point := range span {
		points[i] = dyadicPoint{u: scaledNumerator(point.u, den), v: scaledNumerator(point.v, den)}
	}
	return dyadicSpan{points: points, den: den}
}

// ratLCM returns the least common multiple of two positive integers. A big.Rat
// denominator is always positive, so no sign case arises.
func ratLCM(a, b *big.Int) *big.Int {
	gcd := new(big.Int).GCD(nil, nil, a, b)
	out := new(big.Int).Quo(a, gcd)
	return out.Mul(out, b)
}

// scaledNumerator returns r·den as an exact integer. den is a common multiple
// of every denominator in the span, so the division leaves no remainder.
func scaledNumerator(r *big.Rat, den *big.Int) *big.Int {
	out := new(big.Int).Quo(den, r.Denom())
	return out.Mul(out, r.Num())
}

func (s dyadicSpan) lengthBracket(depth int) (float64, float64) {
	if depth == 0 || len(s.points) < 2 {
		return s.chordLower(), s.polygonUpper()
	}
	left, right := s.split()
	leftLo, leftHi := left.lengthBracket(depth - 1)
	rightLo, rightHi := right.lengthBracket(depth - 1)
	return downRound(leftLo + rightLo), upRound(leftHi + rightHi)
}

// split halves a span by de Casteljau at t = 1/2, exactly: every blend is a
// midpoint, so the arithmetic is a binary bisection over integer numerators.
func (s dyadicSpan) split() (dyadicSpan, dyadicSpan) {
	n := len(s.points)
	work := make([]dyadicPoint, n)
	copy(work, s.points)
	left := make([]dyadicPoint, 0, n)
	right := make([]dyadicPoint, n)
	left = append(left, work[0])
	right[n-1] = work[n-1]
	for round := n - 1; round > 0; round-- {
		for i := range round {
			work[i] = dyadicMidpoint(work[i], work[i+1])
		}
		left = append(left, work[0])
		right[round-1] = work[round-1]
	}
	return dyadicSpan{points: left, den: s.den}, dyadicSpan{points: right, den: s.den}
}

// dyadicMidpoint is (a+b)/2 exactly: the two numerators are raised to their
// common exponent, added, and the exponent goes up by one for the halving.
func dyadicMidpoint(a, b dyadicPoint) dyadicPoint {
	exp := max(a.exp, b.exp)
	return dyadicPoint{
		u:   alignedSum(a.u, exp-a.exp, b.u, exp-b.exp),
		v:   alignedSum(a.v, exp-a.exp, b.v, exp-b.exp),
		exp: exp + 1,
	}
}

// alignedSum is a+b with each numerator first shifted up to the common
// exponent. Shifting is exact, so the sum is the sum of the two values.
func alignedSum(a *big.Int, aShift uint, b *big.Int, bShift uint) *big.Int {
	out := new(big.Int).Lsh(a, aShift)
	if bShift == 0 {
		return out.Add(out, b)
	}
	return out.Add(out, new(big.Int).Lsh(b, bShift))
}

// alignedDifference is a−b under the same alignment.
func alignedDifference(a *big.Int, aShift uint, b *big.Int, bShift uint) *big.Int {
	out := new(big.Int).Lsh(a, aShift)
	if bShift == 0 {
		return out.Sub(out, b)
	}
	return out.Sub(out, new(big.Int).Lsh(b, bShift))
}

// chordLower is a proven lower bound on the distance between the span's two
// ends: the largest float whose square does not exceed the exact squared
// distance.
func (s dyadicSpan) chordLower() float64 {
	return ratSqrtDown(s.squaredDistance(s.points[0], s.points[len(s.points)-1]))
}

// polygonUpper is a proven upper bound on a control polygon's length.
func (s dyadicSpan) polygonUpper() float64 {
	total := 0.0
	for i := 0; i+1 < len(s.points); i++ {
		total = upRound(total + ratSqrtUp(s.squaredDistance(s.points[i], s.points[i+1])))
	}
	return total
}

// squaredDistance is the exact |b−a|² of two split values, handed to the
// outward-rounded square roots as the rational they already read. This is the
// one boundary where the split form becomes a big.Rat: the leaves are where the
// bracket is decided, and everything above them stays free of normalisation.
func (s dyadicSpan) squaredDistance(a, b dyadicPoint) *big.Rat {
	exp := max(a.exp, b.exp)
	du := alignedDifference(b.u, exp-b.exp, a.u, exp-a.exp)
	dv := alignedDifference(b.v, exp-b.exp, a.v, exp-a.exp)
	num := du.Mul(du, du)
	num.Add(num, dv.Mul(dv, dv))
	den := new(big.Int).Mul(s.den, s.den)
	return new(big.Rat).SetFrac(num, den.Lsh(den, 2*exp))
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
