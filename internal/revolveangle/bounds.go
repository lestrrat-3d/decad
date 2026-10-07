package revolveangle

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Extremes returns the held range of c0·cos φ + c1·sin φ over the sweep.
func Extremes(c0, c1, phi0, phi1 float64, full bool) (float64, float64) {
	amp := math.Hypot(c0, c1)
	if full {
		return -amp, amp
	}
	m := func(phi float64) float64 { return c0*math.Cos(phi) + c1*math.Sin(phi) }
	lo := math.Min(m(phi0), m(phi1))
	hi := math.Max(m(phi0), m(phi1))
	if amp == 0 {
		return lo, hi
	}
	star := math.Atan2(c1, c0)
	for _, cand := range []float64{star, star + math.Pi} {
		for k := math.Floor((phi0-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= phi1+1e-12; k += 2 * math.Pi {
			phi := cand + k
			if phi < phi0-1e-12 {
				continue
			}
			lo = math.Min(lo, m(phi))
			hi = math.Max(hi, m(phi))
		}
	}
	return lo, hi
}

// ExtremeBounds proves how far Extremes' held (heldLo, heldHi) can
// sit from the TRUE min/max of m(φ) = c0·cos φ + c1·sin φ over the sweep the
// record DENOTES (docs/evaluator-design.md §6), without ever trusting
// math.Sin/Cos/Atan2/Hypot's accuracy: c0 and c1 are read as exact rationals
// (their own float64 bit patterns — the same convention Extremes' own
// callers already take), each endpoint's own denoted angle is enclosed by
// den.Phi0/den.Phi1 (denotation.go) — falling back to the HELD
// phi0/phi1 read as exact rationals wherever the denotation cannot state one,
// which reproduces today's reading exactly — sin/cos of each denoted angle by
// Angle.SinCosFor, which reads a pure-turn end (a degree-stated
// extent) through proofbound.TurnSinCosInterval, EXACT at every eighth-turn boundary and
// never comparing against π, rather than through the radian-space bracket
// (normal_bound.go's survey2d.RadSinCosInterval, the Cone normal's own primitive) that
// a detour through π would otherwise force even at a quarter turn, and the
// amplitude √(c0²+c1²) by the rational square-root brackets
// circularLengthInterval reads an ArcSeg's radius through (proofbound.RatSqrtDown/
// proofbound.RatSqrtUp).
//
// The true extreme over the denoted sweep always sits at phi0, at phi1, or at
// an interior critical angle where m′(φ) = −c0·sin φ + c1·cos φ = 0. m′ is
// itself a sinusoid whose zeros are spaced exactly π apart, so an interval
// shorter than π contains AT MOST one: if m′ is proven the same sign at both
// endpoints (a certified sign, from the same enclosures — never a float
// comparison) and the interval's own width is proven under π, no interior
// critical angle can exist and the extreme is provably an endpoint.
//
// Where that cannot be certified but the interval's width is proven under 2π
// and m′ is proven STRICTLY opposite signs at the two endpoints, the
// π-spacing of m′'s zeros still decides it: a sign change over a span under
// 2π can only cross an ODD number of the (at most two) zeros that fit, so
// exactly ONE interior zero exists, and it is a maximum where m′ runs + to −
// (a minimum where it runs − to +). A critical value of this sinusoid is
// always exactly ±amp, so the true max (min) is provably the amplitude
// itself — the enclosure narrows to [ampLo, ampHi] ([−ampHi, −ampLo]) — and
// with the interval's one critical point already accounted for, the OTHER
// extreme cannot be interior and needs no widening at all.
//
// A width proven at LEAST a half turn (den.HalfTurnExcessFor, checked in the
// default arm below) decides both extremes by the endpoints' own certified
// slope signs, with no interior angle to locate: a closed interval of width
// ≥ π always contains at least one critical angle, so for every direction at
// least one of {max, min} is exactly the amplitude, and which one is a fact
// about the SIGN of m′ at whichever endpoint is strict. A strictly positive
// slope at phi0 (or strictly negative at phi1) certifies the max is the
// amplitude; the mirrored sign certifies the min; an endpoint whose slope is
// exactly zero is itself a critical angle, so its own value is one extreme
// and the opposite kind's critical angle sits exactly π further inside the
// interval — both extremes are the amplitude. An extreme none of this
// certifies keeps the global widening, which is sound for any φ.
//
// Only where neither arm certifies — a straddling endpoint, or a width not
// proven under 2π — does the enclosure widen to the global amplitude bound on
// both ends (valid for ANY φ, critical or not).
func ExtremeBounds(c0, c1, phi0, phi1 float64, den Sweep, heldLo, heldHi float64, full bool) (float64, float64) {
	c0R, c1R := proofarith.FloatRat(c0), proofarith.FloatRat(c1)
	if c0R == nil || c1R == nil {
		return math.Inf(1), math.Inf(1)
	}
	sq := new(big.Rat).Add(new(big.Rat).Mul(c0R, c0R), new(big.Rat).Mul(c1R, c1R))
	ampLoF, ampHiF := proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
	if proofbound.IsNonFinite(ampLoF) || proofbound.IsNonFinite(ampHiF) {
		return math.Inf(1), math.Inf(1)
	}
	ampLoR, ampHiR := proofarith.FloatRat(ampLoF), proofarith.FloatRat(ampHiF)
	if ampLoR == nil || ampHiR == nil {
		return math.Inf(1), math.Inf(1)
	}
	if full {
		hiIv := proofbound.Interval(ampLoR, ampHiR)
		loIv := proofbound.Interval(new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR))
		return proofbound.IntervalFloatError(loIv, heldLo), proofbound.IntervalFloatError(hiIv, heldHi)
	}
	enc0, ok0 := den.Phi0.EnclosureFor(phi0)
	enc1, ok1 := den.Phi1.EnclosureFor(phi1)
	if !ok0 || !ok1 {
		return math.Inf(1), math.Inf(1)
	}
	sin0, cos0, ok0t := den.Phi0.SinCosFor(phi0)
	sin1, cos1, ok1t := den.Phi1.SinCosFor(phi1)
	if !ok0t || !ok1t {
		return math.Inf(1), math.Inf(1)
	}
	m0 := proofbound.IntervalAdd(proofbound.IntervalScale(cos0, c0R), proofbound.IntervalScale(sin0, c1R))
	m1 := proofbound.IntervalAdd(proofbound.IntervalScale(cos1, c0R), proofbound.IntervalScale(sin1, c1R))
	maxRat := func(a, b *big.Rat) *big.Rat {
		if a.Cmp(b) >= 0 {
			return a
		}
		return b
	}
	minRat := func(a, b *big.Rat) *big.Rat {
		if a.Cmp(b) <= 0 {
			return a
		}
		return b
	}
	// The true max is always >= both endpoints' true values (an endpoint is
	// always a candidate), so the lower end of its enclosure never needs
	// widening; likewise the true min's upper end. Only the "far" end of
	// each — where an unexcluded interior critical angle could push it —
	// widens, and only in the arms below that cannot certify a tighter
	// answer.
	hiLo := maxRat(m0.Lo, m1.Lo)
	hiHi := maxRat(m0.Hi, m1.Hi)
	loHi := minRat(m0.Hi, m1.Hi)
	loLo := minRat(m0.Lo, m1.Lo)

	// m′(φ) = −c0·sin φ + c1·cos φ shares m's own zeros, spaced exactly π
	// apart, so a closed interval narrower than π contains AT MOST one — and
	// if both endpoints proved the SAME sign (equality admitted, since a
	// critical point sitting exactly at an endpoint is that one zero and
	// changes nothing about the OPEN interval between them), that single
	// zero cannot be interior: m′ cannot cross to the opposite sign and back
	// without a second zero, so it holds that one sign throughout and m is
	// monotone on the whole closed interval.
	negC0R := new(big.Rat).Neg(c0R)
	mp0 := proofbound.IntervalAdd(proofbound.IntervalScale(sin0, negC0R), proofbound.IntervalScale(cos0, c1R))
	mp1 := proofbound.IntervalAdd(proofbound.IntervalScale(sin1, negC0R), proofbound.IntervalScale(cos1, c1R))
	widthIv := proofbound.IntervalSub(enc1, enc0)
	widthLessThanPi := widthIv.Hi.Cmp(proofbound.PiLower) < 0
	sameNonPos := mp0.Hi.Sign() <= 0 && mp1.Hi.Sign() <= 0
	sameNonNeg := mp0.Lo.Sign() >= 0 && mp1.Lo.Sign() >= 0
	monotonic := widthLessThanPi && (sameNonPos || sameNonNeg)
	switch {
	case monotonic:
		// No interior critical angle at all: both endpoint enclosures stand
		// as they are.
	case widthIv.Hi.Cmp(proofbound.TwoPiInterval().Lo) < 0 && mp0.Lo.Sign() > 0 && mp1.Hi.Sign() < 0:
		// m′ runs strictly + to strictly −: exactly one interior zero, a
		// maximum. The true max IS the amplitude, and the min cannot be
		// interior, so it keeps its endpoint-only enclosure.
		hiLo, hiHi = ampLoR, ampHiR
	case widthIv.Hi.Cmp(proofbound.TwoPiInterval().Lo) < 0 && mp0.Hi.Sign() < 0 && mp1.Lo.Sign() > 0:
		// The mirror case: a minimum.
		loLo, loHi = new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR)
	default:
		// The reflex arm: a sweep proven at least a half turn wide contains
		// at least one zero of m′ in its CLOSED interval, and each endpoint's
		// own certified slope says which. With α the angular distance from
		// the maximum's direction to phi0, m′(phi0) = −amp·sin α: a strictly
		// positive slope at phi0 puts α in (π, 2π), so the maximum lies
		// within 2π − α < π ≤ width AFTER phi0; a strictly negative one puts
		// α in (0, π), so the minimum lies within π − α < π ≤ width after
		// phi0. Mirrored at phi1 (the extreme lies BEFORE it): a strictly
		// negative slope there certifies the maximum, a strictly positive one
		// the minimum. An endpoint whose slope is exactly zero IS one
		// critical point, so its own value is one extreme and the opposite
		// extreme's critical point sits exactly π further in, inside a
		// width of at least π: both extremes are ±amp. A certified extreme
		// takes the amplitude bracket; an uncertified one keeps the global
		// widening below, which is sound for any φ.
		excess, okExcess := den.HalfTurnExcessFor(phi0, phi1)
		atLeastHalfTurn := okExcess && excess.Lo.Sign() >= 0
		crit0 := mp0.Lo.Sign() == 0 && mp0.Hi.Sign() == 0
		crit1 := mp1.Lo.Sign() == 0 && mp1.Hi.Sign() == 0
		maxIsAmp := atLeastHalfTurn && (crit0 || crit1 || mp0.Lo.Sign() > 0 || mp1.Hi.Sign() < 0)
		minIsAmp := atLeastHalfTurn && (crit0 || crit1 || mp0.Hi.Sign() < 0 || mp1.Lo.Sign() > 0)
		if maxIsAmp {
			hiLo, hiHi = ampLoR, ampHiR
		} else {
			hiHi = maxRat(hiHi, ampHiR)
		}
		if minIsAmp {
			loLo, loHi = new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR)
		} else {
			loLo = minRat(loLo, new(big.Rat).Neg(ampHiR))
		}
	}
	hiIv := proofbound.Interval(hiLo, hiHi)
	loIv := proofbound.Interval(loLo, loHi)
	return proofbound.IntervalFloatError(loIv, heldLo), proofbound.IntervalFloatError(hiIv, heldHi)
}
