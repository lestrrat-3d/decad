package survey2d

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// PullVerdict is the three-valued §6 membership answer for a face's own
// normal-component range against the pull: PullClear is a proven absence of
// opposition (perpendicular or exactly antiparallel included, per §6's own
// carve-outs), PullOpposes is a proven point of opposition, and
// PullUndecided is neither — the range straddles a boundary the reading
// cannot separate.
type PullVerdict int

const (
	PullUndecided PullVerdict = iota
	PullClear
	PullOpposes
)

// OpposesPull applies the zero-allowance normal-component rule to [mn, mx].
// Exact perpendicular is clear, and an exactly antiparallel face separates
// under the pull. The clamp absorbs float overshoot below -1 only.
func OpposesPull(mn, mx float64) bool {
	if mx < -1 {
		mx = -1
	}
	return mn < 0 && mx > -1
}

// DecidePull answers the §6 membership rule for a normal-component range
// [mn, mx] read to within allow: PullClear when the whole range is proven at
// or above zero (mn-allow >= 0) or proven at or below -1 (mx+allow <= -1);
// PullOpposes when the range is proven to include a point strictly below
// zero and a point strictly above -1; PullUndecided otherwise. The allowance
// only ever pushes a straddling range toward undecided, never toward a
// decision — CLAUDE.md's reject-only rule applied to this reading.
func DecidePull(mn, mx, allow float64) PullVerdict {
	switch {
	case mn-allow >= 0, mx+allow <= -1:
		return PullClear
	case mn+allow < 0 && mx-allow > -1:
		return PullOpposes
	default:
		return PullUndecided
	}
}

// DecideRationalComponent decides §6's rule for a component whose EXACT
// value is num / sqrt(scale2 * pull2) — num, scale2 and pull2 all exact
// rationals, scale2 and pull2 both non-negative — without ever taking a
// square root: perpendicular (component == 0) and antiparallel
// (component == -1) are each an exact equality on num and its square, so a
// float division is never asked to decide either. It never returns
// PullUndecided: every input here is exact, so the comparison always
// resolves.
func DecideRationalComponent(num, scale2, pull2 *big.Rat) PullVerdict {
	if num.Sign() >= 0 {
		return PullClear
	}
	lhs := new(big.Rat).Mul(num, num)
	rhs := new(big.Rat).Mul(scale2, pull2)
	if lhs.Cmp(rhs) >= 0 {
		return PullClear
	}
	return PullOpposes
}

// DecideCircularComponent decides §6's rule for sigma*(du*cosθ + dv*sinθ)
// over a window, bracketed as [minLo, minHi] (a bracket on the swept
// function's minimum) and [maxLo, maxHi] (a bracket on its maximum) —
// against sqrt(pull2) rather than 1, since these brackets already carry the
// pull's own (unnormalized) dot product. minLo/maxLo are safe LOWER bounds
// on the true extremes (built from attained sample and endpoint values) and
// minHi/maxHi safe UPPER bounds, so proving the range clear or opposing from
// them never overstates what the enclosure supports.
func DecideCircularComponent(minLo, minHi, maxLo, maxHi, pull2 *big.Rat) PullVerdict {
	switch {
	case minLo.Sign() >= 0:
		return PullClear
	case maxHi.Sign() <= 0 && new(big.Rat).Mul(maxHi, maxHi).Cmp(pull2) >= 0:
		return PullClear
	}
	if minHi.Sign() >= 0 {
		// The minimum's own bracket straddles zero: whether any point at all
		// opposes is not decided from here.
		return PullUndecided
	}
	if maxLo.Sign() > 0 {
		return PullOpposes
	}
	if new(big.Rat).Mul(maxLo, maxLo).Cmp(pull2) < 0 {
		return PullOpposes
	}
	return PullUndecided
}

// WallNormalDecision answers §6's membership rule for one side walk's
// outward normal against the caller's pull. A straight walk's component is
// num / (|t|·|pull|), num = tv·du − tu·dv, for the direction t its recorded
// endpoints denote (lineDirectionEnclosure). It never reads the held tangent
// TanInU, TanInV, which is that difference rounded to float64 and so can turn
// an exact zero into a sign or a small sign into zero. Where an endpoint is
// itself computed, t is an interval, and the verdict is PullUndecided when
// the interval leaves the sign unresolved (DecideIntervalComponent). A
// circular walk's component sweeps sigma·(du·cosθ + dv·sinθ)/|pull| over its
// own [th0, th1], bracketed rather than evaluated. du = m.du·pull and
// dv = m.dv·pull are exact, since m's directions and the caller's pull are
// both held floats. ok is false on any non-finite input, a failed enclosure,
// or a free-form walk.
func WallNormalDecision(w SideWalk, m PlacedFrameMap, pull r3.Vec) (PullVerdict, bool) {
	pv, okP := proofbound.IvVec3Of(pull)
	if !okP {
		return PullUndecided, false
	}
	pull2 := proofbound.IvVec3NormSq(pv).Lo
	du := proofbound.IvVec3Dot(m.Du, pv).Lo
	dv := proofbound.IvVec3Dot(m.Dv, pv).Lo

	switch w.Kind {
	case WalkLine:
		tu, tv, ok := lineDirectionEnclosure(w.SegmentWalk)
		if !ok {
			return PullUndecided, false
		}
		t2 := proofbound.IntervalAdd(proofbound.IntervalSquare(tu), proofbound.IntervalSquare(tv))
		num := proofbound.IntervalSub(proofbound.IntervalScale(tv, du), proofbound.IntervalScale(tu, dv))
		return DecideIntervalComponent(num, t2, pull2), true
	case WalkCircular:
		sigma := big.NewRat(1, 1)
		if w.Th1 < w.Th0 {
			sigma = big.NewRat(-1, 1)
		}
		a := new(big.Rat).Mul(sigma, du)
		b := new(big.Rat).Mul(sigma, dv)
		lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
		minLo, minHi, maxLo, maxHi, ok := CircularNormalRange(a, b, lo, hi, w.Closed)
		if !ok {
			return PullUndecided, false
		}
		return DecideCircularComponent(minLo, minHi, maxLo, maxHi, pull2), true
	default:
		return PullUndecided, false
	}
}

// lineDirectionEnclosure encloses the direction a straight walk's recorded
// endpoints denote: the difference of the two boxes each endpoint's own end
// bound allows. A recorded endpoint states a zero bound, so a walk between two
// recorded points gets its exact difference as a single point.
func lineDirectionEnclosure(w SegmentWalk) (tu, tv proofbound.RatInterval, ok bool) {
	box := func(u, v float64, bound proofbound.WalkEndBound) (proofbound.RatInterval, proofbound.RatInterval, bool) {
		ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
		allow := proofarith.FloatRat(proofbound.WalkEndBoundAllow(bound))
		if ru == nil || rv == nil || allow == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		widen := func(c *big.Rat) proofbound.RatInterval {
			return proofbound.Interval(new(big.Rat).Sub(c, allow), new(big.Rat).Add(c, allow))
		}
		return widen(ru), widen(rv), true
	}
	su, sv, okS := box(w.StartU, w.StartV, w.StartBound)
	eu, ev, okE := box(w.EndU, w.EndV, w.EndBound)
	if !okS || !okE {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return proofbound.IntervalSub(eu, su), proofbound.IntervalSub(ev, sv), true
}

// DecideIntervalComponent is DecideRationalComponent over enclosures: the
// component is num / sqrt(scale2 · pull2) for some num in its interval and
// scale2 in its, both taken together, and pull2 is exact. A verdict holds only
// when it holds for every value the intervals allow, and every other case is
// PullUndecided. Point intervals decide exactly as DecideRationalComponent
// does, which never answers PullUndecided.
func DecideIntervalComponent(num, scale2 proofbound.RatInterval, pull2 *big.Rat) PullVerdict {
	if num.Lo.Sign() >= 0 {
		return PullClear
	}
	if num.Hi.Sign() <= 0 {
		// num² is smallest at num.Hi and largest at num.Lo.
		least := new(big.Rat).Mul(num.Hi, num.Hi)
		if least.Cmp(new(big.Rat).Mul(scale2.Hi, pull2)) >= 0 {
			return PullClear
		}
	}
	if num.Hi.Sign() < 0 {
		most := new(big.Rat).Mul(num.Lo, num.Lo)
		if most.Cmp(new(big.Rat).Mul(scale2.Lo, pull2)) < 0 {
			return PullOpposes
		}
	}
	return PullUndecided
}

// CapNormalDecision answers §6's rule for a planar face whose outward normal
// is an exact `sign` multiple of the placed frame's own n direction (sign is
// always ±1: a prism's two caps, or a cup's kept cap/pocket floor/rims under
// their own sOpen sign, shell_cup.go's cupUndercuts). m.dn's own held length
// need not be exactly one — a placed frame's image of a unit vector is only
// near-unit — so the antiparallel arm compares squares against |m.dn|²
// rather than assuming a unit reading, exactly as the wall reader compares
// against a wall's own tangent length squared.
func CapNormalDecision(m PlacedFrameMap, pull r3.Vec, sign float64) (PullVerdict, bool) {
	pv, okP := proofbound.IvVec3Of(pull)
	rSign := proofarith.FloatRat(sign)
	if !okP || rSign == nil {
		return PullUndecided, false
	}
	pull2 := proofbound.IvVec3NormSq(pv).Lo
	scale2 := proofbound.IvVec3NormSq(m.Dn).Lo
	num := new(big.Rat).Mul(rSign, proofbound.IvVec3Dot(m.Dn, pv).Lo)
	return DecideRationalComponent(num, scale2, pull2), true
}

// CircularNormalRange encloses a*cosθ + b*sinθ over θ ∈ [lo, hi] (lo <= hi,
// both held floats so exact): a bracket [minLo, minHi] on the function's
// minimum and a bracket [maxLo, maxHi] on its maximum, each attained
// wherever the search below can prove it. a and b are exact — no sampling is
// involved building them, unlike a cap-blend patch's recovered coefficients
// (capblend_normal.go) — so nothing here is charged an allowance; the only
// imprecision is the necessarily-irrational sine and cosine the window's own
// angles carry.
//
// The window is cut into four arcs and searched for each of the two critical
// directions (a, b) and (-a, -b) with capblend_normal.go's
// proofbound.WindowReachesDirection, called unmodified: the same robust cross-product
// containment test that function already proves sound against a 200k-sample
// brute force, rather than a second implementation of the same idea.
// wholeTurn (the walk's own structural flag, SideWalk.closed) skips the
// search entirely: a full turn always attains both extremes.
//
// This does not reuse harmonicWindowRange itself: that function's window is
// measured from φ = θ - th0 in a frame its own three sampled coefficients are
// already anchored to, and rotating OUR exact (a, b) into that frame would
// need cos(lo) and sin(lo) — themselves irrational for a generic lo — turning
// an exact input into an interval one for no reason, since a and b already
// hold everywhere over [lo, hi] with no anchor at all.
func CircularNormalRange(a, b *big.Rat, lo, hi float64, wholeTurn bool) (minLo, minHi, maxLo, maxHi *big.Rat, ok bool) {
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return nil, nil, nil, nil, false
	}
	width := new(big.Rat).Sub(rhi, rlo)
	if width.Sign() < 0 {
		return nil, nil, nil, nil, false
	}
	amp, okAmp := proofbound.IntervalSqrt(proofbound.PointInterval(proofbound.RatAdd(proofbound.RatMul(a, a), proofbound.RatMul(b, b))))
	if !okAmp {
		return nil, nil, nil, nil, false
	}
	peakLo, peakHi := amp.Lo, amp.Hi
	troughLo, troughHi := new(big.Rat).Neg(amp.Hi), new(big.Rat).Neg(amp.Lo)
	if wholeTurn {
		return troughLo, troughHi, peakLo, peakHi, true
	}

	const arcs = 4
	sins, coss := make([]proofbound.RatInterval, arcs+1), make([]proofbound.RatInterval, arcs+1)
	for j := range arcs + 1 {
		theta := new(big.Rat).Add(rlo, proofbound.RatMul(width, big.NewRat(int64(j), arcs)))
		sin, cos, okT := proofbound.RadSinCosInterval(theta)
		if !okT {
			return nil, nil, nil, nil, false
		}
		sins[j], coss[j] = sin, cos
		at := proofbound.IntervalAdd(proofbound.IntervalScale(cos, a), proofbound.IntervalScale(sin, b))
		if j == 0 {
			minLo, minHi, maxLo, maxHi = at.Lo, at.Hi, at.Lo, at.Hi
			continue
		}
		minLo, minHi = proofbound.RatMin(minLo, at.Lo), proofbound.RatMin(minHi, at.Hi)
		maxLo, maxHi = proofbound.RatMax(maxLo, at.Lo), proofbound.RatMax(maxHi, at.Hi)
	}

	sure, maybe := proofbound.WindowReachesDirection(coss, sins, a, b)
	if maybe {
		maxHi = proofbound.RatMax(maxHi, peakHi)
	}
	if sure {
		maxLo = proofbound.RatMax(maxLo, peakLo)
	}
	sure, maybe = proofbound.WindowReachesDirection(coss, sins, new(big.Rat).Neg(a), new(big.Rat).Neg(b))
	if maybe {
		minLo = proofbound.RatMin(minLo, troughLo)
	}
	if sure {
		minHi = proofbound.RatMin(minHi, troughHi)
	}
	return minLo, minHi, maxLo, maxHi, true
}
