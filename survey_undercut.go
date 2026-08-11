package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the receiver-side half of the undercut survey's fu155 fix
// (docs/verification-design.md §6, docs/modify-reach-design.md §12 Table DX
// row DX7): the exact three-valued reader for a body's own unchanged walls
// and caps, which prismUndercuts, cupUndercuts and capBlendUndercuts's
// receiver-wall loop all now share.
//
// Before this file, a receiver face's normal-component range was read as a
// bare float (survey.go's now-removed wallNormalRange) and handed to
// opposesPull's strict `m < 0 && M > -1` test — sound only where floating
// point lands exactly where the test expects, which a genuinely
// perpendicular or antiparallel face need not do (fu155's own repro:
// -4.6811112914356013e-17 and -0.99999999999999989 for faces whose exact
// components are 0 and -1). The cap-blend patch loop already carried a
// bounded three-valued answer; this file brings the receiver's own faces to
// the same standard, over the rationals rather than a float allowance,
// because both carve-outs are exactly decidable from held numbers alone: a
// straight wall's plane-local tangent, a circular wall's swept angle, and the
// placed frame's own exact directions (newPlacedFrameMap,
// capblend_normal.go).
//
// The caller's ORIGINAL pull is used throughout, never its normalized form:
// normalizing rounds, and that rounding is exactly what pushed the
// antiparallel wall of the repro to -0.99999999999999989 instead of the
// exact -1 its tangent and the pull's own direction agree on. Squaring both
// sides of every comparison against -1 (or 0) is what lets this file decide
// without ever taking a square root of an irrational quantity: a component
// c = num / sqrt(scale2 * pull2) satisfies c >= 0 iff num >= 0, and (for
// num <= 0) c <= -1 iff num^2 >= scale2*pull2 — an exact rational test
// either way.
//
// revolveUndercuts is NOT converted: it carries the same defect through
// sweepExtremes, a genuinely different reader, and fixing it is fu188's
// scope, not this one's.

// pullVerdict is the three-valued §6 membership answer for a face's own
// normal-component range against the pull: pullClear is a proven absence of
// opposition (perpendicular or exactly antiparallel included, per §6's own
// carve-outs), pullOpposes is a proven point of opposition, and
// pullUndecided is neither — the range straddles a boundary the reading
// cannot separate.
type pullVerdict int

const (
	pullUndecided pullVerdict = iota
	pullClear
	pullOpposes
)

// decidePull answers the §6 membership rule for a normal-component range
// [mn, mx] read to within allow: pullClear when the whole range is proven at
// or above zero (mn-allow >= 0) or proven at or below -1 (mx+allow <= -1);
// pullOpposes when the range is proven to include a point strictly below
// zero and a point strictly above -1; pullUndecided otherwise. The allowance
// only ever pushes a straddling range toward undecided, never toward a
// decision — CLAUDE.md's reject-only rule applied to this reading.
func decidePull(mn, mx, allow float64) pullVerdict {
	switch {
	case mn-allow >= 0, mx+allow <= -1:
		return pullClear
	case mn+allow < 0 && mx-allow > -1:
		return pullOpposes
	default:
		return pullUndecided
	}
}

// listVerdict folds one face's verdict into the running faces list and
// undecided flag prismUndercuts, cupUndercuts and capBlendUndercuts's
// receiver-wall loop each build over their own faces: ok=false (a
// non-finite input or a failed enclosure) propagates as a total refusal to
// the caller, matching every other refusal path these surveys already take.
func listVerdict(faces *[]*Face, undecided *bool, f *Face, verdict pullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case pullOpposes:
		*faces = append(*faces, f)
	case pullUndecided:
		*undecided = true
	}
	return true
}

// decideRationalComponent decides §6's rule for a component whose EXACT
// value is num / sqrt(scale2 * pull2) — num, scale2 and pull2 all exact
// rationals, scale2 and pull2 both non-negative — without ever taking a
// square root: perpendicular (component == 0) and antiparallel
// (component == -1) are each an exact equality on num and its square, so a
// float division is never asked to decide either. It never returns
// pullUndecided: every input here is exact, so the comparison always
// resolves.
func decideRationalComponent(num, scale2, pull2 *big.Rat) pullVerdict {
	if num.Sign() >= 0 {
		return pullClear
	}
	lhs := new(big.Rat).Mul(num, num)
	rhs := new(big.Rat).Mul(scale2, pull2)
	if lhs.Cmp(rhs) >= 0 {
		return pullClear
	}
	return pullOpposes
}

// decideCircularComponent decides §6's rule for sigma*(du*cosθ + dv*sinθ)
// over a window, bracketed as [minLo, minHi] (a bracket on the swept
// function's minimum) and [maxLo, maxHi] (a bracket on its maximum) —
// against sqrt(pull2) rather than 1, since these brackets already carry the
// pull's own (unnormalized) dot product. minLo/maxLo are safe LOWER bounds
// on the true extremes (built from attained sample and endpoint values) and
// minHi/maxHi safe UPPER bounds, so proving the range clear or opposing from
// them never overstates what the enclosure supports.
func decideCircularComponent(minLo, minHi, maxLo, maxHi, pull2 *big.Rat) pullVerdict {
	switch {
	case minLo.Sign() >= 0:
		return pullClear
	case maxHi.Sign() <= 0 && new(big.Rat).Mul(maxHi, maxHi).Cmp(pull2) >= 0:
		return pullClear
	}
	if minHi.Sign() >= 0 {
		// The minimum's own bracket straddles zero: whether any point at all
		// opposes is not decided from here.
		return pullUndecided
	}
	if maxLo.Sign() > 0 {
		return pullOpposes
	}
	if new(big.Rat).Mul(maxLo, maxLo).Cmp(pull2) < 0 {
		return pullOpposes
	}
	return pullUndecided
}

// wallNormalDecision answers §6's membership rule for one side walk's
// outward normal against the caller's pull, decided exactly over the
// rationals: a straight walk's plane-local tangent (tu, tv) gives a single
// exact component num / (|t|*|pull|), num = tv*du - tu*dv; a circular
// walk's sweeps sigma*(du*cosθ + dv*sinθ)/|pull| over its own [th0, th1],
// bracketed rather than evaluated. du = m.du·pull and dv = m.dv·pull are
// exact — no sampling is involved building them, since m's directions and
// the caller's pull are both held floats. ok is false on any non-finite
// input or a failed enclosure.
func wallNormalDecision(w sideWalk, m placedFrameMap, pull r3.Vec) (pullVerdict, bool) {
	pv, okP := ivVec3Of(pull)
	if !okP {
		return pullUndecided, false
	}
	pull2 := ivVec3NormSq(pv).lo
	du := ivVec3Dot(m.du, pv).lo
	dv := ivVec3Dot(m.dv, pv).lo

	if !w.isCircular() {
		tu, tv := floatRat(w.tanInU), floatRat(w.tanInV)
		if tu == nil || tv == nil {
			return pullUndecided, false
		}
		t2 := ratAdd(ratMul(tu, tu), ratMul(tv, tv))
		num := new(big.Rat).Sub(new(big.Rat).Mul(tv, du), new(big.Rat).Mul(tu, dv))
		return decideRationalComponent(num, t2, pull2), true
	}

	sigma := big.NewRat(1, 1)
	if w.th1 < w.th0 {
		sigma = big.NewRat(-1, 1)
	}
	a := new(big.Rat).Mul(sigma, du)
	b := new(big.Rat).Mul(sigma, dv)
	lo, hi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
	minLo, minHi, maxLo, maxHi, ok := circularNormalRange(a, b, lo, hi, w.closed)
	if !ok {
		return pullUndecided, false
	}
	return decideCircularComponent(minLo, minHi, maxLo, maxHi, pull2), true
}

// capNormalDecision answers §6's rule for a planar face whose outward normal
// is an exact `sign` multiple of the placed frame's own n direction (sign is
// always ±1: a prism's two caps, or a cup's kept cap/pocket floor/rims under
// their own sOpen sign, shell_cup.go's cupUndercuts). m.dn's own held length
// need not be exactly one — a placed frame's image of a unit vector is only
// near-unit — so the antiparallel arm compares squares against |m.dn|²
// rather than assuming a unit reading, exactly as the wall reader compares
// against a wall's own tangent length squared.
func capNormalDecision(m placedFrameMap, pull r3.Vec, sign float64) (pullVerdict, bool) {
	pv, okP := ivVec3Of(pull)
	rSign := floatRat(sign)
	if !okP || rSign == nil {
		return pullUndecided, false
	}
	pull2 := ivVec3NormSq(pv).lo
	scale2 := ivVec3NormSq(m.dn).lo
	num := new(big.Rat).Mul(rSign, ivVec3Dot(m.dn, pv).lo)
	return decideRationalComponent(num, scale2, pull2), true
}

// circularNormalRange encloses a*cosθ + b*sinθ over θ ∈ [lo, hi] (lo <= hi,
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
// windowReachesDirection, called unmodified: the same robust cross-product
// containment test that function already proves sound against a 200k-sample
// brute force, rather than a second implementation of the same idea.
// wholeTurn (the walk's own structural flag, sideWalk.closed) skips the
// search entirely: a full turn always attains both extremes.
//
// This does not reuse harmonicWindowRange itself: that function's window is
// measured from φ = θ - th0 in a frame its own three sampled coefficients are
// already anchored to, and rotating OUR exact (a, b) into that frame would
// need cos(lo) and sin(lo) — themselves irrational for a generic lo — turning
// an exact input into an interval one for no reason, since a and b already
// hold everywhere over [lo, hi] with no anchor at all.
func circularNormalRange(a, b *big.Rat, lo, hi float64, wholeTurn bool) (minLo, minHi, maxLo, maxHi *big.Rat, ok bool) {
	rlo, rhi := floatRat(lo), floatRat(hi)
	if rlo == nil || rhi == nil {
		return nil, nil, nil, nil, false
	}
	width := new(big.Rat).Sub(rhi, rlo)
	if width.Sign() < 0 {
		return nil, nil, nil, nil, false
	}
	amp, okAmp := intervalSqrt(pointInterval(ratAdd(ratMul(a, a), ratMul(b, b))))
	if !okAmp {
		return nil, nil, nil, nil, false
	}
	peakLo, peakHi := amp.lo, amp.hi
	troughLo, troughHi := new(big.Rat).Neg(amp.hi), new(big.Rat).Neg(amp.lo)
	if wholeTurn {
		return troughLo, troughHi, peakLo, peakHi, true
	}

	const arcs = 4
	sins, coss := make([]ratInterval, arcs+1), make([]ratInterval, arcs+1)
	for j := range arcs + 1 {
		theta := new(big.Rat).Add(rlo, ratMul(width, big.NewRat(int64(j), arcs)))
		sin, cos, okT := radSinCosInterval(theta)
		if !okT {
			return nil, nil, nil, nil, false
		}
		sins[j], coss[j] = sin, cos
		at := intervalAdd(intervalScale(cos, a), intervalScale(sin, b))
		if j == 0 {
			minLo, minHi, maxLo, maxHi = at.lo, at.hi, at.lo, at.hi
			continue
		}
		minLo, minHi = ratMin(minLo, at.lo), ratMin(minHi, at.hi)
		maxLo, maxHi = ratMax(maxLo, at.lo), ratMax(maxHi, at.hi)
	}

	sure, maybe := windowReachesDirection(coss, sins, a, b)
	if maybe {
		maxHi = ratMax(maxHi, peakHi)
	}
	if sure {
		maxLo = ratMax(maxLo, peakLo)
	}
	sure, maybe = windowReachesDirection(coss, sins, new(big.Rat).Neg(a), new(big.Rat).Neg(b))
	if maybe {
		minLo = ratMin(minLo, troughLo)
	}
	if sure {
		minHi = ratMin(minHi, troughHi)
	}
	return minLo, minHi, maxLo, maxHi, true
}
