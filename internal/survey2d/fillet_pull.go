package survey2d

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the undercut survey's reading of a loop fillet's pipe patch
// (docs/loop-fillet-design.md Table DF's DF7). A patch's outward normal at
// azimuth θ and tube angle φ ∈ [0, π/2] is cos φ·ν + sin φ·σ·û(θ), with ν the
// outward normal of the face F the band meets, σ the band's side and û(θ)
// the inward normal of F's region at the loop point at azimuth θ. Its component
// along the pull is cos φ·A + sin φ·B(θ). Both functions below are
// nondecreasing in A and in B(θ), so the extremes over the whole patch come
// from A's and B's own ends.

// PullBracket brackets the minimum and maximum of a harmonic component,
// MinLo <= min <= MinHi and MaxLo <= max <= MaxHi, in units of the pull's own
// length (the convention DecideCircularComponent reads).
type PullBracket struct{ MinLo, MinHi, MaxLo, MaxHi *big.Rat }

// PullSquared is the pull's exact squared length.
func PullSquared(pull r3.Vec) (*big.Rat, bool) {
	pv, ok := proofbound.IvVec3Of(pull)
	if !ok {
		return nil, false
	}
	return proofbound.IvVec3NormSq(pv).Lo, true
}

// InwardRange brackets coef·(û·pull) over the walk w of F's loop, û its
// inward normal (into F's material). m is F's placed frame.
func InwardRange(w SideWalk, m PlacedFrameMap, pull r3.Vec, coef int64) (PullBracket, bool) {
	pv, ok := proofbound.IvVec3Of(pull)
	if !ok {
		return PullBracket{}, false
	}
	du := proofbound.IvVec3Dot(m.Du, pv).Lo
	dv := proofbound.IvVec3Dot(m.Dv, pv).Lo
	c := big.NewRat(coef, 1)
	switch w.Kind {
	case WalkLine:
		tu, tv, ok := lineDirectionEnclosure(w.SegmentWalk)
		if !ok {
			return PullBracket{}, false
		}
		t2 := proofbound.IntervalAdd(proofbound.IntervalSquare(tu), proofbound.IntervalSquare(tv))
		length, ok := proofbound.IntervalSqrt(t2)
		if !ok {
			return PullBracket{}, false
		}
		// The left normal of t is (−tv, tu).
		num := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalScale(tu, dv), proofbound.IntervalScale(tv, du)), c)
		q, ok := proofbound.IntervalQuo(num, length)
		if !ok {
			return PullBracket{}, false
		}
		return PullBracket{q.Lo, q.Hi, q.Lo, q.Hi}, true
	case WalkCircular:
		s := big.NewRat(1, 1)
		if w.Th1 < w.Th0 {
			s = big.NewRat(-1, 1)
		}
		// Inward is −s·ρ̂(θ).
		k := new(big.Rat).Mul(c, new(big.Rat).Neg(s))
		a, b := new(big.Rat).Mul(k, du), new(big.Rat).Mul(k, dv)
		var win circularWindow
		if !w.Closed {
			if win, ok = circularWindowOf(w); !ok {
				return PullBracket{}, false
			}
		}
		minLo, minHi, maxLo, maxHi, ok := circularNormalRange(a, b, win, w.Closed)
		return PullBracket{minLo, minHi, maxLo, maxHi}, ok
	}
	return PullBracket{}, false
}

// inwardAt is the exact inward normal of w at its start (atEnd false) or end.
func inwardAt(w SideWalk, atEnd bool) ([2]*big.Rat, bool) {
	su, sv := proofarith.FloatRat(w.StartU), proofarith.FloatRat(w.StartV)
	eu, ev := proofarith.FloatRat(w.EndU), proofarith.FloatRat(w.EndV)
	if su == nil || sv == nil || eu == nil || ev == nil {
		return [2]*big.Rat{}, false
	}
	if w.Kind == WalkLine {
		tu, tv := new(big.Rat).Sub(eu, su), new(big.Rat).Sub(ev, sv)
		return [2]*big.Rat{new(big.Rat).Neg(tv), tu}, true
	}
	cu, cv := proofarith.FloatRat(w.CU), proofarith.FloatRat(w.CV)
	if cu == nil || cv == nil {
		return [2]*big.Rat{}, false
	}
	pu, pv := su, sv
	if atEnd {
		pu, pv = eu, ev
	}
	ru, rv := new(big.Rat).Sub(pu, cu), new(big.Rat).Sub(pv, cv)
	if w.Th1 > w.Th0 {
		ru, rv = ru.Neg(ru), rv.Neg(rv)
	}
	return [2]*big.Rat{ru, rv}, true
}

// CornerRange brackets coef·(û·pull) over the window of directions û turns
// through at the reflex corner where prev arrives at cur: from the arriving
// walk's inward normal clockwise to the leaving walk's (a turn short of a half
// turn).
func CornerRange(prev, cur SideWalk, m PlacedFrameMap, pull r3.Vec, coef int64) (PullBracket, bool) {
	pv, ok := proofbound.IvVec3Of(pull)
	n1, ok1 := inwardAt(prev, true)
	n2, ok2 := inwardAt(cur, false)
	if !ok || !ok1 || !ok2 {
		return PullBracket{}, false
	}
	var win circularWindow
	// Counter-clockwise from the leaving normal to the arriving one.
	for _, n := range [][2]*big.Rat{n2, n1} {
		x, y := proofbound.PointInterval(n[0]), proofbound.PointInterval(n[1])
		l, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(proofbound.IntervalSquare(x), proofbound.IntervalSquare(y)))
		if !ok || l.Lo.Sign() <= 0 {
			return PullBracket{}, false
		}
		win.add(x, y, l)
	}
	c := big.NewRat(coef, 1)
	a := new(big.Rat).Mul(c, proofbound.IvVec3Dot(m.Du, pv).Lo)
	b := new(big.Rat).Mul(c, proofbound.IvVec3Dot(m.Dv, pv).Lo)
	minLo, minHi, maxLo, maxHi, ok := circularNormalRange(a, b, win, false)
	return PullBracket{minLo, minHi, maxLo, maxHi}, ok
}

// PatchPullVerdict decides §6's membership rule for a pipe patch whose normal
// component is cos φ·A + sin φ·B, φ ∈ [0, π/2], A = sign·(F's frame normal ·
// pull) and B ranging over b.
func PatchPullVerdict(m PlacedFrameMap, pull r3.Vec, sign int64, b PullBracket) (PullVerdict, bool) {
	pv, ok := proofbound.IvVec3Of(pull)
	pull2, ok2 := PullSquared(pull)
	if !ok || !ok2 {
		return PullUndecided, false
	}
	a := proofbound.IntervalScale(proofbound.IvVec3Dot(m.Dn, pv), big.NewRat(sign, 1))
	maxPhi := func(a, b *big.Rat, up bool) (*big.Rat, bool) {
		if a.Sign() > 0 && b.Sign() > 0 {
			r, ok := proofbound.IntervalSqrt(proofbound.PointInterval(proofbound.RatAdd(proofbound.RatMul(a, a), proofbound.RatMul(b, b))))
			if !ok {
				return nil, false
			}
			if up {
				return r.Hi, true
			}
			return r.Lo, true
		}
		return proofbound.RatMax(a, b), true
	}
	minPhi := func(a, b *big.Rat, up bool) (*big.Rat, bool) {
		if a.Sign() < 0 && b.Sign() < 0 {
			r, ok := proofbound.IntervalSqrt(proofbound.PointInterval(proofbound.RatAdd(proofbound.RatMul(a, a), proofbound.RatMul(b, b))))
			if !ok {
				return nil, false
			}
			if up {
				return new(big.Rat).Neg(r.Lo), true
			}
			return new(big.Rat).Neg(r.Hi), true
		}
		return proofbound.RatMin(a, b), true
	}
	maxLo, o1 := maxPhi(a.Lo, b.MaxLo, false)
	maxHi, o2 := maxPhi(a.Hi, b.MaxHi, true)
	minLo, o3 := minPhi(a.Lo, b.MinLo, false)
	minHi, o4 := minPhi(a.Hi, b.MinHi, true)
	if !o1 || !o2 || !o3 || !o4 {
		return PullUndecided, false
	}
	return DecideCircularComponent(minLo, minHi, maxLo, maxHi, pull2), true
}
