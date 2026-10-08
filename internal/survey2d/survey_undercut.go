package survey2d

import (
	"math"
	"math/big"
	"slices"

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
// circular walk's component sweeps sigma·(du·cosθ + dv·sinθ)/|pull| over the
// window its record denotes, bracketed rather than evaluated. The window's
// ends are the directions from the recorded centre to the walk's two ends,
// each widened by its own end bound (circularWindowOf). It never reads the
// held Th0 and Th1 as the window: they are a float multiple of 2π for a
// CircleSeg and math.Atan2 for an ArcSeg, so a window end can sit an ulp past
// a direction where the component changes sign. Where an end's enclosure
// leaves that sign open, the verdict is PullUndecided. A window that cannot be
// cut into arcs shorter than a half turn (circularWindow.unbracketed) is
// decided from the ends whose directions it encloses alone. du = m.du·pull and
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
		var win circularWindow
		if !w.Closed {
			var ok bool
			if win, ok = circularWindowOf(w); !ok {
				return PullUndecided, false
			}
		}
		minLo, minHi, maxLo, maxHi, ok := circularNormalRange(a, b, win, w.Closed)
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

// circularWindow encloses a circular walk's denoted window counterclockwise,
// from its low angle to its high angle, by the directions that cut it into
// arcs each shorter than a half turn. Direction j is (xs[j], ys[j]) scaled by
// 1/ls[j]: the first and last are the vectors from the recorded centre to the
// window's two denoted ends, kept unnormalized so that an exact end keeps its
// exact sign against any exact direction, and the ones between are proven to
// lie strictly inside the window.
//
// unbracketed marks a window that cannot be cut into arcs shorter than a half
// turn: an end's box reaches the centre or its angle cannot be enclosed, or
// the ends' angle enclosures overlap while the held sweep is a half turn or
// more. xs, ys and ls then hold only the ends whose directions are enclosed,
// zero, one or two of them, and no arc between them is read.
type circularWindow struct {
	xs, ys, ls  []proofbound.RatInterval
	unbracketed bool
}

func (cw *circularWindow) add(x, y, l proofbound.RatInterval) {
	cw.xs = append(cw.xs, x)
	cw.ys = append(cw.ys, y)
	cw.ls = append(cw.ls, l)
}

// circularWindowOf encloses the window a circular walk's record denotes. Its
// ends are the points the walk's end bounds enclose, read as directions from
// the recorded centre. Three angles split the part of the window both ends'
// angle enclosures prove covered into four equal arcs. A window too narrow
// for that keeps its two ends alone, one arc shorter than a half turn, when
// its held sweep is under a half turn, and is unbracketed otherwise. An end
// whose box reaches the centre, whose end bound states no finite reach, or
// whose angle EndAngleEnclosure refuses leaves the window unbracketed without
// that end. ok is false where a centre, end or held angle is not finite.
func circularWindowOf(w SideWalk) (circularWindow, bool) {
	type end struct {
		u, v, held float64
		bound      proofbound.WalkEndBound
	}
	low := end{w.StartU, w.StartV, w.Th0, w.StartBound}
	high := end{w.EndU, w.EndV, w.Th1, w.EndBound}
	if w.Th1 < w.Th0 {
		low, high = high, low
	}
	if slices.ContainsFunc([]float64{w.CU, w.CV, low.u, low.v, low.held, high.u, high.v, high.held}, proofbound.IsNonFinite) {
		return circularWindow{}, false
	}
	type enclosed struct {
		x, y, l, angle proofbound.RatInterval
		ok             bool
	}
	var ends [2]enclosed
	for i, e := range []end{low, high} {
		reach := proofbound.WalkEndBoundAllow(e.bound)
		x, y, l, okD := endDirection(w.CU, w.CV, e.u, e.v, reach)
		angle, okA := EndAngleEnclosure(w.CU, w.CV, e.u, e.v, reach, e.held)
		ends[i] = enclosed{x: x, y: y, l: l, angle: angle, ok: okD && okA}
	}
	if !ends[0].ok || !ends[1].ok {
		cw := circularWindow{unbracketed: true}
		for _, e := range ends {
			if e.ok {
				cw.add(e.x, e.y, e.l)
			}
		}
		return cw, true
	}
	var cw circularWindow
	cw.add(ends[0].x, ends[0].y, ends[0].l)
	inner := new(big.Rat).Sub(ends[1].angle.Lo, ends[0].angle.Hi)
	switch {
	case inner.Sign() > 0:
		one := proofbound.PointInterval(big.NewRat(1, 1))
		for j := int64(1); j <= 3; j++ {
			theta := new(big.Rat).Add(ends[0].angle.Hi, proofbound.RatMul(inner, big.NewRat(j, 4)))
			sin, cos, ok := proofbound.RadSinCosInterval(theta)
			if !ok {
				return circularWindow{}, false
			}
			cw.add(cos, sin, one)
		}
	case math.Abs(w.Th1-w.Th0) >= math.Pi:
		cw.unbracketed = true
	}
	cw.add(ends[1].x, ends[1].y, ends[1].l)
	return cw, true
}

// endDirection encloses the vector from the centre (cU, cV) to every point
// within reach of (u, v) on each axis, and that vector's length. ok is false
// where the box reaches the centre, reach is not finite, or a coordinate does
// not lift.
func endDirection(cU, cV, u, v, reach float64) (x, y, l proofbound.RatInterval, ok bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	rcu, rcv := proofarith.FloatRat(cU), proofarith.FloatRat(cV)
	allow := proofarith.FloatRat(reach)
	if ru == nil || rv == nil || rcu == nil || rcv == nil || allow == nil || allow.Sign() < 0 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	widen := func(c, centre *big.Rat) proofbound.RatInterval {
		d := new(big.Rat).Sub(c, centre)
		return proofbound.Interval(new(big.Rat).Sub(d, allow), new(big.Rat).Add(d, allow))
	}
	x, y = widen(ru, rcu), widen(rv, rcv)
	l, ok = proofbound.IntervalSqrt(proofbound.IntervalAdd(proofbound.IntervalSquare(x), proofbound.IntervalSquare(y)))
	if !ok || l.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	return x, y, l, true
}

// EndAngleEnclosure encloses the exact angle about (cU, cV) of every point
// within reach of (u, v), on the branch nearest held.
//
// The point and the centre are float64s, so the direction between them is an
// exact rational and proofbound.Atan2Interval encloses its angle with no libm
// accuracy assumed. A point within reach turns that angle by at most
// arcsin(reach/ρ) ≤ (π/2)·reach/ρ, ρ the point's distance from the centre,
// which widens the enclosure. A reach at or past ρ says nothing about the
// angle and answers false, as does a point on the centre.
//
// The branch is the one nearest held. Every caller's held angle is a float
// Atan2 of the same direction, a float multiple of 2π, or such an angle
// unwrapped by whole turns, so it lies within a few ulps of its own branch and
// more than π from any other.
func EndAngleEnclosure(cU, cV, u, v, reach, held float64) (proofbound.RatInterval, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	rcu, rcv := proofarith.FloatRat(cU), proofarith.FloatRat(cV)
	if ru == nil || rv == nil || rcu == nil || rcv == nil || !(reach >= 0) || proofbound.IsNonFinite(reach) || proofbound.IsNonFinite(held) {
		return proofbound.RatInterval{}, false
	}
	dU, dV := new(big.Rat).Sub(ru, rcu), new(big.Rat).Sub(rv, rcv)
	if dU.Sign() == 0 && dV.Sign() == 0 {
		return proofbound.RatInterval{}, false
	}
	iv := proofbound.Atan2Interval(dV, dU, false)
	mid, _ := new(big.Rat).Quo(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(2, 1)).Float64()
	if turns := math.Round((held - mid) / (2 * math.Pi)); turns != 0 {
		iv = proofbound.IntervalAdd(iv, proofbound.IntervalScale(proofbound.TwoPiInterval(), new(big.Rat).SetFloat64(turns)))
	}
	if reach > 0 {
		rhoLower := proofbound.RatSqrtDown(new(big.Rat).Add(new(big.Rat).Mul(dU, dU), new(big.Rat).Mul(dV, dV)))
		if !(reach < rhoLower) {
			return proofbound.RatInterval{}, false
		}
		turn := new(big.Rat).Quo(
			proofbound.RatMul(proofbound.PiUpper, proofarith.FloatRat(reach)),
			proofbound.RatMul(big.NewRat(2, 1), proofarith.FloatRat(rhoLower)),
		)
		iv = proofbound.IntervalWiden(iv, turn)
	}
	return iv, true
}

// circularNormalRange encloses a*cosθ + b*sinθ over the window win encloses: a
// bracket [minLo, minHi] on the function's minimum and a bracket
// [maxLo, maxHi] on its maximum, each attained wherever the search below can
// prove it. a and b are exact, with no sampling involved building them,
// unlike a cap-blend patch's recovered coefficients (capblend_normal.go), so
// nothing here is charged an allowance. At each of the window's directions the
// function is (a·x + b·y)/l, so an exact end on an exact perpendicular reads
// exactly zero.
//
// The window's arcs are searched for each of the two critical directions
// (a, b) and (-a, -b) with capblend_normal.go's
// proofbound.WindowReachesDirection, called unmodified: the same robust
// cross-product containment test that function already proves sound against
// a 200k-sample brute force. Its signs do not change under a positive scale,
// so it reads the unnormalized directions as they are. wholeTurn (the walk's
// own structural flag, SideWalk.closed) skips the search entirely: a full turn
// always attains both extremes, and win is not read.
//
// An unbracketed window's interior is not read at all, so the function's
// peak and trough may lie anywhere in it. Each extreme then spans from the
// amplitude's bound to the values its enclosed ends attain, or the whole
// [−amplitude, amplitude] where no end is enclosed. A pull with a = b = 0
// has amplitude zero, so both brackets close on zero.
func circularNormalRange(a, b *big.Rat, win circularWindow, wholeTurn bool) (minLo, minHi, maxLo, maxHi *big.Rat, ok bool) {
	amp, okAmp := proofbound.IntervalSqrt(proofbound.PointInterval(proofbound.RatAdd(proofbound.RatMul(a, a), proofbound.RatMul(b, b))))
	if !okAmp {
		return nil, nil, nil, nil, false
	}
	peakLo, peakHi := amp.Lo, amp.Hi
	troughLo, troughHi := new(big.Rat).Neg(amp.Hi), new(big.Rat).Neg(amp.Lo)
	if wholeTurn {
		return troughLo, troughHi, peakLo, peakHi, true
	}
	if win.unbracketed {
		minLo, minHi, maxLo, maxHi = troughLo, peakHi, troughLo, peakHi
		for j := range win.xs {
			at, okAt := proofbound.IntervalQuo(proofbound.IntervalAdd(proofbound.IntervalScale(win.xs[j], a), proofbound.IntervalScale(win.ys[j], b)), win.ls[j])
			if !okAt {
				return nil, nil, nil, nil, false
			}
			minLo, minHi = proofbound.RatMin(minLo, at.Lo), proofbound.RatMin(minHi, at.Hi)
			maxLo, maxHi = proofbound.RatMax(maxLo, at.Lo), proofbound.RatMax(maxHi, at.Hi)
		}
		return minLo, minHi, maxLo, maxHi, true
	}
	if len(win.xs) < 2 {
		return nil, nil, nil, nil, false
	}
	for j := range win.xs {
		at, okAt := proofbound.IntervalQuo(proofbound.IntervalAdd(proofbound.IntervalScale(win.xs[j], a), proofbound.IntervalScale(win.ys[j], b)), win.ls[j])
		if !okAt {
			return nil, nil, nil, nil, false
		}
		if j == 0 {
			minLo, minHi, maxLo, maxHi = at.Lo, at.Hi, at.Lo, at.Hi
			continue
		}
		minLo, minHi = proofbound.RatMin(minLo, at.Lo), proofbound.RatMin(minHi, at.Hi)
		maxLo, maxHi = proofbound.RatMax(maxLo, at.Lo), proofbound.RatMax(maxHi, at.Hi)
	}

	sure, maybe := proofbound.WindowReachesDirection(win.xs, win.ys, a, b)
	if maybe {
		maxHi = proofbound.RatMax(maxHi, peakHi)
	}
	if sure {
		maxLo = proofbound.RatMax(maxLo, peakLo)
	}
	sure, maybe = proofbound.WindowReachesDirection(win.xs, win.ys, new(big.Rat).Neg(a), new(big.Rat).Neg(b))
	if maybe {
		minLo = proofbound.RatMin(minLo, troughLo)
	}
	if sure {
		minHi = proofbound.RatMin(minHi, troughHi)
	}
	return minLo, minHi, maxLo, maxHi, true
}
