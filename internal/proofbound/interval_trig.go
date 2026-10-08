package proofbound

import (
	"encoding/binary"
	"math/big"
	"sync"
)

// TurnGridShift is the dyadic grid the radian-to-turn conversion lands on
// before turn_trig.go's series runs. π's own in-tree bounds carry
// seventy-odd digits, so the quotient by 2π is a rational nothing needs to
// square that wide; rounding it down to a 2⁻⁹⁶ grid and charging the whole
// gap back through the sine's own Lipschitz constant keeps the series input
// small while leaving the enclosure valid.
const TurnGridShift = 96

// RadSinCosInterval encloses sin(x) and cos(x) for an exact rational RADIAN
// value x, which is what a Cone's half angle is once it has been read out in
// radians.
//
// TurnSinCosInterval answers for a rational TURN, and a radian value is not
// one: dividing by 2π lands on an interval, since π is
// itself only enclosed. So the turn is rounded DOWN onto a dyadic grid, the
// series is evaluated at that one point, and both readings are widened by
// 2π·w with w the whole remaining gap — sound because |sin(2πt) − sin(2πt₀)|
// ≤ 2π|t − t₀| and the same for the cosine, so no monotonicity over the gap
// need be argued.
//
// The enclosure is a pure function of x, and tessellation asks for the same
// angle many times over (every revolve on one angular plan reads the same
// samples), so nonzero readings are memoised process-wide on x's exact value
// (radSinCosMemo). Every call returns freshly allocated endpoints the caller
// may mutate; the memo never hands out the values it holds.
func RadSinCosInterval(x *big.Rat) (RatInterval, RatInterval, bool) {
	if x.Sign() == 0 {
		zero, one := new(big.Rat), big.NewRat(1, 1)
		return Interval(zero, zero), Interval(one, one), true
	}
	key := radSinCosKey(x)
	if e, ok := radSinCosMemo.get(key); ok {
		return e.sin, e.cos, e.ok
	}
	sin, cos, ok := radSinCosIntervalUncached(x)
	radSinCosMemo.put(key, radSinCosEntry{sin: sin, cos: cos, ok: ok})
	return sin, cos, ok
}

// RadSinCosSpan encloses sine and cosine over a rational radian interval.
// Both functions are 1-Lipschitz, so the interval width widens the reading
// at its lower endpoint without assuming monotonicity over the span.
func RadSinCosSpan(x RatInterval) (RatInterval, RatInterval, bool) {
	width := new(big.Rat).Sub(x.Hi, x.Lo)
	if width.Sign() < 0 {
		return RatInterval{}, RatInterval{}, false
	}
	sin, cos, ok := RadSinCosInterval(x.Lo)
	if !ok {
		return RatInterval{}, RatInterval{}, false
	}
	return IntervalWiden(sin, width), IntervalWiden(cos, width), true
}

// RadTanSpan encloses tan(x) over a rational radian interval: RadSinCosSpan's
// sine enclosure divided by its cosine enclosure over exact rationals
// (docs/draft-design.md §8.1). ok is false where either enclosure cannot be
// built or the cosine enclosure reaches zero, since no box then encloses the
// quotient.
func RadTanSpan(x RatInterval) (RatInterval, bool) {
	sin, cos, ok := RadSinCosSpan(x)
	if !ok {
		return RatInterval{}, false
	}
	return IntervalQuo(sin, cos)
}

// radSinCosMemoCap bounds the memo. The whole apitest suite reads about nine
// thousand distinct angles, each entry four rationals of a few hundred bits;
// a full memo is cleared before the next insert.
const radSinCosMemoCap = 16384

type radSinCosEntry struct {
	sin, cos RatInterval
	ok       bool
}

// clone returns the entry with every endpoint freshly allocated.
func (e radSinCosEntry) clone() radSinCosEntry {
	if !e.ok {
		return e
	}
	return radSinCosEntry{sin: Interval(e.sin.Lo, e.sin.Hi), cos: Interval(e.cos.Lo, e.cos.Hi), ok: true}
}

type radSinCosMemoMap struct {
	mu      sync.Mutex
	entries map[string]radSinCosEntry
}

var radSinCosMemo = &radSinCosMemoMap{entries: map[string]radSinCosEntry{}}

// get returns a fresh copy of the reading memoised for key, or ok false when
// none is held.
func (m *radSinCosMemoMap) get(key string) (radSinCosEntry, bool) {
	m.mu.Lock()
	e, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return e, false
	}
	return e.clone(), true
}

// put records a copy of e, so the caller keeps sole ownership of e itself.
func (m *radSinCosMemoMap) put(key string, e radSinCosEntry) {
	held := e.clone()
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.entries) >= radSinCosMemoCap {
		clear(m.entries)
	}
	m.entries[key] = held
}

// radSinCosKey encodes x's exact value: its sign, then its reduced numerator's
// magnitude length-prefixed, then its denominator.
func radSinCosKey(x *big.Rat) string {
	num, den := x.Num().Bytes(), x.Denom().Bytes()
	buf := make([]byte, 0, 1+binary.MaxVarintLen64+len(num)+len(den))
	buf = append(buf, byte(x.Sign()+1))
	buf = binary.AppendUvarint(buf, uint64(len(num)))
	buf = append(buf, num...)
	buf = append(buf, den...)
	return string(buf)
}

// radSinCosIntervalUncached is RadSinCosInterval's reading for a nonzero x
// without the memo; RadSinCosInterval's doc comment states the argument.
func radSinCosIntervalUncached(x *big.Rat) (RatInterval, RatInterval, bool) {
	twoPi := TwoPiInterval()
	turn, ok := IntervalQuo(PointInterval(x), twoPi)
	if !ok {
		return RatInterval{}, RatInterval{}, false
	}
	base := RatFloorGrid(turn.Lo, TurnGridShift)
	gap := new(big.Rat).Sub(turn.Hi, base)
	if gap.Sign() < 0 {
		return RatInterval{}, RatInterval{}, false
	}
	sin, cos := TurnSinCosInterval(base)
	slop := new(big.Rat).Mul(twoPi.Hi, gap)
	return IntervalWiden(sin, slop), IntervalWiden(cos, slop), true
}

// WindowReachesDirection decides, over arcs each shorter than a half turn,
// whether the direction (dx, dy) lies inside the window the given cosines and
// sines bound. It returns the proven answer first and the possible one second,
// and never collapses the two: a straddling cross product leaves the direction
// possible but unproven, which is exactly the case an extreme may only widen an
// enclosure with rather than fix an end of it.
//
// A zero direction — the constant form, a = b = 0 — makes both cross products
// exactly zero and so reads as proven inside, which is right: every azimuth
// attains the constant.
func WindowReachesDirection(coss, sins []RatInterval, dx, dy *big.Rat) (bool, bool) {
	sure, maybe := false, false
	for j := 0; j+1 < len(coss); j++ {
		// The cross product of the arc's start with the direction, then of the
		// direction with the arc's end: both non-negative places it between them.
		from := IntervalSub(IntervalScale(coss[j], dy), IntervalScale(sins[j], dx))
		to := IntervalSub(IntervalScale(sins[j+1], dx), IntervalScale(coss[j+1], dy))
		if from.Lo.Sign() >= 0 && to.Lo.Sign() >= 0 {
			sure = true
		}
		if from.Hi.Sign() >= 0 && to.Hi.Sign() >= 0 {
			maybe = true
		}
	}
	return sure, maybe
}
