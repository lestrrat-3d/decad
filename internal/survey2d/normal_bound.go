package survey2d

import (
	"encoding/binary"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// IvVec3 encloses a 3D vector coordinate-wise. A vector built from held
// float64s alone encloses it EXACTLY — every interval is a point — and only a
// square root or a sine widens one.
type IvVec3 [3]proofbound.RatInterval

// IvVec3Of encloses a held vector exactly, one point interval per coordinate.
func IvVec3Of(v r3.Vec) (IvVec3, bool) {
	x, y, z := proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)
	if x == nil || y == nil || z == nil {
		return IvVec3{}, false
	}
	return IvVec3{proofbound.PointInterval(x), proofbound.PointInterval(y), proofbound.PointInterval(z)}, true
}

func IvVec3Add(a, b IvVec3) IvVec3 {
	return IvVec3{proofbound.IntervalAdd(a[0], b[0]), proofbound.IntervalAdd(a[1], b[1]), proofbound.IntervalAdd(a[2], b[2])}
}

func IvVec3Mul(a IvVec3, s proofbound.RatInterval) IvVec3 {
	return IvVec3{proofbound.IntervalMul(a[0], s), proofbound.IntervalMul(a[1], s), proofbound.IntervalMul(a[2], s)}
}

func IvVec3Dot(a, b IvVec3) proofbound.RatInterval {
	return proofbound.IntervalAdd(proofbound.IntervalAdd(proofbound.IntervalMul(a[0], b[0]), proofbound.IntervalMul(a[1], b[1])), proofbound.IntervalMul(a[2], b[2]))
}

// IvVec3NormSq is |v|² — IntervalSquare per coordinate, never proofbound.IntervalMul with
// itself, so a coordinate straddling zero cannot contribute a negative low
// end.
func IvVec3NormSq(a IvVec3) proofbound.RatInterval {
	return proofbound.IntervalAdd(proofbound.IntervalAdd(IntervalSquare(a[0]), IntervalSquare(a[1])), IntervalSquare(a[2]))
}

// TurnGridShift is the dyadic grid the radian-to-turn conversion lands on
// before internal/proofbound/moments_trig.go's series runs. π's own in-tree bounds carry
// seventy-odd digits, so the quotient by 2π is a rational nothing needs to
// square that wide; rounding it down to a 2⁻⁹⁶ grid and charging the whole
// gap back through the sine's own Lipschitz constant keeps the series input
// small while leaving the enclosure valid.
const TurnGridShift = 96

// RadSinCosInterval encloses sin(x) and cos(x) for an exact rational RADIAN
// value x, which is what a Cone's half angle is once it has been read out in
// radians.
//
// internal/proofbound/moments_trig.go's proofbound.TurnSinCosInterval answers for a rational TURN, and a
// radian value is not one: dividing by 2π lands on an interval, since π is
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
func RadSinCosInterval(x *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	if x.Sign() == 0 {
		zero, one := new(big.Rat), big.NewRat(1, 1)
		return proofbound.Interval(zero, zero), proofbound.Interval(one, one), true
	}
	key := radSinCosKey(x)
	if e, ok := radSinCosMemo.get(key); ok {
		return e.sin, e.cos, e.ok
	}
	sin, cos, ok := radSinCosIntervalUncached(x)
	radSinCosMemo.put(key, radSinCosEntry{sin: sin, cos: cos, ok: ok})
	return sin, cos, ok
}

// radSinCosMemoCap bounds the memo. The whole apitest suite reads about nine
// thousand distinct angles, each entry four rationals of a few hundred bits;
// a full memo is cleared before the next insert.
const radSinCosMemoCap = 16384

type radSinCosEntry struct {
	sin, cos proofbound.RatInterval
	ok       bool
}

// clone returns the entry with every endpoint freshly allocated.
func (e radSinCosEntry) clone() radSinCosEntry {
	if !e.ok {
		return e
	}
	return radSinCosEntry{sin: proofbound.Interval(e.sin.Lo, e.sin.Hi), cos: proofbound.Interval(e.cos.Lo, e.cos.Hi), ok: true}
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
func radSinCosIntervalUncached(x *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	twoPi := proofbound.TwoPiInterval()
	turn, ok := IntervalQuo(proofbound.PointInterval(x), twoPi)
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	base := RatFloorGrid(turn.Lo, TurnGridShift)
	gap := new(big.Rat).Sub(turn.Hi, base)
	if gap.Sign() < 0 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	sin, cos := proofbound.TurnSinCosInterval(base)
	slop := new(big.Rat).Mul(twoPi.Hi, gap)
	return IntervalWiden(sin, slop), IntervalWiden(cos, slop), true
}

// IntervalWiden grows an enclosure by a non-negative rational on both ends.
func IntervalWiden(a proofbound.RatInterval, w *big.Rat) proofbound.RatInterval {
	return proofbound.Interval(new(big.Rat).Sub(a.Lo, w), new(big.Rat).Add(a.Hi, w))
}

// RatFloorGrid rounds a rational DOWN onto the 2⁻ˢʰⁱᶠᵗ grid, so the result is
// never above the input and sits within 2⁻ˢʰⁱᶠᵗ of it. Rounding down in one
// direction only is what lets the caller charge the whole gap from one end.
func RatFloorGrid(x *big.Rat, shift uint) *big.Rat {
	scale := new(big.Int).Lsh(big.NewInt(1), shift)
	num := new(big.Int).Mul(x.Num(), scale)
	// Denom is positive for every big.Rat, so Div is the floor.
	q := new(big.Int).Div(num, x.Denom())
	return new(big.Rat).SetFrac(q, scale)
}
