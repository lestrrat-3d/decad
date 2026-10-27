package proofbound

import (
	"encoding/binary"
	"math/big"
	"sync"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file owns the atan, atan2, and pi enclosures used by certified readings.
// Basic rational interval operations live in internal/proof/interval.go.
//
// Every constant here is a PROVEN enclosure rather than a rounded literal, so
// a value computed through these operations encloses the true answer no
// matter how the float arithmetic beside it rounded. turn_trig.go owns the
// sine/cosine enclosure of an exact turn, which is proven without ever
// comparing against pi and so does not belong here.

type RatInterval struct {
	Lo *big.Rat
	Hi *big.Rat
}

// The trigonometric enclosures retain private endpoints in the root package.
// Arithmetic delegates to the exact interval operations in internal/proof.
func ToProofInterval(a RatInterval) proofarith.RatInterval {
	return proofarith.RatInterval{Lo: a.Lo, Hi: a.Hi}
}

func FromProofInterval(a proofarith.RatInterval) RatInterval {
	return RatInterval{Lo: a.Lo, Hi: a.Hi}
}

func Interval(lo, hi *big.Rat) RatInterval {
	return FromProofInterval(proofarith.Interval(lo, hi))
}

func IntervalOwned(lo, hi *big.Rat) RatInterval {
	return FromProofInterval(proofarith.OwnedInterval(lo, hi))
}

func PointInterval(value *big.Rat) RatInterval {
	return FromProofInterval(proofarith.PointInterval(value))
}

func IntervalAdd(a, b RatInterval) RatInterval {
	return FromProofInterval(proofarith.AddInterval(ToProofInterval(a), ToProofInterval(b)))
}

func IntervalNeg(a RatInterval) RatInterval {
	return FromProofInterval(proofarith.NegInterval(ToProofInterval(a)))
}

func IntervalSub(a, b RatInterval) RatInterval {
	return FromProofInterval(proofarith.SubInterval(ToProofInterval(a), ToProofInterval(b)))
}

func IntervalScale(a RatInterval, scale *big.Rat) RatInterval {
	return FromProofInterval(proofarith.ScaleInterval(ToProofInterval(a), scale))
}

func IntervalMul(a, b RatInterval) RatInterval {
	return FromProofInterval(proofarith.MulInterval(ToProofInterval(a), ToProofInterval(b)))
}

// IntervalAbsUpper is the largest magnitude an interval permits.
func IntervalAbsUpper(a RatInterval) *big.Rat {
	lo, hi := new(big.Rat).Abs(a.Lo), new(big.Rat).Abs(a.Hi)
	if proofarith.CmpRat(lo, hi) >= 0 {
		return lo
	}
	return hi
}

func IntervalFloatError(a RatInterval, held float64) float64 {
	return proofarith.IntervalFloatError(ToProofInterval(a), held)
}

var (
	PiLower = MustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406286")
	PiUpper = MustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406287")
)

// QuarterPiIv, HalfPiIv and TwoPiIv cache the pi-interval scalings every
// Atan2Interval/AtanPositiveInterval call and several circular brackets
// rebuild, each of which otherwise pays a GCD normalization on PiLower/
// PiUpper's ~250-bit operands per call. They are declared from IntervalScale
// directly, never from their own accessor — that would be an initialization
// cycle.
var (
	QuarterPiIv = IntervalScale(Interval(PiLower, PiUpper), big.NewRat(1, 4))
	HalfPiIv    = IntervalScale(Interval(PiLower, PiUpper), big.NewRat(1, 2))
	TwoPiIv     = IntervalScale(Interval(PiLower, PiUpper), big.NewRat(2, 1))
)

// QuarterPiInterval, HalfPiInterval and TwoPiInterval hand out copies of the
// cached pi-multiple intervals above. They MUST go through interval, which
// copies both endpoints: RatInterval holds pointers, so returning the cached
// value directly would let a caller mutate a package-level constant.
func QuarterPiInterval() RatInterval { return Interval(QuarterPiIv.Lo, QuarterPiIv.Hi) }
func HalfPiInterval() RatInterval    { return Interval(HalfPiIv.Lo, HalfPiIv.Hi) }
func TwoPiInterval() RatInterval     { return Interval(TwoPiIv.Lo, TwoPiIv.Hi) }

func MustRatDecimal(value string) *big.Rat {
	out, ok := new(big.Rat).SetString(value)
	if !ok {
		panic("decad: invalid in-tree rational constant")
	}
	return out
}

// FixedMulDown multiplies two non-negative fixed-point values and rounds the
// 2*TrigFixedBits-wide product down to TrigFixedBits, an inward (floor)
// rescale.
func FixedMulDown(a, b *big.Int) *big.Int {
	p := new(big.Int).Mul(a, b)
	return p.Rsh(p, TrigFixedBits)
}

// FixedMulUp multiplies two non-negative fixed-point values and rounds the
// product up, FixedMulDown's outward (ceiling) mirror.
// A nonzero bit below the fixed-point boundary is the remainder: shift the
// product down and add one exactly when that remainder is nonzero.
func FixedMulUp(a, b *big.Int) *big.Int {
	p := new(big.Int).Mul(a, b)
	roundUp := p.Sign() != 0 && p.TrailingZeroBits() < TrigFixedBits
	p.Rsh(p, TrigFixedBits)
	if roundUp {
		p.Add(p, big.NewInt(1))
	}
	return p
}

// FixedDivDown divides a non-negative fixed-point value by a positive
// integer, rounding down (Quo truncates toward zero, which is floor for a
// non-negative dividend).
func FixedDivDown(a *big.Int, d int64) *big.Int { return new(big.Int).Quo(a, big.NewInt(d)) }

// FixedDivUp is FixedDivDown's outward (ceiling) mirror.
func FixedDivUp(a *big.Int, d int64) *big.Int {
	q := new(big.Int).Add(a, big.NewInt(d-1))
	return q.Quo(q, big.NewInt(d))
}

// AtanSmallInterval bounds atan(x) for |x| <= 1/2, evaluating the same
// 64-term alternating Maclaurin series turn_trig.go's TrigFixedSeries
// uses for sin/cos, on the same fixed-point 2^-TrigFixedBits grid, rather
// than over big.Rat: a big.Rat pays a GCD normalization per operation, which
// dominates cost at this series' 10^4-bit numerators.
//
// The negative case folds to the positive one first (IntervalNeg), so every
// operand below is non-negative and Rsh/Quo's toward-zero rounding is floor.
// x is enclosed by xLo <= x*2^P <= xHi (FixedFloor/FixedCeil). By induction,
// carrying a lower chain (powerLo, rounded down at every step) and an upper
// chain (powerHi, rounded up at every step): powerLo_n <= x^(2n+1)*2^P <=
// powerHi_n for every n, so termLo_n <= (x^(2n+1)/(2n+1))*2^P <= termHi_n.
// The lower accumulator adds termLo on even n and subtracts termHi on odd n;
// the upper accumulator mirrors it (adds termHi, subtracts termLo) — each
// choice is the one that can only push its own side outward, never inward —
// so after all 64 terms, lo <= S_64(x)*2^P <= hi where S_64 is the 64-term
// partial sum.
//
// On [0, 1/2] the terms strictly decrease in magnitude, and the last summed
// term (n = 63) is subtracted, so the alternating-series remainder theorem
// gives S_64(x) <= atan(x) <= S_64(x) + x^129/129. The loop's final powerHi
// is the outward enclosure of x^129*2^P, and FixedDivUp(powerHi, 129) charges
// that remainder without narrowing it.
//
// Each power step's outward rescale (FixedMulUp against x2Hi, whose own
// factor has magnitude <=1) and each term's outward division together add at
// most 3 grid units of extra width per step (1 from the multiply's ceiling,
// 1 from the shared x2Hi rounding carried through a <=1-magnitude factor, 1
// from the divide's ceiling) — the same accounting TrigFixedSeries's own
// comment uses. Over 64 terms that is under 64*3 = 192 < 2^8 units, i.e.
// under 2^-(TrigFixedBits-8) = 2^-192, well inside the 2^-187 budget this
// function publishes and far below the 2^-136 series remainder already
// dominating the bound at x = 1/2.
func AtanSmallInterval(x *big.Rat) RatInterval {
	if x.Sign() < 0 {
		return IntervalNeg(AtanSmallInterval(new(big.Rat).Neg(x)))
	}
	xLo, xHi := FixedFloor(x), FixedCeil(x)
	x2Lo := FixedMulDown(xLo, xLo)
	x2Hi := FixedMulUp(xHi, xHi)
	powerLo, powerHi := new(big.Int).Set(xLo), new(big.Int).Set(xHi)
	lo, hi := new(big.Int), new(big.Int)
	for n := range 64 {
		d := int64(2*n + 1)
		tLo := FixedDivDown(powerLo, d)
		tHi := FixedDivUp(powerHi, d)
		if n%2 == 0 {
			lo.Add(lo, tLo)
			hi.Add(hi, tHi)
		} else {
			lo.Sub(lo, tHi)
			hi.Sub(hi, tLo)
		}
		powerLo = FixedMulDown(powerLo, x2Lo)
		powerHi = FixedMulUp(powerHi, x2Hi)
	}
	hi.Add(hi, FixedDivUp(powerHi, 129))
	return Interval(FixedToRat(lo), FixedToRat(hi))
}

func AtanPositiveInterval(x *big.Rat) RatInterval {
	if proofarith.CmpRat(x, big.NewRat(1, 2)) <= 0 {
		return AtanSmallInterval(x)
	}
	// atan(x) = π/4 + atan((x-1)/(x+1)); for x in (1/2,1], the transformed
	// argument lies in [-1/3,0), inside the fast alternating-series range.
	q := proofarith.QuoRat(new(big.Rat),
		proofarith.SubRat(new(big.Rat), x, big.NewRat(1, 1)),
		proofarith.AddRat(new(big.Rat), x, big.NewRat(1, 1)),
	)
	return IntervalAdd(QuarterPiInterval(), AtanSmallInterval(q))
}

// Atan2Interval encloses atan2(y, x) for exact rationals y and x.
// negativeZeroY picks -π over π when y is zero and x negative.
//
// Off the axes the enclosure is a deterministic function of y's and x's
// numerators and denominators (atan2IntervalSeries reads nothing else), and
// the circular brackets ask for the same few hundred float-held directions
// thousands of times in one fillet or chamfer, so it is memoised
// process-wide on those exact integers (atan2Memo). A memo hit therefore
// returns the very endpoints a fresh evaluation would. Every call returns
// freshly allocated endpoints the caller may mutate; the memo never hands
// out the values it holds. The axis cases are cheap and stay unmemoised.
func Atan2Interval(y, x *big.Rat, negativeZeroY bool) RatInterval {
	if x.Sign() == 0 || y.Sign() == 0 {
		return atan2IntervalAxis(y, x, negativeZeroY)
	}
	key := atan2Key(y, x)
	if iv, ok := atan2Memo.get(key); ok {
		return iv
	}
	iv := atan2IntervalSeries(y, x)
	atan2Memo.put(key, iv)
	return iv
}

// atan2IntervalAxis is Atan2Interval for y or x zero.
func atan2IntervalAxis(y, x *big.Rat, negativeZeroY bool) RatInterval {
	if x.Sign() == 0 {
		halfPi := HalfPiInterval()
		if y.Sign() < 0 {
			return IntervalNeg(halfPi)
		}
		return halfPi
	}
	if x.Sign() < 0 {
		if negativeZeroY {
			return IntervalNeg(Interval(PiLower, PiUpper))
		}
		return Interval(PiLower, PiUpper)
	}
	return PointInterval(new(big.Rat))
}

// atan2IntervalSeries is Atan2Interval for y and x both nonzero, without the
// memo: the octant's AtanPositiveInterval reading mapped to the quadrant.
func atan2IntervalSeries(y, x *big.Rat) RatInterval {
	ax, ay := new(big.Rat).Abs(x), new(big.Rat).Abs(y)
	var base RatInterval
	if proofarith.CmpRat(ay, ax) <= 0 {
		base = AtanPositiveInterval(proofarith.QuoRat(new(big.Rat), ay, ax))
	} else {
		base = IntervalSub(HalfPiInterval(), AtanPositiveInterval(proofarith.QuoRat(new(big.Rat), ax, ay)))
	}
	switch {
	case x.Sign() > 0 && y.Sign() > 0:
		return base
	case x.Sign() > 0 && y.Sign() < 0:
		return IntervalNeg(base)
	case x.Sign() < 0 && y.Sign() > 0:
		return IntervalSub(Interval(PiLower, PiUpper), base)
	default:
		return IntervalAdd(IntervalNeg(Interval(PiLower, PiUpper)), base)
	}
}

// atan2MemoCap bounds the memo, as radSinCosMemoCap bounds the sine/cosine
// one; a full memo is cleared before the next insert.
const atan2MemoCap = 16384

type atan2MemoMap struct {
	mu      sync.Mutex
	entries map[string]RatInterval
}

var atan2Memo = &atan2MemoMap{entries: map[string]RatInterval{}}

// get returns a fresh copy of the enclosure memoised for key, or ok false
// when none is held.
func (m *atan2MemoMap) get(key string) (RatInterval, bool) {
	m.mu.Lock()
	iv, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return RatInterval{}, false
	}
	return Interval(iv.Lo, iv.Hi), true
}

// put records a copy of iv, so the caller keeps sole ownership of iv itself.
func (m *atan2MemoMap) put(key string, iv RatInterval) {
	held := Interval(iv.Lo, iv.Hi)
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.entries) >= atan2MemoCap {
		clear(m.entries)
	}
	m.entries[key] = held
}

// atan2Key encodes y's and then x's exact representation: per rational its
// sign, then its numerator's magnitude and its denominator, each
// length-prefixed, so two keys are equal exactly when both numerators and
// both denominators are.
func atan2Key(y, x *big.Rat) string {
	bits := y.Num().BitLen() + y.Denom().BitLen() + x.Num().BitLen() + x.Denom().BitLen()
	buf := make([]byte, 0, 2*(1+2*binary.MaxVarintLen64)+bits/8+4)
	buf = appendRatKey(buf, y)
	buf = appendRatKey(buf, x)
	return string(buf)
}

func appendRatKey(buf []byte, q *big.Rat) []byte {
	buf = append(buf, byte(q.Sign()+1))
	buf = appendNatKey(buf, q.Num())
	return appendNatKey(buf, q.Denom())
}

// appendNatKey appends |v| as a uvarint byte length followed by its
// big-endian bytes, written in place without an intermediate slice.
func appendNatKey(buf []byte, v *big.Int) []byte {
	n := (v.BitLen() + 7) / 8
	buf = binary.AppendUvarint(buf, uint64(n))
	start := len(buf)
	buf = append(buf, make([]byte, n)...)
	v.FillBytes(buf[start:])
	return buf
}
