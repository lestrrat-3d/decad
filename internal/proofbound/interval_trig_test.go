package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"strconv"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// requireSameInterval asserts two enclosures are the same exact rationals,
// equal by Cmp and by their canonical RatString.
func requireSameInterval(t *testing.T, want, got proofbound.RatInterval, msg string) {
	t.Helper()
	require.Zero(t, want.Lo.Cmp(got.Lo), msg)
	require.Zero(t, want.Hi.Cmp(got.Hi), msg)
	require.Equal(t, want.Lo.RatString(), got.Lo.RatString(), msg)
	require.Equal(t, want.Hi.RatString(), got.Hi.RatString(), msg)
}

// TestRadSinCosIntervalMemoMatchesUncached pins the memo to the reading it
// replaces: every angle, on its first (filling) call and on later (hitting)
// calls, must return the uncached enclosure exactly, and a caller mutating the
// returned endpoints must not reach the memo.
func TestRadSinCosIntervalMemoMatchesUncached(t *testing.T) {
	angles := map[string]*big.Rat{
		"third":      big.NewRat(1, 3),
		"negative":   big.NewRat(-7, 5),
		"many-turns": big.NewRat(1000003, 7),
		"tiny":       big.NewRat(1, 1<<40),
		"pi/2-float": proofarith.FloatRat(math.Pi / 2),
		"2pi/24":     proofarith.FloatRat(2 * math.Pi / 24),
		"-pi/3":      proofarith.FloatRat(-math.Pi / 3),
	}
	rng := rand.New(rand.NewPCG(3, 4))
	for i := range 16 {
		angles["random/"+strconv.Itoa(i)] = big.NewRat(rng.Int64N(2_000_000)-1_000_000, rng.Int64N(99_999)+1)
	}

	for name, x := range angles {
		t.Run(name, func(t *testing.T) {
			wantSin, wantCos, ok := proofbound.RadSinCosIntervalUncached(x)
			require.True(t, ok)
			for call := range 3 {
				sin, cos, ok := proofbound.RadSinCosInterval(x)
				require.True(t, ok, "call %d", call)
				msg := "call " + strconv.Itoa(call)
				requireSameInterval(t, wantSin, sin, msg)
				requireSameInterval(t, wantCos, cos, msg)
				require.True(t, proofbound.RadSinCosMemoHolds(x), msg)
				// The caller owns the endpoints: mutating them must not reach
				// the memo.
				one := big.NewRat(1, 1)
				sin.Lo.Add(sin.Lo, one)
				sin.Hi.Add(sin.Hi, one)
				cos.Lo.Add(cos.Lo, one)
				cos.Hi.Add(cos.Hi, one)
			}
		})
	}

	// Angles differing only in sign, or sharing a numerator, must not share
	// a memo entry: fill with one, then read the other.
	t.Run("distinct-keys", func(t *testing.T) {
		pairs := [][2]*big.Rat{
			{big.NewRat(-11, 13), big.NewRat(11, 13)},
			{big.NewRat(17, 19), big.NewRat(17, 23)},
		}
		for _, pair := range pairs {
			_, _, ok := proofbound.RadSinCosInterval(pair[0])
			require.True(t, ok)
			wantSin, wantCos, ok := proofbound.RadSinCosIntervalUncached(pair[1])
			require.True(t, ok)
			sin, cos, ok := proofbound.RadSinCosInterval(pair[1])
			require.True(t, ok)
			requireSameInterval(t, wantSin, sin, pair[1].RatString())
			requireSameInterval(t, wantCos, cos, pair[1].RatString())
		}
	})

	t.Run("zero", func(t *testing.T) {
		sin, cos, ok := proofbound.RadSinCosInterval(new(big.Rat))
		require.True(t, ok)
		require.Zero(t, sin.Lo.Sign())
		require.Zero(t, sin.Hi.Sign())
		require.Zero(t, cos.Lo.Cmp(big.NewRat(1, 1)))
		require.Zero(t, cos.Hi.Cmp(big.NewRat(1, 1)))
	})
}

// seriesTan is tan(x) to about 75 significant digits: the Taylor series of
// sin and cos at 256 bits, summed until a term falls below 2⁻³⁰⁰. It is the
// reference RadTanSpan's enclosure is checked against, and shares no code with
// it.
func seriesTan(x *big.Rat) *big.Rat {
	const prec = 256
	fx := new(big.Float).SetPrec(prec).SetRat(x)
	x2 := new(big.Float).SetPrec(prec).Mul(fx, fx)
	sum := func(term *big.Float, first int64) *big.Float {
		total := new(big.Float).SetPrec(prec).Set(term)
		eps := new(big.Float).SetPrec(prec).SetMantExp(big.NewFloat(1), -300)
		for k := first; ; k += 2 {
			term = new(big.Float).SetPrec(prec).Mul(term, x2)
			term.Quo(term, new(big.Float).SetPrec(prec).SetInt64(-k*(k+1)))
			total.Add(total, term)
			if new(big.Float).Abs(term).Cmp(eps) < 0 {
				return total
			}
		}
	}
	sin := sum(fx, 2)
	cos := sum(new(big.Float).SetPrec(prec).SetInt64(1), 1)
	tan, _ := new(big.Float).SetPrec(prec).Quo(sin, cos).Rat(nil)
	return tan
}

// TestRadTanSpanEnclosesSeriesReference pins the draft's certified tangent
// (agent-docs/draft-design.md §8.1): over every span, the enclosure holds the series
// reference at both ends of the span, and over a one-point span it is a box
// far narrower than one float ulp of the tangent wherever the angle is a degree
// or more; the turn grid's fixed absolute width dominates a tinier angle's
// tangent. Shown to fail first: with
// RadTanSpan's body replaced by the point interval at math.Tan of the span's
// float, the reference fell outside the box at every angle.
func TestRadTanSpanEnclosesSeriesReference(t *testing.T) {
	deg := func(d float64) *big.Rat { return proofarith.FloatRat(d * math.Pi / 180) }
	for _, x := range []*big.Rat{deg(3), deg(5), deg(-5), deg(10), deg(45), deg(89), big.NewRat(1, 1<<50)} {
		t.Run(x.FloatString(20), func(t *testing.T) {
			width := new(big.Rat).SetFrac64(1, 1<<40)
			for _, span := range []proofbound.RatInterval{
				proofbound.PointInterval(x),
				proofbound.Interval(x, new(big.Rat).Add(x, width)),
			} {
				tan, ok := proofbound.RadTanSpan(span)
				require.True(t, ok)
				for _, end := range []*big.Rat{span.Lo, span.Hi} {
					ref := seriesTan(end)
					require.LessOrEqual(t, tan.Lo.Cmp(ref), 0, "the enclosure's low end passes tan at %s", end.FloatString(20))
					require.GreaterOrEqual(t, tan.Hi.Cmp(ref), 0, "the enclosure's high end falls short of tan at %s", end.FloatString(20))
				}
			}
			if new(big.Rat).Abs(x).Cmp(deg(1)) < 0 {
				return
			}
			tan, ok := proofbound.RadTanSpan(proofbound.PointInterval(x))
			require.True(t, ok)
			ref, _ := seriesTan(x).Float64()
			gap, _ := new(big.Rat).Sub(tan.Hi, tan.Lo).Float64()
			require.Less(t, gap, 1e-6*ulpOf(ref), "a one-point span encloses its tangent well inside one float ulp")
		})
	}
	// A cosine enclosure reaching zero has no quotient to box.
	_, ok := proofbound.RadTanSpan(proofbound.Interval(deg(89), deg(91)))
	require.False(t, ok)
}

func ulpOf(v float64) float64 {
	v = math.Abs(v)
	return math.Nextafter(v, math.Inf(1)) - v
}

func requireTrigDyadic(t *testing.T, x *big.Rat) {
	t.Helper()
	d := x.Denom()
	require.Equal(t, d.BitLen()-1, int(d.TrailingZeroBits()), "denominator is not a power of two")
}

// TestTrigWideningRoundsOutward checks the exact rounding direction, relative
// error, significand size and input ownership, including values far outside
// float64's range and values on both sides of the significand cutoff.
func TestTrigWideningRoundsOutward(t *testing.T) {
	rel := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), proofbound.TrigFixedBits-2))
	mantissas := []*big.Int{big.NewInt(1), big.NewInt(3)}
	for _, p := range []uint{199, 200, 201} {
		x := new(big.Int).Lsh(big.NewInt(1), p)
		mantissas = append(mantissas,
			new(big.Int).Sub(x, big.NewInt(1)), new(big.Int).Add(x, big.NewInt(1)))
	}
	denominators := []*big.Int{big.NewInt(1), big.NewInt(3),
		new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 257), big.NewInt(1))}
	for _, e := range []int{-4096, -1075, -200, -1, 0, 199, 200, 201, 1024, 4096} {
		for _, m := range mantissas {
			for _, d := range denominators {
				n, den := new(big.Int).Set(m), new(big.Int).Set(d)
				if e >= 0 {
					n.Lsh(n, uint(e))
				} else {
					den.Lsh(den, uint(-e))
				}
				q := new(big.Rat).SetFrac(n, den)
				before := new(big.Rat).Set(q)
				got := proofbound.TrigWidening(q)
				require.GreaterOrEqual(t, got.Cmp(q), 0, "inward rounding at exponent %d", e)
				extra := new(big.Rat).Sub(got, q)
				require.LessOrEqual(t, extra.Cmp(new(big.Rat).Mul(q, rel)), 0, "relative error at exponent %d", e)
				requireTrigDyadic(t, got)
				require.LessOrEqual(t, got.Num().BitLen()-int(got.Num().TrailingZeroBits()), proofbound.TrigFixedBits)
				require.Zero(t, before.Cmp(q), "input changed")
				got.SetInt64(0)
				require.Zero(t, before.Cmp(q), "result shares input storage")
			}
		}
	}
	zero := new(big.Rat)
	got := proofbound.TrigWidening(zero)
	require.Zero(t, got.Sign())
	got.SetInt64(1)
	require.Zero(t, zero.Sign())
}

// radSinCosExactWidening is the rational construction before dyadic widening.
// It returns its exact widening as well as the two enclosures.
func radSinCosExactWidening(t *testing.T, x *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, *big.Rat) {
	t.Helper()
	if x.Sign() == 0 {
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1)), new(big.Rat)
	}
	pi := proofbound.TwoPiInterval()
	turn, ok := proofbound.IntervalQuo(proofbound.PointInterval(x), pi)
	require.True(t, ok)
	base := proofbound.RatFloorGrid(turn.Lo, proofbound.TurnGridShift)
	gap := new(big.Rat).Sub(turn.Hi, base)
	slop := new(big.Rat).Mul(pi.Hi, gap)
	sin, cos := proofbound.TurnSinCosInterval(base)
	return proofbound.IntervalWiden(sin, slop), proofbound.IntervalWiden(cos, slop), slop
}

// TestRadSinCosDyadicEnclosesExactWidening checks that outward rounding only
// expands the previous exact construction, by at most its stated relative
// rounding allowance. Both endpoints must stay dyadic for later arithmetic.
func TestRadSinCosDyadicEnclosesExactWidening(t *testing.T) {
	angles := []*big.Rat{new(big.Rat), big.NewRat(1, 3), big.NewRat(-7, 5), big.NewRat(1000003, 7),
		proofarith.FloatRat(math.Pi / 2), proofarith.FloatRat(2 * math.Pi)}
	for _, e := range []uint{200, 1075, 4096} {
		x := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), e))
		angles = append(angles, x, new(big.Rat).Neg(x))
	}
	rel := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), proofbound.TrigFixedBits-2))
	for _, x := range angles {
		before := new(big.Rat).Set(x)
		for _, width := range []*big.Rat{new(big.Rat), big.NewRat(1, 7),
			new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(3), 2048))} {
			oldSin, oldCos, slop := radSinCosExactWidening(t, x)
			oldSin, oldCos = proofbound.IntervalWiden(oldSin, width), proofbound.IntervalWiden(oldCos, width)
			span := proofbound.Interval(x, new(big.Rat).Add(x, width))
			sin, cos, ok := proofbound.RadSinCosSpan(span)
			require.True(t, ok)
			allow := new(big.Rat).Mul(new(big.Rat).Add(slop, width), rel)
			for _, pair := range [][2]proofbound.RatInterval{{oldSin, sin}, {oldCos, cos}} {
				old, got := pair[0], pair[1]
				for _, extra := range []*big.Rat{new(big.Rat).Sub(old.Lo, got.Lo), new(big.Rat).Sub(got.Hi, old.Hi)} {
					require.GreaterOrEqual(t, extra.Sign(), 0, "previous enclosure shrank")
					require.LessOrEqual(t, extra.Cmp(allow), 0, "exceeded rounding allowance")
				}
				requireTrigDyadic(t, got.Lo)
				requireTrigDyadic(t, got.Hi)
			}
		}
		require.Zero(t, before.Cmp(x), "input changed")
	}
}
