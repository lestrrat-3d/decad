package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"sync"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// atan2IntervalReference is Atan2Interval evaluated with no memo: the octant's
// AtanPositiveInterval reading mapped to the quadrant, and the axis cases.
func atan2IntervalReference(y, x *big.Rat, negativeZeroY bool) proofbound.RatInterval {
	pi := func() proofbound.RatInterval { return proofbound.Interval(proofbound.PiLower, proofbound.PiUpper) }
	if x.Sign() == 0 {
		halfPi := proofbound.HalfPiInterval()
		if y.Sign() < 0 {
			return proofbound.IntervalNeg(halfPi)
		}
		return halfPi
	}
	if y.Sign() == 0 {
		if x.Sign() < 0 {
			if negativeZeroY {
				return proofbound.IntervalNeg(pi())
			}
			return pi()
		}
		return proofbound.PointInterval(new(big.Rat))
	}
	ax, ay := new(big.Rat).Abs(x), new(big.Rat).Abs(y)
	var base proofbound.RatInterval
	if proofarith.CmpRat(ay, ax) <= 0 {
		base = proofbound.AtanPositiveInterval(proofarith.QuoRat(new(big.Rat), ay, ax))
	} else {
		base = proofbound.IntervalSub(proofbound.HalfPiInterval(),
			proofbound.AtanPositiveInterval(proofarith.QuoRat(new(big.Rat), ax, ay)))
	}
	switch {
	case x.Sign() > 0 && y.Sign() > 0:
		return base
	case x.Sign() > 0 && y.Sign() < 0:
		return proofbound.IntervalNeg(base)
	case x.Sign() < 0 && y.Sign() > 0:
		return proofbound.IntervalSub(pi(), base)
	default:
		return proofbound.IntervalAdd(proofbound.IntervalNeg(pi()), base)
	}
}

type atan2Case struct {
	y, x *big.Rat
	neg  bool
}

func floatCase(y, x float64) atan2Case {
	return atan2Case{y: new(big.Rat).SetFloat64(y), x: new(big.Rat).SetFloat64(x)}
}

// atan2DifferentialCases spans every Atan2Interval branch: random floats over
// the whole exponent range, random wide rationals, both axes with each sign
// and negative-zero flag, tiny, huge and subnormal magnitudes, and directions
// one ulp either side of the octant (|y| = |x|) and AtanPositiveInterval's
// 1/2 split.
func atan2DifferentialCases() []atan2Case {
	rng := rand.New(rand.NewPCG(2026, 1028))
	var out []atan2Case
	signed := func(v float64) float64 {
		if rng.IntN(2) == 0 {
			return -v
		}
		return v
	}
	for range 400 {
		y := signed(math.Ldexp(rng.Float64()+0.5, rng.IntN(2*1000)-1000))
		x := signed(math.Ldexp(rng.Float64()+0.5, rng.IntN(2*1000)-1000))
		out = append(out, floatCase(y, x))
	}
	for range 400 {
		a := rng.Float64() * 2 * math.Pi
		r := 0.5 + 4*rng.Float64()
		out = append(out, floatCase(r*math.Sin(a), r*math.Cos(a)))
	}
	randInt := func(bits int) *big.Int {
		v := new(big.Int)
		for range bits/63 + 1 {
			v.Lsh(v, 63)
			v.Or(v, big.NewInt(rng.Int64()))
		}
		v.Add(v, big.NewInt(1))
		if rng.IntN(2) == 0 {
			v.Neg(v)
		}
		return v
	}
	for range 200 {
		y := new(big.Rat).SetFrac(randInt(rng.IntN(300)), new(big.Int).Abs(randInt(rng.IntN(300))))
		x := new(big.Rat).SetFrac(randInt(rng.IntN(300)), new(big.Int).Abs(randInt(rng.IntN(300))))
		out = append(out, atan2Case{y: y, x: x})
	}
	specials := []float64{
		0, 1, -1, 3, -3, 1e-300, -1e-300, 1e300, -1e300,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		0x1p-1022, -0x1p-1022, 0x1.fffffffffffffp-1023, math.MaxFloat64, -math.MaxFloat64,
	}
	for _, y := range specials {
		for _, x := range specials {
			c := floatCase(y, x)
			out = append(out, c, atan2Case{y: c.y, x: c.x, neg: true})
		}
	}
	for _, v := range []float64{1, 3, 1e-310, 1e300, 0.7} {
		for _, sy := range []float64{1, -1} {
			for _, sx := range []float64{1, -1} {
				for _, f := range []float64{
					1, math.Nextafter(1, 2), math.Nextafter(1, 0),
					0.5, math.Nextafter(0.5, 1), math.Nextafter(0.5, 0),
					2, math.Nextafter(2, 3), math.Nextafter(2, 1),
				} {
					out = append(out, floatCase(sy*v*f, sx*v), floatCase(sy*v, sx*v*f))
				}
			}
		}
	}
	return out
}

// TestAtan2IntervalMatchesUnmemoisedReference checks every Atan2Interval call
// — the first, a repeat after the caller mutated the first result, and a
// repeat with the other negative-zero flag — returns the reference's exact
// numerators and denominators.
func TestAtan2IntervalMatchesUnmemoisedReference(t *testing.T) {
	t.Parallel()
	for i, c := range atan2DifferentialCases() {
		want := atan2IntervalReference(c.y, c.x, c.neg)
		first := proofbound.Atan2Interval(c.y, c.x, c.neg)
		requireSameInterval(t, want, first, "first call")
		first.Lo.SetInt64(int64(i) + 7)
		first.Hi.SetInt64(-int64(i) - 7)
		requireSameInterval(t, want, proofbound.Atan2Interval(c.y, c.x, c.neg), "repeat call")
		requireSameInterval(t, atan2IntervalReference(c.y, c.x, !c.neg),
			proofbound.Atan2Interval(c.y, c.x, !c.neg), "flipped flag")
	}
}

// TestAtan2IntervalMemoConcurrentReaders checks concurrent callers sharing
// directions each get the reference enclosure and private endpoints.
func TestAtan2IntervalMemoConcurrentReaders(t *testing.T) {
	t.Parallel()
	cases := atan2DifferentialCases()[400:440]
	want := make([]proofbound.RatInterval, len(cases))
	for i, c := range cases {
		want[i] = atan2IntervalReference(c.y, c.x, c.neg)
	}
	var wg sync.WaitGroup
	got := make([][]proofbound.RatInterval, 4)
	for g := range got {
		wg.Go(func() {
			for _, c := range cases {
				iv := proofbound.Atan2Interval(c.y, c.x, c.neg)
				got[g] = append(got[g], iv)
				iv.Lo.Add(iv.Lo, big.NewRat(1, 1))
				iv.Lo.Sub(iv.Lo, big.NewRat(1, 1))
			}
		})
	}
	wg.Wait()
	for g := range got {
		for i := range cases {
			requireSameInterval(t, want[i], got[g][i], "concurrent reader")
		}
	}
}
