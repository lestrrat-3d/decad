package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// projectedAllowQuotientForm computes CellTwistAreaProjectedAllow through the
// exact rational quotient |A⊥|² + |B⊥|² = Σ(|V|² − (W·V)²/|W|²) and keeps it
// as the oracle for the single-quotient form the library uses, which forms
// the numerator in dyadics and divides once.
func projectedAllowQuotientForm(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !proofbound.FiniteVec(vLo) || !proofbound.FiniteVec(vHi) ||
		!proofbound.FiniteVec(wLo) || !proofbound.FiniteVec(wHi) {
		return math.Inf(1)
	}
	da := proofbound.HeldDelta(vHi, vLo)
	g := proofbound.HeldDelta(wLo, vLo)
	twist := proofarith.DvSub(proofbound.HeldDelta(vLo, vHi), proofbound.HeldDelta(wLo, wHi))
	a := proofarith.DvCross(da, twist)
	b := proofarith.DvCross(twist, g)
	n0 := proofarith.DvCross(da, g)
	var w proofarith.DyV3
	for i := range w {
		w[i] = proofarith.DyAdd(proofarith.DyAdd(proofarith.DyShift(n0[i], 1), a[i]), b[i])
	}
	ww := proofarith.DvDot(w, w).Rat()
	if ww.Sign() <= 0 {
		return math.Inf(1)
	}
	wa := proofarith.DvDot(w, a).Rat()
	wb := proofarith.DvDot(w, b).Rat()
	perp := func(v proofarith.DyV3, wv *big.Rat) *big.Rat {
		out := proofarith.DvDot(v, v).Rat()
		return proofarith.SubRat(out, out, proofarith.QuoRat(new(big.Rat), proofarith.MulRat(new(big.Rat), wv, wv), ww))
	}
	num := proofarith.AddRat(new(big.Rat), perp(a, wa), perp(b, wb))
	if num.Sign() <= 0 {
		return 0
	}
	c := proofarith.SubRat(new(big.Rat), ww, new(big.Rat).Abs(wa))
	proofarith.SubRat(c, c, new(big.Rat).Abs(wb))
	if c.Sign() <= 0 {
		return math.Inf(1)
	}
	wLen, ok := proofbound.RatOf(proofbound.RatSqrtUp(ww))
	if !ok {
		return math.Inf(1)
	}
	proofarith.MulRat(num, num, wLen)
	return proofbound.RatFloatUp(proofarith.QuoRat(num, num, proofarith.MulRat(c, c, big.NewRat(2, 1))))
}

// TestCellTwistAreaProjectedAllowMatchesQuotientForm checks that forming the
// numerator in dyadics and dividing once returns the same float64, bit for
// bit, as the rational quotient form: finite, withdrawn (+Inf) and zero
// results alike, on near-planar twisted cells, arbitrary and degenerate quads,
// untwisted parallelograms and coordinates at exponent extremes.
func TestCellTwistAreaProjectedAllowMatchesQuotientForm(t *testing.T) {
	rng := rand.New(rand.NewPCG(5, 3029))
	kinds := map[string]int{}
	check := func(vLo, vHi, wLo, wHi r3.Vec) {
		t.Helper()
		got := proofbound.CellTwistAreaProjectedAllow(vLo, vHi, wLo, wHi)
		want := projectedAllowQuotientForm(vLo, vHi, wLo, wHi)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%v %v %v %v", vLo, vHi, wLo, wHi)
		switch {
		case math.IsInf(got, 1):
			kinds["inf"]++
		case got == 0:
			kinds["zero"]++
		default:
			kinds["finite"]++
		}
	}
	jitter := func(scale float64) r3.Vec {
		return r3.Vec{X: (rng.Float64() - 0.5) * scale, Y: (rng.Float64() - 0.5) * scale, Z: (rng.Float64() - 0.5) * scale}
	}
	for range 4000 {
		// Near-planar twisted cells like a coil wall, at several scales.
		scale := math.Ldexp(1, rng.IntN(20)-10)
		base := jitter(100)
		u := jitter(scale)
		v := jitter(scale)
		tw := jitter(scale * math.Ldexp(1, -rng.IntN(30)))
		check(base, base.Add(u), base.Add(v), base.Add(u).Add(v).Add(tw))
		// Arbitrary quads, including degenerate ones.
		check(jitter(10), jitter(10), jitter(10), jitter(10))
		p := jitter(1)
		check(p, p, p, p)
		check(p, p.Add(u), p, p.Add(u))
		// An untwisted integer parallelogram has A = B = 0, so the arm is 0.
		iv := func() r3.Vec {
			return r3.Vec{X: float64(rng.IntN(9) - 4), Y: float64(rng.IntN(9) - 4), Z: float64(rng.IntN(9) - 4)}
		}
		o, du, dv := iv(), iv(), iv()
		check(o, o.Add(du), o.Add(dv), o.Add(du).Add(dv))
		// Exponent extremes: tiny and huge cells, and mixed magnitudes.
		huge := math.Ldexp(1, 200+rng.IntN(600))
		small := math.Ldexp(1, -200-rng.IntN(600))
		check(jitter(huge), jitter(huge), jitter(huge), jitter(huge))
		check(jitter(small), jitter(small), jitter(small), jitter(small))
		check(jitter(huge), jitter(small), jitter(huge), jitter(small))
	}
	// Non-finite input withdraws the arm.
	check(r3.Vec{X: math.NaN()}, r3.Vec{}, r3.Vec{}, r3.Vec{})
	check(r3.Vec{}, r3.Vec{X: math.Inf(1)}, r3.Vec{}, r3.Vec{})
	t.Logf("kinds %v", kinds)
	require.Positive(t, kinds["finite"])
	require.Positive(t, kinds["inf"])
	require.Positive(t, kinds["zero"])
}
