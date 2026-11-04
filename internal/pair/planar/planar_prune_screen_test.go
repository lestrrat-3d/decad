package planar

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// prunedExact is pruned without the tight-box screen: the float test, then the
// exact comparisons, the reference the screen must agree with.
func prunedExact(k *planarKernel, alo, ahi, blo, bhi [3]proof.Dyadic, fa, fb floatBox) bool {
	up, bounded := k.boundUp()
	if !bounded {
		return false
	}
	if gapSquaredBelowLimit(fa, fb, up) > up {
		return true
	}
	if !boxesApart(alo, ahi, blo, bhi) {
		return false
	}
	gap := frac{num: boxGapSquared(alo, ahi, blo, bhi), den: proof.DyInt(1)}
	beyond := func(bound frac) bool {
		if bound.num.Sign() == 0 && bound.den.Sign() > 0 {
			return true
		}
		return fracCmp(gap, bound) > 0
	}
	return (k.hasBest && beyond(k.best)) || (k.hasCap && beyond(k.cap))
}

// tightBox draws an exact box with float corners and its tight float copy.
func tightBox(rng *rand.Rand, scale float64) ([3]proof.Dyadic, [3]proof.Dyadic, floatBox) {
	var lo, hi [3]proof.Dyadic
	var box floatBox
	box.tight = true
	for axis := range 3 {
		a, b := (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale
		if rng.IntN(4) == 0 {
			b = a
		}
		a, b = math.Min(a, b), math.Max(a, b)
		lo[axis], hi[axis] = proof.MustDyOf(a), proof.MustDyOf(b)
		box.lo[axis], box.hi[axis] = a, b
	}
	return lo, hi, box
}

func setBest(k *planarKernel, f frac) {
	k.best, k.bestUp, k.bestLo, k.hasBest = f, fracAbove(f), fracBelow(f), true
}

func setCap(k *planarKernel, f frac) {
	k.cap, k.capUp, k.capLo, k.hasCap = f, fracAbove(f), fracBelow(f), true
}

// TestPrunedTightScreenKeepsAnswers holds pruned to the exact-only reference on
// random tight boxes against random running minima and caps, bounds equal to
// the boxes' own gap included, and counts the pairs the screen settled.
//
// Leg shown to fail: gapSquaredAbove rounding down, or clearlyWithin ignoring
// the cap, answers false for a pair the exact comparison prunes.
func TestPrunedTightScreenKeepsAnswers(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(401, 409))
	screened, pruned := 0, 0
	for range 20000 {
		scale := math.Pow(10, float64(rng.IntN(6)-2))
		alo, ahi, fa := tightBox(rng, scale)
		blo, bhi, fb := tightBox(rng, scale)
		gap := boxGapSquared(alo, ahi, blo, bhi)
		var k planarKernel
		pick := func() frac {
			switch rng.IntN(5) {
			case 0:
				return frac{num: gap, den: proof.DyInt(1)}
			case 1:
				return frac{num: proof.DyMul(gap, proof.DyInt(3)), den: proof.DyInt(2)}
			case 2:
				return frac{num: proof.MustDyOf(rng.Float64() * scale * scale), den: proof.DyInt(1)}
			case 3:
				// A bound one part in 2^70 below the gap: beyond it, but too
				// close for the float test to say so.
				return frac{num: proof.DyMul(gap, proof.DyAdd(proof.DyShift(proof.DyInt(1), 70), proof.DyInt(-1))),
					den: proof.DyShift(proof.DyInt(1), 70)}
			default:
				return frac{num: proof.DyInt(int64(rng.IntN(3))), den: proof.MustDyOf(1 + rng.Float64())}
			}
		}
		if rng.IntN(3) > 0 {
			setBest(&k, pick())
		}
		if rng.IntN(3) > 0 {
			setCap(&k, pick())
		}
		want := prunedExact(&k, alo, ahi, blo, bhi, fa, fb)
		got := k.pruned(alo, ahi, blo, bhi, fa, fb)
		require.Equal(t, want, got)
		if want {
			pruned++
		}
		if _, bounded := k.boundUp(); bounded && !want && fa.tight && fb.tight && k.clearlyWithin(fa, fb) {
			screened++
		}
	}
	require.Positive(t, screened, "premise: the screen settles some pairs")
	require.Positive(t, pruned, "premise: some pairs are pruned")
}

func TestGapSquaredAboveAndFracBelowBoundExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 7))
	for range 5000 {
		alo, ahi, fa := tightBox(rng, 10)
		blo, bhi, fb := tightBox(rng, 10)
		exact := boxGapSquared(alo, ahi, blo, bhi)
		_, exactHi := proof.FloatBounds(exact)
		require.GreaterOrEqual(t, gapSquaredAbove(fa, fb), exactHi)
		f := frac{num: proof.MustDyOf(rng.Float64() * 5), den: proof.MustDyOf(0.5 + rng.Float64()*3)}
		below := fracBelow(f)
		require.LessOrEqual(t, below, fracAbove(f))
		require.LessOrEqual(t, fracCmp(frac{num: proof.MustDyOf(below), den: proof.DyInt(1)}, f), 0)
	}
}
