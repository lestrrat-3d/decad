package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacySqrtUp is the DySqrtUp walk kept as the differential reference: two
// exact tests per step, f² ≤ d and f² = d.
func legacySqrtUp(d proof.Dyadic) float64 {
	if d.Sign() <= 0 {
		return 0
	}
	f := proof.DySqrtSeed(d)
	if math.IsNaN(f) || math.IsInf(f, 0) {
		f = math.MaxFloat64
	}
	for range proof.SqrtAdjustLimit {
		if !proof.DySquareAtMost(f, d) || proof.DySquareEquals(f, d) {
			return f
		}
		f = proof.NextUp(f)
	}
	return math.Inf(1)
}

// randomNormVector draws a vector whose components have the given mantissa
// widths, random signs and exponents in [-span, span].
func randomNormVector(t *testing.T, rng *rand.Rand, widths [3]int, span int) proof.DyV3 {
	t.Helper()
	var v proof.DyV3
	for axis, width := range widths {
		if width == 0 {
			continue
		}
		mantissa := new(big.Int)
		buf := make([]byte, (width+7)/8)
		for i := range buf {
			buf[i] = byte(rng.UintN(256))
		}
		mantissa.SetBytes(buf)
		mantissa.Rsh(mantissa, uint(len(buf)*8-width))
		mantissa.SetBit(mantissa, width-1, 1)
		mantissa.SetBit(mantissa, 0, 1)
		if rng.IntN(2) == 0 {
			mantissa.Neg(mantissa)
		}
		v[axis] = dyadicFromParts(t, mantissa, rng.IntN(2*span+1)-span)
	}
	return v
}

func requireNormUp(t *testing.T, v proof.DyV3, shift int, label string) {
	t.Helper()
	want := legacySqrtUp(proof.DyShift(proof.DvDot(v, v), shift))
	got := proof.DvNormUp(v, shift)
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%s: DvNormUp(shift %d)", label, shift)
	require.Equal(t, math.Float64bits(want),
		math.Float64bits(proof.DySqrtUp(proof.DyShift(proof.DvDot(v, v), shift))), "%s: DySqrtUp", label)
}

func TestDvNormUpMatchesPreviousRoot(t *testing.T) {
	t.Parallel()
	shifts := []int{-2, 0, -1, 1, 7, -9}
	t.Run("widths", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(401, 409))
		widths := []int{0, 1, 2, 31, 53, 54, 63, 64, 65, 100, 108, 112, 126, 127, 128, 200}
		for sample := range 3000 {
			var w [3]int
			for axis := range w {
				w[axis] = widths[rng.IntN(len(widths))]
			}
			// Narrow exponent spans keep the squares inside the word sum;
			// wide ones leave it for the general path.
			span := []int{0, 3, 40, 140, 1100}[sample%5]
			v := randomNormVector(t, rng, w, span)
			requireNormUp(t, v, shifts[sample%len(shifts)], "widths")
		}
	})
	t.Run("range-edges", func(t *testing.T) {
		t.Parallel()
		// Components whose magnitude 2^(width+exp) puts the root in the
		// subnormal range, or at the top of the float range.
		rng := rand.New(rand.NewPCG(457, 461))
		for sample := range 4000 {
			var w [3]int
			for axis := range w {
				w[axis] = 40 + rng.IntN(170)
			}
			v := randomNormVector(t, rng, w, 0)
			top := []int{-1080 + rng.IntN(70), 1010 + rng.IntN(20)}[sample%2]
			for axis := range v {
				v[axis] = proof.DyShift(v[axis], top-w[axis]-rng.IntN(3))
			}
			requireNormUp(t, v, shifts[sample%len(shifts)], "range-edges")
		}
	})
	t.Run("float-cross", func(t *testing.T) {
		t.Parallel()
		edge := []float64{
			0, math.Copysign(0, -1), 1, -1, 0.5, 3, -7.25, 1 + 0x1p-52, 31.7, -12.3, 9.000000000000002,
			math.SmallestNonzeroFloat64, 0x1p-1022, math.Float64frombits(0x000fffffffffffff),
			0x1p-600, 0x1p-300, 0x1p300, 0x1p500, 1e15, -1e15, 1e-15, math.MaxFloat64, -math.MaxFloat64,
		}
		rng := rand.New(rand.NewPCG(419, 421))
		pick := func() float64 {
			if rng.IntN(3) == 0 {
				return (rng.Float64()*2 - 1) * math.Pow(10, float64(rng.IntN(13)-6))
			}
			return edge[rng.IntN(len(edge))]
		}
		for sample := range 6000 {
			a := r3.Vec{X: pick(), Y: pick(), Z: pick()}
			b := r3.Vec{X: pick(), Y: pick(), Z: pick()}
			c := r3.Vec{X: pick(), Y: pick(), Z: pick()}
			u := proof.DvSub(proof.DyVec(b), proof.DyVec(a))
			v := proof.DvSub(proof.DyVec(c), proof.DyVec(a))
			requireNormUp(t, proof.DvCross(u, v), -2, "cross")
			requireNormUp(t, u, shifts[sample%len(shifts)], "edge")
		}
	})
	t.Run("degenerate", func(t *testing.T) {
		t.Parallel()
		wide := new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 127), big.NewInt(1))
		one := big.NewInt(1)
		cases := []proof.DyV3{
			{},
			{proof.DyZero(), dyadicFromParts(t, one, 0), proof.DyZero()},
			{dyadicFromParts(t, one, 1023), dyadicFromParts(t, one, 1023), proof.DyZero()},
			{dyadicFromParts(t, wide, 900), dyadicFromParts(t, wide, 900), dyadicFromParts(t, wide, 900)},
			{dyadicFromParts(t, wide, -1200), dyadicFromParts(t, wide, -1200), proof.DyZero()},
			{dyadicFromParts(t, one, -1074), proof.DyZero(), proof.DyZero()},
			// Roots in the subnormal range and below it: the seed underflows
			// to zero and the walk steps up from there.
			{dyadicFromParts(t, wide, -1250), dyadicFromParts(t, wide, -1250), proof.DyZero()},
			{dyadicFromParts(t, wide, -1300), proof.DyZero(), dyadicFromParts(t, wide, -1301)},
			{dyadicFromParts(t, wide, -1190), dyadicFromParts(t, one, -1190), proof.DyZero()},
		}
		for delta := range 260 {
			cases = append(cases, proof.DyV3{
				dyadicFromParts(t, wide, -160),
				dyadicFromParts(t, new(big.Int).Neg(wide), delta-160),
				dyadicFromParts(t, one, delta-200),
			})
		}
		for _, v := range cases {
			for _, shift := range shifts {
				requireNormUp(t, v, shift, "degenerate")
			}
		}
	})
}

func TestDySqrtUpMatchesPreviousWalk(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(431, 433))
	widths := []int{1, 2, 52, 53, 64, 100, 127, 128, 200, 300}
	for sample := range 3000 {
		v := randomNormVector(t, rng, [3]int{widths[sample%len(widths)], 0, 0}, 1100)
		d := proof.DyAbs(v[0])
		require.Equal(t, math.Float64bits(legacySqrtUp(d)), math.Float64bits(proof.DySqrtUp(d)), "sample %d", sample)
		sq := proof.DyMul(d, d)
		require.Equal(t, math.Float64bits(legacySqrtUp(sq)), math.Float64bits(proof.DySqrtUp(sq)), "square %d", sample)
	}
}

func requireSameDyadic(t *testing.T, want, got proof.Dyadic, label string) {
	t.Helper()
	require.Zero(t, proof.DyCmp(want, got), "%s: value differs", label)
	require.Equal(t, want.Exp(), got.Exp(), "%s: exponent differs", label)
	require.Equal(t, want.Sign(), got.Sign(), "%s: sign differs", label)
	if want.Sign() != 0 {
		require.Zero(t, want.Mant().Cmp(got.Mant()), "%s: mantissa differs", label)
	}
}

func TestDvSubHeldMatchesDvSub(t *testing.T) {
	t.Parallel()
	edge := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, 3, -7.25, 1 + 0x1p-52, 1 - 0x1p-53, 2, 31.7, -12.3,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 0x1p-1022, -0x1p-1022,
		math.Float64frombits(0x000fffffffffffff), 0x1p-600, 0x1p600, 1e15, 1e16 + 2, -1e-15,
		math.MaxFloat64, -math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0),
	}
	rng := rand.New(rand.NewPCG(439, 443))
	pick := func() float64 {
		switch rng.IntN(3) {
		case 0:
			return math.Ldexp(rng.Float64()*2-1, rng.IntN(2124)-1100)
		case 1:
			return edge[rng.IntN(len(edge))]
		default:
			return 9 + (rng.Float64()*2-1)*math.Pow(10, float64(-rng.IntN(17)))
		}
	}
	for range 20000 {
		a := r3.Vec{X: pick(), Y: pick(), Z: pick()}
		b := r3.Vec{X: pick(), Y: pick(), Z: pick()}
		if rng.IntN(4) == 0 {
			b.Y = a.Y
		}
		want := proof.DvSub(proof.DyVec(a), proof.DyVec(b))
		got := proof.DvSubHeld(a, b, proof.DyVec(a), proof.DyVec(b))
		for axis := range want {
			requireSameDyadic(t, want[axis], got[axis], "component")
		}
	}
}
