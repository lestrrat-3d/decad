package freeform_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

// referenceBernsteinSplit is the rational de Casteljau triangle, one
// normalised big.Rat operation per product and sum. BernsteinSplit must
// return its exact values and share pointers the way it does.
func referenceBernsteinSplit(values []*big.Rat, t *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(values)
	work := make([]*big.Rat, n)
	copy(work, values)
	left := make([]*big.Rat, 0, n)
	right := make([]*big.Rat, n)
	left = append(left, work[0])
	right[n-1] = work[n-1]
	oneMinusT := new(big.Rat).Sub(big.NewRat(1, 1), t)
	for round := n - 1; round > 0; round-- {
		for i := range round {
			blend := new(big.Rat).Mul(oneMinusT, work[i])
			blend.Add(blend, new(big.Rat).Mul(t, work[i+1]))
			work[i] = blend
		}
		left = append(left, work[0])
		right[round-1] = work[round-1]
	}
	return left, right
}

// referenceBernsteinRestrict is two reference splits: the right branch at a,
// then the left branch at (b−a)/(1−a), with a = 1 read as the endpoint.
func referenceBernsteinRestrict(values []*big.Rat, a, b *big.Rat) []*big.Rat {
	one := big.NewRat(1, 1)
	if a.Cmp(one) == 0 {
		out := make([]*big.Rat, len(values))
		last := values[len(values)-1]
		for i := range out {
			out[i] = new(big.Rat).Set(last)
		}
		return out
	}
	_, right := referenceBernsteinSplit(values, a)
	span := new(big.Rat).Sub(b, a)
	span.Quo(span, new(big.Rat).Sub(one, a))
	left, _ := referenceBernsteinSplit(right, span)
	return left
}

// requireSameRats checks every element is the same canonical rational:
// equal value, numerator and denominator.
func requireSameRats(t *testing.T, want, got []*big.Rat, label string) {
	t.Helper()
	require.Len(t, got, len(want), label)
	for i := range want {
		require.Zero(t, want[i].Cmp(got[i]), "%s[%d]: want %s got %s", label, i, want[i].RatString(), got[i].RatString())
		require.Zero(t, want[i].Num().Cmp(got[i].Num()), "%s[%d] numerator", label, i)
		require.Zero(t, want[i].Denom().Cmp(got[i].Denom()), "%s[%d] denominator", label, i)
	}
}

// requireSameSharing checks the outputs alias the inputs and each other at
// exactly the positions the reference's outputs do.
func requireSameSharing(t *testing.T, values []*big.Rat, want, got [][]*big.Rat) {
	t.Helper()
	flatten := func(rows [][]*big.Rat) []*big.Rat {
		out := append([]*big.Rat(nil), values...)
		for _, row := range rows {
			out = append(out, row...)
		}
		return out
	}
	w, g := flatten(want), flatten(got)
	require.Len(t, g, len(w))
	for i := range w {
		for j := i + 1; j < len(w); j++ {
			require.Equal(t, w[i] == w[j], g[i] == g[j], "sharing of positions %d and %d", i, j)
		}
	}
}

func splitCaseValue(rng *rand.Rand, kind int) *big.Rat {
	switch kind {
	case 0: // a float64 lift: dyadic, any sign, any binade
		f := math.Ldexp(rng.Float64()+0.5, rng.IntN(120)-60)
		if rng.IntN(2) == 0 {
			f = -f
		}
		return new(big.Rat).SetFloat64(f)
	case 1: // general rational with large terms
		num := randBits(rng, 1+rng.IntN(600))
		den := randBits(rng, 1+rng.IntN(600))
		den.Add(den, big.NewInt(1))
		if rng.IntN(2) == 0 {
			num.Neg(num)
		}
		return new(big.Rat).SetFrac(num, den)
	case 2: // edge values
		edges := []float64{0, math.Copysign(0, -1), 1, -1, math.MaxFloat64, -math.MaxFloat64,
			math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 0x1p-1022, 1e300, -1e-300}
		return new(big.Rat).SetFloat64(edges[rng.IntN(len(edges))])
	default: // small integer ratio, shares denominators often
		return big.NewRat(int64(rng.IntN(21)-10), int64(1+rng.IntN(6)))
	}
}

// randBits returns a uniform integer in [0, 2^bits).
func randBits(rng *rand.Rand, bits int) *big.Int {
	words := make([]big.Word, (bits+63)/64)
	for i := range words {
		words[i] = big.Word(rng.Uint64())
	}
	x := new(big.Int).SetBits(words)
	mask := new(big.Int).Lsh(big.NewInt(1), uint(bits))
	return x.Mod(x, mask)
}

func splitCaseRow(rng *rand.Rand, n int) []*big.Rat {
	mode := rng.IntN(5)
	row := make([]*big.Rat, n)
	for i := range row {
		kind := mode
		if mode == 4 {
			kind = rng.IntN(4)
		}
		row[i] = splitCaseValue(rng, kind)
	}
	return row
}

func splitCaseParameter(rng *rand.Rand) *big.Rat {
	switch rng.IntN(7) {
	case 0:
		return big.NewRat(1, 2)
	case 1:
		return big.NewRat(1, int64(2+rng.IntN(9)))
	case 2:
		return new(big.Rat).SetFloat64(rng.Float64())
	case 3:
		k := 1 + rng.IntN(60)
		return new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<k)+1), new(big.Int).Lsh(big.NewInt(1), uint(k)))
	case 4:
		den := randBits(rng, 200)
		den.Add(den, big.NewInt(2))
		num := new(big.Int).Mod(randBits(rng, 202), den)
		return new(big.Rat).SetFrac(num, den)
	case 5:
		return []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 1)}[rng.IntN(2)]
	default: // outside [0, 1]: the split extrapolates
		return big.NewRat(int64(rng.IntN(11)-5), int64(1+rng.IntN(3)))
	}
}

func splitCaseInterval(rng *rand.Rand) (*big.Rat, *big.Rat) {
	switch rng.IntN(6) {
	case 0:
		return big.NewRat(0, 1), big.NewRat(1, 1)
	case 1:
		return big.NewRat(1, 1), big.NewRat(1, 1)
	case 2:
		return big.NewRat(0, 1), splitCaseParameter(rng)
	case 3:
		return splitCaseParameter(rng), big.NewRat(1, 1)
	}
	a, b := splitCaseParameter(rng), splitCaseParameter(rng)
	if a.Cmp(b) > 0 {
		a, b = b, a
	}
	if a.Cmp(big.NewRat(1, 1)) == 0 {
		b = a
	}
	return a, b
}

func TestBernsteinSplitAndRestrictMatchRationalTriangle(t *testing.T) {
	t.Parallel()
	cases := 1200
	if raceDetector {
		cases = 170
	}
	rng := rand.New(rand.NewPCG(7, 11))
	for c := range cases {
		n := 1 + c%17
		values := splitCaseRow(rng, n)
		snapshot := make([]*big.Rat, n)
		for i, v := range values {
			snapshot[i] = new(big.Rat).Set(v)
		}

		at := splitCaseParameter(rng)
		wantL, wantR := referenceBernsteinSplit(values, at)
		gotL, gotR := freeform.BernsteinSplit(values, at)
		requireSameRats(t, wantL, gotL, "split left")
		requireSameRats(t, wantR, gotR, "split right")
		requireSameSharing(t, values, [][]*big.Rat{wantL, wantR}, [][]*big.Rat{gotL, gotR})

		a, b := splitCaseInterval(rng)
		want := referenceBernsteinRestrict(values, a, b)
		got := freeform.BernsteinRestrict(values, a, b)
		requireSameRats(t, want, got, "restrict")
		requireSameSharing(t, values, [][]*big.Rat{want}, [][]*big.Rat{got})

		requireSameRats(t, snapshot, values, "input after the calls")
	}
}

// BenchmarkBernsteinSplitRestrict runs the shapes a loft fillet and cap
// chamfer feed the splits: a degree-6 row of large general rationals halved,
// a cubic restricted to a rational sub-interval, and a degree-15 row
// restricted to a narrow bracket with ~200-bit endpoints.
func BenchmarkBernsteinSplitRestrict(b *testing.B) {
	rng := rand.New(rand.NewPCG(3, 5))
	row := func(n, bits int) []*big.Rat {
		out := make([]*big.Rat, n)
		den := randBits(rng, bits)
		den.Add(den, big.NewInt(1))
		for i := range out {
			num := randBits(rng, bits)
			out[i] = new(big.Rat).SetFrac(num, den)
		}
		return out
	}
	bracket := func(bits uint) (*big.Rat, *big.Rat) {
		den := new(big.Int).Lsh(big.NewInt(1), bits)
		den.Add(den, big.NewInt(3))
		num := new(big.Int).Rsh(den, 1)
		lo := new(big.Rat).SetFrac(num, den)
		hi := new(big.Rat).SetFrac(new(big.Int).Add(num, big.NewInt(1)), den)
		return lo, hi
	}
	half := big.NewRat(1, 2)
	split6 := row(7, 1000)
	cubic := row(4, 170)
	cubicLo, cubicHi := big.NewRat(3, 7), big.NewRat(5, 7)
	wide := row(16, 300)
	wideLo, wideHi := bracket(160)
	b.Run("split/degree6/half", func(b *testing.B) {
		for b.Loop() {
			freeform.BernsteinSplit(split6, half)
		}
	})
	b.Run("restrict/degree3", func(b *testing.B) {
		for b.Loop() {
			freeform.BernsteinRestrict(cubic, cubicLo, cubicHi)
		}
	})
	b.Run("restrict/degree15/narrow", func(b *testing.B) {
		for b.Loop() {
			freeform.BernsteinRestrict(wide, wideLo, wideHi)
		}
	})
}
