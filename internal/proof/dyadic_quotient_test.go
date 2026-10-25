package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func ratFloatUpRef(q *big.Rat) float64 {
	f, _ := q.Float64()
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return f
	}
	if fr := new(big.Rat).SetFloat64(f); fr != nil && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

func randomDyadic(r *rand.Rand) proof.Dyadic {
	// Products of a few floats with wide exponents, so mantissas reach the
	// big.Int path and exponents cover the normal range.
	d := proof.MustDyOf(math.Ldexp(r.Float64()+0.5, r.IntN(200)-100))
	for range r.IntN(4) {
		d = proof.DyMul(d, proof.MustDyOf(math.Ldexp(r.Float64()+0.5, r.IntN(40)-20)))
	}
	if r.IntN(8) == 0 {
		d = proof.DyAdd(d, proof.MustDyOf(math.Ldexp(r.Float64(), r.IntN(60)-30)))
	}
	return d
}

func TestDyQuoFloatUpMatchesRatFloatUp(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(7, 11))
	for i := range 6000 {
		n, d := randomDyadic(r), randomDyadic(r)
		if i%5 == 0 {
			// Exactly representable quotients: n = f·d.
			n = proof.DyMul(proof.MustDyOf(math.Ldexp(r.Float64()+0.5, r.IntN(60)-30)), d)
		}
		q := new(big.Rat).Quo(n.Rat(), d.Rat())
		want := ratFloatUpRef(q)
		got := proof.DyQuoFloatUp(n, d)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "n/d=%s", q.FloatString(30))
	}
}

func TestDyQuoFloatErrorMatchesRationalFloatError(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(3, 5))
	for i := range 6000 {
		n, d := randomDyadic(r), randomDyadic(r)
		q := new(big.Rat).Quo(n.Rat(), d.Rat())
		held, _ := q.Float64()
		switch i % 4 {
		case 1:
			held = math.Nextafter(held, math.Inf(1))
		case 2:
			held = held * (1 + 1e-9)
		case 3:
			held = held * 0.5
		}
		want := proof.RationalFloatError(q, held)
		got := proof.DyQuoFloatError(n, d, held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "n/d=%s held=%g", q.FloatString(30), held)
	}
}

// TestDyQuoFloatBinades sweeps quotients through every normal binade, so both the
// power-of-two table and the Ldexp fallback of the scaling run.
func TestDyQuoFloatBinades(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(13, 17))
	for i := range 8000 {
		e := i%2040 - 1020
		n := proof.MustDyOf(math.Ldexp(r.Float64()+0.5, e))
		d := proof.MustDyOf(r.Float64() + 0.5)
		q := new(big.Rat).Quo(n.Rat(), d.Rat())
		want := ratFloatUpRef(q)
		got := proof.DyQuoFloatUp(n, d)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "n/d=%s", q.FloatString(30))

		held, _ := q.Float64()
		if i%3 == 0 {
			held = math.Nextafter(held, math.Inf(1))
		}
		wantErr := proof.RationalFloatError(q, held)
		gotErr := proof.DyQuoFloatError(n, d, held)
		require.Equal(t, math.Float64bits(wantErr), math.Float64bits(gotErr), "n/d=%s held=%x", q.FloatString(30), held)
	}
}

func quotientMantissaMask(width int) *big.Int {
	return new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), uint(width)), big.NewInt(1))
}

func checkDyQuoFloatReference(t *testing.T, n, d proof.Dyadic, sample int) {
	t.Helper()
	q := new(big.Rat).Quo(n.Rat(), d.Rat())
	require.Equal(t, math.Float64bits(ratFloatUpRef(q)), math.Float64bits(proof.DyQuoFloatUp(n, d)),
		"sample %d: upward rounding differs", sample)
	nearest, _ := q.Float64()
	if math.IsInf(nearest, 0) {
		nearest = 0
	}
	for _, held := range [2]float64{0, nearest} {
		difference := new(big.Rat).Sub(q, new(big.Rat).SetFloat64(held))
		difference.Abs(difference)
		want, _ := difference.Float64()
		if !math.IsInf(want, 0) && new(big.Rat).SetFloat64(want).Cmp(difference) != 0 {
			want = math.Nextafter(want, math.Inf(1))
		}
		require.Equal(t, math.Float64bits(want), math.Float64bits(proof.DyQuoFloatError(n, d, held)),
			"sample %d: error rounding differs", sample)
	}
}

func TestDyQuoFloatLimbBoundaries(t *testing.T) {
	t.Parallel()
	nWidths := []int{1, 53, 64, 65, 106, 118, 119, 127, 128, 181, 255, 511, 512, 513, 600, 1024, 2048}
	dWidths := []int{1, 53, 64, 65, 106, 127, 128, 255, 511, 512, 513, 600}
	sample := 0
	for _, nw := range nWidths {
		for _, dw := range dWidths {
			for _, offset := range []int{-1100, 0, 1100} {
				n := dyadicFromParts(t, quotientMantissaMask(nw), offset-nw)
				d := dyadicFromParts(t, quotientMantissaMask(dw), -dw)
				checkDyQuoFloatReference(t, n, d, sample)
				sample++
			}
		}
	}
}

func TestDyQuoFloatEstimateCorrection(t *testing.T) {
	t.Parallel()
	top := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 63), big.NewInt(1))
	den := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 126), new(big.Int).Lsh(big.NewInt(1), 63))
	den.Add(den, big.NewInt(3))
	for adjustment := range 2 {
		estimate := quotientMantissaMask(54)
		estimate.Sub(estimate, big.NewInt(int64(adjustment)))
		num := new(big.Int).Mul(estimate, top)
		checkDyQuoFloatReference(t, dyadicFromParts(t, num, 0), dyadicFromParts(t, den, 0), adjustment)
	}
	// The low divisor words force an overestimate even when adding its top
	// word to the estimated remainder would carry past a machine word.
	num := new(big.Int).Mul(quotientMantissaMask(54), quotientMantissaMask(64))
	num.Add(num, big.NewInt(10))
	checkDyQuoFloatReference(t, dyadicFromParts(t, num, 0), dyadicFromParts(t, quotientMantissaMask(127), 0), 2)
}

func TestDyQuoFloatReusesNativeLimbs(t *testing.T) {
	for sample, widths := range [][2]int{{5, 2}, {118, 64}, {119, 64}, {256, 128}, {2048, 512}} {
		n := dyadicFromParts(t, quotientMantissaMask(widths[0]), -widths[0])
		d := dyadicFromParts(t, quotientMantissaMask(widths[1]), -widths[1])
		beforeN, beforeD := n.Rat(), d.Rat()
		var result float64
		allocations := testing.AllocsPerRun(100, func() { result = proof.DyQuoFloatUp(n, d) })
		require.Zero(t, allocations, "sample %d", sample)
		want := ratFloatUpRef(new(big.Rat).Quo(beforeN, beforeD))
		require.Equal(t, math.Float64bits(want), math.Float64bits(result), "sample %d", sample)
		require.Zero(t, beforeN.Cmp(n.Rat()), "sample %d: numerator changed", sample)
		require.Zero(t, beforeD.Cmp(d.Rat()), "sample %d: denominator changed", sample)
	}
}
