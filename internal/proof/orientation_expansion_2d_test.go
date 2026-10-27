package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// exactCross2Sign is the sign of (b - a) × (c - a) over the exact rationals
// of six floats.
func exactCross2Sign(au, av, bu, bv, cu, cv float64) int {
	r := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	bau := new(big.Rat).Sub(r(bu), r(au))
	bav := new(big.Rat).Sub(r(bv), r(av))
	cau := new(big.Rat).Sub(r(cu), r(au))
	cav := new(big.Rat).Sub(r(cv), r(av))
	left := new(big.Rat).Mul(bau, cav)
	return left.Cmp(new(big.Rat).Mul(bav, cau))
}

func TestOrient2SignExpansionMatchesExactCross(t *testing.T) {
	t.Parallel()
	check := func(p [6]float64) {
		t.Helper()
		got, certain := proof.Orient2SignExpansion(p[0], p[1], p[2], p[3], p[4], p[5])
		require.True(t, certain, `%v`, p)
		require.Equal(t, exactCross2Sign(p[0], p[1], p[2], p[3], p[4], p[5]), got, `%v`, p)
	}
	// Corners (0, 0), (n, n+1) and (n+1+k, n+2) for n = 2^52 - 3: both
	// products carry about 104 significant bits and differ by kn + k + 1, so
	// the rounded products agree and only the remainders hold the sign.
	n := float64(1<<52 - 3)
	for _, exponent := range []int{-140, 0, 140} {
		s := math.Ldexp(1, exponent)
		for _, k := range []float64{-1, 0, 1} {
			check([6]float64{0, 0, n * s, (n + 1) * s, (n + 1 + k) * s, (n + 2) * s})
		}
	}
	// The cancellation must also survive the cyclic and mirrored orders, here
	// with a off the origin and m = 2^50 + 1.
	m := float64(1<<50 + 1)
	for _, k := range []float64{-1, 0, 1} {
		a, b, c := [2]float64{1, 3}, [2]float64{m + 1, m + 4}, [2]float64{m + 2 + k, m + 5}
		check([6]float64{a[0], a[1], b[0], b[1], c[0], c[1]})
		check([6]float64{b[0], b[1], c[0], c[1], a[0], a[1]})
		check([6]float64{c[0], c[1], b[0], b[1], a[0], a[1]})
		check([6]float64{a[1], a[0], b[1], b[0], c[1], c[0]})
	}
	rng := rand.New(rand.NewPCG(0x2d0e, 0x51a7))
	exponents := [...]int{-200, -100, -1, 0, 1, 100, 199}
	coordinate := func() float64 {
		v := math.Ldexp(1+float64(rng.Uint64()>>11)*0x1p-53, exponents[rng.IntN(len(exponents))])
		if rng.IntN(2) == 0 {
			return -v
		}
		return v
	}
	for range 3000 {
		// a at the origin keeps every difference exact and admitted.
		p := [6]float64{0, 0, coordinate(), coordinate(), coordinate(), coordinate()}
		check(p)
		// Repeated and collinear corners cancel exactly.
		check([6]float64{0, 0, p[2], p[3], p[2], p[3]})
		if math.Abs(p[2]) < 0x1p199 && math.Abs(p[3]) < 0x1p199 {
			check([6]float64{0, 0, p[2], p[3], 2 * p[2], 2 * p[3]})
		}
		check([6]float64{0, 0, p[2], p[3], -p[2], -p[3]})
	}
	for range 2000 {
		// Small integers over 8: nearly every triple is exact, and many are
		// collinear or repeat a corner.
		var p [6]float64
		for i := range p {
			p[i] = float64(rng.IntN(17)-8) / 8
		}
		check(p)
	}
	for range 3000 {
		// Arbitrary corners: whatever the expansion is certain of must be the
		// exact sign.
		var p [6]float64
		for i := range p {
			p[i] = coordinate()
		}
		got, certain := proof.Orient2SignExpansion(p[0], p[1], p[2], p[3], p[4], p[5])
		if !certain {
			require.Zero(t, got)
			continue
		}
		require.Equal(t, exactCross2Sign(p[0], p[1], p[2], p[3], p[4], p[5]), got, `%v`, p)
	}
	for _, scale := range []float64{0x1p-200, 0x1p200} {
		check([6]float64{0, 0, scale, 0, 0, scale})
		check([6]float64{0, 0, 0, scale, scale, 0})
	}
}

func TestOrient2SignExpansionDeclinesRoundedDifferencesAndExtremeScales(t *testing.T) {
	t.Parallel()
	for _, value := range []float64{math.SmallestNonzeroFloat64, math.Nextafter(0x1p-200, 0),
		math.Nextafter(0x1p200, math.Inf(1)), math.MaxFloat64, math.Inf(1), math.NaN()} {
		got, certain := proof.Orient2SignExpansion(0, 0, value, 0, 0, 1)
		require.False(t, certain, `%v`, value)
		require.Zero(t, got)
	}
	// 1 - 2^-100 rounds to 1, so the difference is inexact even though both
	// inputs are tame.
	got, certain := proof.Orient2SignExpansion(0x1p-100, 0, 1, 0, 0, 1)
	require.False(t, certain)
	require.Zero(t, got)
}

func TestOrient2SignExpansionAllocatesNoStorage(t *testing.T) {
	n := float64(1 << 50)
	allocations := testing.AllocsPerRun(100, func() {
		_, _ = proof.Orient2SignExpansion(0, 0, n, n+1, n+2, n+3)
	})
	require.Zero(t, allocations)
}
