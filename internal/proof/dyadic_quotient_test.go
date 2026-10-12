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
