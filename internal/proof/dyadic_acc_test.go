package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// randomAccDyadic returns a zero, an inline-mantissa value or a big-mantissa
// value, with an exponent anywhere in -1100..1100.
func randomAccDyadic(t *testing.T, r *rand.Rand) proof.Dyadic {
	t.Helper()
	exp := r.IntN(2201) - 1100
	var bits int
	switch r.IntN(6) {
	case 0:
		return proof.DyZero()
	case 1, 2:
		bits = 1 + r.IntN(100)
	default:
		bits = 100 + r.IntN(300)
	}
	buf := make([]byte, bits/8+1)
	for i := range buf {
		buf[i] = byte(r.UintN(256))
	}
	m := new(big.Int).SetBytes(buf)
	m.Rsh(m, uint(len(buf)*8-bits))
	if r.IntN(2) == 0 {
		m.Neg(m)
	}
	return dyadicFromParts(t, m, exp)
}

func requireAccMatches(t *testing.T, want proof.Dyadic, acc *proof.DyAcc) {
	t.Helper()
	got := acc.Dyadic()
	require.Zero(t, proof.DyCmp(want, got))
	require.Equal(t, want.Exp(), got.Exp())
	require.Zero(t, want.Rat().Cmp(acc.Rat()))
}

func TestDyAccMatchesDyadicChains(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(31, 37))
	for range 2000 {
		n := 1 + r.IntN(60)
		var sum, prods proof.Dyadic
		var accSum, accProd proof.DyAcc
		for range n {
			x, y := randomAccDyadic(t, r), randomAccDyadic(t, r)
			sum = proof.DyAdd(sum, x)
			accSum.Add(x)
			prods = proof.DyAdd(prods, proof.DyMul(x, y))
			accProd.AddProduct(x, y)
			if r.IntN(10) == 0 {
				// Exact cancellation of the running sum.
				sum = proof.DyAdd(sum, proof.DyNeg(sum))
				accSum.Add(proof.DyNeg(accSum.Dyadic()))
			}
		}
		requireAccMatches(t, sum, &accSum)
		requireAccMatches(t, prods, &accProd)
	}
}

func TestDyAccZeroValueIsZero(t *testing.T) {
	t.Parallel()
	var acc proof.DyAcc
	require.True(t, acc.Dyadic().IsZero())
	require.Zero(t, acc.Rat().Sign())
}

func TestDyAccInlineUpdatesAllocateNothingAfterWarmup(t *testing.T) {
	x, y := proof.MustDyOf(3.141592653589793), proof.MustDyOf(1.4142135623730951)
	for _, tc := range []struct {
		name string
		add  func(*proof.DyAcc)
	}{
		{"sum", func(a *proof.DyAcc) { a.Add(x) }},
		{"product", func(a *proof.DyAcc) { a.AddProduct(x, y) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var acc proof.DyAcc
			// Fill the accumulator's storage before measuring repeated updates.
			for range 1024 {
				tc.add(&acc)
			}
			snapshot := acc.Dyadic()
			before := snapshot.Rat()
			allocations := testing.AllocsPerRun(100, func() { tc.add(&acc) })
			require.Zero(t, allocations)
			require.Zero(t, before.Cmp(snapshot.Rat()), "an update changed the published sum")
		})
	}
}
