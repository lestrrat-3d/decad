package proof_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestDyadicCompareAlignedWideMantissas(t *testing.T) {
	for _, width := range []uint{1, 62, 63, 64, 65, 125, 126, 127, 128, 191, 255, 511, 2047} {
		for _, shift := range []uint{1, 31, 32, 63, 64, 65, 127, 128, 129, 511} {
			for _, delta := range []int64{-1, 0, 1} {
				n := new(big.Int).Lsh(big.NewInt(1), width)
				n.Add(n, big.NewInt(1))
				m := new(big.Int).Lsh(n, shift)
				m.Add(m, big.NewInt(delta))
				a := new(big.Rat).SetInt(n)
				b := new(big.Rat).SetFrac(m, new(big.Int).Lsh(big.NewInt(1), shift))
				for _, negative := range []bool{false, true} {
					if negative {
						a.Neg(a)
						b.Neg(b)
					}
					da, ok := proof.DyOfRat(a)
					require.True(t, ok)
					db, ok := proof.DyOfRat(b)
					require.True(t, ok)
					require.Equal(t, a.Cmp(b), proof.DyCmp(da, db),
						"width %d shift %d delta %d negative %t", width, shift, delta, negative)
					require.Equal(t, b.Cmp(a), proof.DyCmp(db, da),
						"reverse width %d shift %d delta %d negative %t", width, shift, delta, negative)
					require.Zero(t, a.Cmp(da.Rat()), "first input changed")
					require.Zero(t, b.Cmp(db.Rat()), "second input changed")
				}
			}
		}
	}
}

func TestDyadicCompareWideMantissasAllocatesNothing(t *testing.T) {
	n := new(big.Int).Lsh(big.NewInt(1), 511)
	n.Add(n, big.NewInt(1))
	m := new(big.Int).Lsh(n, 65)
	m.Add(m, big.NewInt(1))
	a, ok := proof.DyOfRat(new(big.Rat).SetInt(n))
	require.True(t, ok)
	b, ok := proof.DyOfRat(new(big.Rat).SetFrac(m, new(big.Int).Lsh(big.NewInt(1), 65)))
	require.True(t, ok)
	var comparison int
	allocations := testing.AllocsPerRun(100, func() {
		comparison = proof.DyCmp(a, b)
	})
	require.Equal(t, -1, comparison)
	require.Zero(t, allocations)
}
