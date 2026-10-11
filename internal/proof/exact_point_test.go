package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestXhpVecKeepsRationalFloatRounding(t *testing.T) {
	t.Parallel()
	check := func(num, den *big.Int) {
		t.Helper()
		positive, _ := new(big.Rat).SetFrac(num, den).Float64()
		negative, _ := new(big.Rat).SetFrac(new(big.Int).Neg(num), den).Float64()
		got := proof.XhpVec(proof.Xhp{
			X: num,
			Y: new(big.Int),
			Z: new(big.Int).Neg(num),
			W: den,
		})
		require.Equal(t, math.Float64bits(positive), math.Float64bits(got.X))
		require.Equal(t, uint64(0), math.Float64bits(got.Y))
		require.Equal(t, math.Float64bits(negative), math.Float64bits(got.Z))
	}

	check(big.NewInt(0), big.NewInt(1))
	check(big.NewInt(1), big.NewInt(3))
	check(new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(1)),
		new(big.Int).Lsh(big.NewInt(1), 53))
	check(big.NewInt(3), new(big.Int).Lsh(big.NewInt(1), 1075))
	check(new(big.Int).Lsh(big.NewInt(3), 500), new(big.Int).Lsh(big.NewInt(7), 500))
	for _, shift := range []uint{52, 53, 1022, 1074, 1075, 1100, 2048} {
		check(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), shift))
		check(new(big.Int).Lsh(big.NewInt(1), shift), big.NewInt(3))
	}
	rng := rand.New(rand.NewPCG(0x8db4c214, 0x54fc32a7))
	for range 1000 {
		num := new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()|1), uint(rng.IntN(1100)))
		num.Add(num, new(big.Int).SetUint64(rng.Uint64()))
		den := new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()|1), uint(rng.IntN(1100)))
		den.Add(den, new(big.Int).SetUint64(rng.Uint64()))
		check(num, den)
	}
}
