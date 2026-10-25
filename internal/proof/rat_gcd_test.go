package proof

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRatGCDPowerTwoAndAliasing(t *testing.T) {
	values := []*big.Int{new(big.Int), big.NewInt(1), big.NewInt(-1)}
	for _, exponent := range []uint{1, 31, 32, 63, 64, 127, 128, 511, 512, 1024} {
		p := new(big.Int).Lsh(big.NewInt(1), exponent)
		mixed := new(big.Int).Mul(p, big.NewInt(15))
		values = append(values, p, new(big.Int).Neg(p), mixed, new(big.Int).Neg(mixed))
	}
	for i, a := range values {
		for j, b := range values {
			want := new(big.Int).GCD(nil, nil, a, b)
			for alias := range 3 {
				aa, bb := new(big.Int).Set(a), new(big.Int).Set(b)
				z := new(big.Int)
				switch alias {
				case 1:
					z = aa
				case 2:
					z = bb
				}
				got := ratGCD(z, aa, bb)
				require.Zero(t, want.Cmp(got), "case %d,%d alias %d", i, j, alias)
				if alias != 1 {
					require.Zero(t, a.Cmp(aa), "first input changed: case %d,%d", i, j)
				}
				if alias != 2 {
					require.Zero(t, b.Cmp(bb), "second input changed: case %d,%d", i, j)
				}
			}
		}
		x := new(big.Int).Set(a)
		want := new(big.Int).Abs(a)
		require.Zero(t, want.Cmp(ratGCD(x, x, x)), "both inputs alias: case %d", i)
	}
}
