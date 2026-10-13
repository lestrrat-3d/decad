package meshbool_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRatCoordOfMatchesSetFrac(t *testing.T) {
	rng := rand.New(rand.NewPCG(43, 47))
	coord := func() float64 {
		return (rng.Float64() - 0.5) * float64(uint64(1)<<uint(rng.IntN(40))) * 1e-3
	}
	for range 300 {
		p := proof.XptOf(r3.Vec{X: coord(), Y: coord(), Z: coord()})
		if rng.IntN(3) == 0 {
			p.W = big.NewInt(int64(rng.IntN(1000)*2 + 1))
		}
		for axis, num := range []*big.Int{p.X, p.Y, p.Z} {
			want := new(big.Rat).SetFrac(num, p.W)
			got := meshbool.RatCoordOf(p, axis)
			require.Zero(t, want.Num().Cmp(got.Num()))
			require.Zero(t, want.Denom().Cmp(got.Denom()))
		}
	}
}
