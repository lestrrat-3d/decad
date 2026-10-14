package meshbool_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireSameCacheBits(t *testing.T, want, got float64, msg string) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), "%s: want %v got %v", msg, want, got)
}

// TestXP2FloatCacheMatchesRatFloat64 pins the float cache NewXP2 and
// NewXP2FromXpt store to big.Rat.Float64 of the coordinate, for power-of-two
// denominators (lifted floats, differences of them), ties, huge and tiny
// values, and denominators that are not powers of two.
func TestXP2FloatCacheMatchesRatFloat64(t *testing.T) {
	rng := rand.New(rand.NewPCG(501, 503))
	check := func(u, v *big.Rat) {
		t.Helper()
		wantU, _ := u.Float64()
		wantV, _ := v.Float64()
		p := meshbool.NewXP2(u, v)
		requireSameCacheBits(t, wantU, p.Fu, "Fu")
		requireSameCacheBits(t, wantV, p.Fv, "Fv")
	}
	pow2 := func(k int) *big.Int { return new(big.Int).Lsh(big.NewInt(1), uint(k)) }
	finite := func() float64 {
		for {
			f := math.Float64frombits(rng.Uint64())
			if !math.IsNaN(f) && !math.IsInf(f, 0) {
				return f
			}
		}
	}
	check(new(big.Rat), new(big.Rat))
	check(big.NewRat(1, 3), big.NewRat(-2, 7))
	for range 5000 {
		a, b := proof.FloatRat(finite()), proof.FloatRat(finite())
		check(a, b)
		check(new(big.Rat).Sub(a, b), new(big.Rat).Add(a, b))
		odd := new(big.Int).SetUint64(rng.Uint64()&(1<<54-1) | 1<<53 | 1)
		tie := new(big.Rat).SetFrac(new(big.Int).Lsh(odd, uint(rng.IntN(300))), pow2(rng.IntN(1200)))
		check(tie, new(big.Rat).Neg(tie))
		check(tie, new(big.Rat).Add(tie, big.NewRat(1, 3)))
	}
	for _, k := range []int{1023, 1024, 1100} {
		check(new(big.Rat).SetInt(pow2(k)), new(big.Rat).SetFrac(big.NewInt(1), pow2(k+60)))
	}

	// The same through homogeneous points whose weight is not a power of two.
	for range 3000 {
		a := proof.XptOf(r3.Vec{X: finite() / 1e290, Y: finite() / 1e290, Z: finite() / 1e290})
		b := proof.XptOf(r3.Vec{X: rng.NormFloat64(), Y: rng.NormFloat64() * 1e-9, Z: rng.NormFloat64()})
		td := new(big.Int).SetUint64(rng.Uint64()%1000 + 1)
		tn := new(big.Int).SetUint64(rng.Uint64() % 1000)
		for _, p := range []proof.Xpt{a, b, proof.Xsub(a, b), proof.Xlerp(a, b, tn, td)} {
			for _, axes := range [][2]int{{0, 1}, {1, 2}, {2, 0}} {
				u, v := axes[0], axes[1]
				ur, vr := meshbool.RatCoordOf(p, u), meshbool.RatCoordOf(p, v)
				wantU, _ := ur.Float64()
				wantV, _ := vr.Float64()
				got := meshbool.NewXP2FromXpt(p, u, v)
				requireSameCacheBits(t, wantU, got.Fu, "FromXpt Fu")
				requireSameCacheBits(t, wantV, got.Fv, "FromXpt Fv")
				plain := meshbool.NewXP2(ur, vr)
				require.Equal(t, plain.FloatFinite, got.FloatFinite)
			}
		}
	}
}
