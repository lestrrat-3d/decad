package revolvemesh_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

// TestRevolveAngularHomotopyFactorMemoMatchesUncached pins the memo to the
// reading it replaces: every step, on its first (filling) call and on later
// (hitting) calls, must return the uncached reading exactly — equal by Cmp and
// by RatString — and a caller mutating a returned value must not reach the
// memo.
func TestRevolveAngularHomotopyFactorMemoMatchesUncached(t *testing.T) {
	steps := map[string]proofbound.RatInterval{}
	for _, n := range []int64{8, 16, 24, 64} {
		steps["full/"+big.NewInt(n).String()] = proofbound.IntervalScale(proofbound.TwoPiInterval(), big.NewRat(1, n))
	}
	partial := []struct {
		name   string
		sweep  float64
		chords float64
	}{
		{"quarter/6", math.Pi / 2, 6},
		{"three-quarter/18", 3 * math.Pi / 2, 18},
		{"negative/12", -math.Pi / 3, 12},
		{"tiny/1", 1e-6, 1},
	}
	for _, p := range partial {
		steps[p.name] = proofbound.PointInterval(proofarith.FloatRat(p.sweep / p.chords))
	}
	steps["wide"] = proofbound.Interval(big.NewRat(1, 10), big.NewRat(1, 9))
	steps["straddle"] = proofbound.Interval(big.NewRat(-1, 50), big.NewRat(1, 40))
	rng := rand.New(rand.NewPCG(1, 2))
	for i := range 8 {
		lo := big.NewRat(rng.Int64N(2000)-1000, 997)
		hi := new(big.Rat).Add(lo, big.NewRat(rng.Int64N(50), 100003))
		steps["random/"+big.NewInt(int64(i)).String()] = proofbound.Interval(lo, hi)
	}

	for name, step := range steps {
		t.Run(name, func(t *testing.T) {
			want, err := revolvemesh.RevolveAngularHomotopyFactorUncached(step)
			require.NoError(t, err)
			for call := range 3 {
				got, err := revolvemesh.RevolveAngularHomotopyFactor(step)
				require.NoError(t, err)
				require.Zero(t, got.Cmp(want), "call %d", call)
				require.Equal(t, want.RatString(), got.RatString(), "call %d", call)
				require.True(t, revolvemesh.RevolveHomotopyMemoHolds(step), "call %d", call)
				// The caller owns the value: mutating it must not reach the memo.
				got.Add(got, big.NewRat(1, 1))
			}
		})
	}

	t.Run("refusal", func(t *testing.T) {
		step := proofbound.Interval(big.NewRat(1, 1), new(big.Rat))
		_, want := revolvemesh.RevolveAngularHomotopyFactorUncached(step)
		require.ErrorIs(t, want, revolvemesh.ErrRevolveAngularHomotopy)
		for call := range 2 {
			got, err := revolvemesh.RevolveAngularHomotopyFactor(step)
			require.Nil(t, got, "call %d", call)
			require.Equal(t, want, err, "call %d", call)
		}
	})
}
