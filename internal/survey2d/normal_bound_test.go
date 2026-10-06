package survey2d_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"strconv"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// requireSameInterval asserts two enclosures are the same exact rationals,
// equal by Cmp and by their canonical RatString.
func requireSameInterval(t *testing.T, want, got proofbound.RatInterval, msg string) {
	t.Helper()
	require.Zero(t, want.Lo.Cmp(got.Lo), msg)
	require.Zero(t, want.Hi.Cmp(got.Hi), msg)
	require.Equal(t, want.Lo.RatString(), got.Lo.RatString(), msg)
	require.Equal(t, want.Hi.RatString(), got.Hi.RatString(), msg)
}

// TestRadSinCosIntervalMemoMatchesUncached pins the memo to the reading it
// replaces: every angle, on its first (filling) call and on later (hitting)
// calls, must return the uncached enclosure exactly, and a caller mutating the
// returned endpoints must not reach the memo.
func TestRadSinCosIntervalMemoMatchesUncached(t *testing.T) {
	angles := map[string]*big.Rat{
		"third":      big.NewRat(1, 3),
		"negative":   big.NewRat(-7, 5),
		"many-turns": big.NewRat(1000003, 7),
		"tiny":       big.NewRat(1, 1<<40),
		"pi/2-float": proofarith.FloatRat(math.Pi / 2),
		"2pi/24":     proofarith.FloatRat(2 * math.Pi / 24),
		"-pi/3":      proofarith.FloatRat(-math.Pi / 3),
	}
	rng := rand.New(rand.NewPCG(3, 4))
	for i := range 16 {
		angles["random/"+strconv.Itoa(i)] = big.NewRat(rng.Int64N(2_000_000)-1_000_000, rng.Int64N(99_999)+1)
	}

	for name, x := range angles {
		t.Run(name, func(t *testing.T) {
			wantSin, wantCos, ok := survey2d.RadSinCosIntervalUncached(x)
			require.True(t, ok)
			for call := range 3 {
				sin, cos, ok := survey2d.RadSinCosInterval(x)
				require.True(t, ok, "call %d", call)
				msg := "call " + strconv.Itoa(call)
				requireSameInterval(t, wantSin, sin, msg)
				requireSameInterval(t, wantCos, cos, msg)
				require.True(t, survey2d.RadSinCosMemoHolds(x), msg)
				// The caller owns the endpoints: mutating them must not reach
				// the memo.
				one := big.NewRat(1, 1)
				sin.Lo.Add(sin.Lo, one)
				sin.Hi.Add(sin.Hi, one)
				cos.Lo.Add(cos.Lo, one)
				cos.Hi.Add(cos.Hi, one)
			}
		})
	}

	// Angles differing only in sign, or sharing a numerator, must not share
	// a memo entry: fill with one, then read the other.
	t.Run("distinct-keys", func(t *testing.T) {
		pairs := [][2]*big.Rat{
			{big.NewRat(-11, 13), big.NewRat(11, 13)},
			{big.NewRat(17, 19), big.NewRat(17, 23)},
		}
		for _, pair := range pairs {
			_, _, ok := survey2d.RadSinCosInterval(pair[0])
			require.True(t, ok)
			wantSin, wantCos, ok := survey2d.RadSinCosIntervalUncached(pair[1])
			require.True(t, ok)
			sin, cos, ok := survey2d.RadSinCosInterval(pair[1])
			require.True(t, ok)
			requireSameInterval(t, wantSin, sin, pair[1].RatString())
			requireSameInterval(t, wantCos, cos, pair[1].RatString())
		}
	})

	t.Run("zero", func(t *testing.T) {
		sin, cos, ok := survey2d.RadSinCosInterval(new(big.Rat))
		require.True(t, ok)
		require.Zero(t, sin.Lo.Sign())
		require.Zero(t, sin.Hi.Sign())
		require.Zero(t, cos.Lo.Cmp(big.NewRat(1, 1)))
		require.Zero(t, cos.Hi.Cmp(big.NewRat(1, 1)))
	})
}
