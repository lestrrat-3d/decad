package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

var proofIntegrationFloats = []float64{
	0, 1, -1, 0.5, 0.1, math.Pi, 1e-300, 1e300,
	math.SmallestNonzeroFloat64, math.MaxFloat64,
	math.Nextafter(1, 2), math.Nextafter(1, 0),
}

func ratOfDyadic(t *testing.T, d proofarith.Dyadic) *big.Rat {
	t.Helper()
	if d.Mant() == nil {
		return new(big.Rat)
	}
	out := new(big.Rat).SetInt(d.Mant())
	if d.Exp() >= 0 {
		return out.Mul(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(d.Exp()))))
	}
	return out.Quo(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(-d.Exp()))))
}

// TestDyadicSqrtBracketsMatchTheRationalOnes pins the directed square-root
// walks against the rational ones they restate: the same float, proven by the
// same exact comparison, for a spread of magnitudes.
func TestDyadicSqrtBracketsMatchTheRationalOnes(t *testing.T) {
	t.Parallel()
	for _, f := range proofIntegrationFloats {
		if f <= 0 {
			continue
		}
		d, q := proofarith.MustDyOf(f), proofarith.FloatRat(f)
		require.Equal(t, ratSqrtDown(q), proofarith.DySqrtDown(d), "sqrt down of %v", f)
		require.Equal(t, ratSqrtUp(q), proofarith.DySqrtUp(d), "sqrt up of %v", f)

		// The bracket is PROVEN, not merely close: the down leg squares to at
		// most the value and the up leg to at least it, decided exactly.
		require.True(t, proofarith.DySquareAtMost(proofarith.DySqrtDown(d), d), "the down leg must square to at most %v", f)
		if up := proofarith.DySqrtUp(d); !isNonFinite(up) {
			require.False(t, proofarith.DyCmp(proofarith.DyMul(proofarith.MustDyOf(up), proofarith.MustDyOf(up)), d) < 0,
				"the up leg must square to at least %v", f)
		}
	}
	require.Zero(t, proofarith.DySqrtDown(proofarith.MustDyOf(-1)), "a negative value brackets at zero, as its rational twin does")
	require.Zero(t, proofarith.DySqrtUp(proofarith.MustDyOf(-1)))
}

// TestDyadicDirectedRoundingMatchesTheRationalOnes pins dyFloatDown/dyFloatUp
// against ratFloatDown/ratFloatUp on values that are deliberately NOT floats:
// a mid-ulp third of a sum is where a directed rounding either steps or does
// not, and where an off-by-one would show.
func TestDyadicDirectedRoundingMatchesTheRationalOnes(t *testing.T) {
	t.Parallel()
	for _, f := range proofIntegrationFloats {
		d := proofarith.MustDyOf(f)
		// A value needing more than 53 significant bits: the float itself plus
		// one ulp of its own smallest neighbour, which no float64 holds.
		wide := proofarith.DyAdd(proofarith.DyMul(d, d), proofarith.DyShift(proofarith.MustDyOf(1), -1080))
		require.Equal(t, ratFloatDown(wide.Rat()), proofarith.DyFloatDown(wide), "float down of the widened %v", f)
		require.Equal(t, ratFloatUp(wide.Rat()), proofarith.DyFloatUp(wide), "float up of the widened %v", f)
		require.Equal(t, f, proofarith.DyFloatDown(d), "a value that IS a float rounds to itself, for %v", f)
		require.Equal(t, f, proofarith.DyFloatUp(d), "a value that IS a float rounds to itself, for %v", f)
	}
}
