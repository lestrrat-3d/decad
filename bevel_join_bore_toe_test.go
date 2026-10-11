package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestBevelToeBoreIntersectionUsesRecordedFlank(t *testing.T) {
	root := Point2{U: 23.333333333333332, V: 13.333333333333332}
	toe := Point2{U: 32, V: 1.777777777777775}
	cross, ok := bevelToeBoreIntersection(root, toe, 6)
	require.True(t, ok)
	require.InDelta(t, 28.833333333333332, cross.x, 1e-12)
	require.Positive(t, cross.slopeLo)
	require.GreaterOrEqual(t, cross.slopeHi, cross.slopeLo)

	// The published float allowance contains the exact rational intersection
	// of the held source coordinates, including nonterminating quotients.
	dx := new(big.Rat).Sub(proofarith.FloatRat(toe.U), proofarith.FloatRat(root.U))
	dr := new(big.Rat).Sub(proofarith.FloatRat(root.V), proofarith.FloatRat(toe.V))
	fraction := new(big.Rat).Quo(
		new(big.Rat).Sub(proofarith.FloatRat(root.V), proofarith.FloatRat(6)), dr)
	exact := new(big.Rat).Add(proofarith.FloatRat(root.U), new(big.Rat).Mul(dx, fraction))
	gap := new(big.Rat).Sub(exact, proofarith.FloatRat(cross.x))
	gap.Abs(gap)
	require.LessOrEqual(t, gap.Cmp(proofarith.FloatRat(cross.round)), 0)

	for _, radius := range []float64{toe.V, root.V, 0, root.V + 1} {
		_, ok := bevelToeBoreIntersection(root, toe, radius)
		require.False(t, ok)
	}
	_, ok = bevelToeBoreIntersection(toe, root, 6)
	require.False(t, ok)
	_, ok = bevelToeBoreIntersection(root, toe, math.NaN())
	require.False(t, ok)
}
