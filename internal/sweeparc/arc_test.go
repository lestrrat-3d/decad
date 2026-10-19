package sweeparc_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/stretchr/testify/require"
)

func TestArcAngleCarriesArbitraryInterval(t *testing.T) {
	r0 := sweeparc.RatVec{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}
	r1 := sweeparc.RatVec{big.NewRat(3, 5), big.NewRat(4, 5), new(big.Rat)}
	axis := sweeparc.RatVec{new(big.Rat), new(big.Rat), big.NewRat(1, 1)}
	held, denotation, err := sweeparc.ArcAngle(r0, r1, axis)
	require.NoError(t, err)
	require.NotNil(t, denotation.Span)
	require.True(t, denotation.Valid())

	enclosure, ok := denotation.Enclosure()
	require.True(t, ok)
	require.Positive(t, denotation.Delta(held))
	require.LessOrEqual(t, math.Abs(held-math.Atan2(4, 3)), denotation.Delta(held))
	sin, cos, ok := denotation.SinCosFor(held)
	require.True(t, ok)
	require.True(t, intervalContains(sin, big.NewRat(4, 5)))
	require.True(t, intervalContains(cos, big.NewRat(3, 5)))

	negative, ok := denotation.Neg().Enclosure()
	require.True(t, ok)
	require.Zero(t, negative.Lo.Cmp(new(big.Rat).Neg(enclosure.Hi)))
	require.Zero(t, negative.Hi.Cmp(new(big.Rat).Neg(enclosure.Lo)))
	doubled, ok := denotation.Scale(big.NewRat(2, 1)).Enclosure()
	require.True(t, ok)
	require.Zero(t, doubled.Lo.Cmp(new(big.Rat).Mul(enclosure.Lo, big.NewRat(2, 1))))
	require.Zero(t, doubled.Hi.Cmp(new(big.Rat).Mul(enclosure.Hi, big.NewRat(2, 1))))
}

func intervalContains(interval proofbound.RatInterval, value *big.Rat) bool {
	return interval.Lo.Cmp(value) <= 0 && interval.Hi.Cmp(value) >= 0
}
