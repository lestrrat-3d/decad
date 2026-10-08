package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sweeparc"

	"github.com/stretchr/testify/require"
)

func TestSweepArcAngleDenotationCarriesArbitraryInterval(t *testing.T) {
	t.Parallel()

	r0 := sweepRatVec{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}
	r1 := sweepRatVec{big.NewRat(3, 5), big.NewRat(4, 5), new(big.Rat)}
	axis := sweepRatVec{new(big.Rat), new(big.Rat), big.NewRat(1, 1)}
	held, den, err := sweeparc.ArcAngle(r0, r1, axis)
	require.NoError(t, err)
	require.NotNil(t, den.Span)
	require.True(t, den.Valid())

	enc, ok := den.Enclosure()
	require.True(t, ok)
	require.Positive(t, den.Delta(held))

	want := math.Atan2(4, 3)
	require.LessOrEqual(t, math.Abs(held-want), den.Delta(held))
	sin, cos, ok := den.SinCosFor(held)
	require.True(t, ok)
	require.True(t, intervalContainsRat(sin, big.NewRat(4, 5)))
	require.True(t, intervalContainsRat(cos, big.NewRat(3, 5)))

	neg, ok := den.Neg().Enclosure()
	require.True(t, ok)
	require.Zero(t, neg.Lo.Cmp(new(big.Rat).Neg(enc.Hi)))
	require.Zero(t, neg.Hi.Cmp(new(big.Rat).Neg(enc.Lo)))
	doubled, ok := den.Scale(big.NewRat(2, 1)).Enclosure()
	require.True(t, ok)
	require.Zero(t, doubled.Lo.Cmp(new(big.Rat).Mul(enc.Lo, big.NewRat(2, 1))))
	require.Zero(t, doubled.Hi.Cmp(new(big.Rat).Mul(enc.Hi, big.NewRat(2, 1))))
}

func TestSweepArcAngleDenotationFeedsHalfTurnDecision(t *testing.T) {
	t.Parallel()

	span := proofbound.Interval(big.NewRat(4, 1), big.NewRat(4001, 1000))
	den := revolveangle.Angle{Span: &span}
	sweep := revolveangle.Sweep{Phi0: revolveangle.Zero(), Phi1: den}
	excess, ok := sweep.HalfTurnExcessFor(0, 4)
	require.True(t, ok)
	require.Positive(t, excess.Lo.Sign())
	require.Positive(t, excess.Hi.Sign())
}

func intervalContainsRat(iv proofbound.RatInterval, value *big.Rat) bool {
	return iv.Lo.Cmp(value) <= 0 && iv.Hi.Cmp(value) >= 0
}
