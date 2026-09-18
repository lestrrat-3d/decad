package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestSweepArcAngleDenotationCarriesArbitraryInterval(t *testing.T) {
	t.Parallel()

	r0 := sweepRatVec{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}
	r1 := sweepRatVec{big.NewRat(3, 5), big.NewRat(4, 5), new(big.Rat)}
	axis := sweepRatVec{new(big.Rat), new(big.Rat), big.NewRat(1, 1)}
	held, den, err := sweepArcAngle(r0, r1, axis)
	require.NoError(t, err)
	require.NotNil(t, den.span)
	require.True(t, den.valid())

	enc, ok := den.enclosure()
	require.True(t, ok)
	require.Positive(t, den.delta(held))

	want := math.Atan2(4, 3)
	require.LessOrEqual(t, math.Abs(held-want), den.delta(held))
	sin, cos, ok := den.sinCosFor(held)
	require.True(t, ok)
	require.True(t, intervalContainsRat(sin, big.NewRat(4, 5)))
	require.True(t, intervalContainsRat(cos, big.NewRat(3, 5)))

	neg, ok := den.neg().enclosure()
	require.True(t, ok)
	require.Zero(t, neg.lo.Cmp(new(big.Rat).Neg(enc.hi)))
	require.Zero(t, neg.hi.Cmp(new(big.Rat).Neg(enc.lo)))
	doubled, ok := den.scale(big.NewRat(2, 1)).enclosure()
	require.True(t, ok)
	require.Zero(t, doubled.lo.Cmp(new(big.Rat).Mul(enc.lo, big.NewRat(2, 1))))
	require.Zero(t, doubled.hi.Cmp(new(big.Rat).Mul(enc.hi, big.NewRat(2, 1))))
}

func TestSweepArcAngleDenotationFeedsHalfTurnDecision(t *testing.T) {
	t.Parallel()

	span := interval(big.NewRat(4, 1), big.NewRat(4001, 1000))
	den := angleDenotation{span: &span}
	sweep := sweepDenotation{phi0: zeroAngleDenotation(), phi1: den}
	excess, ok := sweep.halfTurnExcessFor(0, 4)
	require.True(t, ok)
	require.Positive(t, excess.lo.Sign())
	require.Positive(t, excess.hi.Sign())
}

func intervalContainsRat(iv ratInterval, value *big.Rat) bool {
	return iv.lo.Cmp(value) <= 0 && iv.hi.Cmp(value) >= 0
}
