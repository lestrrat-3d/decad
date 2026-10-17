package loftcontact_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// The two stationary wall crossings are interior and ordered. Their exact
// ordering product exceeds the source proof's 8192-bit rational ceiling,
// so the certificate must refuse before constructing that product.
func TestVolumeOrderingPreflightRefusesOversizedProduct(t *testing.T) {
	const maxBits = 8192

	// F/delta = (2^53-1)*2^2045. For the right wall, the raw constant
	// ordering product has odd numerator
	// (M+1)*(M/2-1)*(M-1)*(M+1). Each factor is small enough to form
	// directly; their bit lengths give a lower bound without multiplying them.
	m := new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(1))
	m.Lsh(m, 2045)
	require.Equal(t, 2098, m.BitLen())
	plus := new(big.Int).Add(new(big.Int).Set(m), big.NewInt(1))
	halfMinus := new(big.Int).Sub(new(big.Int).Rsh(new(big.Int).Set(m), 1), big.NewInt(1))
	minus := new(big.Int).Sub(new(big.Int).Set(m), big.NewInt(1))
	for _, factor := range []*big.Int{plus, halfMinus, minus} {
		require.Equal(t, uint(1), factor.Bit(0))
	}
	lowerBits := 2*plus.BitLen() + halfMinus.BitLen() + minus.BitLen() - 3
	require.Greater(t, lowerBits, maxBits)

	f, delta := math.MaxFloat64, math.SmallestNonzeroFloat64
	carrier := [2]sectionrecord.Point2{{U: delta}, {U: f}}
	leftBottom := sectionrecord.Point2{U: f / 4, V: -delta}
	leftTop := sectionrecord.Point2{U: f / 4, V: f}
	rightBottom := sectionrecord.Point2{U: f / 2, V: -delta}
	rightTop := sectionrecord.Point2{U: f / 2, V: f}
	left := [2]sectionrecord.Point2{leftBottom, leftTop}
	right := [2]sectionrecord.Point2{rightBottom, rightTop}

	t.Run("ordered seams", func(t *testing.T) {
		_, admitted, err := loftcontact.CertifyOrderedLineSeams(t.Context(),
			proofbound.NewWorkBudget(t.Context()), left, left, right, right, carrier)
		require.NoError(t, err)
		require.False(t, admitted, "an oversized exact ordering product must be refused")
	})

	t.Run("carrier occupancy", func(t *testing.T) {
		bottom := []sectionrecord.LineSeg{
			{Start: leftBottom, End: rightBottom, TStart: 0, TEnd: 1},
			{Start: rightBottom, End: rightTop, TStart: 0, TEnd: 1},
			{Start: rightTop, End: leftTop, TStart: 0, TEnd: 1},
			{Start: leftTop, End: leftBottom, TStart: 0, TEnd: 1},
		}
		crossing, _, _, admitted, err := loftcontact.CertifyCarrierOccupancy(t.Context(),
			proofbound.NewWorkBudget(t.Context()), bottom, bottom, carrier)
		require.NoError(t, err)
		require.False(t, admitted, "carrier occupancy must refuse the oversized seam ordering")
		require.Equal(t, [2]int{}, crossing)
	})
}
