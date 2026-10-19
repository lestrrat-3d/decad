package loftcontact_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/stretchr/testify/require"
)

func TestLoftArithmeticCmpBounds(t *testing.T) {
	cmp := func(a, b *big.Rat) (int, bool) {
		arithmetic := &loftcontact.Arithmetic{OK: true}
		comparison := arithmetic.Cmp(a, b)
		return comparison, arithmetic.OK
	}
	comparison, bounded := cmp(big.NewRat(3, 2), big.NewRat(6, 4))
	require.True(t, bounded)
	require.Zero(t, comparison)

	large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8190))
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8190))
	require.True(t, (&loftcontact.Arithmetic{OK: true}).Valid(large))
	require.True(t, (&loftcontact.Arithmetic{OK: true}).Valid(tiny))
	_, bounded = cmp(large, big.NewRat(1, 4))
	require.False(t, bounded)
	_, bounded = cmp(big.NewRat(4, 1), tiny)
	require.False(t, bounded)
	_, bounded = cmp(tiny, big.NewRat(4, 1))
	require.False(t, bounded)

	atLimit := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8188))
	comparison, bounded = cmp(atLimit, big.NewRat(1, 4))
	require.True(t, bounded)
	require.Positive(t, comparison)
	comparison, bounded = cmp(big.NewRat(1, 4), atLimit)
	require.True(t, bounded)
	require.Negative(t, comparison)
	atDenominatorLimit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8188))
	comparison, bounded = cmp(big.NewRat(4, 1), atDenominatorLimit)
	require.True(t, bounded)
	require.Positive(t, comparison)
	_, bounded = cmp(nil, big.NewRat(1, 1))
	require.False(t, bounded)
}

func TestLoftReplayFloat64RefusesOversizedAlignment(t *testing.T) {
	denominator := new(big.Int).Lsh(big.NewInt(1), 8140)
	arithmetic := &loftcontact.Arithmetic{OK: true}
	_, exact := arithmetic.Float64(new(big.Rat).SetFrac(big.NewInt(1), denominator))
	require.False(t, exact)
	require.False(t, arithmetic.OK)

	// The last permitted numerator alignment is exactly 8,192 bits.
	denominator = new(big.Int).Lsh(big.NewInt(1), 8137)
	arithmetic = &loftcontact.Arithmetic{OK: true}
	_, _ = arithmetic.Float64(new(big.Rat).SetFrac(big.NewInt(1), denominator))
	require.True(t, arithmetic.OK)
}
