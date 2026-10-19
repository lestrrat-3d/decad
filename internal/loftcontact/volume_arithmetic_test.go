package loftcontact_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/stretchr/testify/require"
)

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
