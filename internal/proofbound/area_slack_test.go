package proofbound_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestAreaSlack(t *testing.T) {
	// published [9, 11], held [10, 12]: distances |9-12| = 3 and |11-10| = 1.
	got, ok := proofbound.AreaSlack(10, 1, big.NewRat(10, 1), big.NewRat(12, 1))
	require.True(t, ok)
	require.GreaterOrEqual(t, got, 3.0)
	require.Less(t, got, 3.0+1e-9)

	// published [9, 11], held [8, 9]: distances |9-9| = 0 and |11-8| = 3.
	got, ok = proofbound.AreaSlack(10, 1, big.NewRat(8, 1), big.NewRat(9, 1))
	require.True(t, ok)
	require.GreaterOrEqual(t, got, 3.0)
	require.Less(t, got, 3.0+1e-9)

	_, ok = proofbound.AreaSlack(10, 1, nil, nil)
	require.False(t, ok)
	_, ok = proofbound.AreaSlack(10, -1, big.NewRat(1, 1), big.NewRat(2, 1))
	require.False(t, ok)
	_, ok = proofbound.AreaSlack(math.Inf(1), 1, big.NewRat(1, 1), big.NewRat(2, 1))
	require.False(t, ok)
}
