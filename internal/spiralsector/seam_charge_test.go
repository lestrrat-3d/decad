package spiralsector

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func chargePremise(yMin, yMax, zMax, rMax, span int64) seamChargePremise {
	r := func(v int64) *big.Rat { return big.NewRat(v, 1) }
	return seamChargePremise{yMin: r(yMin), yMax: r(yMax), zMax: r(zMax), rMax: r(rMax), spanMax: r(span)}
}

func TestSeamChargePremiseCoversTheSourceDefaultAndMore(t *testing.T) {
	e := big.NewRat(1, 1_000_000_000_000)
	premise := chargePremise(13, 15, 5, 16, 4)

	// The fixed premises of the source-default gear: Q_t above 25.
	require.True(t, premise.covers(e, big.NewRat(25, 1)))
	// A smaller Q_t still keeps the move below the published charges: the root
	// moves by span*(2*(15+5+16)*e)/Q_t, which stays under 1e-10 mm down to Q_t = 2.9.
	require.True(t, premise.covers(e, big.NewRat(24, 1)))
	require.True(t, premise.covers(e, big.NewRat(3, 1)))
	// Q_t of 2 moves the root by 1.4e-10 mm, past the 1e-10 mm charge.
	require.False(t, premise.covers(e, big.NewRat(2, 1)))
	require.False(t, premise.covers(e, big.NewRat(1, 1000)))
	// A radius error of 1e-9 mm is past the charge even with Q_t = 25.
	require.False(t, premise.covers(big.NewRat(1, 1_000_000_000), big.NewRat(25, 1)))
	// No positive Q_t proves nothing.
	require.False(t, premise.covers(e, big.NewRat(0, 1)))
	require.False(t, premise.covers(e, nil))
}
