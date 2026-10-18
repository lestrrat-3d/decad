package spiralsector

import (
	"context"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// The station scan must return cancellation before reading an incomplete
// source record. Both moved root-cell proofs use this poll.
func TestRootCellsPropagateCancellation(t *testing.T) {
	trim := &loftFacesTrimRecord{StationGroup: []int{4}}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, check := range []struct {
		name string
		fn   func(context.Context, *loftFacesTrimRecord, *big.Rat) (bool, error)
	}{
		{name: "heel", fn: spiralHeelRootCells},
		{name: "toe", fn: spiralToeRootCells},
	} {
		t.Run(check.name, func(t *testing.T) {
			admitted, err := check.fn(ctx, trim, big.NewRat(1, 1))
			require.False(t, admitted)
			require.ErrorIs(t, err, context.Canceled)
		})
	}
}
