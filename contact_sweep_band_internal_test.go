package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestPlanarReplayLowerGapChargesTravel reads the replay's lower gap inside a
// certified clear span directly. A sound sweep leaves no span whose margin
// falls below a pose rounding, so no public fixture can show the travel charge
// failing; this one does: deleting the charge reads the span's end gaps
// unreduced, 3 mm instead of 2 mm.
func TestPlanarReplayLowerGapChargesTravel(t *testing.T) {
	replay := planarReplay{travel: big.NewRat(4, 1), spans: []planarClearSpan{
		{from: new(big.Rat), to: big.NewRat(1, 2), left: big.NewRat(3, 1), right: big.NewRat(1, 1)},
		{from: big.NewRat(1, 2), to: big.NewRat(1, 1), axis: big.NewRat(1, 8)},
	}}
	// At 1/4: the left end's 3 mm less 4·(1/4), against the right end's
	// 1 mm less 4·(1/4).
	require.Zero(t, big.NewRat(2, 1).Cmp(replay.lowerGap(big.NewRat(1, 4))))
	// A hull-separated span reads its hull gap anywhere inside it.
	require.Zero(t, big.NewRat(1, 8).Cmp(replay.lowerGap(big.NewRat(3, 4))))
	require.Nil(t, (&planarReplay{}).lowerGap(big.NewRat(1, 2)))
}
