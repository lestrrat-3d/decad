package loftmesh

import (
	"fmt"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestLoftStationCapScalesWithPairs pins S15's cap
// min(max(512, 64·P), 8192) (agent-docs/loft-gear-bounds-design.md §7) and the
// shared extra-cell budget that reserves one cell per pair from it.
func TestLoftStationCapScalesWithPairs(t *testing.T) {
	t.Parallel()
	for _, row := range []struct {
		p    uint64
		want int
	}{
		{p: 3, want: 512},
		{p: 6, want: 512},
		{p: 8, want: 512},
		{p: 9, want: 576},
		{p: 48, want: 3072},
		{p: 120, want: 7680},
		{p: 127, want: 8128},
		{p: 128, want: 8192},
		{p: 240, want: 8192},
		{p: 10_000, want: 8192},
	} {
		t.Run(fmt.Sprintf("P=%d", row.p), func(t *testing.T) {
			require.Equal(t, row.want, StationCap(row.p))
			require.Equal(t, max(0, row.want-int(row.p)), StationExtraBudget(row.p), //nolint:gosec // p is a small table constant.
				"S15 reserves every pair's first station, then shares the remainder")
		})
	}
}
