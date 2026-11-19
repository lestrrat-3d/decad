package smoothloft

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCircularFlatHalvesKeepsRecordedJoins(t *testing.T) {
	p := func(x, y int64) twistSpan { return twistLineSpan(twistTestPoint(1, x), twistTestPoint(1, y)) }
	fit := []twistSpan{twistTestLinePatch(twistTestPoint(1, 3), twistTestPoint(-1, 0))[0],
		twistTestLinePatch(twistTestPoint(-1, 0), twistTestPoint(1, -3))[0]}
	// The three original joins are at -3, -1, 1 and 3; the midpoint
	// additionally splits the middle line without dropping its endpoints.
	edges := [][]twistSpan{{p(-3, -1)}, {p(-1, 1)}, {p(1, 3)}, fit}
	groups, intervals, ok := circularFlatHalves(edges, 3)
	require.True(t, ok)
	require.Len(t, groups[0], 2)
	require.Len(t, groups[3], 2)
	require.True(t, twistSame(groups[3][0][3], twistTestPoint(1, -1)))
	require.True(t, twistSame(groups[3][1][3], twistTestPoint(1, 0)))
	require.True(t, twistSame(groups[0][0][3], twistTestPoint(1, 1)))
	require.Zero(t, intervals[3][0].hi.Cmp(big.NewRat(2, 3)))
	require.Zero(t, intervals[0][0].hi.Cmp(big.NewRat(1, 3)))
	bad := append([][]twistSpan(nil), edges...)
	bad[1] = []twistSpan{twistLineSpan(twistTestPoint(1, -1), twistTestPoint(2, 1))}
	_, _, ok = circularFlatHalves(bad, 3)
	require.False(t, ok)
}
