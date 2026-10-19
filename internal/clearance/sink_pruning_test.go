package clearance

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestPruningSinkSkipsOnlyBoundsStrictlyBeyondTheBest(t *testing.T) {
	t.Parallel()

	sink := NewPruningSink(0.5)
	sink.Contribs = append(sink.Contribs,
		GapContrib{Lo: 4, Hi: math.Inf(1)},
		GapContrib{Lo: 5, Hi: 5, Exact: true},
	)
	require.False(t, sink.Pruned(5.5))
	require.True(t, sink.Pruned(math.Nextafter(5.5, 6)))
	require.Equal(t, 1, sink.Skipped)
}

func TestPruningSinkLeavesUnboundedAndUnusableBoundsUnpruned(t *testing.T) {
	t.Parallel()

	sink := NewPruningSink(0)
	sink.Contribs = append(sink.Contribs, GapContrib{Lo: 5, Hi: 5, Exact: true})
	require.False(t, sink.Pruned(5))
	require.False(t, sink.Pruned(math.Inf(1)))
	require.False(t, sink.Pruned(math.NaN()))
	require.Zero(t, sink.Skipped)

	noUpperBound := NewPruningSink(0)
	noUpperBound.Contribs = append(noUpperBound.Contribs, GapContrib{Lo: 4, Hi: math.Inf(1)})
	require.False(t, noUpperBound.Pruned(1e300))

	var zeroValue CellSink
	zeroValue.Contribs = append(zeroValue.Contribs, GapContrib{Lo: 5, Hi: 5, Exact: true})
	require.False(t, zeroValue.Pruned(100))
}
