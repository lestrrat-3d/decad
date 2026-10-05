package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRatIntervalCachedPiCopiesRemainIndependent(t *testing.T) {
	t.Parallel()
	wantLo := new(big.Rat).Set(quarterPiIv.lo)
	wantHi := new(big.Rat).Set(quarterPiIv.hi)

	got := quarterPiInterval()
	got.lo.SetInt64(0)
	got.hi.SetInt64(1)
	require.Zero(t, quarterPiIv.lo.Cmp(wantLo))
	require.Zero(t, quarterPiIv.hi.Cmp(wantHi))

	second := quarterPiInterval()
	require.Zero(t, second.lo.Cmp(wantLo))
	require.Zero(t, second.hi.Cmp(wantHi))
	require.NotSame(t, got.lo, second.lo)
	require.NotSame(t, got.hi, second.hi)
}
