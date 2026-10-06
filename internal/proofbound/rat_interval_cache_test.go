package proofbound

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRatIntervalCachedPiCopiesRemainIndependent(t *testing.T) {
	t.Parallel()
	wantLo := new(big.Rat).Set(QuarterPiIv.Lo)
	wantHi := new(big.Rat).Set(QuarterPiIv.Hi)

	got := QuarterPiInterval()
	got.Lo.SetInt64(0)
	got.Hi.SetInt64(1)
	require.Zero(t, QuarterPiIv.Lo.Cmp(wantLo))
	require.Zero(t, QuarterPiIv.Hi.Cmp(wantHi))

	second := QuarterPiInterval()
	require.Zero(t, second.Lo.Cmp(wantLo))
	require.Zero(t, second.Hi.Cmp(wantHi))
	require.NotSame(t, got.Lo, second.Lo)
	require.NotSame(t, got.Hi, second.Hi)
}
