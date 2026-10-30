package twistsweep

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestSumTwistOffsetAreasMatchesSequentialAddition(t *testing.T) {
	var terms [64]proofbound.RatInterval
	want := proofbound.PointInterval(new(big.Rat))
	for index := range terms {
		lo := new(big.Rat).SetFrac64(int64(index+1), int64(index+67))
		hi := new(big.Rat).Add(lo, new(big.Rat).SetFrac64(1, int64(index+101)))
		terms[index] = proofbound.Interval(lo, hi)
		want = proofbound.IntervalAdd(want, terms[index])
	}

	got := sumTwistOffsetAreas(terms)
	require.Zero(t, got.Lo.Cmp(want.Lo))
	require.Zero(t, got.Hi.Cmp(want.Hi))
}
