package circularbounds_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSharedCircleTurnMatchesIndependentEndpointIntervals(t *testing.T) {
	first, ok := circularbounds.PrepareEndpoint(circularbounds.CircleSeg{
		Center: circularbounds.Point2{U: math.Nextafter(3, 4), V: -2}, Radius: units.Millimeters(2.5),
	})
	require.True(t, ok)
	second, ok := circularbounds.PrepareEndpoint(circularbounds.CircleSeg{
		Center: circularbounds.Point2{U: -7, V: math.Nextafter(1, 2)}, Radius: units.Millimeters(0.125),
	})
	require.True(t, ok)
	require.True(t, first.CanShareCircleTurn(second))

	largeDen := new(big.Int).Lsh(big.NewInt(1), 220)
	params := []*big.Rat{
		big.NewRat(0, 1), big.NewRat(1, 4), big.NewRat(1, 3), big.NewRat(3, 4),
		new(big.Rat).SetFrac(new(big.Int).Sub(largeDen, big.NewInt(1)), largeDen),
		new(big.Rat).SetFrac(big.NewInt(1), largeDen),
	}
	for _, param := range params {
		before := param.RatString()
		wantU0, wantV0, ok0 := first.At(param)
		wantU1, wantV1, ok1 := second.At(param)
		require.True(t, ok0 && ok1)
		gotU0, gotV0, gotU1, gotV1 := first.AtSharedCircleTurn(second, param)
		for i, pair := range [][2]proofbound.RatInterval{
			{gotU0, wantU0}, {gotV0, wantV0}, {gotU1, wantU1}, {gotV1, wantV1},
		} {
			require.Zero(t, pair[0].Lo.Cmp(pair[1].Lo), "lower interval %d at %s", i, before)
			require.Zero(t, pair[0].Hi.Cmp(pair[1].Hi), "upper interval %d at %s", i, before)
		}
		require.Equal(t, before, param.RatString(), "the shared proof must not change its input")
	}

	memo := proofbound.NewTurnTrigMemo()
	for _, m := range []int{32, 281, 345} {
		for _, k := range []int{1, m / 4, m - 1} {
			for _, param := range []*big.Rat{
				big.NewRat(int64(k), int64(m)),
				new(big.Rat).Sub(big.NewRat(1, 1), big.NewRat(int64(k), int64(m))),
			} {
				before := param.RatString()
				wantU0, wantV0, wantU1, wantV1 := first.AtSharedCircleTurn(second, param)
				gotU0, gotV0, gotU1, gotV1 := first.AtSharedCircleTurnWithMemo(second, param, memo)
				for i, pair := range [][2]proofbound.RatInterval{
					{gotU0, wantU0}, {gotV0, wantV0}, {gotU1, wantU1}, {gotV1, wantV1},
				} {
					require.Zero(t, pair[0].Lo.Cmp(pair[1].Lo), "m=%d interval=%d lower at %s", m, i, before)
					require.Zero(t, pair[0].Hi.Cmp(pair[1].Hi), "m=%d interval=%d upper at %s", m, i, before)
				}
				require.Equal(t, before, param.RatString(), "m=%d input turn", m)
			}
		}
	}
	turn := big.NewRat(1, 281)
	wantU0, wantV0, wantU1, wantV1 := first.AtSharedCircleTurn(second, turn)
	gotU0, gotV0, gotU1, gotV1 := first.AtSharedCircleTurnWithMemo(second, turn, memo)
	for _, interval := range []*proofbound.RatInterval{&gotU0, &gotV0, &gotU1, &gotV1} {
		interval.Lo.SetInt64(123)
		interval.Hi.SetInt64(456)
	}
	againU0, againV0, againU1, againV1 := first.AtSharedCircleTurnWithMemo(second, turn, memo)
	for i, pair := range [][2]proofbound.RatInterval{
		{againU0, wantU0}, {againV0, wantV0}, {againU1, wantU1}, {againV1, wantV1},
	} {
		require.Zero(t, pair[0].Lo.Cmp(pair[1].Lo), "owned interval %d lower", i)
		require.Zero(t, pair[0].Hi.Cmp(pair[1].Hi), "owned interval %d upper", i)
	}
	independentU0, independentV0, ok0 := first.At(turn)
	independentU1, independentV1, ok1 := second.At(turn)
	require.True(t, ok0 && ok1)
	for i, pair := range [][2]proofbound.RatInterval{
		{independentU0, wantU0}, {independentV0, wantV0},
		{independentU1, wantU1}, {independentV1, wantV1},
	} {
		require.Zero(t, pair[0].Lo.Cmp(pair[1].Lo), "endpoint %d lower after result mutation", i)
		require.Zero(t, pair[0].Hi.Cmp(pair[1].Hi), "endpoint %d upper after result mutation", i)
	}

	zero, ok := circularbounds.PrepareEndpoint(circularbounds.CircleSeg{Radius: units.Millimeters(0)})
	require.True(t, ok)
	arc, ok := circularbounds.PrepareEndpoint(circularbounds.ArcSeg{
		Center: circularbounds.Point2{}, Start: circularbounds.Point2{U: 1}, End: circularbounds.Point2{V: 1},
	})
	require.True(t, ok)
	require.False(t, first.CanShareCircleTurn(zero))
	require.False(t, first.CanShareCircleTurn(arc))
}
