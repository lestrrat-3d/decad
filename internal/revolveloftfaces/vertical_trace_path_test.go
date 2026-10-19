package revolveloftfaces_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/stretchr/testify/require"
)

func traceRay(y, z *big.Rat) revolveloftfaces.ShellRay {
	return revolveloftfaces.ShellRay{Y: y, Z: z, SeamIndex: -1}
}

// The line Y = 1 from Z = -1 to Z = 1 crosses the rays (4/5, -3/5), (1, 0)
// and (4/5, 3/5) at the exact source parameters 1/8, 1/2 and 7/8.
func verticalTraceLine() revolveloftfaces.VerticalTraceSourcePiece {
	return revolveloftfaces.VerticalTraceSourcePiece{
		Piece: loftface.VerticalPiece{Source: loftface.Interval{
			Lo: big.NewRat(0, 1), Hi: big.NewRat(1, 1)}},
		Y:    []*big.Rat{big.NewRat(1, 1), big.NewRat(1, 1)},
		Z:    []*big.Rat{big.NewRat(-1, 1), big.NewRat(1, 1)},
		From: big.NewRat(0, 1), To: big.NewRat(1, 1),
	}
}

func TestVerticalTraceSourceHits(t *testing.T) {
	rays := []revolveloftfaces.ShellRay{
		traceRay(big.NewRat(4, 5), big.NewRat(3, 5)),
		traceRay(big.NewRat(-1, 1), big.NewRat(0, 1)),
		traceRay(big.NewRat(1, 1), big.NewRat(0, 1)),
		traceRay(big.NewRat(4, 5), big.NewRat(-3, 5)),
	}
	t.Run("orders exact crossings by source parameter", func(t *testing.T) {
		hits, ok := revolveloftfaces.VerticalTraceSourceHits(verticalTraceLine(), rays, [2]int{-1, -1})
		require.True(t, ok)
		require.Len(t, hits, 3, "the ray with Y <= 0 is skipped")
		wantRays := []int{3, 2, 0}
		wantS := []*big.Rat{big.NewRat(1, 8), big.NewRat(1, 2), big.NewRat(7, 8)}
		for i, hit := range hits {
			require.Equal(t, wantRays[i], hit.Ray)
			require.Zero(t, hit.Lo.Cmp(wantS[i]), "hit %d lo = %s", i, hit.Lo.RatString())
			require.Zero(t, hit.Hi.Cmp(wantS[i]), "hit %d hi = %s", i, hit.Hi.RatString())
		}
	})
	t.Run("skips excluded endpoint rays", func(t *testing.T) {
		hits, ok := revolveloftfaces.VerticalTraceSourceHits(verticalTraceLine(), rays, [2]int{3, 0})
		require.True(t, ok)
		require.Len(t, hits, 1)
		require.Equal(t, 2, hits[0].Ray)
	})
	t.Run("reversed piece reverses the order", func(t *testing.T) {
		piece := verticalTraceLine()
		piece.From, piece.To = piece.To, piece.From
		hits, ok := revolveloftfaces.VerticalTraceSourceHits(piece, rays, [2]int{-1, -1})
		require.True(t, ok)
		require.Len(t, hits, 3)
		require.Equal(t, []int{0, 2, 3}, []int{hits[0].Ray, hits[1].Ray, hits[2].Ray})
	})
	t.Run("refuses two owners of one crossing", func(t *testing.T) {
		duplicate := append([]revolveloftfaces.ShellRay(nil), rays...)
		duplicate[0] = duplicate[2]
		_, ok := revolveloftfaces.VerticalTraceSourceHits(verticalTraceLine(), duplicate, [2]int{-1, -1})
		require.False(t, ok)
	})
	t.Run("refuses a ray through the path endpoint", func(t *testing.T) {
		endpoint := []revolveloftfaces.ShellRay{traceRay(big.NewRat(1, 1), big.NewRat(-1, 1))}
		_, ok := revolveloftfaces.VerticalTraceSourceHits(verticalTraceLine(), endpoint, [2]int{-1, -1})
		require.False(t, ok)
	})
}

func TestMeridianRayPoint(t *testing.T) {
	point, bound, ok := revolveloftfaces.MeridianRayPoint(
		[2]*big.Rat{big.NewRat(3, 1), big.NewRat(4, 1)}, big.NewRat(2, 1), big.NewRat(10, 1))
	require.True(t, ok)
	require.InDelta(t, 2.0, point.X, 0)
	require.InDelta(t, 6.0, point.Y, 1e-12)
	require.InDelta(t, 8.0, point.Z, 1e-12)
	require.GreaterOrEqual(t, bound, 0.0)
	require.Less(t, bound, 1e-12)
	_, _, ok = revolveloftfaces.MeridianRayPoint(
		[2]*big.Rat{big.NewRat(3, 1), big.NewRat(4, 1)}, big.NewRat(2, 1), big.NewRat(0, 1))
	require.False(t, ok, "a zero radius has no meridian point")
}
