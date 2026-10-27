package smoothloft

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func twistTestPoint(x, y int64) freeform.RatPoint {
	return freeform.RatPoint{U: rint(x), V: rint(y)}
}

// twistTestLinePatch is the patch of a line from a to b that is fixed in
// height: both height spans see the same segment.
func twistTestLinePatch(a, b freeform.RatPoint) twistPatch {
	span := twistLineSpan(a, b)
	return twistPatchOf(span, span, twistTestZeroSlopes(), twistTestZeroSlopes(), rint(1))
}

func twistTestZeroSlopes() twistSpan {
	z := twistTestPoint(0, 0)
	return twistSpan{z, z, z, z}
}

func TestTwistedPatchPolarProof(t *testing.T) {
	t.Parallel()
	zero := twistTestZeroSlopes()
	prove := func(a, b freeform.RatPoint) error {
		span := twistLineSpan(a, b)
		patch := twistPatchOf(span, span, zero, zero, rint(1))
		return patch.provePolarPositive(t.Context(), proofbound.NewWorkBudget(t.Context()))
	}
	// A counterclockwise edge seen from the origin is admitted.
	require.NoError(t, prove(twistTestPoint(4, -2), twistTestPoint(4, 2)))
	// The same edge walked clockwise, and an edge through the origin, refuse.
	require.ErrorIs(t, prove(twistTestPoint(4, 2), twistTestPoint(4, -2)), decaderr.ErrUnsupported)
	require.ErrorIs(t, prove(twistTestPoint(-4, -2), twistTestPoint(4, 2)), decaderr.ErrUnsupported)
}

func TestTwistedPatchEvaluatesRecordedSections(t *testing.T) {
	t.Parallel()
	a, b := twistTestPoint(4, -2), twistTestPoint(4, 2)
	c, d := twistTestPoint(0, 5), twistTestPoint(-1, 7)
	first, second := twistLineSpan(a, b), twistLineSpan(c, d)
	// With zero slopes the blend still meets both sections at its ends.
	zero := twistTestZeroSlopes()
	patch := twistPatchOf(first, second, zero, zero, rint(1))
	for _, tc := range []struct {
		u, t int64
		want freeform.RatPoint
	}{{0, 0, a}, {1, 0, b}, {0, 1, c}, {1, 1, d}} {
		got := patch.At(rint(tc.u), rint(tc.t))
		require.Zero(t, got.U.Cmp(tc.want.U))
		require.Zero(t, got.V.Cmp(tc.want.V))
	}
}

func TestTwistedSlopesUseExactSpacing(t *testing.T) {
	t.Parallel()
	points := []freeform.RatPoint{twistTestPoint(0, 0), twistTestPoint(1, 0), twistTestPoint(4, 0)}
	zs := []*big.Rat{rint(0), rint(1), rint(3)}
	slopes := twistSlopes(points, zs)
	require.Zero(t, slopes[0].U.Cmp(rint(1)))
	require.Zero(t, slopes[1].U.Cmp(big.NewRat(4, 3)))
	require.Zero(t, slopes[2].U.Cmp(big.NewRat(3, 2)))
}

func TestTwistedCountForMeetsLimit(t *testing.T) {
	t.Parallel()
	limit := big.NewRat(1, 300)
	for _, x := range []*big.Rat{rint(0), big.NewRat(1, 2), rint(1), big.NewRat(37, 10)} {
		n := twistCountFor(x, limit)
		require.GreaterOrEqual(t, n, 1)
		require.LessOrEqual(t, div(x, mul(rint(8), rint(int64(n*n)))).Cmp(limit), 0)
		if n > 1 {
			require.Greater(t, div(x, mul(rint(8), rint(int64((n-1)*(n-1))))).Cmp(limit), 0)
		}
	}
}

func TestTwistedBoundsOfLinePatch(t *testing.T) {
	t.Parallel()
	b := twistTestLinePatch(twistTestPoint(4, -2), twistTestPoint(4, 2)).bounds()
	require.Zero(t, b.muu.Sign())
	require.Zero(t, b.mtt.Sign())
	require.Zero(t, b.mut.Sign())
	require.Zero(t, b.bu.Cmp(rint(4)))
}
