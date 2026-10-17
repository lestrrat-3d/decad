package decad

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func loftOutputAdversarialPoint(u, v int64) loftPartitionPoint {
	return loftPartitionPoint{u: big.NewRat(u, 1), v: big.NewRat(v, 1)}
}

func TestLoftOutputCapConcaveEscapingDiagonal(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 1), p(1, 1), p(1, 4), p(0, 4)}
	budget := proofbound.NewWorkBudget(t.Context())

	// Both endpoints lie on the boundary, but the diagonal crosses the missing corner.
	inside, err := loftOutputEdgeInside(budget, polygon[2], polygon[4], polygon)
	require.NoError(t, err)
	require.False(t, inside)

	tri := [3]loftPartitionPoint{polygon[1], polygon[2], polygon[4]}
	inside, err = loftOutputTriangleInside(budget, polygon, tri)
	require.NoError(t, err)
	require.False(t, inside)

	inside, err = loftOutputCertifyCapTiling(budget, polygon, [][3]int{{1, 2, 4}})
	require.NoError(t, err)
	require.False(t, inside)
}

func TestLoftOutputCapEqualAreaOverlapAndGap(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 2), p(4, 4), p(0, 4), p(0, 2)}
	tris := [][3]int{{0, 1, 3}, {0, 2, 4}}
	budget := proofbound.NewWorkBudget(t.Context())

	// The two distinct triangles each cover half the polygon's area, but overlap
	// near the center and leave another region uncovered.
	require.Zero(t, new(big.Rat).Sub(
		new(big.Rat).Add(loftPartitionCross(polygon[0], polygon[1], polygon[3]),
			loftPartitionCross(polygon[0], polygon[2], polygon[4])),
		new(big.Rat).Mul(loftOutputPolygonArea(polygon), big.NewRat(2, 1))).Sign())
	for _, tri := range tris {
		inside, err := loftOutputTriangleInside(budget, polygon,
			[3]loftPartitionPoint{polygon[tri[0]], polygon[tri[1]], polygon[tri[2]]})
		require.NoError(t, err)
		require.True(t, inside)
	}
	valid, err := loftOutputCertifyCapTiling(budget, polygon, tris)
	require.NoError(t, err)
	require.False(t, valid)
}

func TestLoftOutputCapCollinearBoundarySplit(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(2, 0), p(4, 0), p(4, 4), p(0, 4)}
	budget := proofbound.NewWorkBudget(t.Context())

	inside, err := loftOutputEdgeInside(budget, polygon[0], polygon[2], polygon)
	require.NoError(t, err)
	require.True(t, inside)

	// The first tiling matches both collinear boundary segments. The second
	// covers the same area but has an unsplit edge over the boundary vertex.
	valid, err := loftOutputCertifyCapTiling(budget, polygon,
		[][3]int{{4, 0, 1}, {4, 1, 2}, {4, 2, 3}})
	require.NoError(t, err)
	require.True(t, valid)
	valid, err = loftOutputCertifyCapTiling(budget, polygon,
		[][3]int{{0, 2, 3}, {0, 3, 4}})
	require.NoError(t, err)
	require.False(t, valid)
}

func TestLoftOutputCapPredicatesCancelInFlight(t *testing.T) {
	p := loftOutputAdversarialPoint
	polygon := []loftPartitionPoint{p(0, 0), p(4, 0), p(4, 4), p(0, 4)}
	for _, tc := range []struct {
		name string
		call func(*proofbound.WorkBudget) (bool, error)
	}{
		{"edge", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputEdgeInside(b, polygon[0], polygon[2], polygon)
		}},
		{"triangle", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputTriangleInside(b, polygon,
				[3]loftPartitionPoint{polygon[0], polygon[1], polygon[2]})
		}},
		{"tiling", func(b *proofbound.WorkBudget) (bool, error) {
			return loftOutputCertifyCapTiling(b, polygon, [][3]int{{0, 1, 2}, {0, 2, 3}})
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(t.Context())
			defer cancel()
			steps := 0
			budget := &proofbound.WorkBudget{
				StepFn: func() error {
					steps++
					if steps == 3 {
						cancel()
					}
					return ctx.Err()
				},
				ErrFn: ctx.Err,
			}
			valid, err := tc.call(budget)
			require.ErrorIs(t, err, context.Canceled)
			require.False(t, valid)
			require.Equal(t, 3, steps)
		})
	}
}
