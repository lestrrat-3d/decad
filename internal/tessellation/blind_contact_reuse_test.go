package tessellation

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// BlindContactReuseFixtureForTest is shared with the independent loft audit.
func BlindContactReuseFixtureForTest() ([]r3.Vec, [][3]int) {
	const wide = 0x1p31
	return []r3.Vec{
		{}, {X: wide}, {Y: wide},
		{X: wide + 1024, Y: 1, Z: -1}, {X: wide + 1024, Y: 1, Z: 1}, {X: wide + 2048, Y: 1},
		{}, {X: 2}, {Y: 2}, {Y: -2},
		{}, {X: 4}, {Y: 4}, {X: 1, Y: 1, Z: -1}, {X: 1, Y: 1, Z: 1}, {X: 2, Y: 1},
	}, [][3]int{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {7, 6, 9}, {10, 11, 12}, {13, 14, 15}}
}

func requireBlindScratchWidths(t *testing.T, s *blindContactScratch) {
	t.Helper()
	check := func(x *blindContactInt) {
		if x.known {
			requireBlindContactWidth(t, x)
			return
		}
		for _, limb := range x.limbs {
			require.Zero(t, limb)
		}
	}
	for i := range s.points {
		for j := range s.points[i] {
			for k := range s.points[i][j] {
				check(&s.points[i][j][k])
			}
		}
	}
	for i := range s.distances {
		for j := range s.distances[i] {
			check(&s.distances[i][j])
		}
	}
	for i := range s.endpoints {
		for j := range s.endpoints[i] {
			for k := range s.endpoints[i][j] {
				check(&s.endpoints[i][j][k])
			}
		}
	}
	for i := range s.products {
		check(&s.products[i])
	}
	for i := range s.orient {
		check(&s.orient[i])
	}
	for i := range s.dot {
		check(&s.dot[i])
	}
	for i := range s.projected {
		for j := range s.projected[i] {
			check(&s.projected[i][j])
		}
	}
	for i := range s.vectors {
		for j := range s.vectors[i] {
			check(&s.vectors[i][j])
		}
	}
	check(&s.zero)
}

func TestBlindContactScratchReuse(t *testing.T) {
	points, initial := BlindContactReuseFixtureForTest()
	vertices := make([]blindContactVector, len(points))
	for i, p := range points {
		for j, x := range [...]float64{p.X, p.Y, p.Z} {
			require.True(t, blindContactCoordinate(x, &vertices[i][j]))
		}
	}
	for left := range 2 {
		for right := range 2 {
			tris := append([][3]int(nil), initial...)
			for i := range 3 {
				if left != 0 {
					tris[2*i][1], tris[2*i][2] = tris[2*i][2], tris[2*i][1]
				}
				if right != 0 {
					tris[2*i+1][1], tris[2*i+1][2] = tris[2*i+1][2], tris[2*i+1][1]
				}
			}
			normals := make([]blindContactVector, len(tris))
			setup := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
			var setupScratch blindContactScratch
			for i, tri := range tris {
				require.NoError(t, setup.contactNormal(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]], &normals[i], &setupScratch))
			}
			run := func(ctx context.Context, budget *proofbound.ReplayBudget, check bool) ([3]bool, error) {
				var got [3]bool
				var scratch blindContactScratch
				a := blindContactArithmetic{ctx, budget}
				for i := range 3 {
					var err error
					got[i], err = a.contactPair(vertices, points, normals, tris[2*i], tris[2*i+1], 2*i, 2*i+1, &scratch)
					if err != nil {
						return got, err
					}
					if check {
						requireBlindScratchWidths(t, &scratch)
						var fresh blindContactScratch
						want, err := (blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}).contactPair(
							vertices, points, normals, tris[2*i], tris[2*i+1], 2*i, 2*i+1, &fresh)
						require.NoError(t, err)
						require.Equal(t, want, got[i])
					}
				}
				return got, nil
			}
			full := proofbound.NewReplayTrialBudget()
			got, err := run(t.Context(), full, true)
			require.NoError(t, err)
			require.Equal(t, [3]bool{true, true, false}, got)
			work, _ := full.Usage()
			exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
			require.NoError(t, err)
			got, err = run(t.Context(), exact, false)
			require.NoError(t, err)
			require.Equal(t, [3]bool{true, true, false}, got)
			short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
			require.NoError(t, err)
			_, err = run(t.Context(), short, false)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			_, err = run(ctx, proofbound.NewReplayTrialBudget(), false)
			require.ErrorIs(t, err, context.Canceled)
			base, stop := context.WithCancel(t.Context())
			polling := &replayCloseCancel{Context: base, cancel: stop}
			_, err = run(polling, proofbound.NewReplayTrialBudget(), false)
			stop()
			require.ErrorIs(t, err, context.Canceled)
		}
	}
}
