package tessellation

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBlindContactCoplanar(t *testing.T) {
	points := [][2]float64{{0, 0}, {2, 0}, {0, 2}, {0, -2}, {0, 1}, {3, 0}, {3, 2}, {0, 0}, {1, 1}}
	vertices := make([]blindContactVector, len(points))
	floats := make([]r3.Vec, len(points))
	for i, p := range points {
		floats[i] = r3.Vec{X: p[0], Y: p[1]}
		for k, value := range p {
			blindContactCoordinate(value, &vertices[i][k])
		}
	}
	for _, tc := range []struct {
		name   string
		x, y   [3]int
		shared [2]int
		count  int
		want   bool
	}{
		{"opposite edge", [3]int{0, 1, 2}, [3]int{1, 0, 3}, [2]int{0, 1}, 2, true},
		{"overlap at edge", [3]int{0, 1, 2}, [3]int{1, 0, 4}, [2]int{0, 1}, 2, false},
		{"separate", [3]int{0, 1, 2}, [3]int{1, 5, 6}, [2]int{1}, 1, true},
		{"unindexed equal", [3]int{0, 1, 2}, [3]int{7, 3, 5}, [2]int{}, 0, false},
		{"contained", [3]int{0, 1, 2}, [3]int{0, 4, 8}, [2]int{0}, 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
			var slots [8]blindContactInt
			got, err := a.coplanarContactAllowed(vertices, floats, tc.x, tc.y, tc.shared, tc.count, 0, 1, &slots)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestBlindContactCoplanarSharedEdgeWorkLimit(t *testing.T) {
	points := [][2]float64{{0, 0}, {2, 0}, {0, 2}, {0, -2}}
	vertices := make([]blindContactVector, len(points))
	floats := make([]r3.Vec, len(points))
	for i, p := range points {
		floats[i] = r3.Vec{X: p[0], Y: p[1]}
		for j, x := range p {
			blindContactCoordinate(x, &vertices[i][j])
		}
	}
	check := func(b *proofbound.ReplayBudget) (bool, error) {
		var slots [8]blindContactInt
		return (blindContactArithmetic{t.Context(), b}).coplanarContactAllowed(vertices, floats,
			[3]int{0, 1, 2}, [3]int{1, 0, 3}, [2]int{0, 1}, 2, 0, 1, &slots)
	}
	full := proofbound.NewReplayTrialBudget()
	got, err := check(full)
	require.NoError(t, err)
	require.True(t, got)
	work, _ := full.Usage()
	exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
	require.NoError(t, err)
	got, err = check(exact)
	require.NoError(t, err)
	require.True(t, got)
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	_, err = check(short)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
