package tessellation_test

import (
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBlindContactPairDifferential(t *testing.T) {
	rng := rand.New(rand.NewSource(87923))
	accepted, refused := 0, 0
	for shared := range 3 {
		for iteration := range 600 {
			vertices := make([]r3.Vec, 6-shared)
			for i := range vertices {
				vertices[i] = r3.Vec{X: float64(rng.Intn(9) - 4), Y: float64(rng.Intn(9) - 4), Z: float64(rng.Intn(9) - 4)}
				if iteration >= 300 {
					vertices[i].Z = 0
				}
			}
			tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
			for i := range shared {
				tris[1][i] = tris[0][i]
			}
			for i := shared; i < 3; i++ {
				tris[1][i] = 3 + i - shared
			}
			if rng.Intn(2) == 0 {
				tris[0][1], tris[0][2] = tris[0][2], tris[0][1]
			}
			if rng.Intn(2) == 0 {
				tris[1][1], tris[1][2] = tris[1][2], tris[1][1]
			}
			got, err := tessellation.BlindContactPairForTest(t.Context(), vertices, tris)
			if err != nil {
				continue
			}
			_, refErr := loftmesh.AuditLoftPairData(loftmesh.NewLoftAuditData(vertices, tris), tris, 0, 1, loftmesh.LoftAuditShortcuts{})
			require.Equal(t, refErr == nil, got, "shared=%d vertices=%v triangles=%v", shared, vertices, tris)
			if got {
				accepted++
			} else {
				refused++
			}
		}
	}
	require.Greater(t, accepted, 100)
	require.Positive(t, refused)
}

func TestBlindContactPairCasesBothWindings(t *testing.T) {
	vertices := []r3.Vec{
		{}, {X: 4}, {Y: 4}, {X: 4, Y: -4},
		{X: 1, Y: 1, Z: -1}, {X: 1, Y: 1, Z: 1}, {X: 2, Y: 1},
		{Z: 1}, {X: 4, Z: 1},
		{},
		{X: 1, Y: 1}, {X: 2, Y: 1}, {X: 1, Y: 2}, {Y: 4, Z: 1},
	}
	for _, tc := range []struct {
		name string
		tri  [3]int
		want bool
	}{
		{"shared edge opposite sides", [3]int{1, 0, 3}, true},
		{"shared edge same side", [3]int{0, 1, 6}, false},
		{"noncoplanar shared edge", [3]int{0, 1, 7}, true},
		{"shared vertex", [3]int{0, 7, 8}, true},
		{"noncoplanar crossing", [3]int{4, 5, 6}, false},
		{"coplanar containment", [3]int{10, 11, 12}, false},
		{"unindexed equal point", [3]int{9, 7, 8}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			for left := range 2 {
				for right := range 2 {
					tris := [][3]int{{0, 1, 2}, tc.tri}
					if left != 0 {
						tris[0][1], tris[0][2] = tris[0][2], tris[0][1]
					}
					if right != 0 {
						tris[1][1], tris[1][2] = tris[1][2], tris[1][1]
					}
					got, err := tessellation.BlindContactPairForTest(t.Context(), vertices, tris)
					require.NoError(t, err)
					require.Equal(t, tc.want, got)
					_, refErr := loftmesh.AuditLoftPairData(loftmesh.NewLoftAuditData(vertices, tris), tris, 0, 1, loftmesh.LoftAuditShortcuts{})
					require.Equal(t, tc.want, refErr == nil)
				}
			}
		})
	}
	tris := [][3]int{{0, 1, 2}, {7, 8, 13}}
	got, err := tessellation.BlindContactPairForTest(t.Context(), vertices, tris)
	require.True(t, got)
	require.NoError(t, err)
	_, refErr := loftmesh.AuditLoftPairData(loftmesh.NewLoftAuditData(vertices, tris), tris, 0, 1, loftmesh.LoftAuditShortcuts{})
	require.NoError(t, refErr)
}

func TestBlindContactScratchReuseReference(t *testing.T) {
	vertices, initial := tessellation.BlindContactReuseFixtureForTest()
	for left := range 2 {
		for right := range 2 {
			tris := append([][3]int(nil), initial...)
			for i, want := range [...]bool{true, true, false} {
				if left != 0 {
					tris[2*i][1], tris[2*i][2] = tris[2*i][2], tris[2*i][1]
				}
				if right != 0 {
					tris[2*i+1][1], tris[2*i+1][2] = tris[2*i+1][2], tris[2*i+1][1]
				}
				pair := [][3]int{tris[2*i], tris[2*i+1]}
				_, err := loftmesh.AuditLoftPairData(loftmesh.NewLoftAuditData(vertices, pair), pair, 0, 1, loftmesh.LoftAuditShortcuts{})
				require.Equal(t, want, err == nil)
			}
		}
	}
}
