package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPreparedPlaneSeparationMatchesUnprepared(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x8c4d2af1, 0x719cf4ad))
	var verts []r3.Vec
	var tris [][3]int
	for i := range 60 {
		x, y := 20*rng.Float64()-10, 20*rng.Float64()-10
		z := 0.0
		switch i % 3 {
		case 1:
			z = math.Ldexp(1, -45)
		case 2:
			z = 20*rng.Float64() - 10
		}
		start := len(verts)
		verts = append(verts,
			r3.Vec{X: x, Y: y, Z: z},
			r3.Vec{X: x + 1, Y: y, Z: z},
			r3.Vec{X: x, Y: y + 1, Z: z},
		)
		tris = append(tris, [3]int{start, start + 1, start + 2})
	}
	data := NewLoftAuditData(verts, tris)
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			want := LoftPlaneSeparated(data.Corners[i], data.Corners[j])
			require.Equal(t, want, loftPlaneSeparatedPrepared(data, i, j), "pair %d,%d", i, j)
		}
	}
}
