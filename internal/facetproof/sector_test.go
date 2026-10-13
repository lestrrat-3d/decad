package facetproof_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSectorSourcesSeparate(t *testing.T) {
	a := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0}, {X: 0, Y: 0, Z: 2}}
	b := []r3.Vec{{X: 0, Y: 2, Z: 2}, {X: 0, Y: 1.2, Z: 2}, {X: 0, Y: 2, Z: 1.2}}
	shifted := make([]r3.Vec, len(a))
	for i, v := range a {
		shifted[i] = r3.Vec{X: v.X + 10, Y: v.Y, Z: v.Z}
	}
	source := func(verts []r3.Vec, mesh, bound float64) facetproof.SectorSource {
		return facetproof.SectorSource{Verts: verts, MeshBound: mesh, Bound: bound}
	}

	t.Run("projection separates within margin", func(t *testing.T) {
		require.True(t, facetproof.SectorSourcesSeparate(
			[]facetproof.SectorSource{source(a, 0.3, 0.3), source(b, 0.3, 0.3)}))
	})
	t.Run("projection refuses past margin", func(t *testing.T) {
		require.False(t, facetproof.SectorSourcesSeparate(
			[]facetproof.SectorSource{source(a, 0.5, 0.5), source(b, 0.5, 0.5)}))
	})
	t.Run("box path", func(t *testing.T) {
		require.True(t, facetproof.SectorSourcesSeparate(
			[]facetproof.SectorSource{source(a, 0.5, 0.5), source(shifted, 0.5, 0.5)}))
	})
	t.Run("refusals", func(t *testing.T) {
		nanVerts := []r3.Vec{{X: math.NaN()}, {Y: 1}}
		cases := map[string]facetproof.SectorSource{
			"empty verts":     source(nil, 0.1, 0.1),
			"negative mesh":   source(shifted, -1, 0.1),
			"NaN bound":       source(shifted, 0.1, math.NaN()),
			"NaN vertex":      source(nanVerts, 0.1, 0.1),
			"infinite bound":  source(shifted, 0.1, math.Inf(1)),
			"infinite mesh":   source(shifted, math.Inf(1), 0.1),
			"NaN mesh":        source(shifted, math.NaN(), 0.1),
			"infinite vertex": source([]r3.Vec{{Y: math.Inf(1)}}, 0.1, 0.1),
		}
		for name, bad := range cases {
			t.Run(name, func(t *testing.T) {
				require.False(t, facetproof.SectorSourcesSeparate(
					[]facetproof.SectorSource{source(a, 0.1, 0.1), bad}))
			})
		}
	})
}
