package facetproof_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/stretchr/testify/require"
)

// sphereFacets is the facet list of a closed UV sphere with stacks rings
// between two poles of degree slices, in uvSphere's vertex numbering.
func sphereFacets(stacks, slices int) [][3]int {
	ring := func(i, j int) int { return 1 + (i-1)*slices + j%slices }
	south := 1 + (stacks-1)*slices
	var tris [][3]int
	for j := range slices {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
	}
	for i := 1; i < stacks-1; i++ {
		for j := range slices {
			a, b, c, d := ring(i, j), ring(i, j+1), ring(i+1, j+1), ring(i+1, j)
			tris = append(tris, [3]int{a, d, c}, [3]int{a, c, b})
		}
	}
	for j := range slices {
		tris = append(tris, [3]int{south, ring(stacks-1, j+1), ring(stacks-1, j)})
	}
	return tris
}

func BenchmarkClosedFacetEdges(b *testing.B) {
	for _, size := range [][2]int{{6, 8}, {64, 96}, {80, 128}} {
		tris := sphereFacets(size[0], size[1])
		b.Run(fmt.Sprintf("sphere/%d", len(tris)), func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				neighbors, err := facetproof.ClosedFacetEdges(b.Context(), tris)
				require.NoError(b, err)
				require.Len(b, neighbors, 3*len(tris))
			}
		})
	}
}
