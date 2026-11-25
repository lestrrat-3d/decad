package meshbool

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestParityIndexListsEveryFacetThatMattersOnCoarseGrids runs the facet
// contract check of TestParityIndexListsEveryFacetThatMatters on every grid
// below the first one that the coarse retry can settle on, for each fixture
// and for two small rings whose facets span many columns and rows.
func TestParityIndexListsEveryFacetThatMattersOnCoarseGrids(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x31d8, 0x0c56))
	fixtures := parityIndexFixtures()
	v, tr := parityRing(256, 1)
	fixtures = append(fixtures, parityIndexFixture{`tall ring`, v, tr})
	v, tr = parityRing(128, 6)
	fixtures = append(fixtures, parityIndexFixture{`banded ring`, v, tr})
	for _, fx := range fixtures {
		queries := parityCoarseQueries(fx, 0)
		if len(fx.tris) > 1000 {
			queries = parityRingQueries(rng, fx.verts, 120)
		}
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			inners := parityGridInners(len(fx.tris))
			require.Greater(t, len(inners), 1, `the fixture has a coarse grid`)
			pruned, mattered := 0, 0
			for _, inner := range inners[1:] {
				for _, ray := range AxisRays {
					pm := NewParityMesh(fx.verts, fx.tris)
					idx, ok := parityIndexOnGrid(pm, ray.Axis, ray.U, ray.V, inner)
					require.True(t, ok, `grid %d axis %d`, inner, ray.Axis)
					group := flatGroupFirst(pm, ray.Axis, ray.U, ray.V)
					for qi, p := range queries {
						m, pr := requireLeftOutFacetsEndOutside(t, pm, idx, group, ray.Axis, ray.U, ray.V, ray.Dir, p, qi)
						mattered += m
						pruned += pr
					}
				}
			}
			require.Positive(t, mattered, `the queries must reach facets`)
			require.Positive(t, pruned, `the coarse indexes must prune`)
		})
	}
}

// TestParityIndexBuildPinsTheCoarseGridLists pins the start and ids slices of
// the last (coarsest) grid the retry can settle on, for every differential
// fixture and two small rings on each swept axis. The hashes pin the coarse
// grids; TestParityIndexBuildPinsTheFixtureLists pins the first grid.
func TestParityIndexBuildPinsTheCoarseGridLists(t *testing.T) {
	t.Parallel()
	type pin struct {
		sum    uint64
		ns, ni int
	}
	// The rows follow the fixtures in order: the five differential fixtures,
	// then the tall and the banded ring.
	want := [][3]pin{
		{{0x86addd8d255d6698, 50, 387}, {0x2ae2296f3046bf22, 50, 387}, {0xdc5e32f37e458408, 50, 576}},
		{{0x3605c926cc4d3e9a, 50, 1251}, {0xd12d6051adda8f16, 50, 1214}, {0xcb8a0184efe2aa0d, 50, 1256}},
		{{0x1ff8e7e15c8f6bec, 65, 1517}, {0x4be18b5274a333e3, 65, 1142}, {0xd95be0cac3108eaa, 65, 1134}},
		{{0xb6bd33e18b6fd12e, 37, 606}, {0xc6208f174f0b2811, 37, 427}, {0xf4ffc6c5a610ee9e, 37, 408}},
		{{0x96a94402e8592947, 37, 594}, {0xa091a9608b0eb99a, 37, 410}, {0x68c9915c600c8fa3, 37, 394}},
		{{0x4dab3fb1454aa526, 50, 6299}, {0x56f6e345a6372a58, 50, 6299}, {0x77f0f46f070dd614, 50, 8012}},
		{{0x262633cef2d7f9ad, 82, 7187}, {0x17e22ea9498199a5, 82, 7187}, {0xda23a630329507c2, 82, 5172}},
	}
	fixtures := parityIndexFixtures()
	v, tr := parityRing(256, 1)
	fixtures = append(fixtures, parityIndexFixture{`tall ring`, v, tr})
	v, tr = parityRing(128, 6)
	fixtures = append(fixtures, parityIndexFixture{`banded ring`, v, tr})
	require.Len(t, fixtures, len(want))
	for fi, fx := range fixtures {
		inners := parityGridInners(len(fx.tris))
		require.Greater(t, len(inners), 1, fx.name)
		last := inners[len(inners)-1]
		for i, ax := range []struct{ axis, u, v int }{{0, 1, 2}, {1, 2, 0}, {2, 0, 1}} {
			idx, ok := parityIndexOnGrid(NewParityMesh(fx.verts, fx.tris), ax.axis, ax.u, ax.v, last)
			require.True(t, ok, `%s axis %d`, fx.name, ax.axis)
			sum, ns, ni := parityBuildHash(idx)
			require.Equal(t, want[fi][i], pin{sum, ns, ni}, `%s grid %d axis %d`, fx.name, last, ax.axis)
		}
	}
}
