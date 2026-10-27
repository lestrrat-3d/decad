package meshbool

import (
	"encoding/binary"
	"fmt"
	"hash/fnv"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// parityBuildHash folds the built index's start and ids slices into one hash.
func parityBuildHash(idx *ParityIndex) (uint64, int, int) {
	h := fnv.New64a()
	var buf [4]byte
	for _, s := range [][]int32{idx.start, idx.ids} {
		for _, x := range s {
			binary.LittleEndian.PutUint32(buf[:], uint32(x))
			_, _ = h.Write(buf[:])
		}
		_, _ = h.Write([]byte{0xff, 0xff, 0xff, 0xff})
	}
	return h.Sum64(), len(idx.start), len(idx.ids)
}

// TestParityIndexBuildIsStable pins the exact start and ids slices built for a
// 6000-facet closed shell on each swept axis. Under the race detector it pins
// those of a 600-facet shell instead.
func TestParityIndexBuildIsStable(t *testing.T) {
	t.Parallel()
	sides, starts := 1500, 24650
	want := []uint64{0xe233565e4203299, 0xb0a2b33a9c112095, 0x7f885a5cd47948f}
	if raceDetector {
		sides, starts = 150, 2602
		want = []uint64{0x90b4a666739748ba, 0x3636e202af0d22b2, 0x1769950073261d4d}
	}
	verts, tris := parityCylinder(sides)
	pm := NewParityMesh(verts, tris)
	for i, ax := range []struct{ axis, u, v int }{{0, 1, 2}, {1, 2, 0}, {2, 0, 1}} {
		idx := pm.buildParityIndex(ax.axis, ax.u, ax.v)
		require.NotNil(t, idx)
		sum, ns, ni := parityBuildHash(idx)
		require.Equal(t, want[i], sum, "axis %d", ax.axis)
		require.Equal(t, starts, ns)
		require.Positive(t, ni)
	}
}

// TestParityIndexBuildPinsTheFixtureLists pins the start and ids slices of
// every differential fixture on each swept axis, as one append per
// membership followed by a counting sort builds them.
func TestParityIndexBuildPinsTheFixtureLists(t *testing.T) {
	t.Parallel()
	want := map[string][3]struct {
		sum    uint64
		ns, ni int
	}{
		`cylinder`:     {{0x7a0fb474ef3f702e, 485, 1979}, {0xe47bf711e5b78cb4, 485, 1979}, {0x52231c5016c4644f, 485, 3496}},
		`lattice`:      {{0xf7226e03325b490, 577, 7776}, {0xe5f11037b002436b, 577, 8976}, {0x4064da09c4d649ac, 577, 9872}},
		`slanted`:      {{0x10b211fd28025785, 730, 8032}, {0x801f71f93fdf55d1, 730, 7830}, {0x37da594e2d2690c2, 730, 7789}},
		`slanted huge`: {{0xb7bbe6a6b76ab6a3, 442, 3125}, {0xeb7919603e46acbe, 442, 3025}, {0xcc09efa321f31391, 442, 2908}},
		`slanted tiny`: {{0x7fb2cae08056f284, 442, 2925}, {0x8f0b8a0262a919, 442, 2617}, {0x1c9cf79ee3f42ce, 442, 2634}},
	}
	fixtures := parityIndexFixtures()
	require.Len(t, fixtures, len(want))
	for _, fx := range fixtures {
		pins, ok := want[fx.name]
		require.True(t, ok, fx.name)
		for i, ax := range []struct{ axis, u, v int }{{0, 1, 2}, {1, 2, 0}, {2, 0, 1}} {
			idx := NewParityMesh(fx.verts, fx.tris).buildParityIndex(ax.axis, ax.u, ax.v)
			require.NotNil(t, idx, `%s axis %d`, fx.name, ax.axis)
			sum, ns, ni := parityBuildHash(idx)
			require.Equal(t, pins[i].sum, sum, `%s axis %d`, fx.name, ax.axis)
			require.Equal(t, pins[i].ns, ns, `%s axis %d`, fx.name, ax.axis)
			require.Equal(t, pins[i].ni, ni, `%s axis %d`, fx.name, ax.axis)
		}
	}
}

// TestParityIndexFillMatchesTheMembershipSort expands every placement's runs
// into single (cell, facet) memberships in facet order, counting-sorts them
// one membership at a time, and requires the result to equal the lists fill
// builds from the runs. The fixtures hold runs along v and along u.
func TestParityIndexFillMatchesTheMembershipSort(t *testing.T) {
	t.Parallel()
	fixtures := parityIndexFixtures()
	v, tr := parityCylinder(150)
	fixtures = append(fixtures, parityIndexFixture{`cylinder 600`, v, tr})
	alongRuns, acrossRuns := 0, 0
	for _, fx := range fixtures {
		for _, ax := range []struct{ axis, u, v int }{{0, 1, 2}, {1, 2, 0}, {2, 0, 1}} {
			pm := NewParityMesh(fx.verts, fx.tris)
			idx, pl := pm.placeParityIndex(ax.axis, ax.u, ax.v)
			require.NotNil(t, idx, `%s axis %d`, fx.name, ax.axis)
			c := pl.c
			// The run slice keeps the capacity it was allocated with: the
			// bound covers every run the placement records.
			_, bound, ok := pm.parityFacetKinds(idx, ax.u, ax.v, pm.FacetBoxes(ax.axis))
			require.True(t, ok)
			require.Equal(t, min(bound, pm.membershipLimit()+c), cap(pl.runs), `%s axis %d`, fx.name, ax.axis)
			var cells, facets []int32
			for ti := range len(pl.facetRuns) - 1 {
				for _, r := range pl.runs[pl.facetRuns[ti]:pl.facetRuns[ti+1]] {
					require.NotZero(t, r[1], `%s axis %d: an empty run`, fx.name, ax.axis)
					cell, k, step := int(r[0]), int(r[1]), 1
					if k < 0 {
						k, step = -k, c
						acrossRuns++
					} else {
						alongRuns++
						require.Equal(t, cell/c, (cell+k-1)/c, `%s axis %d: a run along v stays in its column`, fx.name, ax.axis)
					}
					for range k {
						require.Less(t, cell, c*c)
						cells = append(cells, int32(cell))
						facets = append(facets, int32(ti))
						cell += step
					}
				}
			}
			require.Len(t, cells, pl.memberships)
			start := make([]int32, c*c+1)
			for _, cell := range cells {
				start[cell+1]++
			}
			for k := 1; k <= c*c; k++ {
				start[k] += start[k-1]
			}
			ids := make([]int32, len(cells))
			next := append([]int32(nil), start[:c*c]...)
			for e, cell := range cells {
				ids[next[cell]] = facets[e]
				next[cell]++
			}
			pl.fill(idx)
			require.Equal(t, start, idx.start, `%s axis %d`, fx.name, ax.axis)
			require.Equal(t, ids, idx.ids, `%s axis %d`, fx.name, ax.axis)
		}
	}
	require.Positive(t, alongRuns)
	require.Positive(t, acrossRuns)
}

// BenchmarkParityQueries runs a batch of all-facets parity queries on a fresh
// 20 000-facet mesh per op, so every op pays the mesh's box and index
// construction once. For the x ray the side facets of the upright cylinder
// project to tall triangles, and those of the cylinder lying along x project
// to segments whose whole lines are listed.
func BenchmarkParityQueries(b *testing.B) {
	upright, tris := parityCylinder(5000)
	along := make([]r3.Vec, len(upright))
	for i, p := range upright {
		along[i] = r3.Vec{X: p.Z, Y: p.X, Z: p.Y}
	}
	rng := rand.New(rand.NewPCG(0xbe, 0xef))
	queries := make([]proof.Xpt, 2048)
	for i := range queries {
		queries[i] = proof.XptOf(r3.Vec{X: 7*rng.Float64() - 3.5, Y: 7*rng.Float64() - 3.5, Z: 7*rng.Float64() - 3.5})
	}
	for _, mesh := range []struct {
		name  string
		verts []r3.Vec
	}{{`triangles`, upright}, {`segments`, along}} {
		for _, q := range []int{64, 2048} {
			b.Run(fmt.Sprintf(`%s/q=%d`, mesh.name, q), func(b *testing.B) {
				ctx := b.Context()
				b.ReportAllocs()
				for range b.N {
					pm := NewParityMesh(mesh.verts, tris)
					for _, p := range queries[:q] {
						if _, _, err := MeshParityAllContext(ctx, p, pm); err != nil {
							b.Fatal(err)
						}
					}
				}
			})
		}
	}
}

func BenchmarkBuildParityIndex(b *testing.B) {
	verts, tris := parityCylinder(1500)
	pm := NewParityMesh(verts, tris)
	require.NotNil(b, pm.buildParityIndex(0, 1, 2))
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		_ = pm.buildParityIndex(0, 1, 2)
	}
}
