package meshbool_test

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func stitchMapTetrahedra(count int) []meshbool.KeptFacet {
	kept := make([]meshbool.KeptFacet, 0, 4*count)
	for i := range count {
		x := float64(i * 4)
		points := [4]proof.Xpt{
			proof.XptOf(r3.Vec{X: x}), proof.XptOf(r3.Vec{X: x + 1}),
			proof.XptOf(r3.Vec{X: x, Y: 1}), proof.XptOf(r3.Vec{X: x, Z: 1}),
		}
		for _, tri := range [4][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}} {
			kept = append(kept, meshbool.KeptFacet{
				V:   [3]proof.Xpt{points[tri[0]], points[tri[1]], points[tri[2]]},
				Src: i,
			})
		}
	}
	return kept
}

func TestStitchMapTetrahedronFixture(t *testing.T) {
	t.Parallel()
	got, err := meshbool.StitchFacetsContext(t.Context(), stitchMapTetrahedra(2))
	require.NoError(t, err)
	require.Equal(t, []r3.Vec{
		{}, {Y: 1}, {X: 1}, {Z: 1},
		{X: 4}, {X: 4, Y: 1}, {X: 5}, {X: 4, Z: 1},
	}, got.Verts)
	require.Equal(t, [][3]int{
		{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {2, 1, 3},
		{4, 5, 6}, {4, 6, 7}, {4, 7, 5}, {6, 5, 7},
	}, got.Tris)
	require.Equal(t, []int{0, 0, 0, 0, 1, 1, 1, 1}, got.Src)
	require.Equal(t, uint64(0), math.Float64bits(got.Round))
	require.Equal(t, uint64(0), math.Float64bits(got.DropArea))
	for _, bound := range got.VertexBound {
		require.Equal(t, uint64(0), math.Float64bits(bound))
	}
	require.Len(t, got.VertexBound, 8)
	require.Equal(t, uint64(0x4012ed9eba161339), math.Float64bits(got.PreArea))
}

func TestStitchMapLargeFixture(t *testing.T) {
	t.Parallel()
	const components = 1025 // 4100 triangles exercise the fresh-map fallback.
	kept := stitchMapTetrahedra(components)
	got, err := meshbool.StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Len(t, got.Verts, 4*components)
	require.Len(t, got.Tris, len(kept))
	require.Equal(t, [3]int{4*components - 2, 4*components - 3, 4*components - 1}, got.Tris[len(got.Tris)-1])
	require.Equal(t, components-1, got.Src[len(got.Src)-1])
	require.Equal(t, uint64(0), math.Float64bits(got.Round))
	require.Equal(t, uint64(0), math.Float64bits(got.DropArea))
}

func stitchMapCollapsedTetrahedron() []meshbool.KeptFacet {
	w := new(big.Int).Lsh(big.NewInt(1), 60)
	base := new(big.Int).Mul(big.NewInt(40), w)
	point := func(dx, dy, dz int64) proof.Xpt {
		return proof.Xpt{
			X: new(big.Int).Add(base, big.NewInt(dx)),
			Y: new(big.Int).Add(base, big.NewInt(dy)),
			Z: new(big.Int).Add(base, big.NewInt(dz)),
			W: new(big.Int).Set(w),
		}
	}
	p, q, r, s := point(0, 0, 0), point(1, 0, 0), point(0, 1, 0), point(0, 0, 1)
	return []meshbool.KeptFacet{
		{V: [3]proof.Xpt{p, r, q}}, {V: [3]proof.Xpt{p, q, s}},
		{V: [3]proof.Xpt{p, s, r}}, {V: [3]proof.Xpt{q, r, s}},
	}
}

func TestStitchFacetsClosureRefusals(t *testing.T) {
	t.Parallel()
	closed := stitchMapTetrahedra(1)
	for _, tc := range []struct {
		name string
		kept []meshbool.KeptFacet
		kind error
		want string
	}{
		{"missing directed edge", closed[:3], decaderr.ErrBooleanFailed, "stitched boundary does not close"},
		{"duplicate directed edge", append(append([]meshbool.KeptFacet{}, closed...), closed[0]),
			decaderr.ErrBooleanFailed, "stitched boundary does not close"},
		{"tiny component has no extent", stitchMapCollapsedTetrahedron(),
			decaderr.ErrBooleanFailed, "stitched boundary has no extent"},
		{"component collapses", append(append([]meshbool.KeptFacet{}, closed...), stitchMapCollapsedTetrahedron()...),
			decaderr.ErrUnsupported, "whole component of the result welds away"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := meshbool.StitchFacetsContext(t.Context(), tc.kept)
			require.Nil(t, got)
			require.ErrorIs(t, err, tc.kind)
			require.ErrorContains(t, err, tc.want)
		})
	}
	t.Run("cancelled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		got, err := meshbool.StitchFacetsContext(ctx, closed)
		require.Nil(t, got)
		require.ErrorIs(t, err, context.Canceled)
	})
	t.Run("no facets", func(t *testing.T) {
		got, err := meshbool.StitchFacetsContext(t.Context(), nil)
		require.Nil(t, got)
		require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	})
}

func BenchmarkStitchFacetsClosureMaps(b *testing.B) {
	for _, count := range []int{1, 32} {
		b.Run(fmt.Sprintf("%d_triangles", 4*count), func(b *testing.B) {
			kept := stitchMapTetrahedra(count)
			b.ReportAllocs()
			for b.Loop() {
				got, err := meshbool.StitchFacetsContext(b.Context(), kept)
				if err != nil {
					b.Fatal(err)
				}
				if len(got.Tris) != len(kept) {
					b.Fatalf("got %d triangles, want %d", len(got.Tris), len(kept))
				}
			}
		})
	}
}
