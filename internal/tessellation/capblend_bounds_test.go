package tessellation_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestComposeCapBlendBounds(t *testing.T) {
	type face struct{ name string }
	f1, f2 := &face{"a"}, &face{"b"}
	tris := [][3]int{{0, 1, 2}, {1, 2, 3}, {3, 4, 5}}
	store := []float64{0.5, 1, 2, 4, 8, 16}

	got, err := tessellation.ComposeCapBlendBounds([]*face{f1, f1, f2}, tris,
		map[*face]float64{f1: 0.25}, store, "test")
	require.NoError(t, err)
	require.Len(t, got, 2)
	require.GreaterOrEqual(t, got[f1], 4.25, "f1: extra 0.25 plus max store 4")
	require.Less(t, got[f1], 4.26)
	require.GreaterOrEqual(t, got[f2], 16.0, "f2: no extra, max store 16")
	require.Less(t, got[f2], 16.01)

	t.Run("non-finite", func(t *testing.T) {
		inf := 1e308
		_, err := tessellation.ComposeCapBlendBounds([]*face{f1}, tris[:1],
			map[*face]float64{f1: inf * 10}, store, "test")
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a test face's composed displacement is not finite")
	})
}

func TestRequireCapBlendFacetAreas(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 2}}
	require.NoError(t, tessellation.RequireCapBlendFacetAreas(verts, [][3]int{{0, 1, 2}}, "x"))

	err := tessellation.RequireCapBlendFacetAreas(verts, [][3]int{{0, 1, 2}, {0, 1, 3}}, "x")
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "facet 1 of this x mesh has zero area")
}

func TestRequireCapBlendFacetAreasMatchesExactRational(t *testing.T) {
	large := math.Ldexp(1, 1001)
	step := math.Ldexp(1, 950)
	near := math.Nextafter(2, math.Inf(1))
	tests := []struct {
		name string
		tri  [3]r3.Vec
	}{
		{name: "ordinary", tri: [3]r3.Vec{{}, {X: 3, Y: 1}, {X: 1, Y: 4}}},
		{name: "collinear", tri: [3]r3.Vec{{}, {X: 1, Y: 1, Z: 1}, {X: 2, Y: 2, Z: 2}}},
		{name: "near collinear", tri: [3]r3.Vec{{}, {X: 1, Y: 1, Z: 1}, {X: 2, Y: 2, Z: near}}},
		{name: "extreme finite", tri: [3]r3.Vec{{}, {X: large, Y: large}, {X: large, Y: large + step}}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			wantPositive := tessellation.CapBlendTwiceAreaSq(tc.tri[0], tc.tri[1], tc.tri[2]).Sign() > 0
			err := tessellation.RequireCapBlendFacetAreas(tc.tri[:], [][3]int{{0, 1, 2}}, "test")
			require.Equal(t, wantPositive, err == nil)
		})
	}
}

func TestPrismPatternMeshRoot(t *testing.T) {
	// 5 stations, each a bottom/top pair; the root wall "r" owns the last 2.
	const z0, z1 = 0.0, 3.0
	var verts []r3.Vec
	for j := range 5 {
		verts = append(verts, r3.Vec{X: float64(j), Z: z0}, r3.Vec{X: float64(j), Z: z1})
	}
	src := func(names ...string) []string {
		var out []string
		for _, n := range names {
			out = append(out, n, n)
		}
		return out
	}
	good := src("a", "b", "c", "r", "r")
	ntri := len(good)

	got, err := tessellation.PrismPatternMeshRoot(verts, ntri, good, []string{"r"}, z0, z1)
	require.NoError(t, err)
	require.Equal(t, 3, got)

	cases := []struct {
		name   string
		verts  []r3.Vec
		ntri   int
		source []string
		roots  []string
		z1     float64
		want   string
	}{
		{"short mesh", verts[:6], ntri, good, []string{"r"}, z1, "no paired axial section mesh"},
		{"source mismatch", verts, ntri, good[:8], []string{"r"}, z1, "no paired axial section mesh"},
		{"duplicate root", verts, ntri, good, []string{"r", "r"}, z1, "duplicate root wall"},
		{"lost root", verts, ntri, good, nil, z1, "lost its root wall"},
		{"station z", verts, ntri, good, []string{"r"}, z1 + 1, "lost its shared axial stations"},
		{"split run", verts, ntri, src("a", "r", "c", "r", "r"), []string{"r"}, z1, "not one trailing run"},
		{"root too early", verts, ntri, src("a", "r", "r", "r", "r"), []string{"r"}, z1, "no exterior path"},
		{"root last", verts, ntri, src("a", "b", "c", "d", "r"), []string{"r"}, z1, "no exterior path"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := tessellation.PrismPatternMeshRoot(tc.verts, tc.ntri, tc.source, tc.roots, z0, tc.z1)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
