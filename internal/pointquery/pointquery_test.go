package pointquery

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMeshDistanceSquared(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(0, 2, 0),
	}
	tris := [][3]int{{0, 1, 2}}
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want string
	}{
		{name: "plane projection", at: r3.NewVec(0.5, 0.5, 3), want: "9"},
		{name: "edge", at: r3.NewVec(2, 2, 0), want: "2"},
		{name: "vertex", at: r3.NewVec(-1, -1, 0), want: "2"},
		{name: "on facet", at: r3.NewVec(0.5, 0.5, 0), want: "0"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			distance, err := MeshDistanceSquared(t.Context(), tc.at, verts, tris)
			require.NoError(t, err)
			require.Equal(t, tc.want, distance.RatString())
		})
	}
}

func TestLocateUsesObliqueRayAndVolumeBound(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0),
		r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2),
		r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}
	tris := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
		{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
		{2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	for _, tc := range []struct {
		name  string
		at    r3.Vec
		bound float64
		sym   float64
		want  Location
	}{
		{name: "exact center", at: r3.NewVec(1, 1, 1), want: Inside},
		{name: "certified center", at: r3.NewVec(1, 1, 1), bound: 0.1, sym: 2, want: Inside},
		{name: "uncertain volume", at: r3.NewVec(1, 1, 1), bound: 0.1, sym: 3, want: Undecided},
		{name: "exact facet", at: r3.NewVec(0, 1, 1), want: OnBoundary},
		{name: "uncertain facet", at: r3.NewVec(0, 1, 1), bound: 0.1, want: Undecided},
		{name: "outside", at: r3.NewVec(4, 1, 1), want: Outside},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := Locate(t.Context(), tc.at, verts, tris, tc.bound, tc.sym)
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}
