package meshbool

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// wedgePrism is a closed convex prism over a counterclockwise polygon.
func wedgePrism(t *testing.T, polygon [][2]float64, z0, z1 float64) *BoolMesh {
	t.Helper()
	n := len(polygon)
	var verts []r3.Vec
	for _, p := range polygon {
		verts = append(verts, r3.Vec{X: p[0], Y: p[1], Z: z0})
	}
	for _, p := range polygon {
		verts = append(verts, r3.Vec{X: p[0], Y: p[1], Z: z1})
	}
	var tris [][3]int
	for i := 1; i+1 < n; i++ {
		tris = append(tris, [3]int{n, n + i, n + i + 1}, [3]int{0, i + 1, i})
	}
	for i := range n {
		j := (i + 1) % n
		tris = append(tris, [3]int{i, j, n + j}, [3]int{i, n + j, n + i})
	}
	bm, err := PrepContext(t.Context(), verts, tris, make([]int, len(tris)))
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(bm.Verts)))
	return bm
}

func wedgeAt(lo, hi float64) [][2]float64 {
	pt := func(deg float64) [2]float64 {
		return [2]float64{2 * math.Cos(deg*math.Pi/180), 2 * math.Sin(deg*math.Pi/180)}
	}
	return [][2]float64{{0, 0}, pt(lo), pt(hi)}
}

func sharedEdgeOf(t *testing.T, bm *BoolMesh) (facet, edge int) {
	t.Helper()
	for i, tri := range bm.Tris {
		for k := range 3 {
			u, v := bm.Verts[tri[k]], bm.Verts[tri[(k+1)%3]]
			if u.X == 0 && u.Y == 0 && v.X == 0 && v.Y == 0 && u.Z != v.Z {
				return i, k
			}
		}
	}
	require.Fail(t, "no origin edge")
	return 0, 0
}

type vouch bool

func (v vouch) CertifySharedEdge(context.Context, int, int, int, int) (bool, error) {
	return bool(v), nil
}
