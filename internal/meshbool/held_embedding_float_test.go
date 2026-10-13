package meshbool

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func floatCertMesh(verts []r3.Vec, tris [][3]int) HeldRounding {
	h := HeldRounding{Verts: verts, Tris: tris, Exact: make([]proof.Xpt, len(verts))}
	for i, v := range verts {
		h.Exact[i] = proof.XptOf(v)
	}
	return h
}

// floatCertSphere is a UV sphere of about 2 000 facets, rotated by 0.3 rad
// about (1, 2, 3). Its vertices are the rounded floats.
func floatCertSphere() HeldRounding {
	const rings, segs = 22, 46
	axis := r3.NewVec(1, 2, 3).Scale(1 / math.Sqrt(14))
	rot := func(v r3.Vec) r3.Vec {
		c, s := math.Cos(0.3), math.Sin(0.3)
		return v.Scale(c).Add(axis.Cross(v).Scale(s)).Add(axis.Scale(axis.Dot(v) * (1 - c)))
	}
	verts := []r3.Vec{rot(r3.NewVec(0, 0, 1))}
	for r := 1; r < rings; r++ {
		phi := math.Pi * float64(r) / rings
		for s := range segs {
			th := 2 * math.Pi * float64(s) / segs
			verts = append(verts, rot(r3.NewVec(math.Sin(phi)*math.Cos(th), math.Sin(phi)*math.Sin(th), math.Cos(phi))))
		}
	}
	verts = append(verts, rot(r3.NewVec(0, 0, -1)))
	south := len(verts) - 1
	ring := func(r, s int) int { return 1 + (r-1)*segs + s%segs }
	var tris [][3]int
	for s := range segs {
		tris = append(tris, [3]int{0, ring(1, s), ring(1, s+1)})
		tris = append(tris, [3]int{south, ring(rings-1, s+1), ring(rings-1, s)})
	}
	for r := 1; r < rings-1; r++ {
		for s := range segs {
			tris = append(tris, [3]int{ring(r, s), ring(r+1, s), ring(r+1, s+1)})
			tris = append(tris, [3]int{ring(r, s), ring(r+1, s+1), ring(r, s+1)})
		}
	}
	return floatCertMesh(verts, tris)
}

// floatCertGrid is a flat triangulated grid: every neighbour is coplanar.
func floatCertGrid() HeldRounding {
	const n = 14
	var verts []r3.Vec
	for y := 0; y <= n; y++ {
		for x := 0; x <= n; x++ {
			verts = append(verts, r3.NewVec(float64(x)*0.25, float64(y)*0.5, 1))
		}
	}
	var tris [][3]int
	for y := range n {
		for x := range n {
			a, b, c, d := y*(n+1)+x, y*(n+1)+x+1, (y+1)*(n+1)+x, (y+1)*(n+1)+x+1
			tris = append(tris, [3]int{a, b, d}, [3]int{a, d, c})
		}
	}
	return floatCertMesh(verts, tris)
}

// floatCertFan is a fan around one vertex with a sliver and a collinear spoke.
func floatCertFan() HeldRounding {
	verts := []r3.Vec{{}, r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0)}
	for k := 1; k <= 9; k++ {
		th := float64(k) * 0.6
		verts = append(verts, r3.NewVec(math.Cos(th), math.Sin(th), 0.1*float64(k%3)))
	}
	verts = append(verts, r3.NewVec(1, 1e-15, 0), r3.NewVec(0.5, 0, 1e-17), r3.NewVec(1, 0, 0.5))
	var tris [][3]int
	for k := 1; k < len(verts)-1; k++ {
		tris = append(tris, [3]int{0, k, k + 1})
	}
	tris = append(tris, [3]int{0, len(verts) - 1, 1}, [3]int{0, 2, len(verts) - 2})
	return floatCertMesh(verts, tris)
}

// TestHeldFloatCertificatesAgreeWithExact runs every box-overlapping pair that
// shares one or two indices through the float certificates and the exact
// path, and requires every pair a certificate decides to be one the exact
// path decides as (true, true) and accepts.
func TestHeldFloatCertificatesAgreeWithExact(t *testing.T) {
	t.Parallel()
	meshes := map[string]HeldRounding{"sphere": floatCertSphere(), "grid": floatCertGrid(), "fan": floatCertFan()}
	for name, h := range meshes {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			plain := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
			for i := range h.Tris {
				if e.facetOpenFloat(i) {
					_, _, n := e.facet(i)
					require.False(t, zeroXpt(n), "facet %d", i)
				}
			}
			var pairs, float int
			for i := range h.Tris {
				seen := map[int]bool{}
				for _, v := range h.Tris[i] {
					for _, j := range e.inc[v] {
						if j <= i || seen[j] || !BoxesOverlap(e.triBox(i), e.triBox(j)) {
							continue
						}
						seen[j] = true
						var sharedI, sharedJ []int
						for a, va := range h.Tris[i] {
							for b, vb := range h.Tris[j] {
								if va == vb {
									sharedI, sharedJ = append(sharedI, a), append(sharedJ, b)
								}
							}
						}
						if len(sharedI) == 0 || len(sharedI) == 3 {
							continue
						}
						pairs++
						if e.pairApartFloat(i, j, sharedI, sharedJ) {
							float++
							ok, decided, _ := plain.pairSeparated(i, j, sharedI, sharedJ)
							require.True(t, decided && ok, "pair %d,%d", i, j)
							require.True(t, plain.pairEmbeddedExact(i, j, sharedI, sharedJ), "pair %d,%d", i, j)
							require.True(t, e.pairEmbedded(i, j), "pair %d,%d", i, j)
						}
					}
				}
			}
			require.NotZero(t, pairs)
			t.Logf("%s: %d pairs, %d decided by floats", name, pairs, float)
		})
	}
}
