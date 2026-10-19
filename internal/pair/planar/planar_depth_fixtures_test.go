package planar_test

import (
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/proof"
)

func dy(f float64) proof.Dyadic { return proof.MustDyOf(f) }

func noPoll() error { return nil }

func boxSolid(lo, hi [3]float64) planar.PlanarSolid {
	var solid planar.PlanarSolid
	for index := range 8 {
		point := lo
		for axis := range 3 {
			if index>>axis&1 == 1 {
				point[axis] = hi[axis]
			}
		}
		solid.Verts = append(solid.Verts, proof.DyV3{dy(point[0]), dy(point[1]), dy(point[2])})
	}
	solid.Tris = boxTriangles(0)
	return solid
}

func boxTriangles(base int) [][3]int {
	triangles := [][3]int{
		{0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}, {0, 1, 5}, {0, 5, 4},
		{2, 6, 7}, {2, 7, 3}, {0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6},
	}
	for i := range triangles {
		for j := range 3 {
			triangles[i][j] += base
		}
	}
	return triangles
}

func hollowSolid(lo, hi, cavityLo, cavityHi [3]float64) planar.PlanarSolid {
	outer := boxSolid(lo, hi)
	cavity := boxSolid(cavityLo, cavityHi)
	outer.Verts = append(outer.Verts, cavity.Verts...)
	for _, triangle := range boxTriangles(8) {
		outer.Tris = append(outer.Tris, [3]int{triangle[0], triangle[2], triangle[1]})
	}
	return outer
}
