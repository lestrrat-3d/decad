package orthoshell

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// HasConcaveEdge reports whether an orthogonal polyhedron (verts and tris, closed, wound outward, every triangle
// square to an axis) has an edge where its material turns inward: along some line of the solid's grid, three of the
// four cells around the line hold material and one does not. Every edge of such a solid is sharp, so a concave
// edge is a concave feature of radius zero, and a solid with none is convex along all its edges.
func HasConcaveEdge(verts []r3.Vec, tris [][3]int) (bool, error) {
	if len(verts) == 0 || len(tris) == 0 {
		return false, fmt.Errorf("%w: no triangles", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return false, err
	}
	sg, err := newSolid(verts, tris, axes, signs, [3][]float64{})
	if err != nil {
		return false, err
	}
	for a := range 3 {
		u, w := other(a)
		// A grid line parallel to axis a runs through the corner (iu, iw) of the cells in the plane square to a.
		for iw := 0; iw <= sg.ud[w]; iw++ {
			for iu := 0; iu <= sg.ud[u]; iu++ {
				for k := range sg.ud[a] {
					count := 0
					for _, d := range [4][2]int{{-1, -1}, {0, -1}, {-1, 0}, {0, 0}} {
						cu, cw := iu+d[0], iw+d[1]
						if cu < 0 || cw < 0 || cu >= sg.ud[u] || cw >= sg.ud[w] {
							continue
						}
						var idx [3]int
						idx[a], idx[u], idx[w] = k, cu, cw
						if sg.occ[sg.at(idx[0], idx[1], idx[2])] {
							count++
						}
					}
					if count == 3 {
						return true, nil
					}
				}
			}
		}
	}
	return false, nil
}
