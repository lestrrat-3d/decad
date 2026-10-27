package orthoshell

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// MinWall returns the thinnest wall of an orthogonal polyhedron (verts and tris, closed, wound outward, every triangle
// square to an axis): the shortest run of material along any of the three axes, taken over every line of the solid's
// grid and every maximal stretch of occupied cells on it. For faces that all square to an axis that is the least
// distance between two opposite faces whose footprints overlap, which is what a wall is. The runs are sums of grid
// gaps, each a float difference, so the result is the least thickness to the rounding of those sums; the second
// result is an allowance above it for that rounding. A solid with no material is ErrDegenerate.
func MinWall(verts []r3.Vec, tris [][3]int) (float64, float64, error) {
	if len(verts) == 0 || len(tris) == 0 {
		return 0, 0, fmt.Errorf("%w: no triangles", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return 0, 0, err
	}
	sg, err := newSolid(verts, tris, axes, signs, [3][]float64{})
	if err != nil {
		return 0, 0, err
	}
	thinnest := math.Inf(1)
	extent := 0.0
	for a := range 3 {
		extent = math.Max(extent, sg.u.c[a][len(sg.u.c[a])-1]-sg.u.c[a][0])
		// Walk every grid line parallel to axis a.
		b, c := other(a)
		for jc := range sg.ud[c] {
			for jb := range sg.ud[b] {
				run := 0.0
				for k := range sg.ud[a] {
					var idx [3]int
					idx[a], idx[b], idx[c] = k, jb, jc
					gap := sg.u.c[a][k+1] - sg.u.c[a][k]
					if sg.occ[sg.at(idx[0], idx[1], idx[2])] {
						run += gap
						continue
					}
					if run > 0 {
						thinnest = math.Min(thinnest, run)
					}
					run = 0
				}
				if run > 0 {
					thinnest = math.Min(thinnest, run)
				}
			}
		}
	}
	if math.IsInf(thinnest, 1) {
		return 0, 0, fmt.Errorf("%w: the solid holds no material", ErrDegenerate)
	}
	return thinnest, 1e-12 * math.Max(1, extent), nil
}
