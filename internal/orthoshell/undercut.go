package orthoshell

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// Undercuts returns, for each triangle of an orthogonal polyhedron (verts and tris, closed, wound outward, every
// triangle square to an axis), whether it is blocked against a pull along the world axis `axis`: a triangle square to
// that axis must be released along its own outward normal, and is blocked when material lies beyond it in that
// direction on any line over its footprint. The test is made on the solid's grid, on every line over every cell the
// triangle covers, so a triangle is shown free as well as blocked. A triangle whose normal is square to the pull is seen
// edge on and is never blocked. The pull's sense does not matter: the halves of a mould pull apart. A solid with no
// material is ErrDegenerate.
func Undercuts(verts []r3.Vec, tris [][3]int, axis int) ([]bool, error) {
	if len(verts) == 0 || len(tris) == 0 || axis < 0 || axis > 2 {
		return nil, fmt.Errorf("%w: no triangles, or no pull axis", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return nil, err
	}
	sg, err := newSolid(verts, tris, axes, signs, [3][]float64{})
	if err != nil {
		return nil, err
	}
	u, w := other(axis)
	blocked := make([]bool, len(tris))
	// occupied[column] lists, from low to high along the pull axis, the cells that hold material.
	colAt := func(cu, cw int) []bool {
		col := make([]bool, sg.ud[axis])
		for k := range col {
			var idx [3]int
			idx[axis], idx[u], idx[w] = k, cu, cw
			col[k] = sg.occ[sg.at(idx[0], idx[1], idx[2])]
		}
		return col
	}
	columns := map[[2]int][]bool{}
	column := func(cu, cw int) []bool {
		key := [2]int{cu, cw}
		if c, ok := columns[key]; ok {
			return c
		}
		c := colAt(cu, cw)
		columns[key] = c
		return c
	}
	filled := false
	for _, occupied := range sg.occ {
		filled = filled || occupied
	}
	if !filled {
		return nil, fmt.Errorf("%w: the solid holds no material", decaderr.ErrDegenerate)
	}
	for i, tri := range tris {
		if axes[i] != axis {
			continue
		}
		p := indexOf(sg.u.c[axis], comp(verts[tri[0]], axis), sg.eps)
		if p < 0 {
			return nil, fmt.Errorf("%w: a triangle plane is missing from the grid", ErrUnsupported)
		}
		a := [2]float64{comp(verts[tri[0]], u), comp(verts[tri[0]], w)}
		b := [2]float64{comp(verts[tri[1]], u), comp(verts[tri[1]], w)}
		c := [2]float64{comp(verts[tri[2]], u), comp(verts[tri[2]], w)}
		lo := [2]float64{math.Min(a[0], math.Min(b[0], c[0])), math.Min(a[1], math.Min(b[1], c[1]))}
		hi := [2]float64{math.Max(a[0], math.Max(b[0], c[0])), math.Max(a[1], math.Max(b[1], c[1]))}
		cu0 := max(0, sort.SearchFloat64s(sg.u.c[u], lo[0]+sg.eps)-1)
		cw0 := max(0, sort.SearchFloat64s(sg.u.c[w], lo[1]+sg.eps)-1)
		for cw := cw0; cw < sg.ud[w] && sg.u.c[w][cw] < hi[1]; cw++ {
			for cu := cu0; cu < sg.ud[u] && sg.u.c[u][cu] < hi[0]; cu++ {
				if clipArea(a, b, c, sg.u.c[u][cu], sg.u.c[u][cu+1], sg.u.c[w][cw], sg.u.c[w][cw+1]) <= 0 {
					continue
				}
				col := column(cu, cw)
				// Beyond the plane, along the triangle's outward normal.
				if signs[i] > 0 {
					for k := p; k < len(col) && !blocked[i]; k++ {
						blocked[i] = col[k]
					}
				} else {
					for k := p - 1; k >= 0 && !blocked[i]; k-- {
						blocked[i] = col[k]
					}
				}
				if blocked[i] {
					break
				}
			}
			if blocked[i] {
				break
			}
		}
	}
	return blocked, nil
}
