package orthoshell

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// Part is one orthogonal polyhedron (Verts and Tris, closed, wound outward, every triangle square to an axis).
type Part struct {
	Verts []r3.Vec
	Tris  [][3]int
}

// UnionMany returns the boundary mesh of the union of orthogonal polyhedra: every part is read onto the one grid of
// all their coordinates and the cells any of them occupies are kept. Like Combine, every corner of a face of the
// result is made of the parts' coordinates, so the result is exact, and parts that touch along a face, overlap in a
// coplanar face or share an edge are decided like any other. A result with no cell is ErrDegenerate, and one whose
// kept cells meet only along an edge may be non-manifold, which the caller's solid proof refuses.
func UnionMany(parts []Part) ([]r3.Vec, [][3]int, error) {
	if len(parts) < 2 {
		return nil, nil, fmt.Errorf("%w: a union needs at least two parts", ErrUnsupported)
	}
	axes := make([][]int, len(parts))
	signs := make([][]int, len(parts))
	var all []r3.Vec
	for i, p := range parts {
		if len(p.Verts) == 0 || len(p.Tris) == 0 {
			return nil, nil, fmt.Errorf("%w: each part needs triangles", ErrUnsupported)
		}
		var err error
		if axes[i], signs[i], err = classify(p.Verts, p.Tris); err != nil {
			return nil, nil, err
		}
		all = append(all, p.Verts...)
	}
	eps := gridEps(all)
	corners := make([][3][]float64, len(parts))
	var pooled [3][]float64
	for i, p := range parts {
		corners[i] = cornerCoords(p.Verts, p.Tris, axes[i], eps)
		for a := range 3 {
			pooled[a] = append(pooled[a], corners[i][a]...)
		}
	}
	for a := range 3 {
		if nearCoincident(pooled[a], eps) {
			return nil, nil, fmt.Errorf("%w: a coordinate of one part lies within the grid tolerance of a different one of another along axis %d",
				ErrUnsupported, a)
		}
	}
	var first *solidGrid
	var in []bool
	for i, p := range parts {
		// The lines of every other part, so that all the parts are read onto one grid.
		var extra [3][]float64
		for j := range parts {
			if j == i {
				continue
			}
			for a := range 3 {
				extra[a] = append(extra[a], corners[j][a]...)
			}
		}
		g, err := newSolidEps(p.Verts, p.Tris, axes[i], signs[i], extra, eps)
		if err != nil {
			return nil, nil, err
		}
		if first == nil {
			first, in = g, make([]bool, len(g.occ))
		} else {
			for a := range 3 {
				if len(g.u.c[a]) != len(first.u.c[a]) {
					return nil, nil, fmt.Errorf("%w: the parts' grids differ along axis %d", ErrUnsupported, a)
				}
				for k := range g.u.c[a] {
					if g.u.c[a][k] != first.u.c[a][k] {
						return nil, nil, fmt.Errorf("%w: the parts' grids differ along axis %d", ErrUnsupported, a)
					}
				}
			}
		}
		for k, occupied := range g.occ {
			in[k] = in[k] || occupied
		}
	}
	kept := false
	for _, c := range in {
		kept = kept || c
	}
	if !kept {
		return nil, nil, fmt.Errorf("%w: the union holds no cell", ErrDegenerate)
	}
	return boundary(first.u, first.ud, in)
}
