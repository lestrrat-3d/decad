package orthoshell

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// Op is the Boolean operation Combine performs.
type Op int

const (
	// Union keeps the cells either solid occupies.
	Union Op = iota
	// Cut keeps the cells the first solid occupies and the second does not.
	Cut
	// Intersect keeps the cells both solids occupy.
	Intersect
)

// Combine returns the boundary mesh of the Boolean of two orthogonal polyhedra (each verts and tris, closed,
// wound outward, every triangle square to an axis): both are read onto the one grid of their coordinates, the
// cells the operation keeps are chosen, and the faces between kept and empty cells are meshed. Every
// corner of a face of the result is made of coordinates of the operands, and a point the fanning of a grid
// cell adds lies inside its face, so the result is the exact union, difference or common part, and operands
// that touch along a face, overlap in a coplanar face or share an edge are decided like any other pair. A result with no cell is ErrDegenerate. The mesh may be non-manifold where kept cells meet only along
// an edge or at a corner, which the caller's solid proof refuses.
func Combine(vertsA []r3.Vec, trisA [][3]int, vertsB []r3.Vec, trisB [][3]int, op Op) ([]r3.Vec, [][3]int, error) {
	if len(vertsA) == 0 || len(trisA) == 0 || len(vertsB) == 0 || len(trisB) == 0 {
		return nil, nil, fmt.Errorf("%w: each operand needs triangles", ErrUnsupported)
	}
	axesA, signsA, err := classify(vertsA, trisA)
	if err != nil {
		return nil, nil, err
	}
	axesB, signsB, err := classify(vertsB, trisB)
	if err != nil {
		return nil, nil, err
	}
	eps := gridEps(append(append([]r3.Vec(nil), vertsA...), vertsB...))
	ga, err := newSolidEps(vertsA, trisA, axesA, signsA, cornerCoords(vertsB, trisB, axesB, eps), eps)
	if err != nil {
		return nil, nil, err
	}
	gb, err := newSolidEps(vertsB, trisB, axesB, signsB, cornerCoords(vertsA, trisA, axesA, eps), eps)
	if err != nil {
		return nil, nil, err
	}
	for a := range 3 {
		if len(ga.u.c[a]) != len(gb.u.c[a]) {
			return nil, nil, fmt.Errorf("%w: the operands' grids differ along axis %d", ErrUnsupported, a)
		}
		for i := range ga.u.c[a] {
			if ga.u.c[a][i] != gb.u.c[a][i] {
				return nil, nil, fmt.Errorf("%w: the operands' grids differ along axis %d", ErrUnsupported, a)
			}
		}
	}
	in := make([]bool, len(ga.occ))
	kept := false
	for i := range in {
		switch op {
		case Union:
			in[i] = ga.occ[i] || gb.occ[i]
		case Cut:
			in[i] = ga.occ[i] && !gb.occ[i]
		case Intersect:
			in[i] = ga.occ[i] && gb.occ[i]
		default:
			return nil, nil, fmt.Errorf("%w: unknown operation %d", ErrUnsupported, op)
		}
		kept = kept || in[i]
	}
	if !kept {
		return nil, nil, fmt.Errorf("%w: the result holds no cell", ErrDegenerate)
	}
	return boundary(ga.u, ga.ud, in)
}
