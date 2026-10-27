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

// pairGrids reads two orthogonal polyhedra onto the one grid of both their coordinates: each is read with the other's
// corner coordinates as extra lines, so the two grids are the same.
func pairGrids(vertsA []r3.Vec, trisA [][3]int, vertsB []r3.Vec, trisB [][3]int) (*solidGrid, *solidGrid, error) {
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
	// The two solids' own coordinates must also keep clear of each other: a face of one almost on a face of the
	// other is neither the same plane nor a different one on the grid.
	ca, cb := cornerCoords(vertsA, trisA, axesA, eps), cornerCoords(vertsB, trisB, axesB, eps)
	for a := range 3 {
		if nearCoincident(append(append([]float64(nil), ca[a]...), cb[a]...), eps) {
			return nil, nil, fmt.Errorf("%w: a coordinate of one operand lies within the grid tolerance of a different one of the other along axis %d",
				ErrUnsupported, a)
		}
	}
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
	return ga, gb, nil
}

// Combine returns the boundary mesh of the Boolean of two orthogonal polyhedra (each verts and tris, closed,
// wound outward, every triangle square to an axis): both are read onto the one grid of their coordinates, the
// cells the operation keeps are chosen, and the faces between kept and empty cells are meshed. Every
// corner of a face of the result is made of coordinates of the operands, and a point the fanning of a grid
// cell adds lies inside its face, so the result is the exact union, difference or common part, and operands
// that touch along a face, overlap in a coplanar face or share an edge are decided like any other pair. A result with no cell is ErrDegenerate. The mesh may be non-manifold where kept cells meet only along
// an edge or at a corner, which the caller's solid proof refuses.
func Combine(vertsA []r3.Vec, trisA [][3]int, vertsB []r3.Vec, trisB [][3]int, op Op) ([]r3.Vec, [][3]int, error) {
	ga, gb, err := pairGrids(vertsA, trisA, vertsB, trisB)
	if err != nil {
		return nil, nil, err
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

// OverlapVolume returns the volume two orthogonal polyhedra share: both are read onto one grid and the volumes of the
// cells both occupy are summed, a float sum whose second result is a bound above its rounding. Parts that touch only along a face or an edge
// share no volume. The grid is the one of Combine, so two coordinates of the parts that differ by a nonzero amount
// within its tolerance are ErrUnsupported.
func OverlapVolume(vertsA []r3.Vec, trisA [][3]int, vertsB []r3.Vec, trisB [][3]int) (float64, float64, error) {
	ga, gb, err := pairGrids(vertsA, trisA, vertsB, trisB)
	if err != nil {
		return 0, 0, err
	}
	volume := 0.0
	cells := 0.0
	for k := range ga.occ {
		if !ga.occ[k] || !gb.occ[k] {
			continue
		}
		cells++
		i := k % ga.ud[0]
		j := (k / ga.ud[0]) % ga.ud[1]
		l := k / (ga.ud[0] * ga.ud[1])
		volume += (ga.u.c[0][i+1] - ga.u.c[0][i]) * (ga.u.c[1][j+1] - ga.u.c[1][j]) * (ga.u.c[2][l+1] - ga.u.c[2][l])
	}
	// Each cell volume is two products of differences, each rounded once, and the sum adds one rounding per cell.
	const unit = 1.0 / (1 << 53)
	return volume, (cells + 8) * 8 * unit * volume, nil
}
