package orthoshell

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// Offset returns the boundary mesh of an orthogonal polyhedron (verts and tris, closed, wound outward, every
// triangle square to an axis) with every face moved along its normal by d: outward for a positive d and inward
// for a negative one, the neighbouring faces extended or trimmed to meet. For faces that all square to an axis
// that is the solid dilated by the cube of half-side d, or eroded by it: a convex edge stays sharp and a reflex
// one fills, a slot narrower than twice d closes, and a part thinner than twice -d is gone. The new
// coordinates are the float sums x +/- d of the solid's own. An erosion that leaves nothing is ErrDegenerate.
func Offset(verts []r3.Vec, tris [][3]int, d float64) ([]r3.Vec, [][3]int, error) {
	if d == 0 || math.IsNaN(d) || math.IsInf(d, 0) {
		return nil, nil, fmt.Errorf("%w: the offset distance must be finite and not zero", ErrDegenerate)
	}
	none := make([]bool, len(tris))
	r, rd, walls, inside, err := wallCells(verts, tris, none, math.Abs(d), d > 0)
	if err != nil {
		return nil, nil, err
	}
	kept := make([]bool, len(walls))
	left := false
	for i := range kept {
		if d > 0 {
			kept[i] = walls[i] || inside[i]
		} else {
			kept[i] = inside[i] && !walls[i]
		}
		left = left || kept[i]
	}
	if !left {
		return nil, nil, fmt.Errorf("%w: the offset leaves no solid", ErrDegenerate)
	}
	return boundary(r, rd, kept)
}
