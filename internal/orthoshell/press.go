package orthoshell

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// Press moves one planar face of an orthogonal polyhedron along its normal by d, outward for a
// positive d and inward for a negative one, the neighbouring walls following it, and returns the
// boundary mesh of the result. face marks the triangles of the face, which must lie on one world-axis
// plane with one outward direction. The result is the solid joined to the prism the face sweeps (outward)
// or less it (inward), computed on the grid of the solid's coordinates and the plane moved to, so
// it is exact in the cells and its coordinates are the floats the sum P + d gives. A prism that
// would enter material outward, or leave the solid inward, is ErrDegenerate: the face would pass
// through another part or out through the opposite wall.
func Press(verts []r3.Vec, tris [][3]int, face []bool, d float64) ([]r3.Vec, [][3]int, error) {
	if !(d != 0) || math.IsInf(d, 0) || math.IsNaN(d) {
		return nil, nil, fmt.Errorf("%w: the distance must be finite and not zero", ErrDegenerate)
	}
	if len(verts) == 0 || len(tris) == 0 || len(face) != len(tris) {
		return nil, nil, fmt.Errorf("%w: no triangles, or a face mark for each is missing", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return nil, nil, err
	}
	var members []int
	for i, marked := range face {
		if marked {
			members = append(members, i)
		}
	}
	if len(members) == 0 {
		return nil, nil, fmt.Errorf("%w: no face is marked", ErrUnsupported)
	}
	a, sign := axes[members[0]], signs[members[0]]
	plane := comp(verts[tris[members[0]][0]], a)
	span := 0.0
	for _, v := range verts {
		span = math.Max(span, math.Abs(comp(v, a)))
	}
	tol := 1e-9 * math.Max(1, span)
	for _, i := range members {
		if axes[i] != a || signs[i] != sign || math.Abs(comp(verts[tris[i][0]], a)-plane) > tol {
			return nil, nil, fmt.Errorf("%w: the marked triangles are not one planar face", ErrUnsupported)
		}
	}
	target := plane + float64(sign)*d
	var extra [3][]float64
	extra[a] = []float64{target}
	sg, err := newSolid(verts, tris, axes, signs, extra)
	if err != nil {
		return nil, nil, err
	}
	v, w := other(a)
	ud := sg.ud
	footprint := make([]float64, ud[v]*ud[w])
	for _, i := range members {
		tri := tris[i]
		pa := [2]float64{comp(verts[tri[0]], v), comp(verts[tri[0]], w)}
		pb := [2]float64{comp(verts[tri[1]], v), comp(verts[tri[1]], w)}
		pc := [2]float64{comp(verts[tri[2]], v), comp(verts[tri[2]], w)}
		vmin := math.Min(pa[0], math.Min(pb[0], pc[0]))
		vmax := math.Max(pa[0], math.Max(pb[0], pc[0]))
		wmin := math.Min(pa[1], math.Min(pb[1], pc[1]))
		wmax := math.Max(pa[1], math.Max(pb[1], pc[1]))
		// Include the cell before each lower bound within eps, so merged grid lines remain conservative.
		cv0 := max(0, sort.SearchFloat64s(sg.u.c[v], vmin-sg.eps)-1)
		cw0 := max(0, sort.SearchFloat64s(sg.u.c[w], wmin-sg.eps)-1)
		for cw := cw0; cw < ud[w] && sg.u.c[w][cw] < wmax; cw++ {
			for cv := cv0; cv < ud[v] && sg.u.c[v][cv] < vmax; cv++ {
				v0, v1, w0, w1 := sg.u.c[v][cv], sg.u.c[v][cv+1], sg.u.c[w][cw], sg.u.c[w][cw+1]
				footprint[cw*ud[v]+cv] += clipArea(pa, pb, pc, v0, v1, w0, w1) / ((v1 - v0) * (w1 - w0))
			}
		}
	}
	p := indexOf(sg.u.c[a], plane, sg.eps)
	q := indexOf(sg.u.c[a], target, sg.eps)
	if p < 0 || q < 0 {
		return nil, nil, fmt.Errorf("%w: the moved plane is missing from the grid", ErrUnsupported)
	}
	lo, hi := min(p, q), max(p, q)
	occ := append([]bool(nil), sg.occ...)
	for cw := range ud[w] {
		for cv := range ud[v] {
			if footprint[cw*ud[v]+cv] <= 0.5 {
				continue
			}
			for ca := lo; ca < hi; ca++ {
				var idx [3]int
				idx[a], idx[v], idx[w] = ca, cv, cw
				cell := sg.at(idx[0], idx[1], idx[2])
				switch {
				case d > 0 && occ[cell]:
					return nil, nil, fmt.Errorf("%w: the face would move into material", ErrDegenerate)
				case d < 0 && !occ[cell]:
					return nil, nil, fmt.Errorf("%w: the face would move out through the opposite wall", ErrDegenerate)
				}
				occ[cell] = d > 0
			}
		}
	}
	left := false
	for _, in := range occ {
		left = left || in
	}
	if !left {
		return nil, nil, fmt.Errorf("%w: the move leaves no solid", ErrDegenerate)
	}
	return boundary(sg.u, ud, occ)
}
