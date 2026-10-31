// Package orthoshell finds the cavity of a shell of an orthogonal polyhedron: a
// closed solid whose faces all square to a world axis (agent-docs/orthogonal-shell-design.md).
// It works on plain millimetre data and touches no body, document or sketch.
package orthoshell

import (
	"errors"
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

var (
	// ErrUnsupported is returned for a solid this construction does not take: a face not square to
	// an axis, a grid too large, or a cavity whose boundary touches itself along an edge.
	ErrUnsupported = errors.New("orthoshell: unsupported solid")
	// ErrNotOrthogonal is returned for a solid with a triangle that is not square to an axis; it
	// is the signal that this construction does not apply, not that the solid is bad.
	ErrNotOrthogonal = errors.New("orthoshell: a triangle is not square to an axis")
	// ErrDegenerate is returned when the walls leave no cavity.
	ErrDegenerate = errors.New("orthoshell: no cavity")
)

// MaxCells bounds the refined grid.
const MaxCells = 4_000_000

type grid struct {
	c [3][]float64 // sorted coordinates of each axis
}

func (g grid) dims() [3]int { return [3]int{len(g.c[0]) - 1, len(g.c[1]) - 1, len(g.c[2]) - 1} }

func uniqueSorted(vals []float64, eps float64) []float64 {
	sort.Float64s(vals)
	out := vals[:0:0]
	for _, v := range vals {
		if len(out) > 0 && v-out[len(out)-1] <= eps {
			continue
		}
		out = append(out, v)
	}
	return out
}

// nearCoincident reports whether two of vals differ by a nonzero amount within eps. A grid merges coordinates that close
// into one line, which would move a face of the solid; a solid whose vertex coordinates do that is refused instead,
// so that a result built on the grid stands on the coordinates it was given, exactly.
func nearCoincident(vals []float64, eps float64) bool {
	sorted := append([]float64(nil), vals...)
	sort.Float64s(sorted)
	for i := 1; i < len(sorted); i++ {
		if d := sorted[i] - sorted[i-1]; d > 0 && d <= eps {
			return true
		}
	}
	return false
}

// indexOf finds the coordinate within eps of v.
func indexOf(coords []float64, v, eps float64) int {
	i := sort.SearchFloat64s(coords, v-eps)
	if i < len(coords) && math.Abs(coords[i]-v) <= eps {
		return i
	}
	return -1
}

// other returns the two axes square to a, in the cyclic order (a+1, a+2).
func other(a int) (int, int) { return (a + 1) % 3, (a + 2) % 3 }

// triAxis returns the axis a triangle's three corners share a coordinate on and the sign of its
// normal along it, or ok false for a triangle not square to an axis.
func triAxis(v0, v1, v2 r3.Vec) (axis, sign int, ok bool) {
	p := [3][3]float64{{v0.X, v0.Y, v0.Z}, {v1.X, v1.Y, v1.Z}, {v2.X, v2.Y, v2.Z}}
	axis = -1
	for a := range 3 {
		if p[0][a] == p[1][a] && p[1][a] == p[2][a] {
			if axis >= 0 {
				return 0, 0, false // a triangle of no area
			}
			axis = a
		}
	}
	if axis < 0 {
		return 0, 0, false
	}
	u, w := other(axis)
	area := (p[1][u]-p[0][u])*(p[2][w]-p[0][w]) - (p[1][w]-p[0][w])*(p[2][u]-p[0][u])
	switch {
	case area > 0:
		return axis, 1, true
	case area < 0:
		return axis, -1, true
	default:
		return 0, 0, false
	}
}

// clipArea is the area of the triangle (a, b, c) inside the rectangle [x0,x1]×[y0,y1].
func clipArea(a, b, c [2]float64, x0, x1, y0, y1 float64) float64 {
	// Each clip emits at most two vertices per input edge, so four clips need at most 3*2^4 = 48 slots.
	var scratchA, scratchB [48][2]float64
	scratchA[0], scratchA[1], scratchA[2] = a, b, c
	in, out := &scratchA, &scratchB
	inLen := 3

	for pass := range 4 {
		axis := pass / 2
		bound := [4]float64{x0, x1, y0, y1}[pass]
		keepGreater := pass%2 == 0
		outLen := 0
		for i := range inLen {
			p := in[i]
			q := in[(i+1)%inLen]
			ip, iq := p[axis] >= bound, q[axis] >= bound
			if !keepGreater {
				ip, iq = p[axis] <= bound, q[axis] <= bound
			}
			switch {
			case ip && iq:
				out[outLen] = q
				outLen++
			case ip && !iq, !ip && iq:
				var intersection [2]float64
				if axis == 0 {
					t := (bound - p[0]) / (q[0] - p[0])
					intersection = [2]float64{bound, p[1] + t*(q[1]-p[1])}
				} else {
					t := (bound - p[1]) / (q[1] - p[1])
					intersection = [2]float64{p[0] + t*(q[0]-p[0]), bound}
				}
				out[outLen] = intersection
				outLen++
				if !ip && iq {
					out[outLen] = q
					outLen++
				}
			}
		}
		inLen = outLen
		in, out = out, in
	}
	if inLen < 3 {
		return 0
	}
	area := 0.0
	for i := range inLen {
		p := in[i]
		q := in[(i+1)%inLen]
		area += p[0]*q[1] - q[0]*p[1]
	}
	return math.Abs(area) / 2
}

// solidGrid is an orthogonal polyhedron read onto the grid of its own coordinates, with any extra
// ones: u holds the coordinates of each axis, ud the cell counts, and occ which cells are inside.
type solidGrid struct {
	u   grid
	ud  [3]int
	occ []bool
	eps float64
}

func (g *solidGrid) at(i, j, k int) int { return (k*g.ud[1]+j)*g.ud[0] + i }

func comp(v r3.Vec, a int) float64 { return [3]float64{v.X, v.Y, v.Z}[a] }

// classify returns, for each triangle, the axis it squares to and the sign of its normal along it.
func classify(verts []r3.Vec, tris [][3]int) (axes, signs []int, err error) {
	axes = make([]int, len(tris))
	signs = make([]int, len(tris))
	for i, tri := range tris {
		a, s, ok := triAxis(verts[tri[0]], verts[tri[1]], verts[tri[2]])
		if !ok {
			return nil, nil, fmt.Errorf("%w: triangle %d", ErrNotOrthogonal, i)
		}
		axes[i], signs[i] = a, s
	}
	return axes, signs, nil
}

// cornerCoords returns the coordinates, on each axis, of the vertices of a solid that lie on an edge or at a
// corner. A vertex that lies in one plane only is interior to a face, a point a mesh adds to fan the face, and
// would put a grid line through the solid where there is no corner: only vertices on two planes give lines.
func cornerCoords(verts []r3.Vec, tris [][3]int, axes []int, eps float64) [3][]float64 {
	// A vertex that lies in one plane only is interior to a face, a point a mesh adds to fan the face, and
	// would put a grid line through the solid where there is no corner: only vertices on two planes, on an
	// edge or at a corner, give grid lines.
	type placed struct {
		axis  int
		plane float64
		many  bool
	}
	seen := make([]placed, len(verts))
	marked := make([]bool, len(verts))
	for i, tri := range tris {
		plane := comp(verts[tri[0]], axes[i])
		for _, vi := range tri {
			switch {
			case !marked[vi]:
				seen[vi], marked[vi] = placed{axis: axes[i], plane: plane}, true
			case seen[vi].axis != axes[i] || math.Abs(seen[vi].plane-plane) > eps:
				seen[vi].many = true
			}
		}
	}
	var coords [3][]float64
	for i, v := range verts {
		if !seen[i].many {
			continue
		}
		coords[0] = append(coords[0], v.X)
		coords[1] = append(coords[1], v.Y)
		coords[2] = append(coords[2], v.Z)
	}
	return coords
}

// newSolid reads the solid onto the grid of its vertex coordinates and the extra ones on each axis.
func newSolid(verts []r3.Vec, tris [][3]int, axes, signs []int, extra [3][]float64) (*solidGrid, error) {
	return newSolidEps(verts, tris, axes, signs, extra, gridEps(verts))
}

// gridEps is the tolerance within which two coordinates are one grid line: a billionth of the larger of 1 mm
// and the extent of verts.
func gridEps(verts []r3.Vec) float64 {
	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return 1e-9 * math.Max(1, math.Max(hi.X-lo.X, math.Max(hi.Y-lo.Y, hi.Z-lo.Z)))
}

// newSolidEps is newSolid with the grid tolerance stated, so that two solids can be read onto one grid.
func newSolidEps(verts []r3.Vec, tris [][3]int, axes, signs []int, extra [3][]float64, eps float64) (*solidGrid, error) {
	coords := cornerCoords(verts, tris, axes, eps)
	for a := range 3 {
		if nearCoincident(coords[a], eps) {
			return nil, fmt.Errorf("%w: two vertex coordinates along axis %d differ by less than the grid tolerance %g", ErrUnsupported, a, eps)
		}
		coords[a] = append(coords[a], extra[a]...)
	}
	var u grid
	for a := range 3 {
		u.c[a] = uniqueSorted(append([]float64(nil), coords[a]...), eps)
		if len(u.c[a]) < 2 {
			return nil, fmt.Errorf("%w: the solid has no extent along axis %d", ErrUnsupported, a)
		}
	}
	ud := u.dims()
	if ud[0]*ud[1]*ud[2] > MaxCells {
		return nil, fmt.Errorf("%w: the solid needs more than %d grid cells", ErrUnsupported, MaxCells)
	}
	at := func(i, j, k int) int { return (k*ud[1]+j)*ud[0] + i }
	// cover[p] holds, for the x-facing triangles at plane p, their signed coverage of each (y, z) cell.
	cover := make([]float64, (ud[0]+1)*ud[1]*ud[2])
	coverAt := func(p, j, k int) int { return (k*ud[1]+j)*(ud[0]+1) + p }
	for i, tri := range tris {
		if axes[i] != 0 {
			continue
		}
		p := indexOf(u.c[0], verts[tri[0]].X, eps)
		if p < 0 {
			return nil, fmt.Errorf("%w: a triangle plane is missing from the grid", ErrUnsupported)
		}
		a := [2]float64{verts[tri[0]].Y, verts[tri[0]].Z}
		b := [2]float64{verts[tri[1]].Y, verts[tri[1]].Z}
		c := [2]float64{verts[tri[2]].Y, verts[tri[2]].Z}
		j0 := max(0, sort.SearchFloat64s(u.c[1], math.Min(a[0], math.Min(b[0], c[0]))-eps)-1)
		k0 := max(0, sort.SearchFloat64s(u.c[2], math.Min(a[1], math.Min(b[1], c[1]))-eps)-1)
		for k := k0; k < ud[2] && u.c[2][k] < math.Max(a[1], math.Max(b[1], c[1])); k++ {
			for j := j0; j < ud[1] && u.c[1][j] < math.Max(a[0], math.Max(b[0], c[0])); j++ {
				y0, y1, z0, z1 := u.c[1][j], u.c[1][j+1], u.c[2][k], u.c[2][k+1]
				frac := clipArea(a, b, c, y0, y1, z0, z1) / ((y1 - y0) * (z1 - z0))
				cover[coverAt(p, j, k)] += float64(-signs[i]) * frac
			}
		}
	}
	occ := make([]bool, ud[0]*ud[1]*ud[2])
	volume := 0.0
	for k := range ud[2] {
		for j := range ud[1] {
			depth := 0.0
			for i := range ud[0] {
				depth += cover[coverAt(i, j, k)]
				r := math.Round(depth)
				if math.Abs(depth-r) > 1e-6 || r < 0 || r > 1 {
					return nil, fmt.Errorf("%w: the solid is not a closed outward-wound polyhedron on its grid", ErrUnsupported)
				}
				if r == 1 {
					occ[at(i, j, k)] = true
					volume += (u.c[0][i+1] - u.c[0][i]) * (u.c[1][j+1] - u.c[1][j]) * (u.c[2][k+1] - u.c[2][k])
				}
			}
			if math.Abs(depth+cover[coverAt(ud[0], j, k)]) > 1e-6 {
				return nil, fmt.Errorf("%w: a column of the solid does not close", ErrUnsupported)
			}
		}
	}
	// The grid's volume must equal the mesh's own, or the occupancy misread the solid.
	meshVolume := 0.0
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		meshVolume += a.Dot(b.Cross(c)) / 6
	}
	if math.Abs(volume-meshVolume) > 1e-9*math.Max(1, math.Abs(meshVolume)) {
		return nil, fmt.Errorf("%w: the grid volume %g differs from the mesh volume %g", ErrUnsupported, volume, meshVolume)
	}
	return &solidGrid{u: u, ud: ud, occ: occ, eps: eps}, nil
}

// Walls returns the boundary mesh of the walls of a shell of thickness t: the solid (verts and
// tris, closed, wound outward, every triangle square to an axis) less its cavity, the set of points
// whose cube of half-side t lies in the solid joined to the slab of twice t outward from each
// triangle that removed marks. A point cube-eroded from that union is at least t from every wall and
// clear of the openings, so the cavity is the solid's inward offset with its walls mitered at
// every corner, and the walls are what is left of the solid around it. The mesh is closed,
// conforming (a grid cell's face is a quad of four triangles fanned from a point inside it) and
// wound outward from the walls, and a cavity with no volume is ErrDegenerate. The coordinates of
// the offsets are float sums x ± t, so a wall is t thick to the rounding of that sum.
//
// With outward set the walls grow off the solid instead: they are the cells within a cube of
// half-side t of the solid, less the solid, and over a removed face the footprint's prism is left
// open, so the solid is the cavity.
func Walls(verts []r3.Vec, tris [][3]int, removed []bool, t float64, outward bool) ([]r3.Vec, [][3]int, error) {
	r, rd, walls, _, err := wallCells(verts, tris, removed, t, outward)
	if err != nil {
		return nil, nil, err
	}
	return boundary(r, rd, walls)
}

// WallsPerDirection is Walls for an inward shell whose walls differ by direction: tNeg[a] is the thickness of the
// walls whose outward normal is -a and tPos[a] of those whose outward normal is +a. A wall is as thick as its own
// direction says to the rounding of the sum x +/- t, and the walls of two directions meet in a mitred corner.
func WallsPerDirection(verts []r3.Vec, tris [][3]int, removed []bool, tNeg, tPos [3]float64) ([]r3.Vec, [][3]int, error) {
	if len(verts) == 0 || len(tris) == 0 || len(removed) != len(tris) {
		return nil, nil, fmt.Errorf("%w: no triangles, or a removed mark for each is missing", ErrUnsupported)
	}
	r, rd, walls, _, err := wallCellsAniso(verts, tris, removed, tNeg, tPos, false)
	if err != nil {
		return nil, nil, err
	}
	return boundary(r, rd, walls)
}

// CenteredWalls returns the boundary mesh of a shell whose wall straddles the solid's surface, t/2
// each side: the inward walls of thickness t/2 joined to the outward walls of thickness t/2, which
// are the cells within a cube of half-side t/2 of the surface. Both are computed on the same
// refined grid, since it depends only on the solid, the removed faces and t/2, so the union is a
// cell-wise OR. A removed face opens both halves through it. The result is closed, conforming and
// wound outward from the wall.
func CenteredWalls(verts []r3.Vec, tris [][3]int, removed []bool, t float64) ([]r3.Vec, [][3]int, error) {
	h := t / 2
	r, rd, in, _, err := wallCells(verts, tris, removed, h, false)
	if err != nil {
		return nil, nil, err
	}
	r2, rd2, out, _, err := wallCells(verts, tris, removed, h, true)
	if err != nil {
		return nil, nil, err
	}
	if rd != rd2 || len(in) != len(out) {
		return nil, nil, fmt.Errorf("%w: the two halves of the wall are on different grids", ErrUnsupported)
	}
	for a := range 3 {
		if len(r.c[a]) != len(r2.c[a]) {
			return nil, nil, fmt.Errorf("%w: the two halves of the wall are on different grids", ErrUnsupported)
		}
	}
	for i := range in {
		in[i] = in[i] || out[i]
	}
	return boundary(r, rd, in)
}

// wallCells returns the refined grid of Walls and which of its cells are wall.
func wallCells(verts []r3.Vec, tris [][3]int, removed []bool, t float64, outward bool) (grid, [3]int, []bool, []bool, error) {
	return wallCellsAniso(verts, tris, removed, [3]float64{t, t, t}, [3]float64{t, t, t}, outward)
}

// wallCellsAniso is wallCells with a thickness for each of the six axis directions: tNeg[a] is the thickness of a wall
// whose outward normal is -a and tPos[a] of one whose outward normal is +a. The cavity is the solid eroded by the box
// that reaches tNeg[a] toward -a and tPos[a] toward +a, so each wall is as thick as its own direction says.
func wallCellsAniso(verts []r3.Vec, tris [][3]int, removed []bool, tNeg, tPos [3]float64, outward bool) (grid, [3]int, []bool, []bool, error) {
	for a := range 3 {
		if !(tNeg[a] > 0) || !(tPos[a] > 0) || math.IsInf(tNeg[a], 0) || math.IsInf(tPos[a], 0) {
			return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the thickness must be positive and finite", ErrDegenerate)
		}
	}
	t := tPos[0] // the isotropic outward reach; an outward shell passes one thickness for every direction
	if len(verts) == 0 || len(tris) == 0 || len(removed) != len(tris) {
		return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: no triangles, or a removed mark for each is missing", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return grid{}, [3]int{}, nil, nil, err
	}
	var extra [3][]float64
	for i, tri := range tris {
		if removed[i] {
			a := axes[i]
			extra[a] = append(extra[a], comp(verts[tri[0]], a)+float64(signs[i])*(tNeg[a]+tPos[a]))
		}
	}
	sg, err := newSolid(verts, tris, axes, signs, extra)
	if err != nil {
		return grid{}, [3]int{}, nil, nil, err
	}
	u, ud, occ, eps, at := sg.u, sg.ud, sg.occ, sg.eps, sg.at
	occS := append([]bool(nil), occ...)
	// The slab over each removed face: the cells between its plane and twice t out, over its footprint.
	// A face is several triangles that may each cover only part of a cell, so the footprint is summed
	// over the triangles of one plane before a cell is taken.
	type slabKey struct{ axis, p, sign int }
	slabs := map[slabKey][]int{}
	for i := range tris {
		if !removed[i] {
			continue
		}
		p := indexOf(u.c[axes[i]], comp(verts[tris[i][0]], axes[i]), eps)
		if p < 0 {
			return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: a removed face's plane is missing from the grid", ErrUnsupported)
		}
		k := slabKey{axes[i], p, signs[i]}
		slabs[k] = append(slabs[k], i)
	}
	type opening struct {
		axis, sign int
		plane      float64
		footprint  []float64
	}
	var openings []opening
	for key, members := range slabs {
		a := key.axis
		v, w := other(a)
		plane := u.c[a][key.p]
		q := indexOf(u.c[a], plane+float64(key.sign)*(tNeg[a]+tPos[a]), eps)
		if q < 0 {
			return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: a removed face's slab is missing from the grid", ErrUnsupported)
		}
		lo2, hi2 := min(key.p, q), max(key.p, q)
		footprint := make([]float64, ud[v]*ud[w])
		for _, i := range members {
			tri := tris[i]
			pa := [2]float64{comp(verts[tri[0]], v), comp(verts[tri[0]], w)}
			pb := [2]float64{comp(verts[tri[1]], v), comp(verts[tri[1]], w)}
			pc := [2]float64{comp(verts[tri[2]], v), comp(verts[tri[2]], w)}
			for cw := range ud[w] {
				for cv := range ud[v] {
					v0, v1, w0, w1 := u.c[v][cv], u.c[v][cv+1], u.c[w][cw], u.c[w][cw+1]
					footprint[cw*ud[v]+cv] += clipArea(pa, pb, pc, v0, v1, w0, w1) / ((v1 - v0) * (w1 - w0))
				}
			}
		}
		openings = append(openings, opening{axis: a, sign: key.sign, plane: plane, footprint: footprint})
		for cw := range ud[w] {
			for cv := range ud[v] {
				if footprint[cw*ud[v]+cv] <= 0.5 {
					continue
				}
				for ca := lo2; ca < hi2; ca++ {
					var idx [3]int
					idx[a], idx[v], idx[w] = ca, cv, cw
					occ[at(idx[0], idx[1], idx[2])] = true
				}
			}
		}
	}
	// prefix[i][j][k] counts occupied cells below (i, j, k), so a box of cells is all occupied when
	// its sum equals its size.
	src := occ
	if outward {
		src = occS
	}
	pre := make([]int32, (ud[0]+1)*(ud[1]+1)*(ud[2]+1))
	pat := func(i, j, k int) int { return (k*(ud[1]+1)+j)*(ud[0]+1) + i }
	for k := range ud[2] {
		for j := range ud[1] {
			for i := range ud[0] {
				n := int32(0)
				if src[at(i, j, k)] {
					n = 1
				}
				pre[pat(i+1, j+1, k+1)] = n + pre[pat(i, j+1, k+1)] + pre[pat(i+1, j, k+1)] + pre[pat(i+1, j+1, k)] -
					pre[pat(i, j, k+1)] - pre[pat(i, j+1, k)] - pre[pat(i+1, j, k)] + pre[pat(i, j, k)]
			}
		}
	}
	box := func(i0, i1, j0, j1, k0, k1 int) int32 {
		return pre[pat(i1, j1, k1)] - pre[pat(i0, j1, k1)] - pre[pat(i1, j0, k1)] - pre[pat(i1, j1, k0)] +
			pre[pat(i0, j0, k1)] + pre[pat(i0, j1, k0)] + pre[pat(i1, j0, k0)] - pre[pat(i0, j0, k0)]
	}
	// The refined grid adds every coordinate ± t.
	var r grid
	for a := range 3 {
		var all []float64
		for _, c := range u.c[a] {
			// A wall of either thickness has its inner face at the solid's coordinate plus that thickness, and its cube edge at minus.
			all = append(all, c-tNeg[a], c-tPos[a], c, c+tNeg[a], c+tPos[a])
		}
		r.c[a] = uniqueSorted(all, eps)
	}
	rd := r.dims()
	if rd[0]*rd[1]*rd[2] > MaxCells {
		return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the cavity needs more than %d grid cells", ErrUnsupported, MaxCells)
	}
	// span returns the range of u-cells whose interior meets the open interval (c - t, c + t) on axis a,
	// or ok false when it reaches past the grid.
	span := func(a int, c float64) (int, int, bool) {
		lo3, hi3 := c-tNeg[a], c+tPos[a]
		if lo3 < u.c[a][0]-eps || hi3 > u.c[a][len(u.c[a])-1]+eps {
			return 0, 0, false
		}
		i0 := max(0, sort.SearchFloat64s(u.c[a], lo3+eps)-1) // first cell whose right edge is beyond lo3
		i1 := sort.SearchFloat64s(u.c[a], hi3-eps)           // one past the last cell whose left edge is below hi3
		return i0, i1, i0 < i1
	}
	// reach returns the range of u-cells whose interior meets the open interval (c - t, c + t) on axis a,
	// clipped to the grid, for the outward walls, which may reach past it; the range is empty
	// when nothing is there.
	reach := func(a int, c float64) (int, int) {
		i0 := max(0, sort.SearchFloat64s(u.c[a], c-t+eps)-1)
		i1 := min(ud[a], sort.SearchFloat64s(u.c[a], c+t-eps))
		return i0, i1
	}
	rat := func(i, j, k int) int { return (k*rd[1]+j)*rd[0] + i }
	in := make([]bool, rd[0]*rd[1]*rd[2])
	hasCavity := false
	var spans [3][][2]int
	var spanOK [3][]bool
	for a := range 3 {
		spans[a] = make([][2]int, rd[a])
		spanOK[a] = make([]bool, rd[a])
		for i := range rd[a] {
			c := (r.c[a][i] + r.c[a][i+1]) / 2
			if outward {
				i0, i1 := reach(a, c)
				spans[a][i], spanOK[a][i] = [2]int{i0, i1}, i0 < i1
				continue
			}
			i0, i1, ok := span(a, c)
			spans[a][i], spanOK[a][i] = [2]int{i0, i1}, ok
		}
	}
	for k := range rd[2] {
		if !spanOK[2][k] {
			continue
		}
		for j := range rd[1] {
			if !spanOK[1][j] {
				continue
			}
			for i := range rd[0] {
				if !spanOK[0][i] {
					continue
				}
				x, y, z := spans[0][i], spans[1][j], spans[2][k]
				size := int32((x[1] - x[0]) * (y[1] - y[0]) * (z[1] - z[0]))
				sum := box(x[0], x[1], y[0], y[1], z[0], z[1])
				if (!outward && sum == size) || (outward && sum > 0) {
					in[rat(i, j, k)] = true
					hasCavity = true
				}
			}
		}
	}
	if !hasCavity {
		if outward {
			return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the solid has no reach", ErrDegenerate)
		}
		return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the walls meet", ErrDegenerate)
	}
	// The walls are the solid's own cells that the cavity does not take.
	var uCell [3][]int
	for a := range 3 {
		uCell[a] = make([]int, rd[a])
		for i := range rd[a] {
			c := (r.c[a][i] + r.c[a][i+1]) / 2
			m := sort.SearchFloat64s(u.c[a], c) - 1
			if m < 0 || m >= ud[a] {
				m = -1
			}
			uCell[a][i] = m
		}
	}
	walls := make([]bool, len(in))
	inside := make([]bool, len(in)) // the solid's own cells on the refined grid
	solid := false
	hollow := false // some cavity cell lies in the solid itself, not only in the slab over an opening
	for k := range rd[2] {
		for j := range rd[1] {
			for i := range rd[0] {
				x, y, z := uCell[0][i], uCell[1][j], uCell[2][k]
				inside[rat(i, j, k)] = x >= 0 && y >= 0 && z >= 0 && occS[at(x, y, z)]
				hollow = hollow || (inside[rat(i, j, k)] && in[rat(i, j, k)])
				if outward {
					// Within reach of the solid, and neither in it nor in the opening over a removed face.
					if !in[rat(i, j, k)] || (x >= 0 && y >= 0 && z >= 0 && occ[at(x, y, z)]) {
						continue
					}
				} else if x < 0 || y < 0 || z < 0 || !occS[at(x, y, z)] || in[rat(i, j, k)] {
					continue
				}
				walls[rat(i, j, k)] = true
				solid = true
			}
		}
	}
	if outward {
		// An outward wall ends flush with the plane of a removed face: over the footprint grown by t,
		// nothing of the wall stands beyond that plane.
		for _, op := range openings {
			a := op.axis
			v, w := other(a)
			pre := make([]int32, (ud[v]+1)*(ud[w]+1))
			pp := func(cv, cw int) int { return cw*(ud[v]+1) + cv }
			for cw := range ud[w] {
				for cv := range ud[v] {
					n := int32(0)
					if op.footprint[cw*ud[v]+cv] > 0.5 {
						n = 1
					}
					pre[pp(cv+1, cw+1)] = n + pre[pp(cv, cw+1)] + pre[pp(cv+1, cw)] - pre[pp(cv, cw)]
				}
			}
			for k := range rd[2] {
				for j := range rd[1] {
					for i := range rd[0] {
						idx := [3]int{i, j, k}
						ca := (r.c[a][idx[a]] + r.c[a][idx[a]+1]) / 2
						if d := float64(op.sign) * (ca - op.plane); !(d > 0 && d < t) {
							continue
						}
						cv0, cv1 := reach(v, (r.c[v][idx[v]]+r.c[v][idx[v]+1])/2)
						cw0, cw1 := reach(w, (r.c[w][idx[w]]+r.c[w][idx[w]+1])/2)
						if cv0 >= cv1 || cw0 >= cw1 {
							continue
						}
						if pre[pp(cv1, cw1)]-pre[pp(cv0, cw1)]-pre[pp(cv1, cw0)]+pre[pp(cv0, cw0)] > 0 {
							walls[rat(i, j, k)] = false
						}
					}
				}
			}
		}
		solid = false
		for _, wall := range walls {
			solid = solid || wall
		}
	}
	if !outward && !hollow {
		return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the walls meet", ErrDegenerate)
	}
	if !solid {
		return grid{}, [3]int{}, nil, nil, fmt.Errorf("%w: the walls leave no cells", ErrDegenerate)
	}
	return r, rd, walls, inside, nil
}

// boundary meshes the faces between occupied and empty cells of the grid, wound outward from the occupied cells.
func boundary(g grid, d [3]int, in []bool) ([]r3.Vec, [][3]int, error) {
	at := func(i, j, k int) int { return (k*d[1]+j)*d[0] + i }
	occupied := func(i, j, k int) bool {
		return i >= 0 && j >= 0 && k >= 0 && i < d[0] && j < d[1] && k < d[2] && in[at(i, j, k)]
	}
	vertex := map[[3]int]int{}
	var verts []r3.Vec
	vid := func(p [3]int) int {
		if id, ok := vertex[p]; ok {
			return id
		}
		id := len(verts)
		vertex[p] = id
		verts = append(verts, r3.Vec{X: g.c[0][p[0]], Y: g.c[1][p[1]], Z: g.c[2][p[2]]})
		return id
	}
	var tris [][3]int
	quad := func(a int, plane, cu, cv int, positive bool) {
		// The quad on axis a at grid plane `plane`, over cell (cu, cv) of the axes (a+1, a+2).
		u, v := other(a)
		corner := func(du, dv int) int {
			var p [3]int
			p[a], p[u], p[v] = plane, cu+du, cv+dv
			return vid(p)
		}
		q := [4]int{corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)}
		// The quad is fanned from a point inside it, off its centre and its diagonals, at a place
		// that varies from quad to quad. A face cut along its diagonal leaves the centres of
		// symmetric parts on an edge, and the evaluator's axis rays then cannot classify a
		// probe there; the fan keeps the probes off the edges. The point lies in the quad's own
		// plane, so the face stays planar.
		fu, fv := fanOffset(a, plane, cu, cv)
		var pos [3]float64
		pos[a] = g.c[a][plane]
		pos[u] = g.c[u][cu] + fu*(g.c[u][cu+1]-g.c[u][cu])
		pos[v] = g.c[v][cv] + fv*(g.c[v][cv+1]-g.c[v][cv])
		centre := len(verts)
		verts = append(verts, r3.Vec{X: pos[0], Y: pos[1], Z: pos[2]})
		if !positive {
			q[1], q[3] = q[3], q[1]
		}
		tris = append(tris, [3]int{q[0], q[1], centre}, [3]int{q[1], q[2], centre},
			[3]int{q[2], q[3], centre}, [3]int{q[3], q[0], centre})
	}
	for k := 0; k <= d[2]; k++ {
		for j := 0; j <= d[1]; j++ {
			for i := 0; i <= d[0]; i++ {
				cell := [3]int{i, j, k}
				// The three faces on the low side of cell (i, j, k): between it and the cell below on each axis.
				for a := range 3 {
					below := cell
					below[a]--
					here, there := occupied(cell[0], cell[1], cell[2]), occupied(below[0], below[1], below[2])
					if here == there {
						continue
					}
					u, v := other(a)
					if cell[u] >= d[u] || cell[v] >= d[v] {
						continue
					}
					// here occupied means the outward side is the low one (-a); there occupied means +a.
					quad(a, cell[a], cell[u], cell[v], there)
				}
			}
		}
	}
	// A closed manifold: every edge in exactly two triangles.
	edges := map[[2]int]int{}
	for _, t := range tris {
		for e := range 3 {
			a, b := t[e], t[(e+1)%3]
			if a > b {
				a, b = b, a
			}
			edges[[2]int{a, b}]++
		}
	}
	for _, n := range edges {
		if n != 2 {
			return nil, nil, fmt.Errorf("%w: the cavity's boundary touches itself along an edge", ErrUnsupported)
		}
	}
	return verts, tris, nil
}

// fanOffset returns where, as fractions along a quad's two sides, the fan of its triangles meets:
// a pseudo-random point in the middle half of the quad, fixed by the quad's place so the result is
// the same on every run.
func fanOffset(axis, plane, cu, cv int) (float64, float64) {
	h := uint64(axis+1)*0x9e3779b97f4a7c15 ^ uint64(plane+1)*0xbf58476d1ce4e5b9 ^ uint64(cu+1)*0x94d049bb133111eb ^ uint64(cv+1)*0xd6e8feb86659fd93
	h ^= h >> 31
	h *= 0xbf58476d1ce4e5b9
	h ^= h >> 29
	f := func(x uint64) float64 { return 0.3 + 0.4*float64(x%1000003)/1000003 }
	return f(h), f(h >> 20)
}
