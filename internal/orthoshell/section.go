package orthoshell

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/r3"
)

// SectionLoops returns the loops where the plane normal to axis at coord cuts an orthogonal polyhedron (verts
// and tris, closed, wound outward, every triangle square to an axis), in the world coordinates of the two
// axes ia and iv, which MUST be the two other axes in either order. The plane MUST lie strictly inside one
// layer of the solid's grid, farther than the grid tolerance from every grid line, so the cut is the same
// at either side of it. The loops are the boundary of the union of the layer's occupied cells: an outer loop
// runs counter-clockwise in (ia, iv) and a hole clockwise, and collinear points are merged, so each
// corner of a loop is a coordinate of the solid. The loops come in the order of their lowest, leftmost corners (by iv, then ia). A layer with no cell, a plane on a grid line or outside the
// solid, and a cut whose cells meet only at a corner are ErrUnsupported.
func SectionLoops(verts []r3.Vec, tris [][3]int, axis int, coord float64, ia, iv int) ([][][2]float64, error) {
	if len(verts) == 0 || len(tris) == 0 || axis < 0 || axis > 2 || ia == iv || ia == axis || iv == axis ||
		ia < 0 || ia > 2 || iv < 0 || iv > 2 {
		return nil, fmt.Errorf("%w: a section needs a solid, an axis and the two other axes", ErrUnsupported)
	}
	axes, signs, err := classify(verts, tris)
	if err != nil {
		return nil, err
	}
	sg, err := newSolid(verts, tris, axes, signs, [3][]float64{})
	if err != nil {
		return nil, err
	}
	lines := sg.u.c[axis]
	k := -1
	for i := range len(lines) - 1 {
		if coord > lines[i]+sg.eps && coord < lines[i+1]-sg.eps {
			k = i
		}
	}
	if k < 0 {
		return nil, fmt.Errorf("%w: the section plane lies on a grid line of the solid or outside it", ErrUnsupported)
	}
	ni, nj := sg.ud[ia], sg.ud[iv]
	occ := func(i, j int) bool {
		if i < 0 || j < 0 || i >= ni || j >= nj {
			return false
		}
		var idx [3]int
		idx[axis], idx[ia], idx[iv] = k, i, j
		return sg.occ[sg.at(idx[0], idx[1], idx[2])]
	}
	type vertex struct{ i, j int }
	next := map[vertex]vertex{}
	add := func(a, b vertex) error {
		if _, taken := next[a]; taken {
			return fmt.Errorf("%w: the section has cells that meet only at a corner", ErrUnsupported)
		}
		next[a] = b
		return nil
	}
	filled := false
	for j := range nj {
		for i := range ni {
			if !occ(i, j) {
				continue
			}
			filled = true
			var err error
			if !occ(i, j-1) {
				err = add(vertex{i, j}, vertex{i + 1, j})
			}
			if err == nil && !occ(i+1, j) {
				err = add(vertex{i + 1, j}, vertex{i + 1, j + 1})
			}
			if err == nil && !occ(i, j+1) {
				err = add(vertex{i + 1, j + 1}, vertex{i, j + 1})
			}
			if err == nil && !occ(i-1, j) {
				err = add(vertex{i, j + 1}, vertex{i, j})
			}
			if err != nil {
				return nil, err
			}
		}
	}
	if !filled {
		return nil, fmt.Errorf("%w: the section layer holds no material", ErrUnsupported)
	}
	// The loops are traced from their lowest, leftmost corner in that order, so the result does not depend on map order.
	starts := make([]vertex, 0, len(next))
	for start := range next {
		starts = append(starts, start)
	}
	sort.Slice(starts, func(a, b int) bool {
		if starts[a].j != starts[b].j {
			return starts[a].j < starts[b].j
		}
		return starts[a].i < starts[b].i
	})
	var loops [][][2]float64
	for _, start := range starts {
		if _, live := next[start]; !live {
			continue
		}
		var corners []vertex
		for at := start; ; {
			to, ok := next[at]
			if !ok {
				return nil, fmt.Errorf("%w: a section loop does not close", ErrUnsupported)
			}
			delete(next, at)
			corners = append(corners, at)
			at = to
			if at == start {
				break
			}
		}
		// Merge collinear corners: keep a corner where the direction changes.
		var kept [][2]float64
		for n, c := range corners {
			prev, nxt := corners[(n+len(corners)-1)%len(corners)], corners[(n+1)%len(corners)]
			if (c.i-prev.i)*(nxt.j-c.j) == (c.j-prev.j)*(nxt.i-c.i) {
				continue
			}
			kept = append(kept, [2]float64{sg.u.c[ia][c.i], sg.u.c[iv][c.j]})
		}
		if len(kept) < 4 {
			return nil, fmt.Errorf("%w: a section loop has fewer than four corners", ErrUnsupported)
		}
		loops = append(loops, kept)
	}
	for _, loop := range loops {
		for _, c := range loop {
			if math.IsNaN(c[0]) || math.IsNaN(c[1]) {
				return nil, fmt.Errorf("%w: a section corner is not finite", ErrUnsupported)
			}
		}
	}
	return loops, nil
}
