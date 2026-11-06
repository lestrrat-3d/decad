package export

import (
	"fmt"
	"sort"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

// triangleNormals returns each triangle's unit normal. A triangle whose float cross product is too short to
// normalise, a sliver between held vertices that differ by rounding or by a gap-shift retry, takes the normal of
// the neighbour across its longest edge that has one. The mesh is closed, so every edge has a neighbour.
func triangleNormals(vertices []r3.Vec, triangles [][3]int) ([]r3.Vec, error) {
	normals := make([]r3.Vec, len(triangles))
	known := make([]bool, len(triangles))
	byEdge := make(map[edgeKey][]int, len(triangles)*3/2)
	pending := 0
	for i, t := range triangles {
		a, c, d := vertices[t[0]], vertices[t[1]], vertices[t[2]]
		normals[i], known[i] = c.Sub(a).Cross(d.Sub(a)).Normalize()
		if !known[i] {
			pending++
		}
		for j := range 3 {
			k := orderedEdge(t[j], t[(j+1)%3])
			byEdge[k] = append(byEdge[k], i)
		}
	}
	for pending > 0 {
		progress := false
		for i, t := range triangles {
			if known[i] {
				continue
			}
			order := [3]int{0, 1, 2}
			sort.Slice(order[:], func(p, q int) bool {
				return edgeLength(vertices, t, order[p]) > edgeLength(vertices, t, order[q])
			})
			for _, j := range order {
				for _, n := range byEdge[orderedEdge(t[j], t[(j+1)%3])] {
					if n != i && known[n] {
						normals[i], known[i] = normals[n], true
						break
					}
				}
				if known[i] {
					break
				}
			}
			if known[i] {
				pending--
				progress = true
			}
		}
		if !progress {
			return nil, fmt.Errorf("%w: a run of triangles has no plane", decad.ErrDegenerate)
		}
	}
	return normals, nil
}
