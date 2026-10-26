// Package meshreduce thins an indexed triangle mesh by vertex clustering
// (agent-docs/mesh-import-design.md §9). It works on plain millimetre data and
// touches no body or document.
package meshreduce

import (
	"context"
	"math"

	"github.com/lestrrat-3d/r3"
)

// Result is a reduced mesh.
type Result struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	// MergedVertices counts the input vertices that shared a cell with an
	// earlier one: len(input) − len(Vertices) once unused vertices are counted.
	MergedVertices int
	// DroppedTriangles counts the triangles that collapsed or repeated another.
	DroppedTriangles int
	// MaxMove is the greatest distance from an input vertex to the vertex that
	// replaced it, in millimetres.
	MaxMove float64
}

type cell [3]int64

// Reduce snaps the vertices of each cubic cell of side cellSize, cells aligned
// to the origin, to one vertex at their mean, and rebuilds the triangles over
// those. A triangle whose corners fall in fewer than three cells, or that
// repeats an earlier triangle's cells, is dropped. Vertices that no remaining
// triangle uses are dropped too, and the order of the rest is the order of
// their cells' first vertices, so the result is deterministic. cellSize is
// positive and finite.
func Reduce(ctx context.Context, vertices []r3.Vec, triangles [][3]int, cellSize float64) (Result, error) {
	cellOf := make([]int, len(vertices))
	ids := map[cell]int{}
	var sums []r3.Vec
	var counts []int
	for i, v := range vertices {
		if i%65536 == 0 {
			if err := ctx.Err(); err != nil {
				return Result{}, err
			}
		}
		c := cell{int64(math.Floor(v.X / cellSize)), int64(math.Floor(v.Y / cellSize)), int64(math.Floor(v.Z / cellSize))}
		id, ok := ids[c]
		if !ok {
			id = len(sums)
			ids[c] = id
			sums = append(sums, r3.Vec{})
			counts = append(counts, 0)
		}
		sums[id] = sums[id].Add(v)
		counts[id]++
		cellOf[i] = id
	}
	means := make([]r3.Vec, len(sums))
	for i := range sums {
		means[i] = sums[i].Scale(1 / float64(counts[i]))
	}
	maxMove := 0.0
	for i, v := range vertices {
		maxMove = math.Max(maxMove, v.Sub(means[cellOf[i]]).Len())
	}

	seen := map[[3]int]bool{}
	var kept [][3]int
	dropped := 0
	for i, t := range triangles {
		if i%65536 == 0 {
			if err := ctx.Err(); err != nil {
				return Result{}, err
			}
		}
		a, b, c := cellOf[t[0]], cellOf[t[1]], cellOf[t[2]]
		if a == b || b == c || a == c {
			dropped++
			continue
		}
		key := sorted(a, b, c)
		if seen[key] {
			dropped++
			continue
		}
		seen[key] = true
		kept = append(kept, [3]int{a, b, c})
	}

	// Renumber the cells in the order the kept triangles first use them.
	index := make([]int, len(means))
	for i := range index {
		index[i] = -1
	}
	var out []r3.Vec
	for i, t := range kept {
		for k, c := range t {
			if index[c] < 0 {
				index[c] = len(out)
				out = append(out, means[c])
			}
			kept[i][k] = index[c]
		}
	}
	return Result{Vertices: out, Triangles: kept, MergedVertices: len(vertices) - len(means),
		DroppedTriangles: dropped, MaxMove: maxMove}, nil
}

func sorted(a, b, c int) [3]int {
	if a > b {
		a, b = b, a
	}
	if b > c {
		b, c = c, b
	}
	if a > b {
		a, b = b, a
	}
	return [3]int{a, b, c}
}
