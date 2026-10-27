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
	// Covered says every triangle that collapsed to a segment or a point lies, as
	// that segment or point, on a triangle the result kept, so no part of the input
	// surface is left without a counterpart.
	Covered bool
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
	var collapsed [][3]int
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
			collapsed = append(collapsed, [3]int{a, b, c})
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

	// A collapsed triangle's image is a segment or a point, and the surface is still covered where
	// that segment is an edge of a kept triangle, or the point one of its corners.
	edges := map[[2]int]bool{}
	corners := map[int]bool{}
	for _, t := range kept {
		for k := range 3 {
			x, y := t[k], t[(k+1)%3]
			if x > y {
				x, y = y, x
			}
			edges[[2]int{x, y}] = true
			corners[x] = true
		}
	}
	covered := true
	for _, t := range collapsed {
		switch {
		case t[0] == t[1] && t[1] == t[2]:
			covered = covered && corners[t[0]]
		default:
			x, y := t[0], t[1]
			if x == y {
				y = t[2]
			}
			if x > y {
				x, y = y, x
			}
			covered = covered && edges[[2]int{x, y}]
		}
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
		DroppedTriangles: dropped, MaxMove: maxMove, Covered: covered}, nil
}

// SurfaceBound returns an upper bound, in millimetres, on how far a reduction that moved its
// vertices by at most maxMove is from the mesh it reduced, in both directions, and whether it
// holds. The reduced surface is the image of the input under the map that is affine on each
// input triangle and takes each corner to its replacement, so every input point is within
// maxMove of its image, which is on the result where its triangle survived and on a kept edge or
// corner where it collapsed (Result.Covered); and every point of the result is the image of an
// input point. The bound is maxMove raised above the rounding of the distances it was measured
// with: a relative 1e-12 and eight ulps of the largest coordinate.
func SurfaceBound(r Result, vertices []r3.Vec) (float64, bool) {
	if !r.Covered || len(r.Triangles) == 0 {
		return 0, false
	}
	largest := 0.0
	for _, v := range vertices {
		largest = math.Max(largest, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
	}
	ulp := math.Nextafter(largest, math.Inf(1)) - largest
	return r.MaxMove*(1+1e-12) + 8*ulp, true
}

// ReduceWithin reduces the mesh by the coarsest cell it can prove stays within distance of the
// input surface (SurfaceBound): it tries the cell 0.9·distance/√3, whose every vertex moves less than
// distance, then coarser cells in steps of 1.25, and keeps the proven reduction with the fewest
// triangles. ok is false when no cell can be proven, and the bound of the kept reduction is
// returned with it.
func ReduceWithin(ctx context.Context, vertices []r3.Vec, triangles [][3]int, distance float64) (Result, float64, bool, error) {
	var best Result
	bestBound := 0.0
	found := false
	size := 0.9 * distance / math.Sqrt(3)
	for range 25 {
		r, err := Reduce(ctx, vertices, triangles, size)
		if err != nil {
			return Result{}, 0, false, err
		}
		if bound, ok := SurfaceBound(r, vertices); ok && bound <= distance && (!found || len(r.Triangles) < len(best.Triangles)) {
			best, bestBound, found = r, bound, true
		}
		size *= 1.25
	}
	return best, bestBound, found, nil
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
