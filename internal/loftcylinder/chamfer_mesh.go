package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// ChamferCapTriangles triangulates one held cap annulus between the inset
// outer ring and the bore ring and returns the triangles in mesh indices,
// reversed for the bottom cap.
func ChamferCapTriangles(ctx context.Context, verts []r3.Vec,
	outer, inner []int, bottom bool) ([][3]int, error) {
	points := make([]triangulation.Point2, 0, len(outer)+len(inner))
	outerLocal := make([]int, len(outer))
	for i, index := range outer {
		if index < 0 || index >= len(verts) {
			return nil, fmt.Errorf("%w: cap band has an invalid outer vertex", decaderr.ErrUnsupported)
		}
		p := verts[index]
		outerLocal[i] = len(points)
		points = append(points, triangulation.Point2{U: p.X, V: p.Y})
	}
	loops := [][]int{outerLocal}
	if len(inner) > 0 {
		hole := make([]int, len(inner))
		for i := range inner {
			index := inner[len(inner)-1-i]
			if index < 0 || index >= len(verts) {
				return nil, fmt.Errorf("%w: cap band has an invalid inner vertex", decaderr.ErrUnsupported)
			}
			p := verts[index]
			hole[i] = len(points)
			points = append(points, triangulation.Point2{U: p.X, V: p.Y})
		}
		loops = append(loops, hole)
	}
	tris, err := triangulation.Triangulate(ctx, points, loops)
	if err != nil {
		return nil, triangulation.WrapLoftError(err)
	}
	local := append(append([]int(nil), outer...), reverseInts(inner)...)
	out := make([][3]int, 0, len(tris))
	for _, tri := range tris {
		mapped := [3]int{local[tri[0]], local[tri[1]], local[tri[2]]}
		if bottom {
			mapped[1], mapped[2] = mapped[2], mapped[1]
		}
		out = append(out, mapped)
	}
	return out, nil
}

// ChamferBandTriangles returns the cap band's ruling, diagonal and corner-fan
// triangles, one strip per source edge.
//
// Each old source edge has one offset line. Its side section has a ruling,
// diagonal, then the next ruling. These local strips preserve that order;
// a reflex offset arc receives a fan only at its own source corner.
func ChamferBandTriangles(side, inset, lineStart, arcStart []int, pointCount int) ([][3]int, error) {
	n := len(lineStart)
	if len(side) != 2*n || len(arcStart) != n || len(inset) != pointCount {
		return nil, fmt.Errorf("%w: loft/cylinder band has unmatched source pieces", decaderr.ErrUnsupported)
	}
	var out [][3]int
	for i := range n {
		j := (i + 1) % n
		q, diag, next := side[2*i], side[2*i+1], side[2*j]
		line := inset[lineStart[i]]
		end := inset[lineStart[j]]
		if arcStart[j] >= 0 {
			end = inset[arcStart[j]]
		}
		out = append(out, [3]int{diag, next, end})
		out = append(out, [3]int{diag, end, line})
		out = append(out, [3]int{q, diag, line})
		if arcStart[i] < 0 {
			continue
		}
		// Walk the arc backwards from the next offset line's start. The
		// first triangle above already owns the final arc-to-line segment.
		current := lineStart[i]
		for current != arcStart[i] {
			previous := (current - 1 + len(inset)) % len(inset)
			out = append(out, [3]int{q, inset[current], inset[previous]})
			current = previous
		}
	}
	return out, nil
}

func reverseInts(input []int) []int {
	out := make([]int, len(input))
	for i := range input {
		out[i] = input[len(input)-1-i]
	}
	return out
}

// ChamferCompact drops unused vertices, remaps tris in place and returns the
// kept vertices, their bounds and the old-to-new index.
func ChamferCompact(verts []r3.Vec, tris [][3]int, bounds []float64) ([]r3.Vec, []float64, []int) {
	used := make([]bool, len(verts))
	for _, tri := range tris {
		for _, index := range tri {
			used[index] = true
		}
	}
	index := make([]int, len(verts))
	compacted := make([]r3.Vec, 0, len(verts))
	newBounds := make([]float64, 0, len(bounds))
	for i, yes := range used {
		if !yes {
			continue
		}
		index[i] = len(compacted)
		compacted = append(compacted, verts[i])
		newBounds = append(newBounds, bounds[i])
	}
	for i := range tris {
		for k := range 3 {
			tris[i][k] = index[tris[i][k]]
		}
	}
	return compacted, newBounds, index
}
