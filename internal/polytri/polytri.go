// Package polytri triangulates a planar polygon with holes by ear clipping
// (agent-docs/step-import-design.md). It works on plain two-dimensional data.
package polytri

import (
	"errors"
	"fmt"
	"math"
	"sort"
)

// ErrDegenerate marks a polygon that cannot be triangulated: fewer than three
// points, no area, a hole that touches or crosses the outline, or a boundary
// that crosses itself.
var ErrDegenerate = errors.New("polytri: degenerate polygon")

// Point is a position in the plane.
type Point [2]float64

// Triangulate cuts the polygon whose outer boundary is outer, counter-clockwise,
// and whose holes are clockwise rings, into triangles. It returns the triangles
// as index triples into the points: those of outer first, then each hole's in
// order, the same order Points returns. Every triangle is counter-clockwise. The
// rings do not repeat their first point.
func Triangulate(outer []Point, holes [][]Point) (points []Point, tris [][3]int, err error) {
	for _, ring := range append([][]Point{outer}, holes...) {
		if len(ring) < 3 {
			return nil, nil, fmt.Errorf("%w: a ring of %d points", ErrDegenerate, len(ring))
		}
	}
	points = append(points, outer...)
	ring := make([]int, len(outer))
	for i := range ring {
		ring[i] = i
	}
	if signedArea(points, ring) <= 0 {
		return nil, nil, fmt.Errorf("%w: the outer boundary has no counter-clockwise area", ErrDegenerate)
	}
	// Bridge the holes in, the one with the rightmost point first, so each
	// bridge sees the outline built so far.
	type hole struct {
		indices []int
		right   int
	}
	var pending []hole
	for _, h := range holes {
		first := len(points)
		points = append(points, h...)
		indices := make([]int, len(h))
		for i := range indices {
			indices[i] = first + i
		}
		if signedArea(points, indices) >= 0 {
			return nil, nil, fmt.Errorf("%w: a hole is not clockwise", ErrDegenerate)
		}
		right := 0
		for i, idx := range indices {
			if points[idx][0] > points[indices[right]][0] {
				right = i
			}
		}
		pending = append(pending, hole{indices, right})
	}
	sort.SliceStable(pending, func(a, b int) bool {
		return points[pending[a].indices[pending[a].right]][0] > points[pending[b].indices[pending[b].right]][0]
	})
	for _, h := range pending {
		if ring, err = bridge(points, ring, h.indices, h.right); err != nil {
			return nil, nil, err
		}
	}
	tris, err = clip(points, ring)
	return points, tris, err
}

func signedArea(points []Point, ring []int) float64 {
	total := 0.0
	for i, a := range ring {
		b := ring[(i+1)%len(ring)]
		total += points[a][0]*points[b][1] - points[b][0]*points[a][1]
	}
	return total / 2
}

func cross(o, a, b Point) float64 {
	return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
}

// bridge splices the hole into the outline at a visible outline point: the hole
// is entered at its rightmost point and walked once round, and the bridge runs
// both ways between the two points.
func bridge(points []Point, ring, hole []int, right int) ([]int, error) {
	hp := points[hole[right]]
	// Cast a ray to the right from the hole point and take the nearest crossing
	// outline edge, then the outline point of that edge that lies farther right.
	bestX, bestEdge := math.Inf(1), -1
	for i, a := range ring {
		b := ring[(i+1)%len(ring)]
		pa, pb := points[a], points[b]
		if (pa[1] > hp[1]) == (pb[1] > hp[1]) {
			continue
		}
		x := pa[0] + (hp[1]-pa[1])/(pb[1]-pa[1])*(pb[0]-pa[0])
		if x >= hp[0] && x < bestX {
			bestX, bestEdge = x, i
		}
	}
	if bestEdge < 0 {
		return nil, fmt.Errorf("%w: a hole lies outside the outline", ErrDegenerate)
	}
	a, b := ring[bestEdge], ring[(bestEdge+1)%len(ring)]
	candidate := a
	if points[b][0] > points[a][0] {
		candidate = b
	}
	// Any outline point inside the triangle (hole point, crossing, candidate)
	// would block the bridge: take the one that makes the smallest angle.
	cx := Point{bestX, hp[1]}
	bestPos := -1
	for i, idx := range ring {
		if idx == candidate {
			bestPos = i
		}
	}
	bestTan := math.Inf(1)
	for i, idx := range ring {
		p := points[idx]
		if p[0] < hp[0] || idx == candidate {
			continue
		}
		if inTriangle(hp, cx, points[candidate], p) {
			tan := math.Abs(p[1]-hp[1]) / (p[0] - hp[0] + 1e-300)
			if tan < bestTan {
				bestTan, bestPos = tan, i
			}
		}
	}
	out := make([]int, 0, len(ring)+len(hole)+2)
	out = append(out, ring[:bestPos+1]...)
	for k := range hole {
		out = append(out, hole[(right+k)%len(hole)])
	}
	out = append(out, hole[right], ring[bestPos])
	out = append(out, ring[bestPos+1:]...)
	return out, nil
}

func inTriangle(a, b, c, p Point) bool {
	d1, d2, d3 := cross(a, b, p), cross(b, c, p), cross(c, a, p)
	return d1 >= 0 && d2 >= 0 && d3 >= 0 || d1 <= 0 && d2 <= 0 && d3 <= 0
}

// clip cuts ears off the ring until a triangle is left. A ring point is an ear
// tip when it turns left and no other point of the ring lies inside its triangle.
func clip(points []Point, ring []int) ([][3]int, error) {
	ring = append([]int(nil), ring...)
	var tris [][3]int
	for len(ring) > 3 {
		found := false
		for i := range ring {
			prev, cur, next := ring[(i+len(ring)-1)%len(ring)], ring[i], ring[(i+1)%len(ring)]
			a, b, c := points[prev], points[cur], points[next]
			if cross(a, b, c) <= 0 {
				continue
			}
			blocked := false
			for _, other := range ring {
				p := points[other]
				if other == prev || other == cur || other == next || p == a || p == b || p == c {
					continue
				}
				if inTriangle(a, b, c, p) {
					blocked = true
					break
				}
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{prev, cur, next})
			ring = append(ring[:i], ring[i+1:]...)
			found = true
			break
		}
		if !found {
			// A ring that is all collinear points, or crosses itself, has no ear.
			return nil, fmt.Errorf("%w: the boundary has no ear to cut", ErrDegenerate)
		}
	}
	a, b, c := ring[0], ring[1], ring[2]
	if cross(points[a], points[b], points[c]) > 0 {
		tris = append(tris, [3]int{a, b, c})
	}
	return tris, nil
}
