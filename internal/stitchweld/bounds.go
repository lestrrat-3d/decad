package stitchweld

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// lumpBox encloses one component's held vertices under their own bounds.
type lumpBox struct {
	lo, hi r3.Vec
	have   bool
}

// include charges the vertex's bound once on each side of its held position.
func (box *lumpBox) include(p r3.Vec, bound float64) {
	lo := r3.Vec{X: p.X - bound, Y: p.Y - bound, Z: p.Z - bound}
	hi := r3.Vec{X: p.X + bound, Y: p.Y + bound, Z: p.Z + bound}
	if !box.have {
		box.lo, box.hi, box.have = lo, hi, true
		return
	}
	box.lo = r3.Vec{X: math.Min(box.lo.X, lo.X), Y: math.Min(box.lo.Y, lo.Y), Z: math.Min(box.lo.Z, lo.Z)}
	box.hi = r3.Vec{X: math.Max(box.hi.X, hi.X), Y: math.Max(box.hi.Y, hi.Y), Z: math.Max(box.hi.Z, hi.Z)}
}

// lumpBoxFromFaces visits each distinct vertex once, preserving face and use order.
func lumpBoxFromFaces[F any, V comparable](faces []F, walk func(F, func(V)),
	position func(V) (r3.Vec, float64, bool)) lumpBox {
	var box lumpBox
	seen := map[V]struct{}{}
	visit := func(v V) {
		p, bound, ok := position(v)
		if !ok {
			return
		}
		if _, ok := seen[v]; ok {
			return
		}
		seen[v] = struct{}{}
		box.include(p, bound)
	}
	for _, f := range faces {
		walk(f, visit)
	}
	return box
}

// lumpBoxesSeparated requires a strict gap on at least one coordinate axis.
func lumpBoxesSeparated(a, b lumpBox) bool {
	if !a.have || !b.have {
		return false
	}
	return a.hi.X < b.lo.X || b.hi.X < a.lo.X ||
		a.hi.Y < b.lo.Y || b.hi.Y < a.lo.Y ||
		a.hi.Z < b.lo.Z || b.hi.Z < a.lo.Z
}

// LumpsProvenSeparate compares every connected component's bounded vertex box.
// A touching, nested, or overlapping pair has no proven separation.
func LumpsProvenSeparate[F any, V comparable](components [][]F, walk func(F, func(V)),
	position func(V) (r3.Vec, float64, bool)) bool {
	if len(components) < 2 {
		return true
	}
	boxes := make([]lumpBox, len(components))
	for i, c := range components {
		boxes[i] = lumpBoxFromFaces(c, walk, position)
	}
	for i := range boxes {
		for j := i + 1; j < len(boxes); j++ {
			if !lumpBoxesSeparated(boxes[i], boxes[j]) {
				return false
			}
		}
	}
	return true
}

// BoxCorners expands a box before enumerating its corners in stitch order.
func BoxCorners(boxMin, boxMax r3.Vec, inflate float64) [8]r3.Vec {
	lo := r3.Vec{X: boxMin.X - inflate, Y: boxMin.Y - inflate, Z: boxMin.Z - inflate}
	hi := r3.Vec{X: boxMax.X + inflate, Y: boxMax.Y + inflate, Z: boxMax.Z + inflate}
	return [8]r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z},
		{X: lo.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: lo.Y, Z: hi.Z},
		{X: hi.X, Y: hi.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z},
		{X: lo.X, Y: hi.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z},
	}
}
