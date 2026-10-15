package stitchweld

import (
	"fmt"
	"iter"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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

// ZeroBound reports whether every magnitude in mags is exactly zero. budget
// charges one step per magnitude visited, before the magnitude is read, and
// its error is returned as is. An empty sequence is vacuously zero.
func ZeroBound(budget *proofbound.WorkBudget, mags iter.Seq[float64]) (bool, error) {
	for m := range mags {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if m != 0 {
			return false, nil
		}
	}
	return true, nil
}

// MaxBound returns the largest magnitude in mags, zero for an empty sequence
// or when every magnitude is zero. budget charges one step per magnitude
// visited and its error is returned as is.
func MaxBound(budget *proofbound.WorkBudget, mags iter.Seq[float64]) (float64, error) {
	best := 0.0
	for m := range mags {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		if m > best {
			best = m
		}
	}
	return best, nil
}

// UnstitchBounds publishes one unstitched face's box as the retiring
// receiver's own already-proven Bounds, inflated by its own Bound and then
// carried through xform by its 8 corners (BoxCorners) — the reasoning
// stitchrecord's bounds states for a curved Stitch operand, applied to a single face of
// the body being split rather than a whole operand body. The published Bound
// is exactly delta, on the terms stitchrecord's bounds publishes for a placed stitched
// body. It is also the box PatchEdges reuses for its rebuilt face set.
func UnstitchBounds(srcBounds measurement.Box, xform r3.Transform, delta float64) (measurement.Box, error) {
	inflate := srcBounds.Bound.Base()
	have := false
	var lo, hi r3.Vec
	for _, c := range BoxCorners(srcBounds.Min, srcBounds.Max, inflate) {
		p := xform.Apply(c)
		if !proofbound.FiniteVec(p) {
			return measurement.Box{}, fmt.Errorf(`%w: a placed unstitch bound is not representable`, decaderr.ErrUnsupported)
		}
		if !have {
			lo, hi = p, p
			have = true
			continue
		}
		lo = r3.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = r3.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	return measurement.Box{Min: lo, Max: hi, Exactness: measurement.ExactnessOf(delta), Bound: units.Millimeters(delta)}, nil
}
