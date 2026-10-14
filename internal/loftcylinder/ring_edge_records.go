package loftcylinder

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ringEdgeClass is where an edge's squared radii put it against the blank.
type ringEdgeClass int8

const (
	ringEdgeInner ringEdgeClass = iota
	ringEdgeOuter
	ringEdgeSeam
)

// ringEdgeRecord is what RingEdgesInside reads of one ring edge before any
// tooth level enters: the exact XY ends, and the comparisons of the edge's
// squared radii with the bore and the blank. Each is a function of the edge
// and the two radii alone.
type ringEdgeRecord struct {
	src          [2][3]*big.Int
	built        bool
	amin, bmin   XY
	reachesBore  bool
	witness      bool
	class        ringEdgeClass
	boxed        bool
	xyBox        [2][2]float64
	boreSquared  *big.Rat
	blankSquared *big.Rat
}

// RingEdgeRecords holds the per-edge records of one ring. The direct and the
// reflected check of a ring read the same edges at different tooth levels, so
// they can share one value, and each edge's record is built on its first use.
// The zero value is empty and ready. Each record holds its own copies of the
// edge integers and radii it was built from, and is reused only when the
// current edge and radii equal them by value. A different ring, or an edge
// integer or radius rewritten in place, rebuilds the record.
// It is not safe for concurrent use.
type RingEdgeRecords struct {
	edges []ringEdgeRecord
}

// recordOf returns the record of edge, built if it is not yet.
func (r *RingEdgeRecords) recordOf(index int, edge [2]proofarith.Xpt, bore2, blank2 *big.Rat) *ringEdgeRecord {
	for len(r.edges) <= index {
		r.edges = append(r.edges, ringEdgeRecord{})
	}
	rec := &r.edges[index]
	if rec.built && rec.sameSource(edge) && rec.boreSquared.Cmp(bore2) == 0 && rec.blankSquared.Cmp(blank2) == 0 {
		return rec
	}
	*rec = ringEdgeRecord{src: copyEdgeSource(edge), built: true,
		boreSquared: new(big.Rat).Set(bore2), blankSquared: new(big.Rat).Set(blank2)}
	a, b := ringXYPoint(edge[0]), ringXYPoint(edge[1])
	rec.amin, rec.bmin = a, b
	lazyBore, lazyBlank := proofarith.LazyOf(bore2), proofarith.LazyOf(blank2)
	min2 := sourceRadiusMin2Lazy(a, b)
	rec.reachesBore = proofarith.LazyCmp(min2, lazyBore) <= 0
	ra, rb := sourceDotLazy(a, a), sourceDotLazy(b, b)
	aBelow := proofarith.LazyCmp(ra, lazyBlank) < 0
	bBelow := proofarith.LazyCmp(rb, lazyBlank) < 0
	rec.witness = aBelow || bBelow
	switch {
	case aBelow && bBelow:
		rec.class = ringEdgeInner
	case proofarith.LazyCmp(min2, lazyBlank) > 0:
		rec.class = ringEdgeOuter
	default:
		rec.class = ringEdgeSeam
	}
	return rec
}

// copyEdgeSource copies the X, Y and W integers the record is built from.
// The record owns its copies, so a caller that rewrites an edge's integers
// in place cannot make the record match the changed edge.
func copyEdgeSource(edge [2]proofarith.Xpt) [2][3]*big.Int {
	var src [2][3]*big.Int
	for i, p := range edge {
		src[i] = [3]*big.Int{new(big.Int).Set(p.X), new(big.Int).Set(p.Y), new(big.Int).Set(p.W)}
	}
	return src
}

// sameSource reports whether edge has the values the record was built from.
func (rec *ringEdgeRecord) sameSource(edge [2]proofarith.Xpt) bool {
	for i, p := range edge {
		if rec.src[i][0].Cmp(p.X) != 0 || rec.src[i][1].Cmp(p.Y) != 0 || rec.src[i][2].Cmp(p.W) != 0 {
			return false
		}
	}
	return true
}

// ringXYPoint is the exact XY of a homogeneous point, as RingPoint gives it.
func ringXYPoint(p proofarith.Xpt) XY {
	return XY{X: new(big.Rat).SetFrac(p.X, p.W), Y: new(big.Rat).SetFrac(p.Y, p.W)}
}

// box returns the outward-rounded XY box of the edge, built on first use.
func (rec *ringEdgeRecord) box() [2][2]float64 {
	if rec.boxed {
		return rec.xyBox
	}
	rec.boxed = true
	for axis, ends := range [2][2]*big.Rat{{rec.amin.X, rec.bmin.X}, {rec.amin.Y, rec.bmin.Y}} {
		lo, hi := ends[0], ends[1]
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		rec.xyBox[axis] = [2]float64{proofbound.RatFloatDown(lo), proofbound.RatFloatUp(hi)}
	}
	return rec.xyBox
}

// sourceDotLazy is SourceDot without cancelling common factors.
func sourceDotLazy(a, b XY) proofarith.LazyRat {
	return proofarith.LazyAdd(
		proofarith.LazyMul(proofarith.LazyOf(a.X), proofarith.LazyOf(b.X)),
		proofarith.LazyMul(proofarith.LazyOf(a.Y), proofarith.LazyOf(b.Y)))
}

// sourceRadiusMin2Lazy is sourceRadiusMin2 as an uncancelled fraction, for
// callers that only compare it. With d = b - a, the closest point of the
// segment is a when a.d >= 0, b when -a.d >= d.d, and otherwise the
// foot of the perpendicular, where the squared radius is
// a.a - (a.d)^2 / d.d.
func sourceRadiusMin2Lazy(a, b XY) proofarith.LazyRat {
	d := XY{X: proofarith.SubRat(new(big.Rat), b.X, a.X), Y: proofarith.SubRat(new(big.Rat), b.Y, a.Y)}
	den := sourceDotLazy(d, d)
	if den.Sign() == 0 {
		return sourceDotLazy(a, a)
	}
	along := sourceDotLazy(a, d)
	if along.Sign() >= 0 {
		return sourceDotLazy(a, a)
	}
	if proofarith.LazyCmp(proofarith.LazyNeg(along), den) >= 0 {
		return sourceDotLazy(b, b)
	}
	return proofarith.LazySub(sourceDotLazy(a, a),
		proofarith.LazyQuo(proofarith.LazyMul(along, along), den))
}
