package capcontour

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Point is a rational-interval enclosure of one plane-local (u, v) point.
type Point struct{ U, V proofbound.RatInterval }

// InsideSignOf reads a circular wall's walked sense: positive when its
// material lies inside the circle and negative when it lies outside.
func InsideSignOf(w survey2d.SideWalk) *big.Rat {
	if w.Th1 < w.Th0 {
		return big.NewRat(-1, 1)
	}
	return big.NewRat(1, 1)
}

// CapWallRadiusOffset is the exact cap contour radius change, -insideSign*d.
// A non-finite setback has no rational offset and returns nil.
func CapWallRadiusOffset(w survey2d.SideWalk, d float64) *big.Rat {
	rd := proofarith.FloatRat(d)
	if rd == nil {
		return nil
	}
	return new(big.Rat).Neg(new(big.Rat).Mul(InsideSignOf(w), rd))
}

// ExactPoint lifts a pair of float64 coordinates, which are exact rationals.
func ExactPoint(u, v float64) (Point, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	if ru == nil || rv == nil {
		return Point{}, false
	}
	return Point{U: proofbound.PointInterval(ru), V: proofbound.PointInterval(rv)}, true
}

// Reach is an upper bound on |p − q| over every q the enclosure holds, so a
// point known to lie in the enclosure sits at most this far from p.
func (e Point) Reach(u, v float64) float64 {
	du, okU := AxisSpread(e.U, u)
	dv, okV := AxisSpread(e.V, v)
	if !okU || !okV {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	))
}

// AxisSpread is max(|lo − c|, |hi − c|), the furthest the interval reaches
// from c along one axis.
func AxisSpread(iv proofbound.RatInterval, c float64) (*big.Rat, bool) {
	rc := proofarith.FloatRat(c)
	if rc == nil || iv.Lo == nil || iv.Hi == nil {
		return nil, false
	}
	lo := new(big.Rat).Abs(new(big.Rat).Sub(iv.Lo, rc))
	hi := new(big.Rat).Abs(new(big.Rat).Sub(iv.Hi, rc))
	if lo.Cmp(hi) >= 0 {
		return lo, true
	}
	return hi, true
}

// Union is the smallest box holding both enclosures — what an ambiguous root
// selection reports, so an unresolved choice widens the displacement rather
// than picking a branch the exact arithmetic has not decided.
func Union(a, b Point) Point {
	return Point{U: IntervalHull(a.U, b.U), V: IntervalHull(a.V, b.V)}
}

// IntervalHull returns the smallest interval containing both inputs.
func IntervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) < 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) > 0 {
		hi = b.Hi
	}
	return proofbound.Interval(lo, hi)
}

// UnitVec encloses the EXACT unit vector of a float pair — the value
// normalize2 rounds. The pair itself is exact, so the only widening is the
// length's own outward-rounded square root.
func UnitVec(x, y float64) (Point, bool) {
	rx, ry := proofarith.FloatRat(x), proofarith.FloatRat(y)
	if rx == nil || ry == nil {
		return Point{}, false
	}
	n2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx), new(big.Rat).Mul(ry, ry))
	if n2.Sign() == 0 {
		return Point{}, false
	}
	l, ok := proofbound.IntervalSqrt(proofbound.PointInterval(n2))
	if !ok || l.Lo.Sign() <= 0 {
		return Point{}, false
	}
	u, okU := proofbound.IntervalQuo(proofbound.PointInterval(rx), l)
	v, okV := proofbound.IntervalQuo(proofbound.PointInterval(ry), l)
	if !okU || !okV {
		return Point{}, false
	}
	return Point{U: u, V: v}, true
}

// Nearest encloses intersectOffsets' own "root nearest the corner". A
// candidate whose squared-distance interval starts beyond another's end is
// PROVEN not to be the nearest and is dropped; every candidate the exact
// arithmetic leaves undecided joins the hull, so a near-tangency reports one
// wide honest displacement rather than a branch nothing decided.
func Nearest(cands []Point, vU, vV float64) (Point, bool) {
	corner, ok := ExactPoint(vU, vV)
	if !ok {
		return Point{}, false
	}
	return NearestTo(cands, corner)
}

// NearestTo is Nearest about an enclosed corner: a candidate is dropped
// only when its squared distance to EVERY corner the box holds is proven
// beyond another candidate's farthest.
func NearestTo(cands []Point, corner Point) (Point, bool) {
	if len(cands) == 0 {
		return Point{}, false
	}
	d2 := make([]proofbound.RatInterval, len(cands))
	for i, c := range cands {
		d2[i] = proofbound.IntervalAdd(
			proofbound.IntervalSquare(proofbound.IntervalSub(c.U, corner.U)),
			proofbound.IntervalSquare(proofbound.IntervalSub(c.V, corner.V)),
		)
	}
	best := d2[0].Hi
	for _, iv := range d2[1:] {
		if iv.Hi.Cmp(best) < 0 {
			best = iv.Hi
		}
	}
	var out Point
	found := false
	for i, iv := range d2 {
		if iv.Lo.Cmp(best) > 0 {
			continue
		}
		if !found {
			out, found = cands[i], true
			continue
		}
		out = Union(out, cands[i])
	}
	return out, found
}

// OffsetFootOver encloses the material-side foot v + t·rot90(unit(tu, tv))
// for every offset amount t in span, reading the float pair (tu, tv) as an
// exact direction. A held walk tangent is not one for a line walk, whose
// recorded endpoints state the direction (OffsetFootEnclosure).
func OffsetFootOver(vU, vV, tu, tv float64, span proofbound.RatInterval) (Point, bool) {
	n, ok := UnitVec(tu, tv)
	if !ok {
		return Point{}, false
	}
	v, okV := ExactPoint(vU, vV)
	if !okV {
		return Point{}, false
	}
	return Point{
		U: proofbound.IntervalAdd(v.U, proofbound.IntervalMul(proofbound.IntervalNeg(n.V), span)),
		V: proofbound.IntervalAdd(v.V, proofbound.IntervalMul(n.U, span)),
	}, true
}
