package capcontour

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Carrier is one wall's offset carrier, enclosed: an offset LINE (a point on
// it plus its unit direction) or a concentric CIRCLE (the wall's own exact
// centre and the exact offset radius). It mirrors shell_offset.go's
// offsetCarrier field for field, so the enclosure is of the same object the
// float miter solve intersects.
type Carrier struct {
	IsLine bool
	P, Dir Point
	C      Point
	R      proofbound.RatInterval
}

// Intersect encloses every root of the two offset carriers, dispatching
// exactly as fillet.go's intersectOffsets does over the same three cases.
func Intersect(a, b Carrier) ([]Point, bool) {
	switch {
	case a.IsLine && b.IsLine:
		return lineLine(a, b)
	case a.IsLine:
		return lineCircle(a, b)
	case b.IsLine:
		return lineCircle(b, a)
	default:
		return circleCircle(a, b)
	}
}

func lineLine(a, b Carrier) ([]Point, bool) {
	den := proofbound.IntervalSub(proofbound.IntervalMul(a.Dir.U, b.Dir.V), proofbound.IntervalMul(a.Dir.V, b.Dir.U))
	num := proofbound.IntervalSub(
		proofbound.IntervalMul(proofbound.IntervalSub(b.P.U, a.P.U), b.Dir.V),
		proofbound.IntervalMul(proofbound.IntervalSub(b.P.V, a.P.V), b.Dir.U),
	)
	s, ok := proofbound.IntervalQuo(num, den)
	if !ok {
		return nil, false
	}
	return []Point{{
		U: proofbound.IntervalAdd(a.P.U, proofbound.IntervalMul(s, a.Dir.U)),
		V: proofbound.IntervalAdd(a.P.V, proofbound.IntervalMul(s, a.Dir.V)),
	}}, true
}

func lineCircle(l, c Carrier) ([]Point, bool) {
	fx := proofbound.IntervalSub(l.P.U, c.C.U)
	fy := proofbound.IntervalSub(l.P.V, c.C.V)
	bb := proofbound.IntervalAdd(proofbound.IntervalMul(fx, l.Dir.U), proofbound.IntervalMul(fy, l.Dir.V))
	cc := proofbound.IntervalSub(proofbound.IntervalAdd(proofbound.IntervalSquare(fx), proofbound.IntervalSquare(fy)), proofbound.IntervalSquare(c.R))
	disc := proofbound.IntervalSub(proofbound.IntervalSquare(bb), cc)
	if disc.Hi.Sign() < 0 {
		// The exact carriers miss each other entirely: the float solve reached
		// a root of a system that has none, so there is no denoted point to
		// enclose.
		return nil, false
	}
	sq, ok := proofbound.IntervalSqrt(disc)
	if !ok {
		return nil, false
	}
	nb := proofbound.IntervalNeg(bb)
	out := make([]Point, 0, 2)
	for _, s := range []proofbound.RatInterval{proofbound.IntervalAdd(nb, sq), proofbound.IntervalSub(nb, sq)} {
		out = append(out, Point{
			U: proofbound.IntervalAdd(l.P.U, proofbound.IntervalMul(s, l.Dir.U)),
			V: proofbound.IntervalAdd(l.P.V, proofbound.IntervalMul(s, l.Dir.V)),
		})
	}
	return out, true
}

func circleCircle(a, b Carrier) ([]Point, bool) {
	dx := proofbound.IntervalSub(b.C.U, a.C.U)
	dy := proofbound.IntervalSub(b.C.V, a.C.V)
	dsq := proofbound.IntervalAdd(proofbound.IntervalSquare(dx), proofbound.IntervalSquare(dy))
	dist, ok := proofbound.IntervalSqrt(dsq)
	if !ok || dist.Lo.Sign() <= 0 {
		return nil, false
	}
	mid, okMid := proofbound.IntervalQuo(
		proofbound.IntervalSub(proofbound.IntervalAdd(dsq, proofbound.IntervalSquare(a.R)), proofbound.IntervalSquare(b.R)),
		proofbound.IntervalScale(dist, big.NewRat(2, 1)),
	)
	if !okMid {
		return nil, false
	}
	h2 := proofbound.IntervalSub(proofbound.IntervalSquare(a.R), proofbound.IntervalSquare(mid))
	if h2.Hi.Sign() < 0 {
		return nil, false
	}
	h, okH := proofbound.IntervalSqrt(h2)
	if !okH {
		return nil, false
	}
	along, okA := proofbound.IntervalQuo(mid, dist)
	across, okC := proofbound.IntervalQuo(h, dist)
	if !okA || !okC {
		return nil, false
	}
	baseU := proofbound.IntervalAdd(a.C.U, proofbound.IntervalMul(along, dx))
	baseV := proofbound.IntervalAdd(a.C.V, proofbound.IntervalMul(along, dy))
	offU := proofbound.IntervalMul(across, dy)
	offV := proofbound.IntervalMul(across, dx)
	return []Point{
		{U: proofbound.IntervalSub(baseU, offU), V: proofbound.IntervalAdd(baseV, offV)},
		{U: proofbound.IntervalAdd(baseU, offU), V: proofbound.IntervalSub(baseV, offV)},
	}, true
}

// carrierOverRange is CarrierOver over the float offset range [t0, t1],
// enclosing every carrier the wall's own offset construction occupies as the
// offset amount ranges over it. A line's carrier stays a single line: only its
// anchor point moves, along the line's own FIXED unit normal, so the direction
// needs no widening at all. A circle's carrier stays a single concentric
// circle whose radius encloses the offset radius's own range.
func carrierOverRange(w survey2d.SideWalk, t0, t1 float64) (Carrier, bool) {
	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return Carrier{}, false
	}
	return CarrierOver(w, proofbound.Interval(rt0, rt1))
}

// CarrierOver is carrierOverRange over an offset interval stated exactly:
// every carrier the wall's offset takes as the offset amount ranges over
// span. It is OffsetCarrierEnclosure's carrier.
//
// A line's carrier runs from the start the walk's end bound encloses, along
// the unit direction of the difference of the two enclosed endpoints. It never
// reads the walk's held tangent, which is that difference rounded to float64
// and so tilts the carrier by up to half an ulp of each component. A circle's
// carrier is concentric about the recorded centre, at every radius
// OffsetCircleRadius encloses. That radius starts from the walk's held radius
// widened by its RadiusBound, since an ArcSeg walk holds the math.Hypot of its
// recorded Start − Center, which can sit off the radius the record denotes.
func CarrierOver(w survey2d.SideWalk, span proofbound.RatInterval) (Carrier, bool) {
	return OffsetCarrierEnclosure(w, span)
}

// OffsetSpan is every offset amount a setback held as the float d denotes when
// its unit conversion committed at most dDelta: [d − dDelta, d + dDelta],
// formed exactly. A setback stated in millimetres converts with no rounding,
// and its span is the single point d.
func OffsetSpan(d, dDelta float64) (proofbound.RatInterval, bool) {
	rd := proofarith.FloatRat(d)
	if rd == nil || dDelta < 0 || proofbound.IsNonFinite(dDelta) {
		return proofbound.RatInterval{}, false
	}
	if dDelta == 0 {
		return proofbound.PointInterval(rd), true
	}
	rw := proofarith.FloatRat(dDelta)
	return proofbound.Interval(new(big.Rat).Sub(rd, rw), new(big.Rat).Add(rd, rw)), true
}
