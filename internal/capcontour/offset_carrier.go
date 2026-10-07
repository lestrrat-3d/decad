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

func CarrierOf(w survey2d.SideWalk, d float64) (Carrier, bool) {
	if !w.IsCircular() {
		p, okP := OffsetFoot(w.StartU, w.StartV, w.TanInU, w.TanInV, d)
		dir, okD := UnitVec(w.TanInU, w.TanInV)
		if !okP || !okD {
			return Carrier{}, false
		}
		return Carrier{IsLine: true, P: p, Dir: dir}, true
	}
	r, ok := ExactOffsetRadius(w, d)
	if !ok {
		return Carrier{}, false
	}
	c, okC := ExactPoint(w.CU, w.CV)
	if !okC {
		return Carrier{}, false
	}
	return Carrier{C: c, R: proofbound.PointInterval(r)}, true
}

// ExactOffsetRadius is offsetRadius's own R − insideSign·d taken EXACTLY:
// both operands are float64s, so their difference is a rational with no
// rounding at all, and the float the build holds is the rounding of THIS value.
func ExactOffsetRadius(w survey2d.SideWalk, d float64) (*big.Rat, bool) {
	inside := 1.0
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr, rd := proofarith.FloatRat(w.Radius), proofarith.FloatRat(inside*d)
	if rr == nil || rd == nil {
		return nil, false
	}
	out := new(big.Rat).Sub(rr, rd)
	if out.Sign() <= 0 {
		return nil, false
	}
	return out, true
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

// carrierOverRange generalises CarrierOf to an OFFSET INTERVAL [t0, t1]
// rather than one float, enclosing every carrier the wall's own offset
// construction occupies as the offset amount ranges over it — the same
// closed forms CarrierOf evaluates at one point, evaluated over the whole
// range instead. A line's carrier stays a single line: only its anchor point
// moves, along the line's own FIXED unit normal, so the direction needs no
// widening at all. A circle's carrier stays a single concentric circle whose
// radius now encloses the offset radius's own range rather than one value.
// At t0 == t1 == d it reduces to CarrierOf(w, d)'s own enclosure, since
// both build the offset amount from the identical closed form.
func carrierOverRange(w survey2d.SideWalk, t0, t1 float64) (Carrier, bool) {
	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return Carrier{}, false
	}
	tRange := proofbound.Interval(rt0, rt1)
	if !w.IsCircular() {
		p, okP := offsetFootRange(w.StartU, w.StartV, w.TanInU, w.TanInV, tRange)
		dir, okD := UnitVec(w.TanInU, w.TanInV)
		if !okP || !okD {
			return Carrier{}, false
		}
		return Carrier{IsLine: true, P: p, Dir: dir}, true
	}
	rr := proofarith.FloatRat(w.Radius)
	if rr == nil {
		return Carrier{}, false
	}
	inside := big.NewRat(1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	r := proofbound.IntervalSub(proofbound.PointInterval(rr), proofbound.IntervalMul(tRange, proofbound.PointInterval(inside)))
	if r.Lo.Sign() <= 0 {
		return Carrier{}, false
	}
	c, okC := ExactPoint(w.CU, w.CV)
	if !okC {
		return Carrier{}, false
	}
	return Carrier{C: c, R: r}, true
}
