package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// A crossing can enter another whole Sketch entity through their shared
// endpoint twice in one LoftFaces band. These exact rational brackets cover
// every zero of the quadratic endpoint polynomial in band time. Each simple
// root changes side. The caller must bind the same live source endpoint on
// both adjacent entities before interpreting either change as a joint seam.
type EndpointRoot struct {
	Time      Interval
	X         Interval
	Direction int
	Finite    bool
}

// Bezier hulls enclose both cap numerators and their positive projective
// denominator over a source subspan. X is affine in band time at each source
// point, so the hull of its two time-end ranges encloses the full time slab.
func SpanXRange(first, last Map, span freeform.BezierSpan,
	sLo, sHi, tLo, tHi *big.Rat) (Interval, bool) {
	if len(span) == 0 || sLo == nil || sHi == nil || tLo == nil || tHi == nil ||
		sLo.Sign() < 0 || sLo.Cmp(sHi) > 0 || sHi.Cmp(ratOne()) > 0 ||
		tLo.Sign() < 0 || tLo.Cmp(tHi) > 0 || tHi.Cmp(ratOne()) > 0 {
		return Interval{}, false
	}
	g0, h0 := CapBernsteinRows(first, span)
	g1, h1 := CapBernsteinRows(last, span)
	denominator := polynomial.BernsteinRestrict(polynomial.BernsteinMul(g0, g1), sLo, sHi)
	denRange := coeffRange(denominator)
	if denRange.Lo.Sign() <= 0 {
		return Interval{}, false
	}
	firstX, lastX := polynomial.BernsteinMul(h0[0], g1), polynomial.BernsteinMul(h1[0], g0)
	at := func(t *big.Rat) Interval {
		numerator := polynomial.BernsteinAdd(polynomial.BernsteinScale(firstX, ratSub(ratOne(), t)),
			polynomial.BernsteinScale(lastX, t))

		numRange := coeffRange(polynomial.BernsteinRestrict(numerator, sLo, sHi))
		var out Interval
		for _, valueNum := range [...]*big.Rat{numRange.Lo, numRange.Hi} {
			for _, valueDen := range [...]*big.Rat{denRange.Lo, denRange.Hi} {
				value := ratDiv(valueNum, valueDen)
				if out.Lo == nil || value.Cmp(out.Lo) < 0 {
					out.Lo = value
				}
				if out.Hi == nil || value.Cmp(out.Hi) > 0 {
					out.Hi = value
				}
			}
		}
		return out
	}
	return IntervalHull(at(tLo), at(tHi)), true
}

// coeffRange returns the interval spanned by the coefficients.
func coeffRange(coeff []*big.Rat) Interval {
	var out Interval
	for _, value := range coeff {
		if out.Lo == nil || value.Cmp(out.Lo) < 0 {
			out.Lo = value
		}
		if out.Hi == nil || value.Cmp(out.Hi) > 0 {
			out.Hi = value
		}
	}
	return out
}

// EndpointTimeRoots returns the exact rational brackets of the endpoint
// polynomial roots in band time.
func EndpointTimeRoots(q [3][]*big.Rat,
	atEnd bool) ([]EndpointRoot, bool) {
	if len(q[0]) == 0 || len(q[0]) != len(q[1]) || len(q[0]) != len(q[2]) {
		return nil, false
	}
	index := 0
	if atEnd {
		index = len(q[0]) - 1
	}
	coeff := []*big.Rat{q[0][index], q[1][index], q[2][index]}
	first, last := coeff[0].Sign(), coeff[2].Sign()
	if first == 0 || last == 0 {
		return nil, false
	}
	if first != last {
		root, ok := isolateTimeRoot(coeff, ratZero(), ratOne())
		if !ok {
			return nil, false
		}
		return []EndpointRoot{root}, true
	}
	if coeff[1].Sign() == first {
		return nil, true
	}
	// A quadratic with equal endpoint signs has either zero or two simple
	// interior roots. A bounded dyadic search finds an opposite-sign point;
	// two disjoint sign-changing intervals then prove both roots and exclude
	// further roots by the polynomial degree. If the search misses a narrow
	// opposite-side pocket, refuse instead of guessing from samples.
	for depth := 1; depth <= 12; depth++ {
		denominator := int64(1 << depth)
		for numerator := int64(1); numerator < denominator; numerator += 2 {
			middle := big.NewRat(numerator, denominator)
			sign := polynomial.BernsteinSign(coeff, middle)
			if sign == 0 {
				return nil, false
			}
			if sign != first {
				left, leftOK := isolateTimeRoot(coeff, ratZero(), middle)
				right, rightOK := isolateTimeRoot(coeff, middle, ratOne())
				if !leftOK || !rightOK || left.Time.Hi.Cmp(right.Time.Lo) >= 0 {
					return nil, false
				}
				return []EndpointRoot{left, right}, true
			}
		}
	}
	return nil, false
}

// isolateTimeRoot isolates one sign-changing root in band time.
func isolateTimeRoot(coeff []*big.Rat,
	lo, hi *big.Rat) (EndpointRoot, bool) {
	loSign, hiSign := polynomial.BernsteinSign(coeff, lo), polynomial.BernsteinSign(coeff, hi)
	if loSign == 0 || hiSign == 0 || loSign == hiSign {
		return EndpointRoot{}, false
	}
	direction := hiSign
	for range 64 {
		mid := ratMul(ratAdd(lo, hi), big.NewRat(1, 2))
		sign := polynomial.BernsteinSign(coeff, mid)
		if sign == 0 {
			return EndpointRoot{
				Time: Interval{Lo: mid, Hi: mid}, Direction: direction}, true
		}
		if sign == loSign {
			lo = mid
		} else {
			hi = mid
		}
	}
	return EndpointRoot{
		Time: Interval{Lo: lo, Hi: hi}, Direction: direction}, true
}

// TimePiece is one open or closed piece of band time with one side sign.
type TimePiece struct {
	Lo, Hi         *big.Rat
	LoOpen, HiOpen bool
	Side           int
	RootIndex      int // -1 for a strict-side piece
}

// Quadratic degree plus isolated sign-changing root brackets proves there is
// no unlisted side change in any strict piece. The bracket pieces remain
// undecidable as point cells; the radial-joint source certificate binds each
// entire arc contact there instead.
func PartitionTime(q [3]*big.Rat,
	roots []EndpointRoot) ([]TimePiece, bool) {
	if len(roots) == 0 || len(roots) > 2 || q[0] == nil || q[1] == nil || q[2] == nil {
		return nil, false
	}
	coeff := []*big.Rat{q[0], q[1], q[2]}
	pieces := make([]TimePiece, 0, 2*len(roots)+1)
	previous := ratZero()
	for index, root := range roots {
		if root.Time.Lo == nil || root.Time.Hi == nil ||
			root.Time.Lo.Cmp(previous) <= 0 || root.Time.Hi.Cmp(root.Time.Lo) < 0 ||
			root.Time.Hi.Cmp(ratOne()) >= 0 || root.Direction == 0 {
			return nil, false
		}
		atLo, atHi := polynomial.BernsteinSign(coeff, root.Time.Lo),
			polynomial.BernsteinSign(coeff, root.Time.Hi)
		if root.Time.Lo.Cmp(root.Time.Hi) == 0 {
			if atLo != 0 {
				return nil, false
			}
		} else if atLo != -root.Direction || atHi != root.Direction {
			return nil, false
		}
		side := polynomial.BernsteinSign(coeff, ratMul(ratAdd(previous, root.Time.Lo), big.NewRat(1, 2)))
		if side == 0 || side != -root.Direction {
			return nil, false
		}
		pieces = append(pieces, TimePiece{
			Lo: previous, Hi: root.Time.Lo, LoOpen: index != 0,
			HiOpen: true, Side: side, RootIndex: -1})
		pieces = append(pieces, TimePiece{
			Lo: root.Time.Lo, Hi: root.Time.Hi, RootIndex: index})
		previous = root.Time.Hi
	}
	side := polynomial.BernsteinSign(coeff, ratMul(ratAdd(previous, ratOne()), big.NewRat(1, 2)))
	if side == 0 || side != roots[len(roots)-1].Direction {
		return nil, false
	}
	pieces = append(pieces, TimePiece{
		Lo: previous, Hi: ratOne(), LoOpen: true, Side: side, RootIndex: -1})
	return pieces, true
}
