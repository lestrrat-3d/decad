package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
)

func shortRatOne() *big.Rat { return big.NewRat(1, 1) }

func shortRatAdd(a, b *big.Rat) *big.Rat { return proof.AddRat(new(big.Rat), a, b) }
func shortRatSub(a, b *big.Rat) *big.Rat { return proof.SubRat(new(big.Rat), a, b) }
func shortRatMul(a, b *big.Rat) *big.Rat { return proof.MulRat(new(big.Rat), a, b) }
func shortRatDiv(a, b *big.Rat) *big.Rat { return proof.QuoRat(new(big.Rat), a, b) }

// ShortCapEndpointX bounds the X coordinate at a selected finite cap endpoint.
func ShortCapEndpointX(first, second loftface.Map, span freeform.BezierSpan,
	q [3][]*big.Rat, endpoint int) (loftface.Interval, bool) {
	if endpoint < 0 || endpoint > 1 || len(span) < 2 || len(q[0]) == 0 {
		return loftface.Interval{}, false
	}
	index := 0
	point := span[0]
	if endpoint == 1 {
		index = len(q[0]) - 1
		point = span[len(span)-1]
	}
	curve := []*big.Rat{q[0][index], q[1][index], q[2][index]}
	if curve[0].Sign() >= 0 || curve[2].Sign() <= 0 {
		return loftface.Interval{}, false
	}
	root := loftface.IsolateRootN(curve, 64)
	x0 := first.At(point.U, point.V)[0]
	x1 := second.At(point.U, point.V)[0]
	if x0.Cmp(x1) >= 0 {
		return loftface.Interval{}, false
	}
	return loftface.Interval{
		Lo: shortRatAdd(shortRatMul(shortRatSub(shortRatOne(), root.Lo), x0), shortRatMul(root.Lo, x1)),
		Hi: shortRatAdd(shortRatMul(shortRatSub(shortRatOne(), root.Hi), x0), shortRatMul(root.Hi, x1)),
	}, true
}

// ShortCapTBracket encloses the crossing time of one Bernstein cell, which
// lies in [tLo, 1] because Q is negative at tLo.
func ShortCapTBracket(q [3][]*big.Rat, tLo *big.Rat) (loftface.Interval, bool) {
	if len(q[0]) == 0 || len(q[0]) != len(q[1]) || len(q[0]) != len(q[2]) {
		return loftface.Interval{}, false
	}
	lo, hi := tLo, shortRatOne()
	for range 64 {
		mid := shortRatMul(shortRatAdd(lo, hi), big.NewRat(1, 2))
		coeff := polynomial.BernsteinRowsAt(q, mid)
		negative, positive := true, true
		for _, value := range coeff {
			negative = negative && value.Sign() < 0
			positive = positive && value.Sign() > 0
		}
		switch {
		case negative:
			lo = mid
		case positive:
			hi = mid
		default:
			return loftface.Interval{Lo: lo, Hi: hi}, true
		}
	}
	return loftface.Interval{Lo: lo, Hi: hi}, true
}

// ShortLineBox bounds a projective cap map restricted to one line. Its
// coordinates have extrema at the endpoints when the denominator stays
// positive. Band interpolation is affine in t, so these endpoint ranges
// enclose the whole crossing.
func ShortLineBox(first, second loftface.Map, span freeform.BezierSpan,
	t loftface.Interval) ([3]loftface.Interval, bool) {
	var box [3]loftface.Interval
	if len(span) != 2 || t.Lo == nil || t.Hi == nil ||
		t.Lo.Sign() < 0 || t.Lo.Cmp(t.Hi) > 0 || t.Hi.Cmp(shortRatOne()) > 0 {
		return box, false
	}
	var caps [2][3]loftface.Interval
	for index, mapped := range [...]loftface.Map{first, second} {
		for _, source := range span {
			if mapped.Denominator(source.U, source.V).Sign() <= 0 {
				return box, false
			}
			point := mapped.At(source.U, source.V)
			for axis, value := range point {
				interval := &caps[index][axis]
				if interval.Lo == nil || value.Cmp(interval.Lo) < 0 {
					interval.Lo = value
				}
				if interval.Hi == nil || value.Cmp(interval.Hi) > 0 {
					interval.Hi = value
				}
			}
		}
	}
	for axis := range box {
		for _, time := range []*big.Rat{t.Lo, t.Hi} {
			for _, a := range []*big.Rat{caps[0][axis].Lo, caps[0][axis].Hi} {
				for _, z := range []*big.Rat{caps[1][axis].Lo, caps[1][axis].Hi} {
					value := shortRatAdd(shortRatMul(shortRatSub(shortRatOne(), time), a), shortRatMul(time, z))
					if box[axis].Lo == nil || value.Cmp(box[axis].Lo) < 0 {
						box[axis].Lo = value
					}
					if box[axis].Hi == nil || value.Cmp(box[axis].Hi) > 0 {
						box[axis].Hi = value
					}
				}
			}
		}
	}
	return box, true
}

// ShortCapXRange bounds the finite cap X coordinates over a source span.
func ShortCapXRange(first, second loftface.Map, span freeform.BezierSpan,
	tLo *big.Rat) (loftface.Interval, bool) {
	if loftface.CapXDecreases(first, span) && loftface.CapXDecreases(second, span) {
		var out loftface.Interval
		for i, point := range [...]freeform.RatPoint{span[0], span[len(span)-1]} {
			x0 := first.At(point.U, point.V)[0]
			x1 := second.At(point.U, point.V)[0]
			xLo := shortRatAdd(shortRatMul(shortRatSub(shortRatOne(), tLo), x0), shortRatMul(tLo, x1))
			for _, x := range []*big.Rat{xLo, x1} {
				if i == 0 && out.Lo == nil || x.Cmp(out.Lo) < 0 {
					out.Lo = x
				}
				if i == 0 && out.Hi == nil || x.Cmp(out.Hi) > 0 {
					out.Hi = x
				}
			}
		}
		return out, true
	}
	capRange := func(m loftface.Map) (loftface.Interval, bool) {
		g, h := loftface.CapBernsteinRows(m, span)
		if len(g) == 0 {
			return loftface.Interval{}, false
		}
		var out loftface.Interval
		for i := range g {
			if g[i].Sign() <= 0 {
				return loftface.Interval{}, false
			}
			x := shortRatDiv(h[0][i], g[i])
			if i == 0 || x.Cmp(out.Lo) < 0 {
				out.Lo = x
			}
			if i == 0 || x.Cmp(out.Hi) > 0 {
				out.Hi = x
			}
		}
		return out, true
	}
	a, okA := capRange(first)
	z, okZ := capRange(second)
	if !okA || !okZ {
		return loftface.Interval{}, false
	}
	blend := func(x, y, t *big.Rat) *big.Rat {
		return shortRatAdd(shortRatMul(shortRatSub(shortRatOne(), t), x), shortRatMul(t, y))
	}
	lo, hi := blend(a.Lo, z.Lo, tLo), blend(a.Hi, z.Hi, tLo)
	if z.Lo.Cmp(lo) < 0 {
		lo = z.Lo
	}
	if z.Hi.Cmp(hi) > 0 {
		hi = z.Hi
	}
	return loftface.Interval{Lo: lo, Hi: hi}, true
}

// ShortCubicBox bounds the clipped cubic crossing in sixteen source cells.
func ShortCubicBox(ctx context.Context, first, second loftface.Map,
	span freeform.BezierSpan, q [3][]*big.Rat, tLo, clipLo, clipHi *big.Rat,
	finiteEnd int) ([3]loftface.Interval, bool, error) {
	var result [3]loftface.Interval
	if ctx == nil || len(span) != 4 || clipLo == nil || clipHi == nil ||
		clipLo.Sign() <= 0 || clipLo.Cmp(clipHi) >= 0 ||
		clipHi.Cmp(shortRatOne()) >= 0 || finiteEnd < 0 || finiteEnd > 1 {
		return result, false, nil
	}
	sLo, sHi := new(big.Rat), clipHi
	if finiteEnd == 1 {
		sLo, sHi = clipLo, shortRatOne()
	}
	width := shortRatDiv(shortRatSub(sHi, sLo), big.NewRat(16, 1))
	for cell := range 16 {
		if cell%4 == 0 {
			if err := ctx.Err(); err != nil {
				return result, false, err
			}
		}
		left := shortRatAdd(sLo, shortRatMul(big.NewRat(int64(cell), 1), width))
		right := shortRatAdd(left, width)
		var restricted [3][]*big.Rat
		for row := range restricted {
			restricted[row] = polynomial.BernsteinRestrict(q[row], left, right)
		}
		t, ok := ShortCapTBracket(restricted, tLo)
		if !ok {
			return result, false, nil
		}
		box, ok := loftface.JunctionBox(first, second, span, left, right, t.Lo, t.Hi)
		if !ok {
			return result, false, nil
		}
		for axis := range result {
			if result[axis].Lo == nil || box[axis].Lo.Cmp(result[axis].Lo) < 0 {
				result[axis].Lo = box[axis].Lo
			}
			if result[axis].Hi == nil || box[axis].Hi.Cmp(result[axis].Hi) > 0 {
				result[axis].Hi = box[axis].Hi
			}
		}
	}
	return result, true, nil
}
