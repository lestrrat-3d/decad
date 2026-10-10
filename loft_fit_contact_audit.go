package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

// certifyRecordedFitArcSeparation proves that no interior intersection of the
// retained fit spline and its adjacent recorded connector arc is possible.
// Bernstein signs cheaply exclude most spans. Exact Sturm counts isolate any
// support-circle roots that remain; a root outside the short connector's
// angular wedge cannot be an arc intersection.
func certifyRecordedFitArcSeparation(ctx context.Context, budget *proofbound.WorkBudget,
	fit fitSplineSeg, arc arcSeg, fitArrives bool, work *freeform.FreeformWork) error {
	spans, err := splinebezier.FitSplineBezierSpans(fit, work)
	if err != nil || len(spans) == 0 {
		return fmt.Errorf(`%w: the loft fillet cannot certify the adjacent fit spline`, ErrUnsupported)
	}
	if !(arc.TStart == 0 && arc.TEnd == 1 || arc.TStart == 1 && arc.TEnd == 0) {
		return fmt.Errorf(`%w: the loft fillet connector is not a complete short arc`, ErrUnsupported)
	}
	_, sweep, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(arc))
	if !ok || sweep.Lo.Sign() <= 0 || sweep.Hi.Cmp(proofbound.PiLower) >= 0 {
		return fmt.Errorf(`%w: the loft fillet cannot certify a short connector arc`, ErrUnsupported)
	}
	ox, oy := new(big.Rat).SetFloat64(arc.Center.U), new(big.Rat).SetFloat64(arc.Center.V)
	sx := new(big.Rat).Sub(new(big.Rat).SetFloat64(arc.Start.U), ox)
	sy := new(big.Rat).Sub(new(big.Rat).SetFloat64(arc.Start.V), oy)
	r2 := new(big.Rat).Add(new(big.Rat).Mul(sx, sx), new(big.Rat).Mul(sy, sy))
	if r2.Sign() <= 0 {
		return fmt.Errorf(`%w: the loft fillet connector has no radius`, ErrUnsupported)
	}
	contactAtLow := fit.TStart < fit.TEnd
	if fitArrives {
		contactAtLow = fit.TEnd < fit.TStart
	}
	fitPoint := spans[len(spans)-1][len(spans[len(spans)-1])-1]
	arcParam := arc.TEnd
	if contactAtLow {
		fitPoint = spans[0][0]
	}
	if fitArrives {
		arcParam = arc.TStart
	}
	arcU, arcV, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(arc),
		new(big.Rat).SetFloat64(arcParam))
	if !ok {
		return fmt.Errorf(`%w: the loft fillet connector endpoint is underivable`, ErrUnsupported)
	}
	du := proofbound.IntervalAbsUpper(proofbound.IntervalSub(proofbound.PointInterval(fitPoint.U), arcU))
	dv := proofbound.IntervalAbsUpper(proofbound.IntervalSub(proofbound.PointInterval(fitPoint.V), arcV))
	gap := proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
	if gap > filletTol {
		return fmt.Errorf(`%w: the loft fillet fit/connector endpoint gap is too large`, ErrUnsupported)
	}
	for i, span := range spans {
		u, v := freeform.SpanCoordinatePolys(span)
		u = polynomial.RpSub(u, polynomial.RatPoly{ox})
		v = polynomial.RpSub(v, polynomial.RatPoly{oy})
		h := polynomial.RpSub(polynomial.RpAdd(polynomial.RpMul(u, u), polynomial.RpMul(v, v)),
			polynomial.RatPoly{r2})
		if polynomial.RpDeg(h) < 0 {
			return fmt.Errorf(`%w: the loft fillet fit spline coincides with its connector circle`, ErrUnsupported)
		}
		controls := freeform.RpToBernstein(h, polynomial.RpDeg(h))
		allowStart := contactAtLow && i == 0
		allowEnd := !contactAtLow && i == len(spans)-1
		if loftBernsteinNoInteriorZero(controls, allowStart, allowEnd, 0) {
			continue
		}
		q := polynomial.RpSquareFree(h)
		chain, err := polynomial.SturmChainIntContext(ctx, q)
		if err != nil {
			return err
		}
		if polynomial.RpEval(h, big.NewRat(0, 1)).Sign() == 0 &&
			!loftFitArcOutside(span, arc, big.NewRat(0, 1), big.NewRat(0, 1)) && !allowStart {
			return fmt.Errorf(`%w: the loft fillet connector touches its adjacent fit spline`, ErrUnsupported)
		}
		count := polynomial.SturmCount(chain, big.NewRat(0, 1), big.NewRat(1, 1))
		type job struct {
			lo, hi *big.Rat
			count  int
			depth  int
		}
		stack := []job{{big.NewRat(0, 1), big.NewRat(1, 1), count, 0}}
		nodes := 0
		for len(stack) > 0 {
			if err := budget.Step(); err != nil {
				return err
			}
			nodes++
			if nodes > 512 {
				return fmt.Errorf(`%w: the loft fillet fit/arc contact proof exceeds its work limit`, ErrUnsupported)
			}
			j := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			if j.count == 0 || loftFitArcOutside(span, arc, j.lo, j.hi) {
				continue
			}
			if j.count == 1 && j.hi.Cmp(big.NewRat(1, 1)) == 0 && allowEnd &&
				polynomial.RpEval(h, j.hi).Sign() == 0 {
				continue // the shared trim endpoint is the only root here
			}
			if j.depth >= 128 {
				return fmt.Errorf(`%w: the loft fillet connector may cross its adjacent fit spline`, ErrUnsupported)
			}
			mid := new(big.Rat).Quo(new(big.Rat).Add(j.lo, j.hi), big.NewRat(2, 1))
			left := polynomial.SturmCount(chain, j.lo, mid)
			stack = append(stack, job{mid, j.hi, j.count - left, j.depth + 1},
				job{j.lo, mid, left, j.depth + 1})
		}
	}
	return nil
}

func loftFitArcOutside(span freeform.BezierSpan, arc arcSeg, lo, hi *big.Rat) bool {
	u, v := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	uLo, uHi := freeform.BernsteinHull(freeform.BernsteinRestrict(u, lo, hi))
	vLo, vHi := freeform.BernsteinHull(freeform.BernsteinRestrict(v, lo, hi))
	pointU, pointV := proofbound.Interval(uLo, uHi), proofbound.Interval(vLo, vHi)
	startU := proofbound.PointInterval(new(big.Rat).SetFloat64(arc.Start.U))
	startV := proofbound.PointInterval(new(big.Rat).SetFloat64(arc.Start.V))
	endU := proofbound.PointInterval(new(big.Rat).SetFloat64(arc.End.U))
	endV := proofbound.PointInterval(new(big.Rat).SetFloat64(arc.End.V))
	return loftOutsideShortArc(pointU, pointV,
		new(big.Rat).SetFloat64(arc.Center.U), new(big.Rat).SetFloat64(arc.Center.V),
		startU, startV, endU, endV)
}

func loftBernsteinNoInteriorZero(controls []*big.Rat, allowStart, allowEnd bool, depth int) bool {
	if controls[0].Sign() == 0 && !allowStart ||
		controls[len(controls)-1].Sign() == 0 && !allowEnd {
		return false
	}
	lo, hi := freeform.BernsteinHull(controls)
	if lo.Sign() >= 0 && hi.Sign() > 0 || hi.Sign() <= 0 && lo.Sign() < 0 {
		return true
	}
	if depth >= 80 {
		return false
	}
	left, right := freeform.BernsteinSplit(controls, big.NewRat(1, 2))
	return loftBernsteinNoInteriorZero(left, allowStart, false, depth+1) &&
		loftBernsteinNoInteriorZero(right, false, allowEnd, depth+1)
}
