package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/units"
)

// Each source edge is split without changing its denoted curve. The held
// polygon is only a comparison boundary; its vertices carry the exact source
// and offset positions' outward interval errors.
type twistOffsetStations struct {
	outer, inner               []planePoint
	outerVertex, innerVertex   []float64
	outerSpanLen, innerSpanLen []float64
	outerChord, innerChord     float64
}

func twistOffsetConvexPolygon(points []planePoint) bool {
	if len(points) < 3 {
		return false
	}
	positive := false
	for i := range points {
		a, b, c := points[i], points[(i+1)%len(points)], points[(i+2)%len(points)]
		abx := new(big.Rat).Sub(twistRat(b.U), twistRat(a.U))
		aby := new(big.Rat).Sub(twistRat(b.V), twistRat(a.V))
		bcx := new(big.Rat).Sub(twistRat(c.U), twistRat(b.U))
		bcy := new(big.Rat).Sub(twistRat(c.V), twistRat(b.V))
		sign := new(big.Rat).Sub(new(big.Rat).Mul(abx, bcy),
			new(big.Rat).Mul(aby, bcx)).Sign()
		if sign < 0 {
			return false
		}
		positive = positive || sign > 0
	}
	return positive
}

func twistOffsetArcAngle(edge arcSeg) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	_, sweep, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(edge))
	if !ok || sweep.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	dx := new(big.Rat).Sub(twistRat(edge.Start.U), twistRat(edge.Center.U))
	dy := new(big.Rat).Sub(twistRat(edge.Start.V), twistRat(edge.Center.V))
	start := proofbound.Atan2Interval(dy, dx,
		edge.Start.V-edge.Center.V == 0 && math.Signbit(edge.Start.V-edge.Center.V))
	return start, sweep, true
}

func twistOffsetSourceStation(segment curveSegment, parameter *big.Rat) (
	proofbound.RatInterval, proofbound.RatInterval,
	proofbound.RatInterval, proofbound.RatInterval, error,
) {
	switch edge := segment.(type) {
	case lineSeg:
		sx, sy := twistRat(edge.Start.U), twistRat(edge.Start.V)
		dx := new(big.Rat).Sub(twistRat(edge.End.U), sx)
		dy := new(big.Rat).Sub(twistRat(edge.End.V), sy)
		length, ok := twistOffsetLength(dx, dy)
		if !ok || length.Lo.Sign() <= 0 {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no tangent`, ErrUnsupported)
		}
		nx, ok := proofbound.IntervalQuo(proofbound.PointInterval(new(big.Rat).Neg(dy)), length)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no normal`, ErrUnsupported)
		}
		ny, ok := proofbound.IntervalQuo(proofbound.PointInterval(dx), length)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no normal`, ErrUnsupported)
		}
		x := proofbound.PointInterval(new(big.Rat).Add(sx, new(big.Rat).Mul(parameter, dx)))
		y := proofbound.PointInterval(new(big.Rat).Add(sy, new(big.Rat).Mul(parameter, dy)))
		return x, y, nx, ny, nil
	case arcSeg:
		radius, _, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(edge))
		start, sweep, angleOK := twistOffsetArcAngle(edge)
		if !ok || !angleOK || radius.Lo.Sign() <= 0 {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell arc station has no bounded walk`, ErrUnsupported)
		}
		angle := proofbound.IntervalAdd(start, proofbound.IntervalScale(sweep, parameter))
		sin, cos, ok := proofbound.RadSinCosSpan(angle)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell arc station has no angular enclosure`, ErrUnsupported)
		}
		x := proofbound.IntervalAdd(twistOffsetPoint(edge.Center.U), proofbound.IntervalMul(radius, cos))
		y := proofbound.IntervalAdd(twistOffsetPoint(edge.Center.V), proofbound.IntervalMul(radius, sin))
		return x, y, proofbound.IntervalNeg(cos), proofbound.IntervalNeg(sin), nil
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{},
			proofbound.RatInterval{}, proofbound.RatInterval{},
			fmt.Errorf(`%w: twisted shell station has an unsupported curve`, ErrUnsupported)
	}
}

func twistOffsetPointStation(segment curveSegment, parameter *big.Rat,
	k2, thickness proofbound.RatInterval) (planePoint, planePoint, float64, float64, error) {
	x, y, nx, ny, err := twistOffsetSourceStation(segment, parameter)
	if err != nil {
		return planePoint{}, planePoint{}, 0, 0, err
	}
	// n is the inward unit normal. T=(ny,-nx), hence f=p·T.
	f := proofbound.IntervalSub(proofbound.IntervalMul(x, ny), proofbound.IntervalMul(y, nx))
	g, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(twistOffsetUnit(1),
		proofbound.IntervalMul(k2, proofbound.IntervalSquare(f))))
	if !ok || g.Lo.Sign() <= 0 {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no bounded normal`, ErrUnsupported)
	}
	a, ok := proofbound.IntervalQuo(thickness, g)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no bounded offset`, ErrUnsupported)
	}
	beta, ok := proofbound.IntervalQuo(proofbound.IntervalMul(proofbound.IntervalMul(k2, thickness), f), g)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no bounded offset turn`, ErrUnsupported)
	}
	sin, cos, ok := proofbound.RadSinCosSpan(beta)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no angular enclosure`, ErrUnsupported)
	}
	px := proofbound.IntervalAdd(x, proofbound.IntervalMul(a, nx))
	py := proofbound.IntervalAdd(y, proofbound.IntervalMul(a, ny))
	qx := proofbound.IntervalSub(proofbound.IntervalMul(cos, px), proofbound.IntervalMul(sin, py))
	qy := proofbound.IntervalAdd(proofbound.IntervalMul(sin, px), proofbound.IntervalMul(cos, py))
	point := func(iv proofbound.RatInterval) float64 {
		middle := new(big.Rat).Quo(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(2, 1))
		f, _ := middle.Float64()
		return f
	}
	p, q := planePoint{U: point(x), V: point(y)}, planePoint{U: point(qx), V: point(qy)}
	pErr := proofbound.Radius3D(max(proofbound.IntervalFloatError(x, p.U),
		proofbound.IntervalFloatError(y, p.V)))
	qErr := proofbound.Radius3D(max(proofbound.IntervalFloatError(qx, q.U),
		proofbound.IntervalFloatError(qy, q.V)))
	return p, q, pErr, qErr, nil
}

// twistOffsetSecondDerivative bounds |q”| for arclength s. The identities
// f'=1+κh, h'=-κf, a'=-fβ', and κ'=0 hold on every line or circular arc.
func twistOffsetSecondDerivative(span twistOffsetSpan,
	k2, thickness proofbound.RatInterval) (float64, float64, error) {
	f2 := proofbound.IntervalSquare(span.f)
	g, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(twistOffsetUnit(1),
		proofbound.IntervalMul(k2, f2)))
	if !ok || g.Lo.Sign() <= 0 {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no positive normal`, ErrUnsupported)
	}
	a, ok := proofbound.IntervalQuo(thickness, g)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded offset`, ErrUnsupported)
	}
	fPrime := proofbound.IntervalAdd(twistOffsetUnit(1),
		proofbound.IntervalMul(span.kappa, span.h))
	g3 := proofbound.IntervalMul(proofbound.IntervalSquare(g), g)
	betaPrime, ok := proofbound.IntervalQuo(proofbound.IntervalMul(
		proofbound.IntervalMul(k2, thickness), fPrime), g3)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn`, ErrUnsupported)
	}
	g5 := proofbound.IntervalMul(g3, proofbound.IntervalSquare(g))
	first, ok := proofbound.IntervalQuo(proofbound.IntervalMul(
		proofbound.IntervalMul(proofbound.IntervalMul(k2, thickness),
			proofbound.IntervalSquare(span.kappa)), span.f), g3)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn acceleration`, ErrUnsupported)
	}
	second, ok := proofbound.IntervalQuo(proofbound.IntervalScale(proofbound.IntervalMul(
		proofbound.IntervalMul(proofbound.IntervalMul(proofbound.IntervalMul(k2, k2), thickness),
			span.f), proofbound.IntervalSquare(fPrime)), big.NewRat(3, 1)), g5)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn acceleration`, ErrUnsupported)
	}
	betaSecond := proofbound.IntervalNeg(proofbound.IntervalAdd(first, second))
	lambda := proofbound.IntervalSub(proofbound.IntervalSub(twistOffsetUnit(1),
		proofbound.IntervalMul(a, span.kappa)),
		proofbound.IntervalMul(betaPrime, proofbound.IntervalAdd(span.h, a)))
	turn := proofbound.IntervalAdd(span.kappa, betaPrime)
	// λ' = 2κfβ' + f(β')² − β''(h+a).
	lambdaPrime := proofbound.IntervalSub(proofbound.IntervalAdd(
		proofbound.IntervalScale(proofbound.IntervalMul(
			proofbound.IntervalMul(span.kappa, span.f), betaPrime), big.NewRat(2, 1)),
		proofbound.IntervalMul(span.f, proofbound.IntervalSquare(betaPrime))),
		proofbound.IntervalMul(betaSecond, proofbound.IntervalAdd(span.h, a)))
	bound := new(big.Rat).Add(proofbound.IntervalAbsUpper(lambdaPrime),
		new(big.Rat).Mul(proofbound.IntervalAbsUpper(lambda), proofbound.IntervalAbsUpper(turn)))
	if lambda.Lo.Sign() <= 0 {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no positive speed`, ErrUnsupported)
	}
	return proofbound.RatFloatUp(bound), proofbound.RatFloatUp(lambda.Hi), nil
}

func twistOffsetSampleStations(profile profileRecord, height float64, angle units.Value,
	thickness, thicknessDelta float64, lineParts, arcParts int64) (twistOffsetStations, error) {
	angleIv, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok || height <= 0 || lineParts <= 0 || arcParts <= 0 {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell station parameters are unbounded`, ErrUnsupported)
	}
	k, ok := proofbound.IntervalQuo(angleIv, twistOffsetPoint(height))
	if !ok {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell station turn is unbounded`, ErrUnsupported)
	}
	k2 := proofbound.IntervalSquare(k)
	t := proofbound.Interval(new(big.Rat).Sub(twistRat(thickness), twistRat(thicknessDelta)),
		new(big.Rat).Add(twistRat(thickness), twistRat(thicknessDelta)))
	var out twistOffsetStations
	for _, segment := range profile.Outer.Segments {
		parts := lineParts
		if _, arc := segment.(arcSeg); arc {
			parts = arcParts
		}
		for i := range parts {
			p, q, pErr, qErr, err := twistOffsetPointStation(segment, big.NewRat(i, parts), k2, t)
			if err != nil {
				return twistOffsetStations{}, err
			}
			out.outer, out.inner = append(out.outer, p), append(out.inner, q)
			out.outerVertex, out.innerVertex = append(out.outerVertex, pErr), append(out.innerVertex, qErr)
			var span twistOffsetSpan
			switch edge := segment.(type) {
			case lineSeg:
				span, err = twistOffsetLineSpan(edge, i, i+1, parts)
			case arcSeg:
				span, err = twistOffsetArcSpan(edge, i, i+1, parts)
			}
			if err != nil {
				return twistOffsetStations{}, err
			}
			second, speed, err := twistOffsetSecondDerivative(span, k2, t)
			if err != nil {
				return twistOffsetStations{}, err
			}
			qChord := proofbound.ProductUpper(proofbound.ProductUpper(second,
				proofbound.ProductUpper(proofbound.RatFloatUp(span.length.Hi),
					proofbound.RatFloatUp(span.length.Hi))), 0.125)
			out.innerChord = max(out.innerChord, qChord)
			spanLength := proofbound.RatFloatUp(span.length.Hi)
			out.outerSpanLen = append(out.outerSpanLen, spanLength)
			out.innerSpanLen = append(out.innerSpanLen, proofbound.ProductUpper(spanLength, speed))
			if arc, curved := segment.(arcSeg); curved {
				radius, sweep, arcOK := circularbounds.WalkEnclosures(circularbounds.RecordSegment(arc))
				if !arcOK {
					return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell arc chord has no walk`, ErrUnsupported)
				}
				angle := proofbound.RatFloatUp(new(big.Rat).Quo(sweep.Hi, big.NewRat(parts, 1)))
				out.outerChord = max(out.outerChord, proofbound.ProductUpper(
					proofbound.ProductUpper(proofbound.RatFloatUp(radius.Hi),
						proofbound.ProductUpper(angle, angle)), 0.125))
			}
		}
	}
	if len(out.outer) < 3 || len(out.outer) != len(out.inner) {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell has no paired contour stations`, ErrUnsupported)
	}
	return out, nil
}
