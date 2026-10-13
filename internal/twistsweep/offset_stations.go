package twistsweep

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

type arcSeg = sectionrecord.ArcSeg

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

// planePoint is a plane-local coordinate.
type planePoint = sectionrecord.Point2

type profileRecord = momentinput.Profile

// Each source edge is split without changing its denoted curve. The held
// polygon is only a comparison boundary; its vertices carry the exact source
// and offset positions' outward interval errors.
type twistOffsetStations struct {
	Outer, Inner               []planePoint
	OuterVertex, InnerVertex   []float64
	outerSpanLen, innerSpanLen []float64
	OuterChord, InnerChord     float64
}

// TwistOffsetConvexPolygon reports whether points turn one way throughout,
// with at least one strict turn, so they form a convex polygon with
// at least three vertices.
func TwistOffsetConvexPolygon(points []planePoint) bool {
	if len(points) < 3 {
		return false
	}
	positive := false
	for i := range points {
		a, b, c := points[i], points[(i+1)%len(points)], points[(i+2)%len(points)]
		abx := new(big.Rat).Sub(TwistRat(b.U), TwistRat(a.U))
		aby := new(big.Rat).Sub(TwistRat(b.V), TwistRat(a.V))
		bcx := new(big.Rat).Sub(TwistRat(c.U), TwistRat(b.U))
		bcy := new(big.Rat).Sub(TwistRat(c.V), TwistRat(b.V))
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
	dx := new(big.Rat).Sub(TwistRat(edge.Start.U), TwistRat(edge.Center.U))
	dy := new(big.Rat).Sub(TwistRat(edge.Start.V), TwistRat(edge.Center.V))
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
		sx, sy := TwistRat(edge.Start.U), TwistRat(edge.Start.V)
		dx := new(big.Rat).Sub(TwistRat(edge.End.U), sx)
		dy := new(big.Rat).Sub(TwistRat(edge.End.V), sy)
		length, ok := twistOffsetLength(dx, dy)
		if !ok || length.Lo.Sign() <= 0 {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no tangent`, decaderr.ErrUnsupported)
		}
		nx, ok := proofbound.IntervalQuo(proofbound.PointInterval(new(big.Rat).Neg(dy)), length)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no normal`, decaderr.ErrUnsupported)
		}
		ny, ok := proofbound.IntervalQuo(proofbound.PointInterval(dx), length)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell line station has no normal`, decaderr.ErrUnsupported)
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
				fmt.Errorf(`%w: twisted shell arc station has no bounded walk`, decaderr.ErrUnsupported)
		}
		angle := proofbound.IntervalAdd(start, proofbound.IntervalScale(sweep, parameter))
		sin, cos, ok := proofbound.RadSinCosSpan(angle)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{},
				proofbound.RatInterval{}, proofbound.RatInterval{},
				fmt.Errorf(`%w: twisted shell arc station has no angular enclosure`, decaderr.ErrUnsupported)
		}
		x := proofbound.IntervalAdd(twistOffsetPoint(edge.Center.U), proofbound.IntervalMul(radius, cos))
		y := proofbound.IntervalAdd(twistOffsetPoint(edge.Center.V), proofbound.IntervalMul(radius, sin))
		return x, y, proofbound.IntervalNeg(cos), proofbound.IntervalNeg(sin), nil
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{},
			proofbound.RatInterval{}, proofbound.RatInterval{},
			fmt.Errorf(`%w: twisted shell station has an unsupported curve`, decaderr.ErrUnsupported)
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
			fmt.Errorf(`%w: twisted shell station has no bounded normal`, decaderr.ErrUnsupported)
	}
	a, ok := proofbound.IntervalQuo(thickness, g)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no bounded offset`, decaderr.ErrUnsupported)
	}
	beta, ok := proofbound.IntervalQuo(proofbound.IntervalMul(proofbound.IntervalMul(k2, thickness), f), g)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no bounded offset turn`, decaderr.ErrUnsupported)
	}
	sin, cos, ok := proofbound.RadSinCosSpan(beta)
	if !ok {
		return planePoint{}, planePoint{}, 0, 0,
			fmt.Errorf(`%w: twisted shell station has no angular enclosure`, decaderr.ErrUnsupported)
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
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no positive normal`, decaderr.ErrUnsupported)
	}
	a, ok := proofbound.IntervalQuo(thickness, g)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded offset`, decaderr.ErrUnsupported)
	}
	fPrime := proofbound.IntervalAdd(twistOffsetUnit(1),
		proofbound.IntervalMul(span.kappa, span.h))
	g3 := proofbound.IntervalMul(proofbound.IntervalSquare(g), g)
	betaPrime, ok := proofbound.IntervalQuo(proofbound.IntervalMul(
		proofbound.IntervalMul(k2, thickness), fPrime), g3)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn`, decaderr.ErrUnsupported)
	}
	g5 := proofbound.IntervalMul(g3, proofbound.IntervalSquare(g))
	first, ok := proofbound.IntervalQuo(proofbound.IntervalMul(
		proofbound.IntervalMul(proofbound.IntervalMul(k2, thickness),
			proofbound.IntervalSquare(span.kappa)), span.f), g3)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn acceleration`, decaderr.ErrUnsupported)
	}
	second, ok := proofbound.IntervalQuo(proofbound.IntervalScale(proofbound.IntervalMul(
		proofbound.IntervalMul(proofbound.IntervalMul(proofbound.IntervalMul(k2, k2), thickness),
			span.f), proofbound.IntervalSquare(fPrime)), big.NewRat(3, 1)), g5)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no bounded turn acceleration`, decaderr.ErrUnsupported)
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
		return 0, 0, fmt.Errorf(`%w: twisted shell chord has no positive speed`, decaderr.ErrUnsupported)
	}
	return proofbound.RatFloatUp(bound), proofbound.RatFloatUp(lambda.Hi), nil
}

// TwistOffsetSampleStations samples the twisted wall offset into axial
// stations, with the error bounds the sampled cup mesh charges.
func TwistOffsetSampleStations(profile profileRecord, height float64, angle units.Value,
	thickness, thicknessDelta float64, lineParts, arcParts int64) (twistOffsetStations, error) {
	angleIv, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok || height <= 0 || lineParts <= 0 || arcParts <= 0 {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell station parameters are unbounded`, decaderr.ErrUnsupported)
	}
	k, ok := proofbound.IntervalQuo(angleIv, twistOffsetPoint(height))
	if !ok {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell station turn is unbounded`, decaderr.ErrUnsupported)
	}
	k2 := proofbound.IntervalSquare(k)
	t := proofbound.Interval(new(big.Rat).Sub(TwistRat(thickness), TwistRat(thicknessDelta)),
		new(big.Rat).Add(TwistRat(thickness), TwistRat(thicknessDelta)))
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
			out.Outer, out.Inner = append(out.Outer, p), append(out.Inner, q)
			out.OuterVertex, out.InnerVertex = append(out.OuterVertex, pErr), append(out.InnerVertex, qErr)
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
			out.InnerChord = max(out.InnerChord, qChord)
			spanLength := proofbound.RatFloatUp(span.length.Hi)
			out.outerSpanLen = append(out.outerSpanLen, spanLength)
			out.innerSpanLen = append(out.innerSpanLen, proofbound.ProductUpper(spanLength, speed))
			if arc, curved := segment.(arcSeg); curved {
				radius, sweep, arcOK := circularbounds.WalkEnclosures(circularbounds.RecordSegment(arc))
				if !arcOK {
					return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell arc chord has no walk`, decaderr.ErrUnsupported)
				}
				angle := proofbound.RatFloatUp(new(big.Rat).Quo(sweep.Hi, big.NewRat(parts, 1)))
				out.OuterChord = max(out.OuterChord, proofbound.ProductUpper(
					proofbound.ProductUpper(proofbound.RatFloatUp(radius.Hi),
						proofbound.ProductUpper(angle, angle)), 0.125))
			}
		}
	}
	if len(out.Outer) < 3 || len(out.Outer) != len(out.Inner) {
		return twistOffsetStations{}, fmt.Errorf(`%w: twisted shell has no paired contour stations`, decaderr.ErrUnsupported)
	}
	return out, nil
}
