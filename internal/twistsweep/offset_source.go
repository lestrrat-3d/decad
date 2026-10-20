// Package twistsweep holds the twisted-sweep shell proofs: the twist polygon
// admission, the twisted-wall offset source and stations, the placed-shell
// proof and the sampled cup mesh. agent-docs/shell-twisted-sweep-design.md owns the
// design.
package twistsweep

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/units"
)

// twistOffsetSourceProof describes the inward unit-normal offset of a
// one-line twisted Sweep. The offset is a denoted curve, not a fitted spline.
// Central symmetry makes both first moments exactly zero.
type twistOffsetSourceProof struct {
	Area         proofbound.RatInterval
	ClearanceLo  *big.Rat
	SpeedLo      *big.Rat
	TurnRateLo   *big.Rat
	RadialLo     *big.Rat
	FirstMomentX *big.Rat
	FirstMomentY *big.Rat
}

type twistOffsetSpan struct {
	f, h, kappa, length proofbound.RatInterval
}

type twistOffsetTangent struct{ x, y *big.Rat }

func twistOffsetPoint(x float64) proofbound.RatInterval {
	return proofbound.PointInterval(TwistRat(x))
}

func twistOffsetUnit(x int64) proofbound.RatInterval {
	return proofbound.PointInterval(big.NewRat(x, 1))
}

func twistOffsetLength(dx, dy *big.Rat) (proofbound.RatInterval, bool) {
	return proofbound.SqrtInterval(proofbound.PointInterval(new(big.Rat).Add(
		new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))))
}

func twistOffsetLineSpan(edge lineSeg, first, last, parts int64) (twistOffsetSpan, error) {
	x, y := TwistRat(edge.Start.U), TwistRat(edge.Start.V)
	dx := new(big.Rat).Sub(TwistRat(edge.End.U), x)
	dy := new(big.Rat).Sub(TwistRat(edge.End.V), y)
	length, ok := twistOffsetLength(dx, dy)
	if !ok || length.Lo.Sign() <= 0 {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell line has no positive length`, decaderr.ErrUnsupported)
	}
	dot := new(big.Rat).Add(new(big.Rat).Mul(x, dx), new(big.Rat).Mul(y, dy))
	len2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	parameter := proofbound.Interval(big.NewRat(first, parts), big.NewRat(last, parts))
	f, ok := proofbound.IntervalQuo(proofbound.IntervalAdd(proofbound.PointInterval(dot),
		proofbound.IntervalScale(parameter, len2)), length)
	if !ok {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell line tangent is unbounded`, decaderr.ErrUnsupported)
	}
	hNumerator := new(big.Rat).Sub(new(big.Rat).Mul(y, dx), new(big.Rat).Mul(x, dy))
	h, ok := proofbound.IntervalQuo(proofbound.PointInterval(hNumerator), length)
	if !ok {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell line normal is unbounded`, decaderr.ErrUnsupported)
	}
	return twistOffsetSpan{f: f, h: h, kappa: twistOffsetUnit(0),
		length: proofbound.IntervalScale(length, big.NewRat(last-first, parts))}, nil
}

func twistOffsetArcSpan(edge arcSeg, first, last, parts int64) (twistOffsetSpan, error) {
	radius, sweep, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(edge))
	if !ok || radius.Lo.Sign() <= 0 || sweep.Lo.Sign() <= 0 || sweep.Hi.Cmp(proofbound.PiUpper) > 0 {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell arc has no bounded radius or turn`, decaderr.ErrUnsupported)
	}
	dx := new(big.Rat).Sub(TwistRat(edge.Start.U), TwistRat(edge.Center.U))
	dy := new(big.Rat).Sub(TwistRat(edge.Start.V), TwistRat(edge.Center.V))
	a0 := proofbound.Atan2Interval(dy, dx,
		edge.Start.V-edge.Center.V == 0 && math.Signbit(edge.Start.V-edge.Center.V))
	angle := proofbound.IntervalAdd(a0, proofbound.IntervalMul(sweep,
		proofbound.Interval(big.NewRat(first, parts), big.NewRat(last, parts))))
	sin, cos, ok := proofbound.RadSinCosSpan(angle)
	if !ok {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell arc has no angular enclosure`, decaderr.ErrUnsupported)
	}
	cx, cy := TwistRat(edge.Center.U), TwistRat(edge.Center.V)
	f := proofbound.IntervalAdd(proofbound.IntervalScale(sin, new(big.Rat).Neg(cx)),
		proofbound.IntervalScale(cos, cy))
	h := proofbound.IntervalSub(proofbound.IntervalAdd(
		proofbound.IntervalScale(cos, new(big.Rat).Neg(cx)),
		proofbound.IntervalScale(sin, new(big.Rat).Neg(cy))), radius)
	kappa, ok := proofbound.IntervalQuo(twistOffsetUnit(1), radius)
	if !ok {
		return twistOffsetSpan{}, fmt.Errorf(`%w: twisted shell arc curvature is unbounded`, decaderr.ErrUnsupported)
	}
	return twistOffsetSpan{f: f, h: h, kappa: kappa,
		length: proofbound.IntervalScale(proofbound.IntervalMul(radius, sweep),
			big.NewRat(last-first, parts))}, nil
}

func twistOffsetSegmentEnds(segment curveSegment) (planePoint, planePoint, twistOffsetTangent, twistOffsetTangent, bool) {
	switch edge := segment.(type) {
	case lineSeg:
		dx := new(big.Rat).Sub(TwistRat(edge.End.U), TwistRat(edge.Start.U))
		dy := new(big.Rat).Sub(TwistRat(edge.End.V), TwistRat(edge.Start.V))
		tangent := twistOffsetTangent{dx, dy}
		return edge.Start, edge.End, tangent, tangent, true
	case arcSeg:
		start := twistOffsetTangent{
			new(big.Rat).Sub(TwistRat(edge.Center.V), TwistRat(edge.Start.V)),
			new(big.Rat).Sub(TwistRat(edge.Start.U), TwistRat(edge.Center.U)),
		}
		end := twistOffsetTangent{
			new(big.Rat).Sub(TwistRat(edge.Center.V), TwistRat(edge.End.V)),
			new(big.Rat).Sub(TwistRat(edge.End.U), TwistRat(edge.Center.U)),
		}
		return edge.Start, edge.End, start, end, true
	default:
		return planePoint{}, planePoint{}, twistOffsetTangent{}, twistOffsetTangent{}, false
	}
}

func twistOffsetSmoothClosed(profile profileRecord) bool {
	segments := profile.Outer.Segments
	for i, segment := range segments {
		_, end, _, leaving, ok := twistOffsetSegmentEnds(segment)
		start, _, arriving, _, nextOK := twistOffsetSegmentEnds(segments[(i+1)%len(segments)])
		if !ok || !nextOK || end != start {
			return false
		}
		cross := new(big.Rat).Sub(new(big.Rat).Mul(leaving.x, arriving.y),
			new(big.Rat).Mul(leaving.y, arriving.x))
		dot := new(big.Rat).Add(new(big.Rat).Mul(leaving.x, arriving.x),
			new(big.Rat).Mul(leaving.y, arriving.y))
		if cross.Sign() != 0 || dot.Sign() <= 0 {
			return false
		}
	}
	return true
}

// TwistOffsetSource proves that the normal-offset contour has a positive
// tangent speed and increasing tangent angle, and stays strictly inside the
// authenticated convex source. It integrates its area without a sampled fit.
func TwistOffsetSource(profile profileRecord, height float64, angle units.Value,
	thickness, thicknessDelta float64) (twistOffsetSourceProof, error) {
	poly, err := TwistPolygonOf(profile)
	if err != nil {
		return twistOffsetSourceProof{}, err
	}
	if !poly.HasArc || !twistOffsetSmoothClosed(profile) || height <= 0 ||
		thickness <= 0 || thicknessDelta < 0 || thickness-thicknessDelta <= 0 ||
		thickness+thicknessDelta >= height {
		return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell needs a smooth rounded section and a positive floor`, decaderr.ErrUnsupported)
	}
	theta, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok {
		return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell angle has no enclosure`, decaderr.ErrUnsupported)
	}
	k, ok := proofbound.IntervalQuo(theta, twistOffsetPoint(height))
	if !ok || proofbound.IntervalSquare(k).Lo.Sign() <= 0 {
		return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell turn is not bounded away from zero`, decaderr.ErrUnsupported)
	}
	k2 := proofbound.IntervalSquare(k)
	t := proofbound.Interval(new(big.Rat).Sub(TwistRat(thickness), TwistRat(thicknessDelta)),
		new(big.Rat).Add(TwistRat(thickness), TwistRat(thicknessDelta)))
	area := twistOffsetUnit(0)
	var clearanceLo, speedLo, turnRateLo, radialLo *big.Rat
	const parts = int64(64)
	for _, segment := range profile.Outer.Segments {
		for index := range parts {
			var span twistOffsetSpan
			switch edge := segment.(type) {
			case lineSeg:
				span, err = twistOffsetLineSpan(edge, index, index+1, parts)
			case arcSeg:
				span, err = twistOffsetArcSpan(edge, index, index+1, parts)
			default:
				err = fmt.Errorf(`%w: twisted shell source has an unsupported curve`, decaderr.ErrUnsupported)
			}
			if err != nil {
				return twistOffsetSourceProof{}, err
			}
			f2 := proofbound.IntervalSquare(span.f)
			g, rootOK := proofbound.SqrtInterval(proofbound.IntervalAdd(twistOffsetUnit(1),
				proofbound.IntervalMul(k2, f2)))
			if !rootOK || g.Lo.Sign() <= 0 {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell normal has no positive length`, decaderr.ErrUnsupported)
			}
			a, quoOK := proofbound.IntervalQuo(t, g)
			if !quoOK {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell normal distance is unbounded`, decaderr.ErrUnsupported)
			}
			fPrime := proofbound.IntervalAdd(twistOffsetUnit(1),
				proofbound.IntervalMul(span.kappa, span.h))
			g3 := proofbound.IntervalMul(proofbound.IntervalSquare(g), g)
			betaPrime, quoOK := proofbound.IntervalQuo(proofbound.IntervalMul(
				proofbound.IntervalMul(k2, t), fPrime), g3)
			if !quoOK {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell tangent turn is unbounded`, decaderr.ErrUnsupported)
			}
			lambda := proofbound.IntervalSub(proofbound.IntervalSub(twistOffsetUnit(1),
				proofbound.IntervalMul(a, span.kappa)),
				proofbound.IntervalMul(betaPrime, proofbound.IntervalAdd(span.h, a)))
			turnRate := proofbound.IntervalAdd(span.kappa, betaPrime)
			beta, quoOK := proofbound.IntervalQuo(proofbound.IntervalMul(
				proofbound.IntervalMul(k2, t), span.f), g)
			if !quoOK {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell offset angle is unbounded`, decaderr.ErrUnsupported)
			}
			clearance := new(big.Rat).Sub(a.Lo, new(big.Rat).Mul(
				TwistRat(poly.MaxRadiusL1), proofbound.IntervalAbsUpper(beta)))
			if lambda.Lo.Sign() <= 0 || turnRate.Lo.Sign() <= 0 || clearance.Sign() <= 0 {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell offset has no convex clearance proof`, decaderr.ErrUnsupported)
			}
			radius := proofbound.IntervalSub(proofbound.IntervalNeg(span.h), a)
			if radius.Lo.Sign() <= 0 {
				return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell offset has no positive area density`, decaderr.ErrUnsupported)
			}
			area = proofbound.IntervalAdd(area, proofbound.IntervalScale(proofbound.IntervalMul(
				span.length, proofbound.IntervalMul(lambda, radius)), big.NewRat(1, 2)))
			if clearanceLo == nil || clearance.Cmp(clearanceLo) < 0 {
				clearanceLo = clearance
			}
			if speedLo == nil || lambda.Lo.Cmp(speedLo) < 0 {
				speedLo = lambda.Lo
			}
			if turnRateLo == nil || turnRate.Lo.Cmp(turnRateLo) < 0 {
				turnRateLo = turnRate.Lo
			}
			if radialLo == nil || radius.Lo.Cmp(radialLo) < 0 {
				radialLo = radius.Lo
			}
		}
	}
	if area.Lo.Sign() <= 0 || area.Hi.Cmp(poly.AreaInterval.Lo) >= 0 {
		return twistOffsetSourceProof{}, fmt.Errorf(`%w: twisted shell offset area is not enclosed inside source`, decaderr.ErrUnsupported)
	}
	return twistOffsetSourceProof{Area: area, ClearanceLo: clearanceLo,
		SpeedLo: speedLo, TurnRateLo: turnRateLo, RadialLo: radialLo,
		FirstMomentX: new(big.Rat), FirstMomentY: new(big.Rat)}, nil
}
