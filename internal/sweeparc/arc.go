// Package sweeparc derives the exact circle and bounded angle of a spatial arc.
package sweeparc

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/r3"
)

// Record holds the exact carrier derived from three recorded points.
type Record struct {
	Center       RatVec
	RadiusStart  RatVec
	RadiusMiddle RatVec
	RadiusEnd    RatVec
	Axis         RatVec
}

// RecordArc derives the exact circle through the three stated points.
func RecordArc(start, through, end r3.Vec) (Record, error) {
	p0, pm, p1 := VecOf(start), VecOf(through), VecOf(end)
	a, b := Sub(pm, p0), Sub(p1, p0)
	aa, ab, bb := Dot(a, a), Dot(a, b), Dot(b, b)
	det := new(big.Rat).Sub(
		new(big.Rat).Mul(aa, bb),
		new(big.Rat).Mul(ab, ab),
	)
	if det.Sign() == 0 {
		return Record{}, fmt.Errorf(`%w: a sweep arc requires three non-collinear points`, decaderr.ErrDegenerate)
	}
	twoDet := new(big.Rat).Mul(det, big.NewRat(2, 1))
	alpha := new(big.Rat).Quo(
		new(big.Rat).Mul(bb, new(big.Rat).Sub(aa, ab)),
		twoDet,
	)
	beta := new(big.Rat).Quo(
		new(big.Rat).Mul(aa, new(big.Rat).Sub(bb, ab)),
		twoDet,
	)
	centerRat := Add(p0, Scale(a, alpha), Scale(b, beta))
	r0 := Sub(p0, centerRat)
	rm := Sub(pm, centerRat)
	r1 := Sub(p1, centerRat)
	axisRat := Cross(a, b)
	return Record{
		Center:       centerRat,
		RadiusStart:  r0,
		RadiusMiddle: rm,
		RadiusEnd:    r1,
		Axis:         axisRat,
	}, nil
}

// AxisLine reads the exact carrier's axis in the recorded profile plane.
func AxisLine(center, axis RatVec, origin, u, v r3.Vec) (revolveaxis.Line2, error) {
	rel := Sub(center, VecOf(origin))
	aUExact, aVExact := Dot(rel, VecOf(u)), Dot(rel, VecOf(v))
	dURaw, dVRaw := Dot(axis, VecOf(u)), Dot(axis, VecOf(v))
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(dURaw, dURaw),
		new(big.Rat).Mul(dVRaw, dVRaw),
	)
	if lengthSquared.Sign() == 0 {
		return revolveaxis.Line2{}, fmt.Errorf(`%w: the sweep arc has no axis direction in the profile plane`, decaderr.ErrDegenerate)
	}

	aU, aUBound, ok := Held(aUExact)
	if !ok {
		return revolveaxis.Line2{}, fmt.Errorf(`%w: the sweep arc's axis anchor is outside the representable range`, decaderr.ErrUnsupported)
	}
	aV, aVBound, ok := Held(aVExact)
	if !ok {
		return revolveaxis.Line2{}, fmt.Errorf(`%w: the sweep arc's axis anchor is outside the representable range`, decaderr.ErrUnsupported)
	}
	dU, dV, ok := Normalized2(dURaw, dVRaw, lengthSquared)
	if !ok {
		return revolveaxis.Line2{}, fmt.Errorf(`%w: the sweep arc's axis direction is outside the representable range`, decaderr.ErrUnsupported)
	}
	dUBound, dVBound := revolveaxis.AxisDirectionSqrtBracket(dURaw, dVRaw, dU, dV)
	if !revolveaxis.FiniteAxisValues(aU, aV, aUBound, aVBound, dU, dV, dUBound, dVBound) {
		return revolveaxis.Line2{}, fmt.Errorf(`%w: the sweep arc's axis has no finite publication bound`, decaderr.ErrUnsupported)
	}
	return revolveaxis.Line2{
		AU: aU, AV: aV,
		AUBound: aUBound, AVBound: aVBound,
		DU: dU, DV: dV,
		DUBound: dUBound, DVBound: dVBound,
	}, nil
}

// ArcAngle encloses the directed angle between two carrier radii.
func ArcAngle(r0, r1, axis RatVec) (float64, revolveangle.Angle, error) {
	dot := Dot(r0, r1)
	cross := Cross(r0, r1)
	crossSquared := Dot(cross, cross)
	orientation := Dot(axis, cross).Sign()
	if dot.Sign() == 0 {
		if orientation > 0 {
			return math.Pi / 2, revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 4)}, nil
		}
		return 3 * math.Pi / 2, revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(3, 4)}, nil
	}
	if orientation == 0 {
		if dot.Sign() >= 0 {
			return 0, revolveangle.Angle{}, fmt.Errorf(`%w: the sweep arc closes without a directed span`, decaderr.ErrDegenerate)
		}
		return math.Pi, revolveangle.Angle{Rad: new(big.Rat), Turn: big.NewRat(1, 2)}, nil
	}
	sinMagnitude, ok := proofbound.IntervalSqrt(proofbound.PointInterval(crossSquared))
	if !ok || sinMagnitude.Lo.Sign() <= 0 {
		return 0, revolveangle.Angle{}, fmt.Errorf(`%w: the sweep arc angle has no finite enclosure`, decaderr.ErrUnsupported)
	}
	positive := positiveAtan2Span(sinMagnitude, dot)
	span := positive
	if orientation < 0 {
		span = proofbound.IntervalSub(proofbound.TwoPiInterval(), positive)
	}
	phiRat := new(big.Rat).Quo(new(big.Rat).Add(span.Lo, span.Hi), big.NewRat(2, 1))
	phi, _, ok := Held(phiRat)
	if !ok || phi <= 0 || phi >= 2*math.Pi {
		return 0, revolveangle.Angle{}, fmt.Errorf(`%w: the sweep arc angle is outside the representable range`, decaderr.ErrUnsupported)
	}
	return phi, revolveangle.Angle{Span: &span}, nil
}

func positiveAtan2Span(y proofbound.RatInterval, x *big.Rat) proofbound.RatInterval {
	if x.Sign() >= 0 {
		return proofbound.Interval(proofbound.Atan2Interval(y.Lo, x, false).Lo, proofbound.Atan2Interval(y.Hi, x, false).Hi)
	}
	return proofbound.Interval(proofbound.Atan2Interval(y.Hi, x, false).Lo, proofbound.Atan2Interval(y.Lo, x, false).Hi)
}
