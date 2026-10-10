package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// loftFitCarrierDeparture bounds the parameter shift of both retained
// carriers when their ideal tangent feet replace the recorded float trims.
// Each carrier uses the same monotone normalized traversal before and after
// the trim. A cell endpoint's parameter shift is no larger than its changed
// end's shift, so twice the path departure is a per-cell length allowance.
func loftFitCarrierDeparture(cert loftFitRootCertificate, circle circleSeg,
	numericFitT, numericCircleT float64, fitArrives bool) (
	fitDeparture, circleDeparture float64, circleParam proofbound.RatInterval, err error) {
	refuse := func() (float64, float64, proofbound.RatInterval, error) {
		return 0, 0, proofbound.RatInterval{},
			fmt.Errorf(`%w: the loft fit-circle carrier departure cannot be certified`, ErrUnsupported)
	}
	if !finiteLoftFitValue(numericFitT, numericCircleT, cert.SpeedUpper) || cert.SpeedUpper <= 0 {
		return refuse()
	}
	fitShift := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.Param,
		proofbound.PointInterval(new(big.Rat).SetFloat64(numericFitT))))
	fitDeparture = proofbound.ProductUpper(cert.SpeedUpper, proofbound.RatFloatUp(fitShift))
	radius, unitErr := circle.Radius.In(units.Millimeter)
	if unitErr != nil || !finiteLoftFitValue(radius) || radius <= 0 {
		return refuse()
	}
	u, v, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(circle),
		new(big.Rat).SetFloat64(numericCircleT))
	if !ok {
		return refuse()
	}
	du := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CircleFootU, u))
	dv := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CircleFootV, v))
	chord := proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
	if !finiteLoftFitValue(chord) {
		return refuse()
	}
	// The shorter angular difference between two points on one circle is at
	// most pi/2 times their unit-vector chord distance. Enclose the ideal
	// parameter around the recorded trim and require that whole interval to
	// remain strictly inside the original circle fragment. This also picks
	// the same unwrapped turn branch rather than accepting a nearby point on
	// a different revolution.
	angle := new(big.Rat).Quo(new(big.Rat).Mul(
		proofbound.PiUpper, new(big.Rat).SetFloat64(chord)),
		new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).SetFloat64(radius)))
	turn := new(big.Rat).Quo(angle, proofbound.TwoPiInterval().Lo)
	parameter := new(big.Rat).SetFloat64(numericCircleT)
	paramLo, paramHi := new(big.Rat).Sub(parameter, turn), new(big.Rat).Add(parameter, turn)
	originalLo := new(big.Rat).SetFloat64(min(circle.TStart, circle.TEnd))
	originalHi := new(big.Rat).SetFloat64(max(circle.TStart, circle.TEnd))
	if paramLo.Cmp(originalLo) <= 0 || paramHi.Cmp(originalHi) >= 0 ||
		new(big.Rat).Sub(originalHi, originalLo).Cmp(big.NewRat(1, 2)) >= 0 {
		return refuse()
	}
	circleDeparture = proofbound.RatFloatUp(new(big.Rat).Mul(
		new(big.Rat).SetFloat64(radius), angle))
	other := circle.TStart
	if fitArrives {
		other = circle.TEnd
	}
	remaining := proofbound.RatFloatDown(new(big.Rat).Mul(
		new(big.Rat).Mul(proofbound.TwoPiInterval().Lo, new(big.Rat).SetFloat64(radius)),
		new(big.Rat).Abs(new(big.Rat).Sub(
			new(big.Rat).SetFloat64(other), new(big.Rat).SetFloat64(numericCircleT)))))
	if !finiteLoftFitValue(circleDeparture, remaining) ||
		fitDeparture > filletTol || circleDeparture > filletTol ||
		circleDeparture >= remaining/4 {
		return refuse()
	}
	return fitDeparture, circleDeparture, proofbound.Interval(paramLo, paramHi), nil
}
