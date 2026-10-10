package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// loftFitArcDeparture bounds the entire recorded connector against the
// radius-r tangent connector defined by an isolated fit/circle root. Both
// arcs are compared at the same fraction of their certified short sweeps.
// The endpoint angle differences interpolate linearly, so the largest one
// bounds every intermediate angle difference.
func loftFitArcDeparture(cert loftFitRootCertificate, arc arcSeg, fitArrives bool,
	radius float64) (departure, idealLength float64, err error) {
	refuse := func() (float64, float64, error) {
		return 0, 0, fmt.Errorf(`%w: the loft fit-circle arc departure cannot be certified`, ErrUnsupported)
	}
	r, ok := proofbound.RatOf(radius)
	if !ok || r.Sign() <= 0 {
		return refuse()
	}
	seg := circularbounds.RecordSegment(arc)
	rRec, sweepRec, ok := circularbounds.WalkEnclosures(seg)
	if !ok || rRec.Lo.Sign() <= 0 || sweepRec.Lo.Sign() <= 0 ||
		sweepRec.Hi.Cmp(proofbound.PiLower) >= 0 {
		return refuse()
	}
	caU, caV := cert.FitFootU, cert.FitFootV
	cbU, cbV := cert.CircleFootU, cert.CircleFootV
	if !fitArrives {
		caU, cbU = cbU, caU
		caV, cbV = cbV, caV
	}
	recordedStartU, recordedStartV, ok := circularbounds.EndpointInterval(seg, new(big.Rat).SetFloat64(arc.TStart))
	if !ok {
		return refuse()
	}
	recordedEndU, recordedEndV, ok := circularbounds.EndpointInterval(seg, new(big.Rat).SetFloat64(arc.TEnd))
	if !ok {
		return refuse()
	}
	recordCenterU, recordCenterV := proofbound.PointInterval(new(big.Rat).SetFloat64(arc.Center.U)),
		proofbound.PointInterval(new(big.Rat).SetFloat64(arc.Center.V))
	idealInv := new(big.Rat).Inv(r)
	recordedInv := proofbound.Interval(new(big.Rat).Inv(rRec.Hi), new(big.Rat).Inv(rRec.Lo))
	type vector struct{ u, v proofbound.RatInterval }
	ideal := [2]vector{
		{proofbound.IntervalScale(proofbound.IntervalSub(caU, cert.CenterU), idealInv),
			proofbound.IntervalScale(proofbound.IntervalSub(caV, cert.CenterV), idealInv)},
		{proofbound.IntervalScale(proofbound.IntervalSub(cbU, cert.CenterU), idealInv),
			proofbound.IntervalScale(proofbound.IntervalSub(cbV, cert.CenterV), idealInv)},
	}
	recorded := [2]vector{
		{proofbound.IntervalMul(proofbound.IntervalSub(recordedStartU, recordCenterU), recordedInv),
			proofbound.IntervalMul(proofbound.IntervalSub(recordedStartV, recordCenterV), recordedInv)},
		{proofbound.IntervalMul(proofbound.IntervalSub(recordedEndU, recordCenterU), recordedInv),
			proofbound.IntervalMul(proofbound.IntervalSub(recordedEndV, recordCenterV), recordedInv)},
	}
	cross := proofbound.IntervalSub(
		proofbound.IntervalMul(ideal[0].u, ideal[1].v),
		proofbound.IntervalMul(ideal[0].v, ideal[1].u),
	)
	if arc.TStart < arc.TEnd {
		if cross.Lo.Sign() <= 0 {
			return refuse()
		}
	} else if cross.Hi.Sign() >= 0 {
		return refuse()
	}
	var angleError *big.Rat
	for i := range ideal {
		du := proofbound.IntervalSub(ideal[i].u, recorded[i].u)
		dv := proofbound.IntervalSub(ideal[i].v, recorded[i].v)
		duMax, dvMax := proofbound.IntervalAbsUpper(du), proofbound.IntervalAbsUpper(dv)
		chordSq := new(big.Rat).Add(new(big.Rat).Mul(duMax, duMax), new(big.Rat).Mul(dvMax, dvMax))
		chordUpper := proofbound.RatSqrtUp(chordSq)
		if proofbound.IsNonFinite(chordUpper) {
			return refuse()
		}
		// For unit vectors, angle <= (pi/2)*their chord distance.
		bound := new(big.Rat).Mul(proofbound.PiUpper,
			new(big.Rat).Quo(new(big.Rat).SetFloat64(chordUpper), big.NewRat(2, 1)))
		if angleError == nil || bound.Cmp(angleError) > 0 {
			angleError = bound
		}
	}
	if angleError == nil {
		return refuse()
	}
	margin := new(big.Rat).Sub(proofbound.PiLower, sweepRec.Hi)
	if sweepRec.Lo.Cmp(margin) < 0 {
		margin = sweepRec.Lo
	}
	if new(big.Rat).Mul(angleError, big.NewRat(4, 1)).Cmp(margin) >= 0 {
		return refuse()
	}
	idealSweepUpper := new(big.Rat).Add(sweepRec.Hi, new(big.Rat).Mul(angleError, big.NewRat(2, 1)))
	idealLength = proofbound.RatFloatUp(new(big.Rat).Mul(r, idealSweepUpper))
	centerDU := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CenterU, recordCenterU))
	centerDV := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CenterV, recordCenterV))
	centerErr := proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(centerDU, centerDU), new(big.Rat).Mul(centerDV, centerDV)))
	radiusErr := proofbound.IntervalAbsUpper(proofbound.IntervalSub(rRec, proofbound.PointInterval(r)))
	rMax := proofbound.RatMax(r, rRec.Hi)
	departure = proofbound.AbsSumUpper(centerErr, proofbound.RatFloatUp(radiusErr),
		proofbound.RatFloatUp(new(big.Rat).Mul(rMax, angleError)))
	if math.IsNaN(departure) || math.IsInf(departure, 0) ||
		math.IsNaN(idealLength) || math.IsInf(idealLength, 0) ||
		departure > filletTol {
		return refuse()
	}
	return departure, idealLength, nil
}
