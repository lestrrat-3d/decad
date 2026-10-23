package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// spiralToeRootSeam encloses the source root arc where it crosses the
// blank's finite toe cap. The heel seam and toe seam use the same exact
// X, angular-slope, and endpoint readings.
type spiralToeRootSeam = spiralHeelRootSeam

// At the toe cap R=C-X. The ideal arc uses the source radius recorded by
// Sketch; spiralHeelSourceBox bounds that carrier's discrepancy from the
// actual source arc, including the opposite endpoint's radius.
func spiralToeMapCharge(first, second loftFaceMap, box [4]*big.Rat,
	radiusBound, capC *big.Rat) (bool, bool) {
	if loftface.PositiveDenominator(first, box) != nil || loftface.PositiveDenominator(second, box) != nil {
		return false, false
	}
	stretch := math.Max(first.Stretch, second.Stretch)
	if proofbound.IsNonFinite(stretch) ||
		seamRatMul(seamRatF(stretch), radiusBound).Cmp(big.NewRat(1, 1_000_000_000_000)) >= 0 {
		return false, false
	}
	corners := loftface.BoxCorners(box)
	axis := [3]loftFaceRatVec{{seamRatOne(), seamRatZero(), seamRatZero()},
		{seamRatZero(), seamRatOne(), seamRatZero()}, {seamRatZero(), seamRatZero(), seamRatOne()}}
	var firstRange, secondRange, delta [3]loftFaceInterval
	for i := range axis {
		firstRange[i] = loftface.ScalarRange(first, axis[i], corners)
		secondRange[i] = loftface.ScalarRange(second, axis[i], corners)
		delta[i] = loftface.IntervalSub(secondRange[i], firstRange[i])
	}
	for _, cap := range [][3]loftFaceInterval{firstRange, secondRange} {
		if cap[0].Lo.Cmp(big.NewRat(9, 1)) <= 0 || cap[0].Hi.Cmp(big.NewRat(12, 1)) >= 0 ||
			cap[1].Lo.Cmp(big.NewRat(8, 1)) <= 0 || cap[1].Hi.Cmp(big.NewRat(11, 1)) >= 0 ||
			cap[2].Lo.Cmp(big.NewRat(-4, 1)) <= 0 || cap[2].Hi.Cmp(big.NewRat(4, 1)) >= 0 ||
			seamRatSub(capC, cap[0].Lo).Cmp(big.NewRat(12, 1)) >= 0 {
			return false, false
		}
	}
	for _, span := range delta {
		if span.Lo.Cmp(big.NewRat(-4, 1)) <= 0 || span.Hi.Cmp(big.NewRat(4, 1)) >= 0 {
			return false, false
		}
	}
	if delta[0].Lo.Sign() <= 0 {
		return false, false
	}
	negative := firstRange[2].Hi.Sign() < 0 && secondRange[2].Hi.Sign() < 0
	positive := firstRange[2].Lo.Sign() > 0 && secondRange[2].Lo.Sign() > 0
	return negative || positive, negative
}

// An arc point is within 10^-12 mm of its ideal mapped point. With
// 9<X<12, 8<Y<11, |Z|<4, 8<R<12, and each cap span <4 mm, this changes
// |Q| by less than 5.5*10^-11 mm². Q_t>20 moves its one root by less
// than 2.75*10^-12 and each coordinate by less than 1.2*10^-11 mm.
// The published charges of 10^-10 mm and 10^-9 in Z/Y exceed those
// bounds, since Y>8. Every premise is checked here or below.
func spiralToeIdealDerivative(ctx context.Context, p spiralHeelPolys) (bool, error) {
	delta := [3][]*big.Rat{}
	for axis := range delta {
		delta[axis] = polynomial.BernsteinSub(p.b[axis], p.a[axis])
	}
	for _, at := range [...][3][]*big.Rat{p.a, p.b} {
		radius := polynomial.BernsteinSub(polynomial.BernsteinScale(p.d, p.coneC), at[0])
		derivative := polynomial.BernsteinScale(polynomial.BernsteinAdd(polynomial.BernsteinAdd(
			polynomial.BernsteinMul(at[1], delta[1]), polynomial.BernsteinMul(at[2], delta[2])),

			polynomial.BernsteinMul(radius, delta[0])),

			big.NewRat(2, 1))

		lower := polynomial.BernsteinSub(derivative, polynomial.BernsteinScale(polynomial.BernsteinMul(p.d, p.d), big.NewRat(20, 1)))
		ok, err := polynomial.BernsteinSignByHalving(ctx, lower, 1, 8)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

func spiralToeFixedSlopeAtX(p spiralHeelPolys, x, slope *big.Rat) []*big.Rat {
	target := polynomial.BernsteinSub(polynomial.BernsteinScale(p.d, x), p.a[0])
	l0 := polynomial.BernsteinSub(p.a[2], polynomial.BernsteinScale(p.a[1], slope))
	deltaSlope := polynomial.BernsteinSub(polynomial.BernsteinSub(p.b[2], p.a[2]),
		polynomial.BernsteinScale(polynomial.BernsteinSub(p.b[1], p.a[1]), slope))

	return polynomial.BernsteinAdd(polynomial.BernsteinMul(l0, p.deltaX), polynomial.BernsteinMul(target, deltaSlope))
}

// The retained station cells enclose the actual Sketch arc, rather than
// only the ideal radius used by the polynomial gates.
func spiralToeRootCells(ctx context.Context, trim *loftFacesTrimRecord,
	capC *big.Rat) (bool, error) {
	count := 0
	for j, group := range trim.StationGroup {
		if group != 4 {
			continue
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if j >= len(trim.Bounds[1].SourceDeparture) ||
			j >= len(trim.Bounds[2].SourceDeparture) ||
			j >= len(trim.Bounds[1].ProjectiveDeparture) ||
			j >= len(trim.Bounds[2].ProjectiveDeparture) {
			return false, nil
		}
		lo, hi := trim.Bounds[1], trim.Bounds[2]
		departure := math.Max(proofbound.AbsSumUpper(lo.SourceDeparture[j], lo.ProjectiveDeparture[j]),
			proofbound.AbsSumUpper(hi.SourceDeparture[j], hi.ProjectiveDeparture[j]))
		if proofbound.IsNonFinite(departure) {
			return false, nil
		}
		a := trim.Source.SourceStation[j]
		z := trim.Source.SourceStation[(j+1)%len(trim.StationGroup)]
		if !SpiralCellSide(trim.Maps[1], trim.Maps[2], a, z, seamRatF(departure),
			big.NewRat(-1, 1), capC, false, seamRatZero(), seamRatZero(), 0) ||
			!SpiralCellSide(trim.Maps[1], trim.Maps[2], a, z, seamRatF(departure),
				big.NewRat(-1, 1), capC, true, seamRatOne(), seamRatOne(), 0) {
			return false, nil
		}
		var caps [2][3]loftFaceInterval
		for index, mapped := range [...]loftFaceMap{trim.Maps[1], trim.Maps[2]} {
			first := mapped.At(seamRatF(a.U), seamRatF(a.V))
			second := mapped.At(seamRatF(z.U), seamRatF(z.V))
			for axis := range 3 {
				minimum, maximum := first[axis], second[axis]
				if minimum.Cmp(maximum) > 0 {
					minimum, maximum = maximum, minimum
				}
				caps[index][axis] = loftFaceInterval{
					Lo: seamRatSub(minimum, seamRatF(departure)), Hi: seamRatAdd(maximum, seamRatF(departure))}
			}
		}
		var point, delta [3]loftFaceInterval
		for axis := range 3 {
			point[axis] = loftface.IntervalHull(caps[0][axis], caps[1][axis])
			delta[axis] = loftface.IntervalSub(caps[1][axis], caps[0][axis])
		}
		radius := loftface.IntervalSub(loftFaceInterval{Lo: capC, Hi: capC}, point[0])
		if radius.Lo.Sign() <= 0 || point[1].Lo.Sign() <= 0 {
			return false, nil
		}
		halfDerivative := spiralHeelIntervalAdd(spiralHeelIntervalAdd(
			spiralHeelIntervalMul(point[1], delta[1]),
			spiralHeelIntervalMul(point[2], delta[2])),
			spiralHeelIntervalMul(radius, delta[0]))
		if halfDerivative.Lo.Cmp(big.NewRat(10, 1)) <= 0 {
			return false, nil
		}
		count++
	}
	return count == 15, nil
}

func spiralToeEndpointBox(ctx context.Context, first, second loftFaceMap,
	point planePoint, capC, charge *big.Rat) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	a := first.At(seamRatF(point.U), seamRatF(point.V))
	b := second.At(seamRatF(point.U), seamRatF(point.V))
	delta := loftface.VSub(b, a)
	at := func(t *big.Rat) loftFaceRatVec {
		return loftface.VAdd(a, loftface.VScale(delta, t))
	}
	q := func(t *big.Rat) *big.Rat {
		p := at(t)
		r := seamRatSub(capC, p[0])
		return seamRatSub(seamRatAdd(seamRatMul(p[1], p[1]), seamRatMul(p[2], p[2])), seamRatMul(r, r))
	}
	lo, hi := big.NewRat(9, 10), seamRatOne()
	if q(lo).Sign() >= 0 || q(hi).Sign() <= 0 {
		return empty, false, nil
	}
	for i := range 70 {
		if i%8 == 0 {
			if err := ctx.Err(); err != nil {
				return empty, false, err
			}
		}
		mid := seamRatMul(seamRatAdd(lo, hi), big.NewRat(1, 2))
		if q(mid).Sign() < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	pLo, pHi := at(lo), at(hi)
	var out [3]loftFaceInterval
	for axis := range out {
		minValue, maxValue := pLo[axis], pHi[axis]
		if minValue.Cmp(maxValue) > 0 {
			minValue, maxValue = maxValue, minValue
		}
		out[axis] = loftFaceInterval{Lo: seamRatSub(minValue, charge), Hi: seamRatAdd(maxValue, charge)}
	}
	return out, true, nil
}

// CertifySpiralToeRootSeam accepts only a current first-tooth source proof.
// It certifies one continuous transverse source root-arc crossing with the
// blank toe cap, a finite X and angular-slope range, and both exact ends.
func CertifySpiralToeRootSeam(ctx context.Context,
	proof *spiralFirstJoinSource) (*spiralToeRootSeam, bool, error) {
	if ctx == nil || proof == nil || proof.Tooth == nil || proof.Blank == nil {
		return nil, false, nil
	}
	trim := proof.Tooth
	arc, ok := trim.Source.Profile.Outer.Segments[4].(arcSeg)
	if !ok || len(trim.Maps) != 9 || len(trim.Bounds) != 9 {
		return nil, false, nil
	}
	radius, radiusBound, box, ok := spiralHeelSourceBox(arc)
	if !ok {
		return nil, false, nil
	}
	capC := seamRatAdd(seamRatF(proof.Blank.ToeRoot.U), seamRatF(proof.Blank.ToeRoot.V))
	if seamRatAdd(seamRatF(proof.Blank.ToeOuter.U), seamRatF(proof.Blank.ToeOuter.V)).Cmp(capC) != 0 ||
		capC.Sign() <= 0 || seamRatF(proof.Blank.ToeRoot.U).Cmp(seamRatF(proof.Blank.ToeOuter.U)) >= 0 {
		return nil, false, nil
	}
	mapOK, negative := spiralToeMapCharge(trim.Maps[1], trim.Maps[2], box, radiusBound, capC)
	if !mapOK {
		return nil, false, nil
	}
	if yes, err := spiralToeRootCells(ctx, trim, capC); err != nil || !yes {
		return nil, false, err
	}
	p := spiralHeelBuildPolys(trim.Maps[1], trim.Maps[2], radius, capC)
	for _, gate := range [...]struct {
		coeff []*big.Rat
		sign  int
	}{{p.d, 1}, {p.deltaX, 1}, {p.at(seamRatZero()), -1}, {p.at(seamRatOne()), 1}} {
		if yes, err := polynomial.BernsteinSignByHalving(ctx, gate.coeff, gate.sign, 8); err != nil || !yes {
			return nil, false, err
		}
	}
	if yes, err := spiralToeIdealDerivative(ctx, p); err != nil || !yes {
		return nil, false, err
	}
	coordCharge, slopeCharge := big.NewRat(1, 10_000_000_000), big.NewRat(1, 1_000_000_000)
	var ends [2][3]loftFaceInterval
	var ratios [2]loftFaceInterval
	for end, point := range [...]planePoint{arc.Start, arc.End} {
		endpoint, yes, err := spiralToeEndpointBox(ctx, trim.Maps[1], trim.Maps[2],
			point, capC, coordCharge)
		if err != nil || !yes {
			return nil, false, err
		}
		ratio, yes := spiralHeelRatio(endpoint)
		if !yes {
			return nil, false, nil
		}
		ends[end], ratios[end] = endpoint, ratio
	}
	// The X and slope limits start at the endpoint boxes and move outward along
	// the same ladder as the heel seam until each resultant gate holds.
	idealXLo, yes, err := seamLimit(ends[0][0].Lo, ends[1][0].Lo, -1, seamXLadder,
		func(limit *big.Rat) (bool, error) {
			return polynomial.BernsteinSignByHalving(ctx, p.fixedX(limit), -1, 12)
		})
	if err != nil || !yes {
		return nil, false, err
	}
	idealXHi, yes, err := seamLimit(ends[0][0].Hi, ends[1][0].Hi, 1, seamXLadder,
		func(limit *big.Rat) (bool, error) {
			return polynomial.BernsteinSignByHalving(ctx, p.fixedX(limit), 1, 12)
		})
	if err != nil || !yes {
		return nil, false, err
	}
	slopeHolds := func(sign int) func(*big.Rat) (bool, error) {
		return func(slope *big.Rat) (bool, error) {
			for _, x := range [...]*big.Rat{idealXLo, idealXHi} {
				ok, err := polynomial.BernsteinSignByHalving(ctx, spiralToeFixedSlopeAtX(p, x, slope), sign, 12)
				if err != nil || !ok {
					return false, err
				}
			}
			return true, nil
		}
	}
	idealSLo, yes, err := seamLimit(ratios[0].Lo, ratios[1].Lo, -1, seamSlopeLadder, slopeHolds(1))
	if err != nil || !yes {
		return nil, false, err
	}
	idealSHi, yes, err := seamLimit(ratios[0].Hi, ratios[1].Hi, 1, seamSlopeLadder, slopeHolds(-1))
	if err != nil || !yes {
		return nil, false, err
	}
	seam := &spiralToeRootSeam{
		X:        loftFaceInterval{Lo: seamRatSub(idealXLo, coordCharge), Hi: seamRatAdd(idealXHi, coordCharge)},
		Slope:    loftFaceInterval{Lo: seamRatSub(idealSLo, slopeCharge), Hi: seamRatAdd(idealSHi, slopeCharge)},
		ConeC:    capC,
		Negative: negative,
	}
	if seam.X.Lo.Cmp(seamRatF(proof.Blank.ToeRoot.U)) <= 0 ||
		seam.X.Hi.Cmp(seamRatF(proof.Blank.ToeOuter.U)) >= 0 {
		return nil, false, nil
	}
	for end := range ends {
		if ends[end][0].Lo.Cmp(seam.X.Lo) <= 0 || ends[end][0].Hi.Cmp(seam.X.Hi) >= 0 ||
			ratios[end].Lo.Cmp(seam.Slope.Lo) <= 0 || ratios[end].Hi.Cmp(seam.Slope.Hi) >= 0 {
			return nil, false, nil
		}
		seam.Ends[end] = ends[end]
	}
	return seam, true, nil
}
