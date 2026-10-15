package decad

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
		lfMul(lfF(stretch), radiusBound).Cmp(big.NewRat(1, 1_000_000_000_000)) >= 0 {
		return false, false
	}
	corners := loftface.BoxCorners(box)
	axis := [3]loftFaceRatVec{{lfOne(), lfZero(), lfZero()},
		{lfZero(), lfOne(), lfZero()}, {lfZero(), lfZero(), lfOne()}}
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
			lfSub(capC, cap[0].Lo).Cmp(big.NewRat(12, 1)) >= 0 {
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
		if !spiralCellSide(trim.Maps[1], trim.Maps[2], a, z, lfF(departure),
			big.NewRat(-1, 1), capC, false, lfZero(), lfZero(), 0) ||
			!spiralCellSide(trim.Maps[1], trim.Maps[2], a, z, lfF(departure),
				big.NewRat(-1, 1), capC, true, lfOne(), lfOne(), 0) {
			return false, nil
		}
		var caps [2][3]loftFaceInterval
		for index, mapped := range [...]loftFaceMap{trim.Maps[1], trim.Maps[2]} {
			first := mapped.At(lfF(a.U), lfF(a.V))
			second := mapped.At(lfF(z.U), lfF(z.V))
			for axis := range 3 {
				minimum, maximum := first[axis], second[axis]
				if minimum.Cmp(maximum) > 0 {
					minimum, maximum = maximum, minimum
				}
				caps[index][axis] = loftFaceInterval{
					Lo: lfSub(minimum, lfF(departure)), Hi: lfAdd(maximum, lfF(departure))}
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
	a := first.At(lfF(point.U), lfF(point.V))
	b := second.At(lfF(point.U), lfF(point.V))
	delta := loftface.VSub(b, a)
	at := func(t *big.Rat) loftFaceRatVec {
		return loftface.VAdd(a, loftface.VScale(delta, t))
	}
	q := func(t *big.Rat) *big.Rat {
		p := at(t)
		r := lfSub(capC, p[0])
		return lfSub(lfAdd(lfMul(p[1], p[1]), lfMul(p[2], p[2])), lfMul(r, r))
	}
	lo, hi := big.NewRat(9, 10), lfOne()
	if q(lo).Sign() >= 0 || q(hi).Sign() <= 0 {
		return empty, false, nil
	}
	for i := range 70 {
		if i%8 == 0 {
			if err := ctx.Err(); err != nil {
				return empty, false, err
			}
		}
		mid := lfMul(lfAdd(lo, hi), big.NewRat(1, 2))
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
		out[axis] = loftFaceInterval{Lo: lfSub(minValue, charge), Hi: lfAdd(maxValue, charge)}
	}
	return out, true, nil
}

// certifySpiralToeRootSeam accepts only a current first-tooth source proof.
// It certifies one continuous transverse source root-arc crossing with the
// blank toe cap, a finite X and angular-slope range, and both exact ends.
func certifySpiralToeRootSeam(ctx context.Context,
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
	capC := lfAdd(lfF(proof.Blank.ToeRoot.U), lfF(proof.Blank.ToeRoot.V))
	if lfAdd(lfF(proof.Blank.ToeOuter.U), lfF(proof.Blank.ToeOuter.V)).Cmp(capC) != 0 ||
		capC.Sign() <= 0 || lfF(proof.Blank.ToeRoot.U).Cmp(lfF(proof.Blank.ToeOuter.U)) >= 0 {
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
	}{{p.d, 1}, {p.deltaX, 1}, {p.at(lfZero()), -1}, {p.at(lfOne()), 1}} {
		if yes, err := polynomial.BernsteinSignByHalving(ctx, gate.coeff, gate.sign, 8); err != nil || !yes {
			return nil, false, err
		}
	}
	if yes, err := spiralToeIdealDerivative(ctx, p); err != nil || !yes {
		return nil, false, err
	}
	idealXLo, idealXHi := big.NewRat(21955, 2000), big.NewRat(21957, 2000)
	for _, gate := range [...]struct {
		value *big.Rat
		sign  int
	}{{idealXLo, -1}, {idealXHi, 1}} {
		if yes, err := polynomial.BernsteinSignByHalving(ctx, p.fixedX(gate.value), gate.sign, 12); err != nil || !yes {
			return nil, false, err
		}
	}
	idealSLo, idealSHi := big.NewRat(345, 2000), big.NewRat(307, 1000)
	if negative {
		idealSLo, idealSHi = lfSub(lfZero(), idealSHi), lfSub(lfZero(), idealSLo)
	}
	for _, gate := range [...]struct {
		slope *big.Rat
		sign  int
	}{{idealSLo, 1}, {idealSHi, -1}} {
		for _, x := range [...]*big.Rat{idealXLo, idealXHi} {
			if yes, err := polynomial.BernsteinSignByHalving(ctx, spiralToeFixedSlopeAtX(p, x, gate.slope),
				gate.sign, 12); err != nil || !yes {
				return nil, false, err
			}
		}
	}
	coordCharge, slopeCharge := big.NewRat(1, 10_000_000_000), big.NewRat(1, 1_000_000_000)
	seam := &spiralToeRootSeam{
		X:        loftFaceInterval{Lo: lfSub(idealXLo, coordCharge), Hi: lfAdd(idealXHi, coordCharge)},
		Slope:    loftFaceInterval{Lo: lfSub(idealSLo, slopeCharge), Hi: lfAdd(idealSHi, slopeCharge)},
		ConeC:    capC,
		Negative: negative,
	}
	if seam.X.Lo.Cmp(lfF(proof.Blank.ToeRoot.U)) <= 0 ||
		seam.X.Hi.Cmp(lfF(proof.Blank.ToeOuter.U)) >= 0 {
		return nil, false, nil
	}
	for end, point := range [...]planePoint{arc.Start, arc.End} {
		endpoint, yes, err := spiralToeEndpointBox(ctx, trim.Maps[1], trim.Maps[2],
			point, capC, coordCharge)
		if err != nil || !yes {
			return nil, false, err
		}
		ratio, yes := spiralHeelRatio(endpoint)
		if !yes || endpoint[0].Lo.Cmp(seam.X.Lo) <= 0 ||
			endpoint[0].Hi.Cmp(seam.X.Hi) >= 0 ||
			ratio.Lo.Cmp(seam.Slope.Lo) <= 0 || ratio.Hi.Cmp(seam.Slope.Hi) >= 0 {
			return nil, false, nil
		}
		seam.Ends[end] = endpoint
	}
	return seam, true, nil
}
