package spiralsector

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// A rational half-angle interval encloses the short source arc. These limits
// only admit the current bounded first-tooth geometry; other shapes refuse.
var spiralHeelHalfAngle = big.NewRat(127, 5000)

func spiralHeelCapBern(m loftFaceMap, radius, lo, hi *big.Rat) ([]*big.Rat, [3][]*big.Rat) {
	toBern := func(p [3]*big.Rat) []*big.Rat {
		w := seamRatSub(hi, lo)
		a0 := seamRatAdd(seamRatAdd(p[0], seamRatMul(p[1], lo)), seamRatMul(p[2], seamRatMul(lo, lo)))
		a1 := seamRatMul(w, seamRatAdd(p[1], seamRatMul(big.NewRat(2, 1), seamRatMul(p[2], lo))))
		a2 := seamRatMul(p[2], seamRatMul(w, w))
		return []*big.Rat{a0, seamRatAdd(a0, seamRatMul(a1, big.NewRat(1, 2))), seamRatAdd(seamRatAdd(a0, a1), a2)}
	}
	k := [3]*big.Rat{seamRatOne(), seamRatZero(), seamRatOne()}
	u := [3]*big.Rat{seamRatSub(seamRatZero(), radius), seamRatZero(), radius}
	v := [3]*big.Rat{seamRatZero(), seamRatMul(big.NewRat(2, 1), radius), seamRatZero()}
	var gp [3]*big.Rat
	var hp [3][3]*big.Rat
	for j := range gp {
		gp[j] = seamRatAdd(seamRatAdd(seamRatMul(m.G0, k[j]), seamRatMul(m.GU, u[j])), seamRatMul(m.GV, v[j]))
		for axis := range hp {
			hp[axis][j] = seamRatAdd(seamRatAdd(seamRatMul(m.Constant[axis], gp[j]), seamRatMul(m.Q0[axis], k[j])),
				seamRatAdd(seamRatMul(m.QU[axis], u[j]), seamRatMul(m.QV[axis], v[j])))
		}
	}
	g := toBern(gp)
	var h [3][]*big.Rat
	for axis := range h {
		h[axis] = toBern(hp[axis])
	}
	return g, h
}

// The two cap maps are rational in the source plane. Composing their
// numerators and denominators with u=-r(1-h²)/(1+h²), v=2rh/(1+h²)
// gives degree-four Bernstein cap coordinates. Eliminating band t at fixed X
// or fixed Z/Y leaves a degree-16 polynomial whose exact sign excludes a
// crossing on the selected limit.
type spiralHeelPolys struct {
	d      []*big.Rat
	a, b   [3][]*big.Rat
	deltaX []*big.Rat
	coneC  *big.Rat
}

func spiralHeelSourceBox(arc arcSeg) (*big.Rat, *big.Rat, [4]*big.Rat, bool) {
	var empty [4]*big.Rat
	if arc.Center.U != 0 || arc.Center.V != 0 || arc.TStart != 1 || arc.TEnd != 0 ||
		arc.Start.U >= 0 || arc.End.U >= 0 || arc.Start.V <= 0 || arc.End.V >= 0 {
		return nil, nil, empty, false
	}
	held := math.Hypot(arc.Start.U, arc.Start.V)
	bound := boundarywalk.ArcWalkRadiusBound(arc, held)
	if math.IsNaN(held) || math.IsInf(held, 0) || math.IsNaN(bound) || math.IsInf(bound, 0) {
		return nil, nil, empty, false
	}
	radius := seamRatF(held)
	startR2 := seamRatAdd(seamRatMul(seamRatF(arc.Start.U), seamRatF(arc.Start.U)),
		seamRatMul(seamRatF(arc.Start.V), seamRatF(arc.Start.V)))
	endR2 := seamRatAdd(seamRatMul(seamRatF(arc.End.U), seamRatF(arc.End.U)),
		seamRatMul(seamRatF(arc.End.V), seamRatF(arc.End.V)))
	// The denoted End uses Start's radius and End's ray. Its displacement
	// from the recorded End is |rStart²-rEnd²|/(rStart+rEnd).
	endGap := new(big.Rat).Quo(seamRatAbs(seamRatSub(startR2, endR2)),
		seamRatAdd(seamRatAbs(seamRatF(arc.Start.U)), seamRatAbs(seamRatF(arc.End.U))))
	errorBound := seamRatAdd(seamRatF(bound), endGap)
	rLo, rHi := seamRatSub(radius, errorBound), seamRatAdd(radius, errorBound)
	if rLo.Sign() <= 0 {
		return nil, nil, empty, false
	}
	for _, end := range [...]planePoint{arc.Start, arc.End} {
		// Each endpoint ray is inside the rational half-angle interval.
		// With U<0, its exact radius is at least -U.
		if seamRatAbs(seamRatF(end.V)).Cmp(seamRatMul(spiralHeelHalfAngle,
			seamRatMul(big.NewRat(-2, 1), seamRatF(end.U)))) >= 0 {
			return nil, nil, empty, false
		}
	}
	h2 := seamRatMul(spiralHeelHalfAngle, spiralHeelHalfAngle)
	uUpper := seamRatSub(seamRatZero(), seamRatMul(rLo, new(big.Rat).Quo(seamRatSub(seamRatOne(), h2), seamRatAdd(seamRatOne(), h2))))
	vUpper := new(big.Rat).Quo(seamRatMul(big.NewRat(2, 1), seamRatMul(rHi, spiralHeelHalfAngle)),
		seamRatAdd(seamRatOne(), h2))
	box := [4]*big.Rat{big.NewRat(-514, 25), big.NewRat(-2053, 100),
		big.NewRat(-21, 20), big.NewRat(21, 20)}
	if seamRatSub(seamRatZero(), rHi).Cmp(box[0]) <= 0 || uUpper.Cmp(box[1]) >= 0 ||
		seamRatSub(seamRatZero(), vUpper).Cmp(box[2]) <= 0 || vUpper.Cmp(box[3]) >= 0 {
		return nil, nil, empty, false
	}
	return radius, errorBound, box, true
}

// SpiralHeelCapEightOutside checks the whole final cap map against the blank heel cone.
func SpiralHeelCapEightOutside(capMap loftFaceMap, coneC *big.Rat) bool {
	n0 := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.G0), capMap.Q0)
	nu := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.GU), capMap.QU)
	nv := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.GV), capMap.QV)
	coneVec := func(n loftFaceRatVec, g *big.Rat) loftFaceRatVec {
		return loftFaceRatVec{seamRatSub(seamRatMul(coneC, g), n[0]), n[1], n[2]}
	}
	a, u, v := coneVec(n0, capMap.G0), coneVec(nu, capMap.GU), coneVec(nv, capMap.GV)
	dot := func(x, y loftFaceRatVec) *big.Rat {
		return seamRatSub(seamRatAdd(seamRatMul(x[1], y[1]), seamRatMul(x[2], y[2])), seamRatMul(x[0], y[0]))
	}
	// Q=Y²+Z²-(C-X)² has exactly two positive quadratic terms.
	return dot(a, a).Sign() == 0 && dot(a, u).Sign() == 0 &&
		dot(a, v).Sign() == 0 && dot(u, v).Sign() == 0 &&
		dot(u, u).Sign() > 0 && dot(v, v).Sign() > 0
}

func spiralHeelIntervalAdd(a, b loftFaceInterval) loftFaceInterval {
	return loftFaceInterval{Lo: seamRatAdd(a.Lo, b.Lo), Hi: seamRatAdd(a.Hi, b.Hi)}
}

func spiralHeelIntervalMul(a, b loftFaceInterval) loftFaceInterval {
	values := [4]*big.Rat{seamRatMul(a.Lo, b.Lo), seamRatMul(a.Lo, b.Hi),
		seamRatMul(a.Hi, b.Lo), seamRatMul(a.Hi, b.Hi)}
	lo, hi := values[0], values[0]
	for _, value := range values[1:] {
		if value.Cmp(lo) < 0 {
			lo = value
		}
		if value.Cmp(hi) > 0 {
			hi = value
		}
	}
	return loftFaceInterval{Lo: lo, Hi: hi}
}

// A mapped radius error e changes |Q| by at most (2*(Ymax+Zmax+Rmax)+3e)*e, where
// Ymax, Zmax and Rmax bound the mapped Y, |Z| and R=C-X over the source box. With
// Q_t at least qMin this moves the root t by at most that over qMin, and each root
// coordinate by at most span times that, span being the largest cap-to-cap move.
// Y > Ymin then bounds the change of Z/Y by (1+Zmax/Ymin)*d/(Ymin-d) for a
// coordinate move d. The published charges are 10^-10 mm per coordinate and 10^-9
// in slope. Every premise is read from the maps and gated below.
func spiralHeelMapCharge(first, second loftFaceMap, box [4]*big.Rat,
	radiusBound, coneC *big.Rat) (bool, bool) {
	if loftface.PositiveDenominator(first, box) != nil || loftface.PositiveDenominator(second, box) != nil {
		return false, false
	}
	stretch := math.Max(first.Stretch, second.Stretch)
	if math.IsNaN(stretch) || math.IsInf(stretch, 0) {
		return false, false
	}
	radialError := seamRatMul(seamRatF(stretch), radiusBound)
	if radialError.Cmp(big.NewRat(1, 1_000_000_000_000)) >= 0 {
		return false, false
	}
	corners := loftface.BoxCorners(box)
	axis := [3]loftFaceRatVec{{seamRatOne(), seamRatZero(), seamRatZero()},
		{seamRatZero(), seamRatOne(), seamRatZero()}, {seamRatZero(), seamRatZero(), seamRatOne()}}
	var a, b, delta [3]loftFaceInterval
	for i := range a {
		a[i], b[i] = loftface.ScalarRange(first, axis[i], corners), loftface.ScalarRange(second, axis[i], corners)
		delta[i] = loftface.IntervalSub(b[i], a[i])
	}
	premise := seamChargePremise{}
	for _, cap := range [][3]loftFaceInterval{a, b} {
		if cap[0].Lo.Sign() <= 0 || cap[1].Lo.Sign() <= 0 {
			return false, false
		}
		premise.include(cap[1].Lo, cap[1].Hi, cap[2], seamRatSub(coneC, cap[0].Lo))
	}
	for _, span := range delta {
		premise.span(span)
	}
	if delta[0].Lo.Sign() <= 0 {
		return false, false
	}
	qMin := (*big.Rat)(nil)
	for _, cap := range [][3]loftFaceInterval{a, b} {
		radial := loftFaceInterval{Lo: seamRatSub(coneC, cap[0].Hi), Hi: seamRatSub(coneC, cap[0].Lo)}
		derivative := spiralHeelIntervalAdd(spiralHeelIntervalAdd(
			spiralHeelIntervalMul(cap[1], delta[1]), spiralHeelIntervalMul(cap[2], delta[2])),
			spiralHeelIntervalMul(radial, delta[0]))
		q := seamRatMul(big.NewRat(2, 1), derivative.Lo)
		if qMin == nil || q.Cmp(qMin) < 0 {
			qMin = q
		}
	}
	if !premise.covers(radialError, qMin) {
		return false, false
	}
	negative := a[2].Hi.Sign() < 0 && b[2].Hi.Sign() < 0
	positive := a[2].Lo.Sign() > 0 && b[2].Lo.Sign() > 0
	return negative || positive, negative
}

// seamChargePremise collects the bounds the root-coordinate charge argument reads.
type seamChargePremise struct {
	yMin, yMax, zMax, rMax, spanMax *big.Rat
}

func (p *seamChargePremise) include(yLo, yHi *big.Rat, z loftFaceInterval, radialHi *big.Rat) {
	if p.yMin == nil || yLo.Cmp(p.yMin) < 0 {
		p.yMin = yLo
	}
	if p.yMax == nil || yHi.Cmp(p.yMax) > 0 {
		p.yMax = yHi
	}
	zAbs := seamRatAbs(z.Lo)
	if hi := seamRatAbs(z.Hi); hi.Cmp(zAbs) > 0 {
		zAbs = hi
	}
	if p.zMax == nil || zAbs.Cmp(p.zMax) > 0 {
		p.zMax = zAbs
	}
	if p.rMax == nil || radialHi.Cmp(p.rMax) > 0 {
		p.rMax = radialHi
	}
}

func (p *seamChargePremise) span(s loftFaceInterval) {
	m := seamRatAbs(s.Lo)
	if hi := seamRatAbs(s.Hi); hi.Cmp(m) > 0 {
		m = hi
	}
	if p.spanMax == nil || m.Cmp(p.spanMax) > 0 {
		p.spanMax = m
	}
}

// covers reports whether the published charges, 10^-10 mm per root coordinate and
// 10^-9 in Z/Y, exceed the moves a mapped radius error e can cause when Q_t is at
// least qMin.
func (p *seamChargePremise) covers(e, qMin *big.Rat) bool {
	if p.yMin == nil || qMin == nil || qMin.Sign() <= 0 {
		return false
	}
	three := big.NewRat(3, 1)
	sum := seamRatAdd(seamRatAdd(p.yMax, p.zMax), p.rMax)
	dQ := seamRatMul(seamRatAdd(seamRatMul(big.NewRat(2, 1), sum), seamRatMul(three, e)), e)
	d := seamRatMul(p.spanMax, new(big.Rat).Quo(dQ, qMin))
	if d.Cmp(big.NewRat(1, 10_000_000_000)) >= 0 || d.Cmp(p.yMin) >= 0 {
		return false
	}
	slope := new(big.Rat).Quo(seamRatMul(seamRatAdd(seamRatOne(), new(big.Rat).Quo(p.zMax, p.yMin)), d),
		seamRatSub(p.yMin, d))
	return slope.Cmp(big.NewRat(1, 1_000_000_000)) < 0
}

func spiralHeelBuildPolys(first, second loftFaceMap, radius, coneC *big.Rat) spiralHeelPolys {
	lo := seamRatSub(seamRatZero(), spiralHeelHalfAngle)
	hi := spiralHeelHalfAngle
	g0, h0 := spiralHeelCapBern(first, radius, lo, hi)
	g1, h1 := spiralHeelCapBern(second, radius, lo, hi)
	p := spiralHeelPolys{d: polynomial.BernsteinMul(g0, g1), coneC: coneC}
	for axis := range p.a {
		p.a[axis] = polynomial.BernsteinMul(h0[axis], g1)
		p.b[axis] = polynomial.BernsteinMul(h1[axis], g0)
	}
	p.deltaX = polynomial.BernsteinSub(p.b[0], p.a[0])
	return p
}

func (p spiralHeelPolys) fixedX(x *big.Rat) []*big.Rat {
	target := polynomial.BernsteinSub(polynomial.BernsteinScale(p.d, x), p.a[0])
	den := polynomial.BernsteinMul(p.d, p.deltaX)
	y := polynomial.BernsteinAdd(polynomial.BernsteinMul(p.a[1], p.deltaX), polynomial.BernsteinMul(target, polynomial.BernsteinSub(p.b[1], p.a[1])))
	z := polynomial.BernsteinAdd(polynomial.BernsteinMul(p.a[2], p.deltaX), polynomial.BernsteinMul(target, polynomial.BernsteinSub(p.b[2], p.a[2])))
	radius := seamRatSub(p.coneC, x)
	return polynomial.BernsteinSub(polynomial.BernsteinAdd(polynomial.BernsteinMul(y, y), polynomial.BernsteinMul(z, z)),
		polynomial.BernsteinScale(polynomial.BernsteinMul(den, den), seamRatMul(radius, radius)))
}

func (p spiralHeelPolys) fixedSlope(slope *big.Rat) ([]*big.Rat, []*big.Rat) {
	l0 := polynomial.BernsteinSub(p.a[2], polynomial.BernsteinScale(p.a[1], slope))
	l1 := polynomial.BernsteinSub(p.b[2], polynomial.BernsteinScale(p.b[1], slope))
	delta := polynomial.BernsteinSub(l1, l0)
	var n [3][]*big.Rat
	for axis := range n {
		n[axis] = polynomial.BernsteinSub(polynomial.BernsteinMul(p.a[axis], delta),
			polynomial.BernsteinMul(l0, polynomial.BernsteinSub(p.b[axis], p.a[axis])))
	}
	den := polynomial.BernsteinMul(p.d, delta)
	radius := polynomial.BernsteinSub(polynomial.BernsteinScale(den, p.coneC), n[0])
	q := polynomial.BernsteinSub(polynomial.BernsteinAdd(polynomial.BernsteinMul(n[1], n[1]), polynomial.BernsteinMul(n[2], n[2])),
		polynomial.BernsteinMul(radius, radius))

	return delta, q
}

func (p spiralHeelPolys) at(t *big.Rat) []*big.Rat {
	var n [3][]*big.Rat
	for axis := range n {
		n[axis] = polynomial.BernsteinAdd(polynomial.BernsteinScale(p.a[axis], seamRatSub(seamRatOne(), t)), polynomial.BernsteinScale(p.b[axis], t))
	}
	radius := polynomial.BernsteinSub(polynomial.BernsteinScale(p.d, p.coneC), n[0])
	return polynomial.BernsteinSub(polynomial.BernsteinAdd(polynomial.BernsteinMul(n[1], n[1]), polynomial.BernsteinMul(n[2], n[2])),
		polynomial.BernsteinMul(radius, radius))
}

func spiralHeelRootCells(ctx context.Context, trim *loftFacesTrimRecord, coneC *big.Rat) (bool, error) {
	count := 0
	for j, group := range trim.StationGroup {
		if group != 4 {
			continue
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if j >= len(trim.Bounds[7].SourceDeparture) || j >= len(trim.Bounds[8].SourceDeparture) ||
			j >= len(trim.Bounds[7].ProjectiveDeparture) || j >= len(trim.Bounds[8].ProjectiveDeparture) {
			return false, nil
		}
		lo, hi := trim.Bounds[7], trim.Bounds[8]
		departure := math.Max(proofbound.AbsSumUpper(lo.SourceDeparture[j], lo.ProjectiveDeparture[j]),
			proofbound.AbsSumUpper(hi.SourceDeparture[j], hi.ProjectiveDeparture[j]))
		if proofbound.IsNonFinite(departure) {
			return false, nil
		}
		a := trim.Source.SourceStation[j]
		z := trim.Source.SourceStation[(j+1)%len(trim.StationGroup)]
		if !SpiralCellSide(trim.Maps[7], trim.Maps[8], a, z, seamRatF(departure),
			big.NewRat(-1, 1), coneC, false, seamRatZero(), big.NewRat(19, 20), 8) {
			return false, nil
		}
		count++
	}
	return count == 15, nil
}

func spiralHeelEndpointBox(ctx context.Context, first, second loftFaceMap,
	point planePoint, coneC, charge *big.Rat) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	a := first.At(seamRatF(point.U), seamRatF(point.V))
	b := second.At(seamRatF(point.U), seamRatF(point.V))
	delta := loftface.VSub(b, a)
	at := func(t *big.Rat) loftFaceRatVec { return loftface.VAdd(a, loftface.VScale(delta, t)) }
	q := func(t *big.Rat) *big.Rat {
		p := at(t)
		r := seamRatSub(coneC, p[0])
		return seamRatSub(seamRatAdd(seamRatMul(p[1], p[1]), seamRatMul(p[2], p[2])), seamRatMul(r, r))
	}
	lo, hi := big.NewRat(19, 20), seamRatOne()
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

func spiralHeelRatio(box [3]loftFaceInterval) (loftFaceInterval, bool) {
	if box[1].Lo.Sign() <= 0 {
		return loftFaceInterval{}, false
	}
	values := [4]*big.Rat{
		new(big.Rat).Quo(box[2].Lo, box[1].Lo), new(big.Rat).Quo(box[2].Lo, box[1].Hi),
		new(big.Rat).Quo(box[2].Hi, box[1].Lo), new(big.Rat).Quo(box[2].Hi, box[1].Hi),
	}
	lo, hi := values[0], values[0]
	for _, value := range values[1:] {
		if value.Cmp(lo) < 0 {
			lo = value
		}
		if value.Cmp(hi) > 0 {
			hi = value
		}
	}
	return loftFaceInterval{Lo: lo, Hi: hi}, true
}

// spiralTangentCapEightOutsideBlank is the cone-tangent branch of
// SpiralHeelCapEightOutside. It runs only when the trim record carries a
// cone-tangent end that matches the heel tool cone and the final cap map
// exactly. The cap plane then lies in the closed outside of the tool cone,
// and the tool cone nests strictly outside the unit-slope blank heel cone
// x + r = coneC, so the cap lies strictly outside the blank cone over the
// whole source box (loftface.CapOutsideNestedCone).
func spiralTangentCapEightOutsideBlank(trim *loftFacesTrimRecord, coneC *big.Rat) bool {
	if trim == nil || trim.Tangent == nil || !SpiralTangentRecordCurrent(trim) || len(trim.Maps) != 9 {
		return false
	}
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	return loftface.CapOutsideNestedCone(trim.Maps[8], box, trim.Heel.Apex, trim.Heel.Slope,
		trim.Heel.Far, coneC, seamRatOne())
}

// CertifySpiralHeelRootSeam bounds the first tooth's source root arc at the blank heel cap.
func CertifySpiralHeelRootSeam(ctx context.Context, proof *spiralFirstJoinSource) (*spiralHeelRootSeam, bool, error) {
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
	coneC := seamRatAdd(seamRatF(proof.Blank.Ded.U), seamRatF(proof.Blank.Ded.V))
	if seamRatAdd(seamRatF(proof.Blank.HeelOuter.U), seamRatF(proof.Blank.HeelOuter.V)).Cmp(coneC) != 0 ||
		coneC.Sign() <= 0 || seamRatF(proof.Blank.Ded.U).Cmp(seamRatF(proof.Blank.HeelOuter.U)) >= 0 {
		return nil, false, nil
	}
	mapOK, negative := spiralHeelMapCharge(trim.Maps[7], trim.Maps[8], box, radiusBound, coneC)
	if !mapOK || (!SpiralHeelCapEightOutside(trim.Maps[8], coneC) &&
		!spiralTangentCapEightOutsideBlank(trim, coneC)) {
		return nil, false, nil
	}
	if ok, err := spiralHeelRootCells(ctx, trim, coneC); err != nil || !ok {
		return nil, false, err
	}
	p := spiralHeelBuildPolys(trim.Maps[7], trim.Maps[8], radius, coneC)
	if ok, err := polynomial.BernsteinSignByHalving(ctx, p.deltaX, 1, 0); err != nil || !ok {
		return nil, false, err
	}
	if ok, err := polynomial.BernsteinSignByHalving(ctx, p.at(big.NewRat(19, 20)), -1, 6); err != nil || !ok {
		return nil, false, err
	}
	// The map charge proves these outward rational enlargements cover the
	// actual Sketch arc and both its endpoint crossings.
	coordCharge, slopeCharge := big.NewRat(1, 10_000_000_000), big.NewRat(1, 1_000_000_000)
	var ends [2][3]loftFaceInterval
	var ratios [2]loftFaceInterval
	for end, point := range [...]planePoint{arc.Start, arc.End} {
		endpoint, ok, err := spiralHeelEndpointBox(ctx, trim.Maps[7], trim.Maps[8], point,
			coneC, coordCharge)
		if err != nil || !ok {
			return nil, false, err
		}
		ratio, ok := spiralHeelRatio(endpoint)
		if !ok {
			return nil, false, nil
		}
		ends[end], ratios[end] = endpoint, ratio
	}
	// The seam's X and slope bands start at this tooth's own endpoint boxes and
	// move outward along a ladder until the resultant gate holds at the limit.
	// Each gate proves the resultant has the needed strict sign at its limit, so
	// a limit the geometry cannot support is never believed, and a tighter limit
	// keeps the held-seam proofs that read the band as narrow as the source allows.
	// Strict nonzero at a limit excludes any simultaneous crossing there, even when
	// the solved parameter lies outside the band; the endpoint boxes place the
	// whole continuous one-root branch between the limits.
	idealXLo, ok, err := seamLimit(ends[0][0].Lo, ends[1][0].Lo, -1, seamXLadder,
		func(limit *big.Rat) (bool, error) {
			return polynomial.BernsteinSignByHalving(ctx, p.fixedX(limit), -1, 12)
		})
	if err != nil || !ok {
		return nil, false, err
	}
	idealXHi, ok, err := seamLimit(ends[0][0].Hi, ends[1][0].Hi, 1, seamXLadder,
		func(limit *big.Rat) (bool, error) {
			return polynomial.BernsteinSignByHalving(ctx, p.fixedX(limit), 1, 12)
		})
	if err != nil || !ok {
		return nil, false, err
	}
	slopeLoSign, slopeHiSign, deltaSign := -1, 1, 1
	if negative {
		slopeLoSign, slopeHiSign, deltaSign = 1, -1, -1
	}
	// Q at the unique solution of Z-sY=0 is strictly nonzero. The solution may lie
	// outside the band: nonzero still excludes a simultaneous slope/seam crossing
	// anywhere in it. The endpoint boxes select the side of each limit.
	slopeHolds := func(sign int) func(*big.Rat) (bool, error) {
		return func(limit *big.Rat) (bool, error) {
			delta, q := p.fixedSlope(limit)
			if ok, err := polynomial.BernsteinSignByHalving(ctx, delta, deltaSign, 0); err != nil || !ok {
				return false, err
			}
			return polynomial.BernsteinSignByHalving(ctx, q, sign, 6)
		}
	}
	idealSLo, ok, err := seamLimit(ratios[0].Lo, ratios[1].Lo, -1, seamSlopeLadder, slopeHolds(slopeLoSign))
	if err != nil || !ok {
		return nil, false, err
	}
	idealSHi, ok, err := seamLimit(ratios[0].Hi, ratios[1].Hi, 1, seamSlopeLadder, slopeHolds(slopeHiSign))
	if err != nil || !ok {
		return nil, false, err
	}
	seam := &spiralHeelRootSeam{
		X:        loftFaceInterval{Lo: seamRatSub(idealXLo, coordCharge), Hi: seamRatAdd(idealXHi, coordCharge)},
		Slope:    loftFaceInterval{Lo: seamRatSub(idealSLo, slopeCharge), Hi: seamRatAdd(idealSHi, slopeCharge)},
		ConeC:    coneC,
		Negative: negative,
	}
	if seam.X.Lo.Cmp(seamRatF(proof.Blank.Ded.U)) <= 0 ||
		seam.X.Hi.Cmp(seamRatF(proof.Blank.HeelOuter.U)) >= 0 {
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

// seamXLadder and seamSlopeLadder are the distances, in order, at which a seam
// band limit is tried beyond the endpoint boxes.
var (
	seamXLadder = []*big.Rat{big.NewRat(1, 100_000), big.NewRat(1, 10_000), big.NewRat(3, 10_000),
		big.NewRat(1, 1000), big.NewRat(3, 1000), big.NewRat(1, 100)}
	seamSlopeLadder = []*big.Rat{big.NewRat(1, 100_000), big.NewRat(1, 10_000), big.NewRat(3, 10_000),
		big.NewRat(1, 1000), big.NewRat(1, 100), big.NewRat(1, 10)}
)

// seamLimit returns the nearest limit beyond the outer one of two endpoint
// values, on the side dir (-1 below, +1 above), at which holds is true.
func seamLimit(a, b *big.Rat, dir int, ladder []*big.Rat,
	holds func(limit *big.Rat) (bool, error)) (*big.Rat, bool, error) {
	edge := a
	if dir < 0 && b.Cmp(a) < 0 || dir > 0 && b.Cmp(a) > 0 {
		edge = b
	}
	for _, step := range ladder {
		limit := seamRatAdd(edge, seamRatMul(big.NewRat(int64(dir), 1), step))
		ok, err := holds(limit)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return limit, true, nil
		}
	}
	return nil, false, nil
}

// SpiralHeelTailBox bounds the source seam tail using the blank's exact heel meridian R=C-X.
// On its positive-Y half, a
// positive slope magnitude S gives Y=R/sqrt(1+S²), Z=sign*S*Y. Monotonicity
// in R and S turns the source X/slope intervals into an exact tail box.
func SpiralHeelTailBox(seam *spiralHeelRootSeam, minAbsSlope *big.Rat) ([3]loftFaceInterval, bool) {
	var empty [3]loftFaceInterval
	if seam == nil || minAbsSlope == nil || minAbsSlope.Sign() <= 0 {
		return empty, false
	}
	maxAbsSlope := seam.Slope.Hi
	if seam.Negative {
		maxAbsSlope = seamRatSub(seamRatZero(), seam.Slope.Lo)
	}
	if minAbsSlope.Cmp(maxAbsSlope) >= 0 {
		return empty, false
	}
	rLo, rHi := seamRatSub(seam.ConeC, seam.X.Hi), seamRatSub(seam.ConeC, seam.X.Lo)
	if rLo.Sign() <= 0 {
		return empty, false
	}
	sqrtInterval := func(s *big.Rat) (*big.Rat, *big.Rat, bool) {
		v := seamRatAdd(seamRatOne(), seamRatMul(s, s))
		lo, hi := proofbound.RatSqrtDown(v), proofbound.RatSqrtUp(v)
		if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || lo <= 0 {
			return nil, nil, false
		}
		return seamRatF(lo), seamRatF(hi), true
	}
	sqrtMinLo, sqrtMinHi, ok := sqrtInterval(minAbsSlope)
	if !ok {
		return empty, false
	}
	sqrtMaxLo, sqrtMaxHi, ok := sqrtInterval(maxAbsSlope)
	if !ok {
		return empty, false
	}
	yLo := new(big.Rat).Quo(rLo, sqrtMaxHi)
	yHi := new(big.Rat).Quo(rHi, sqrtMinLo)
	zLo := new(big.Rat).Quo(seamRatMul(rLo, minAbsSlope), sqrtMinHi)
	zHi := new(big.Rat).Quo(seamRatMul(rHi, maxAbsSlope), sqrtMaxLo)
	z := loftFaceInterval{Lo: zLo, Hi: zHi}
	if seam.Negative {
		z = loftFaceInterval{Lo: seamRatSub(seamRatZero(), zHi), Hi: seamRatSub(seamRatZero(), zLo)}
	}
	return [3]loftFaceInterval{seam.X, {Lo: yLo, Hi: yHi}, z}, true
}
