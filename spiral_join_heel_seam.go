package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// spiralHeelRootSeam encloses the true crossing of the source root arc and
// the blank's finite heel outer cone. Endpoints follow ArcSeg Start, End order.
// The intervals include the source arc's recorded-radius uncertainty.
type spiralHeelRootSeam struct {
	X, Slope loftFaceInterval
	Ends     [2][3]loftFaceInterval
	ConeC    *big.Rat
	Negative bool
}

// A rational half-angle interval encloses the short source arc. These limits
// only admit the current bounded first-tooth geometry; other shapes refuse.
var spiralHeelHalfAngle = big.NewRat(127, 5000)

func spiralHeelCapBern(m loftFaceMap, radius, lo, hi *big.Rat) ([]*big.Rat, [3][]*big.Rat) {
	toBern := func(p [3]*big.Rat) []*big.Rat {
		w := lfSub(hi, lo)
		a0 := lfAdd(lfAdd(p[0], lfMul(p[1], lo)), lfMul(p[2], lfMul(lo, lo)))
		a1 := lfMul(w, lfAdd(p[1], lfMul(big.NewRat(2, 1), lfMul(p[2], lo))))
		a2 := lfMul(p[2], lfMul(w, w))
		return []*big.Rat{a0, lfAdd(a0, lfMul(a1, big.NewRat(1, 2))), lfAdd(lfAdd(a0, a1), a2)}
	}
	k := [3]*big.Rat{lfOne(), lfZero(), lfOne()}
	u := [3]*big.Rat{lfSub(lfZero(), radius), lfZero(), radius}
	v := [3]*big.Rat{lfZero(), lfMul(big.NewRat(2, 1), radius), lfZero()}
	var gp [3]*big.Rat
	var hp [3][3]*big.Rat
	for j := range gp {
		gp[j] = lfAdd(lfAdd(lfMul(m.G0, k[j]), lfMul(m.GU, u[j])), lfMul(m.GV, v[j]))
		for axis := range hp {
			hp[axis][j] = lfAdd(lfAdd(lfMul(m.Constant[axis], gp[j]), lfMul(m.Q0[axis], k[j])),
				lfAdd(lfMul(m.QU[axis], u[j]), lfMul(m.QV[axis], v[j])))
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
	radius := lfF(held)
	startR2 := lfAdd(lfMul(lfF(arc.Start.U), lfF(arc.Start.U)),
		lfMul(lfF(arc.Start.V), lfF(arc.Start.V)))
	endR2 := lfAdd(lfMul(lfF(arc.End.U), lfF(arc.End.U)),
		lfMul(lfF(arc.End.V), lfF(arc.End.V)))
	// The denoted End uses Start's radius and End's ray. Its displacement
	// from the recorded End is |rStart²-rEnd²|/(rStart+rEnd).
	endGap := new(big.Rat).Quo(lfAbs(lfSub(startR2, endR2)),
		lfAdd(lfAbs(lfF(arc.Start.U)), lfAbs(lfF(arc.End.U))))
	errorBound := lfAdd(lfF(bound), endGap)
	rLo, rHi := lfSub(radius, errorBound), lfAdd(radius, errorBound)
	if rLo.Sign() <= 0 {
		return nil, nil, empty, false
	}
	for _, end := range [...]planePoint{arc.Start, arc.End} {
		// Each endpoint ray is inside the rational half-angle interval.
		// With U<0, its exact radius is at least -U.
		if lfAbs(lfF(end.V)).Cmp(lfMul(spiralHeelHalfAngle,
			lfMul(big.NewRat(-2, 1), lfF(end.U)))) >= 0 {
			return nil, nil, empty, false
		}
	}
	h2 := lfMul(spiralHeelHalfAngle, spiralHeelHalfAngle)
	uUpper := lfSub(lfZero(), lfMul(rLo, new(big.Rat).Quo(lfSub(lfOne(), h2), lfAdd(lfOne(), h2))))
	vUpper := new(big.Rat).Quo(lfMul(big.NewRat(2, 1), lfMul(rHi, spiralHeelHalfAngle)),
		lfAdd(lfOne(), h2))
	box := [4]*big.Rat{big.NewRat(-514, 25), big.NewRat(-2053, 100),
		big.NewRat(-21, 20), big.NewRat(21, 20)}
	if lfSub(lfZero(), rHi).Cmp(box[0]) <= 0 || uUpper.Cmp(box[1]) >= 0 ||
		lfSub(lfZero(), vUpper).Cmp(box[2]) <= 0 || vUpper.Cmp(box[3]) >= 0 {
		return nil, nil, empty, false
	}
	return radius, errorBound, box, true
}

func spiralHeelCapEightOutside(capMap loftFaceMap, coneC *big.Rat) bool {
	n0 := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.G0), capMap.Q0)
	nu := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.GU), capMap.QU)
	nv := loftface.VAdd(loftface.VScale(capMap.Constant, capMap.GV), capMap.QV)
	coneVec := func(n loftFaceRatVec, g *big.Rat) loftFaceRatVec {
		return loftFaceRatVec{lfSub(lfMul(coneC, g), n[0]), n[1], n[2]}
	}
	a, u, v := coneVec(n0, capMap.G0), coneVec(nu, capMap.GU), coneVec(nv, capMap.GV)
	dot := func(x, y loftFaceRatVec) *big.Rat {
		return lfSub(lfAdd(lfMul(x[1], y[1]), lfMul(x[2], y[2])), lfMul(x[0], y[0]))
	}
	// Q=Y²+Z²-(C-X)² has exactly two positive quadratic terms.
	return dot(a, a).Sign() == 0 && dot(a, u).Sign() == 0 &&
		dot(a, v).Sign() == 0 && dot(u, v).Sign() == 0 &&
		dot(u, u).Sign() > 0 && dot(v, v).Sign() > 0
}

func spiralHeelIntervalAdd(a, b loftFaceInterval) loftFaceInterval {
	return loftFaceInterval{Lo: lfAdd(a.Lo, b.Lo), Hi: lfAdd(a.Hi, b.Hi)}
}

func spiralHeelIntervalMul(a, b loftFaceInterval) loftFaceInterval {
	values := [4]*big.Rat{lfMul(a.Lo, b.Lo), lfMul(a.Lo, b.Hi),
		lfMul(a.Hi, b.Lo), lfMul(a.Hi, b.Hi)}
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

// A mapped radius error below 10^-12 mm changes |Q| by less than
// (2*(15+5+16)+3*10^-12)*10^-12 < 7.3*10^-11 mm².
// With Q_t>25 this moves the root t by <3*10^-12. Each map span is <4 mm,
// so each root coordinate moves by <1.3*10^-11 mm. Y>13 and |Z|<5 then
// bound the Z/Y change below 10^-9. The published charges are 10^-10 mm
// per coordinate and 10^-9 in slope. Every premise is gated below.
func spiralHeelMapCharge(first, second loftFaceMap, box [4]*big.Rat,
	radiusBound, coneC *big.Rat) (bool, bool) {
	if loftface.PositiveDenominator(first, box) != nil || loftface.PositiveDenominator(second, box) != nil {
		return false, false
	}
	stretch := math.Max(first.Stretch, second.Stretch)
	if math.IsNaN(stretch) || math.IsInf(stretch, 0) ||
		lfMul(lfF(stretch), radiusBound).Cmp(big.NewRat(1, 1_000_000_000_000)) >= 0 {
		return false, false
	}
	corners := loftface.BoxCorners(box)
	axis := [3]loftFaceRatVec{{lfOne(), lfZero(), lfZero()},
		{lfZero(), lfOne(), lfZero()}, {lfZero(), lfZero(), lfOne()}}
	var a, b, delta [3]loftFaceInterval
	for i := range a {
		a[i], b[i] = loftface.ScalarRange(first, axis[i], corners), loftface.ScalarRange(second, axis[i], corners)
		delta[i] = loftface.IntervalSub(b[i], a[i])
	}
	for _, cap := range [][3]loftFaceInterval{a, b} {
		if cap[0].Lo.Sign() <= 0 || cap[0].Hi.Cmp(big.NewRat(17, 1)) >= 0 ||
			cap[1].Lo.Cmp(big.NewRat(13, 1)) <= 0 || cap[1].Hi.Cmp(big.NewRat(15, 1)) >= 0 ||
			cap[2].Lo.Cmp(big.NewRat(-5, 1)) <= 0 || cap[2].Hi.Cmp(big.NewRat(5, 1)) >= 0 ||
			lfSub(coneC, cap[0].Lo).Cmp(big.NewRat(16, 1)) >= 0 {
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
	for _, cap := range [][3]loftFaceInterval{a, b} {
		radial := loftFaceInterval{Lo: lfSub(coneC, cap[0].Hi), Hi: lfSub(coneC, cap[0].Lo)}
		derivative := spiralHeelIntervalAdd(spiralHeelIntervalAdd(
			spiralHeelIntervalMul(cap[1], delta[1]), spiralHeelIntervalMul(cap[2], delta[2])),
			spiralHeelIntervalMul(radial, delta[0]))
		if lfMul(big.NewRat(2, 1), derivative.Lo).Cmp(big.NewRat(25, 1)) <= 0 {
			return false, false
		}
	}
	negative := a[2].Hi.Sign() < 0 && b[2].Hi.Sign() < 0
	positive := a[2].Lo.Sign() > 0 && b[2].Lo.Sign() > 0
	return negative || positive, negative
}

func spiralHeelBuildPolys(first, second loftFaceMap, radius, coneC *big.Rat) spiralHeelPolys {
	lo := lfSub(lfZero(), spiralHeelHalfAngle)
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
	radius := lfSub(p.coneC, x)
	return polynomial.BernsteinSub(polynomial.BernsteinAdd(polynomial.BernsteinMul(y, y), polynomial.BernsteinMul(z, z)),
		polynomial.BernsteinScale(polynomial.BernsteinMul(den, den), lfMul(radius, radius)))
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
		n[axis] = polynomial.BernsteinAdd(polynomial.BernsteinScale(p.a[axis], lfSub(lfOne(), t)), polynomial.BernsteinScale(p.b[axis], t))
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
		a := trim.Source.sourceStation[j]
		z := trim.Source.sourceStation[(j+1)%len(trim.StationGroup)]
		if !spiralCellSide(trim.Maps[7], trim.Maps[8], a, z, lfF(departure),
			big.NewRat(-1, 1), coneC, false, lfZero(), big.NewRat(19, 20), 8) {
			return false, nil
		}
		count++
	}
	return count == 15, nil
}

func spiralHeelEndpointBox(ctx context.Context, first, second loftFaceMap,
	point planePoint, coneC, charge *big.Rat) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	a := first.At(lfF(point.U), lfF(point.V))
	b := second.At(lfF(point.U), lfF(point.V))
	delta := loftface.VSub(b, a)
	at := func(t *big.Rat) loftFaceRatVec { return loftface.VAdd(a, loftface.VScale(delta, t)) }
	q := func(t *big.Rat) *big.Rat {
		p := at(t)
		r := lfSub(coneC, p[0])
		return lfSub(lfAdd(lfMul(p[1], p[1]), lfMul(p[2], p[2])), lfMul(r, r))
	}
	lo, hi := big.NewRat(19, 20), lfOne()
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
// spiralHeelCapEightOutside. It runs only when the trim record carries a
// cone-tangent end that matches the heel tool cone and the final cap map
// exactly. The cap plane then lies in the closed outside of the tool cone,
// and the tool cone nests strictly outside the unit-slope blank heel cone
// x + r = coneC, so the cap lies strictly outside the blank cone over the
// whole source box (loftface.CapOutsideNestedCone).
func spiralTangentCapEightOutsideBlank(trim *loftFacesTrimRecord, coneC *big.Rat) bool {
	if trim == nil || trim.Tangent == nil || !spiralTangentRecordCurrent(trim) || len(trim.Maps) != 9 {
		return false
	}
	box, err := lfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	return loftface.CapOutsideNestedCone(trim.Maps[8], box, trim.Heel.apex, trim.Heel.slope,
		trim.Heel.far, coneC, lfOne())
}

func certifySpiralHeelRootSeam(ctx context.Context, proof *spiralFirstJoinSource) (*spiralHeelRootSeam, bool, error) {
	if ctx == nil || proof == nil || proof.Tooth == nil || proof.Blank == nil {
		return nil, false, nil
	}
	trim := proof.Tooth
	arc, ok := trim.Source.profile.Outer.Segments[4].(arcSeg)
	if !ok || len(trim.Maps) != 9 || len(trim.Bounds) != 9 {
		return nil, false, nil
	}
	radius, radiusBound, box, ok := spiralHeelSourceBox(arc)
	if !ok {
		return nil, false, nil
	}
	coneC := lfAdd(lfF(proof.Blank.Ded.U), lfF(proof.Blank.Ded.V))
	if lfAdd(lfF(proof.Blank.HeelOuter.U), lfF(proof.Blank.HeelOuter.V)).Cmp(coneC) != 0 ||
		coneC.Sign() <= 0 || lfF(proof.Blank.Ded.U).Cmp(lfF(proof.Blank.HeelOuter.U)) >= 0 {
		return nil, false, nil
	}
	mapOK, negative := spiralHeelMapCharge(trim.Maps[7], trim.Maps[8], box, radiusBound, coneC)
	if !mapOK || (!spiralHeelCapEightOutside(trim.Maps[8], coneC) &&
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
	idealXLo, idealXHi := big.NewRat(8231, 500), big.NewRat(1646399, 100000)
	// The fixed-X resultant is nonzero at both limits. Its strict sign
	// excludes any simultaneous Q=0 and X=limit, even when the solved t
	// lies outside this band. The endpoint boxes below place the whole
	// continuous one-root branch between the limits.
	for _, gate := range [...]struct {
		value *big.Rat
		sign  int
	}{{idealXLo, -1}, {idealXHi, 1}} {
		if ok, err := polynomial.BernsteinSignByHalving(ctx, p.fixedX(gate.value), gate.sign, 12); err != nil || !ok {
			return nil, false, err
		}
	}
	idealSLo, idealSHi := big.NewRat(137, 1000), big.NewRat(1441, 5000)
	slopeLoSign, slopeHiSign := -1, 1
	if negative {
		idealSLo, idealSHi = lfSub(lfZero(), idealSHi), lfSub(lfZero(), idealSLo)
		slopeLoSign, slopeHiSign = 1, -1
	}
	for _, gate := range [...]struct {
		value *big.Rat
		sign  int
	}{{idealSLo, slopeLoSign}, {idealSHi, slopeHiSign}} {
		// Q at the unique solution of Z-sY=0 is strictly nonzero. The
		// solution may lie outside this band: nonzero still excludes a
		// simultaneous slope/seam crossing anywhere in the band. The
		// endpoint boxes below select the side of each limit.
		delta, q := p.fixedSlope(gate.value)
		deltaSign := 1
		if negative {
			deltaSign = -1
		}
		if ok, err := polynomial.BernsteinSignByHalving(ctx, delta, deltaSign, 0); err != nil || !ok {
			return nil, false, err
		}
		if ok, err := polynomial.BernsteinSignByHalving(ctx, q, gate.sign, 6); err != nil || !ok {
			return nil, false, err
		}
	}
	// The map charge proves these outward rational enlargements cover the
	// actual Sketch arc and both its endpoint crossings.
	coordCharge, slopeCharge := big.NewRat(1, 10_000_000_000), big.NewRat(1, 1_000_000_000)
	seam := &spiralHeelRootSeam{
		X:        loftFaceInterval{Lo: lfSub(idealXLo, coordCharge), Hi: lfAdd(idealXHi, coordCharge)},
		Slope:    loftFaceInterval{Lo: lfSub(idealSLo, slopeCharge), Hi: lfAdd(idealSHi, slopeCharge)},
		ConeC:    coneC,
		Negative: negative,
	}
	if seam.X.Lo.Cmp(lfF(proof.Blank.Ded.U)) <= 0 ||
		seam.X.Hi.Cmp(lfF(proof.Blank.HeelOuter.U)) >= 0 {
		return nil, false, nil
	}
	for end, point := range [...]planePoint{arc.Start, arc.End} {
		endpoint, ok, err := spiralHeelEndpointBox(ctx, trim.Maps[7], trim.Maps[8], point,
			coneC, coordCharge)
		if err != nil || !ok {
			return nil, false, err
		}
		ratio, ok := spiralHeelRatio(endpoint)
		if !ok || endpoint[0].Lo.Cmp(seam.X.Lo) <= 0 || endpoint[0].Hi.Cmp(seam.X.Hi) >= 0 ||
			ratio.Lo.Cmp(seam.Slope.Lo) <= 0 || ratio.Hi.Cmp(seam.Slope.Hi) >= 0 {
			return nil, false, nil
		}
		seam.Ends[end] = endpoint
	}
	return seam, true, nil
}

// The blank's exact heel meridian has R=C-X. On its positive-Y half, a
// positive slope magnitude S gives Y=R/sqrt(1+S²), Z=sign*S*Y. Monotonicity
// in R and S turns the source X/slope intervals into an exact tail box.
func spiralHeelTailBox(seam *spiralHeelRootSeam, minAbsSlope *big.Rat) ([3]loftFaceInterval, bool) {
	var empty [3]loftFaceInterval
	if seam == nil || minAbsSlope == nil || minAbsSlope.Sign() <= 0 {
		return empty, false
	}
	maxAbsSlope := seam.Slope.Hi
	if seam.Negative {
		maxAbsSlope = lfSub(lfZero(), seam.Slope.Lo)
	}
	if minAbsSlope.Cmp(maxAbsSlope) >= 0 {
		return empty, false
	}
	rLo, rHi := lfSub(seam.ConeC, seam.X.Hi), lfSub(seam.ConeC, seam.X.Lo)
	if rLo.Sign() <= 0 {
		return empty, false
	}
	sqrtInterval := func(s *big.Rat) (*big.Rat, *big.Rat, bool) {
		v := lfAdd(lfOne(), lfMul(s, s))
		lo, hi := proofbound.RatSqrtDown(v), proofbound.RatSqrtUp(v)
		if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || lo <= 0 {
			return nil, nil, false
		}
		return lfF(lo), lfF(hi), true
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
	zLo := new(big.Rat).Quo(lfMul(rLo, minAbsSlope), sqrtMinHi)
	zHi := new(big.Rat).Quo(lfMul(rHi, maxAbsSlope), sqrtMaxLo)
	z := loftFaceInterval{Lo: zLo, Hi: zHi}
	if seam.Negative {
		z = loftFaceInterval{Lo: lfSub(lfZero(), zHi), Hi: lfSub(lfZero(), zLo)}
	}
	return [3]loftFaceInterval{seam.X, {Lo: yLo, Hi: yHi}, z}, true
}
