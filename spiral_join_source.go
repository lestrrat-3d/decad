package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// The interval contains the single flank/root crossing on one source cubic
// at one LoftFaces cap. Its endpoints are exact rationals with opposite
// implicit signs; no rounded point is treated as the true crossing.
type spiralRootCrossing struct {
	span   int
	lo, hi *big.Rat
}

// A bounded box around one true flank/root/tool triple junction. The true
// point has X=the corresponding exact toeX or heelX; s and t remain enclosed
// separately because their algebraic roots need not be rational.
type spiralJunction struct {
	band     int
	sLo, sHi *big.Rat
	tLo, tHi *big.Rat
	box      [3]loftFaceInterval
}

// A first curved tooth join may use this only while both input bodies remain
// the original unplaced blank and cone-trimmed LoftFaces tooth. The matcher
// below rechecks the live Sketch source and the exact finite blank meridian.
type spiralFirstJoinSource struct {
	blank      *bevelBlankSource
	tooth      *loftFacesTrimRecord
	rootM      *big.Rat
	rootB      *big.Rat
	toeX       *big.Rat
	heelX      *big.Rat
	toeOffset  *big.Rat
	heelOffset *big.Rat
	toeCapGap  *big.Rat
	heelCapGap *big.Rat
	flank      [2]freeform.BezierSpan // root-containing cubic of groups 0 and 2
	crossings  [2][9]spiralRootCrossing
	junctions  [2][2]spiralJunction // flank 0/2, then toe/heel
	heelSeam   *spiralHeelRootSeam
}

// certifySpiralFirstJoin proves that each of the two fitted flanks crosses
// the blank's recorded root meridian once and transversely through all eight
// continuous LoftFaces bands. It also proves that both root branches move
// strictly from toe to heel. The finite toe/heel transitions are retained as
// signed exact offsets for the cap-strip proof; they are not collapsed to a
// falsely shared ring.
func certifySpiralFirstJoin(ctx context.Context, blank, tooth *Body) (*spiralFirstJoinSource, bool, error) {
	if ctx == nil || blank == nil || tooth == nil {
		return nil, false, nil
	}
	section, ok := exactSpiralBlank(blank)
	if !ok {
		return nil, false, nil
	}
	p, ok := tooth.payload.(facetedPayload)
	if !ok || p.loftFacesTrim == nil || p.xform != r3.Identity() {
		return nil, false, nil
	}
	trim := p.loftFacesTrim
	if !spiralTrimSourceCurrent(trim) || !spiralTrimCapGroupsCurrent(p, trim) {
		return nil, false, nil
	}
	m, b := section.meridianEquation()
	if !spiralSourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	if !spiralOtherSides(ctx, trim, m, b) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		return nil, false, nil
	}
	toeX, ok := spiralRootToolX(m, b, trim.toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := spiralRootToolX(m, b, trim.heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		lfAdd(lfMul(m, toeX), b).Sign() <= 0 ||
		lfAdd(lfMul(m, heelX), b).Sign() <= 0 {
		return nil, false, nil
	}
	toeGap, ok := spiralToolOuterGap(trim.toe, section.toeRoot, section.toeOuter, 1)
	if !ok {
		return nil, false, nil
	}
	heelGap, ok := spiralToolOuterGap(trim.heel, section.ded, section.heelOuter, -1)
	if !ok {
		return nil, false, nil
	}
	proof := &spiralFirstJoinSource{blank: section, tooth: trim, rootM: m, rootB: b,
		toeX: toeX, heelX: heelX,
		toeOffset:  lfSub(toeX, lfF(section.toeRoot.U)),
		heelOffset: lfSub(heelX, lfF(section.ded.U)),
		toeCapGap:  toeGap, heelCapGap: heelGap}
	// The real finite root lies between the tool-clipped ends, with a small
	// overhang at each cap. The later cap proof charges both exact offsets.
	if proof.toeOffset.Sign() >= 0 || proof.heelOffset.Sign() <= 0 {
		return nil, false, nil
	}
	if !spiralOriginalCapsAvoidToolRootCircles(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	for side, group := range [...]int{0, 2} {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		fit := trim.source.profile.Outer.Segments[group].(fitSplineSeg)
		work := freeform.NewFreeformWork()
		work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		spans, err := splinebezier.FitSplineBezierSpans(fit, work)
		if err != nil || len(spans) != 14 {
			return nil, false, nil
		}
		proof.flank[side] = spans[0]
		for band := 0; band < 8; band++ {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			for spanIndex, span := range spans {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
				if spanIndex == 0 {
					if !spiralCrossingSigns(q) ||
						!spiralCapXDecreases(trim.maps[band], span) ||
						!spiralCapXDecreases(trim.maps[band+1], span) ||
						!spiralAllPositive(spiralRootXOrder(trim.maps[band], trim.maps[band+1], span, q)) {
						return nil, false, nil
					}
					if band == 0 {
						proof.crossings[side][0] = spiralIsolateRoot(q[0])
					}
					proof.crossings[side][band+1] = spiralIsolateRoot(q[2])
				} else if !spiralAllPositive(q) {
					return nil, false, nil
				}
			}
		}
		toeRange := spiralCapXRange(trim.maps[0], spans[0], proof.crossings[side][0])
		heelRange := spiralCapXRange(trim.maps[8], spans[0], proof.crossings[side][8])
		if toeRange.hi.Cmp(toeX) >= 0 || heelRange.lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
		for end, targetX := range []*big.Rat{toeX, heelX} {
			junction, ok := spiralIsolateJunction(ctx, trim.maps, spans[0], m, b,
				proof.crossings[side], targetX)
			if !ok {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				return nil, false, nil
			}
			proof.junctions[side][end] = junction
		}
	}
	// The embedded source solid's boundary meets each tool/root circle only
	// at these two flank junctions. All other contour groups stay on one side.
	// The whole original cap 0 is below toeX. The whole cap-8 plane misses
	// its heel tool/root circle. Thus neither end-cap interior can create an
	// extra contact, even though cap 8 spans the heelX coordinate.
	// Q_s and J make both junctions transverse. Circle membership therefore
	// changes at each junction. The long connecting arc crosses Y<0, where
	// the entire source is absent, so the short arc is inside the source.
	// The ordered exact Cut and Intersect define the operative tool-cone cap
	// as this source solid's trace on that cone. The retained cap group IDs
	// identify held facets only; they do not establish the true trace.
	seam, ok, err := certifySpiralHeelRootSeam(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	proof.heelSeam = seam
	return proof, true, nil
}

// The toe cap lies wholly before the toe tool/root circle in X. The heel
// cap may span heelX, so its full supporting plane must miss the entire
// heel tool/root circle. For A+B*cos(theta)+C*sin(theta), A²>B²+C² is an
// exact rational separation proof for every angle, including cap interiors.
func spiralOriginalCapsAvoidToolRootCircles(trim *loftFacesTrimRecord,
	m, b, toeX, heelX *big.Rat) bool {
	box, err := lfSourceBox(trim.source)
	if err != nil || len(trim.maps) != 9 {
		return false
	}
	for _, end := range [...]int{0, 8} {
		if lfPositiveDenominator(trim.maps[end], box) != nil {
			return false
		}
	}
	corners := lfBoxCorners(box)
	xAxis := loftFaceRatVec{lfOne(), lfZero(), lfZero()}
	if lfScalarRange(trim.maps[0], xAxis, corners).hi.Cmp(toeX) >= 0 {
		return false
	}
	return spiralCapPlaneAvoidsRootCircle(trim.maps[8], corners, m, b, heelX)
}

func spiralCapPlaneAvoidsRootCircle(cap loftFaceMap, corners [4][2]*big.Rat,
	m, b, circleX *big.Rat) bool {
	p0 := cap.at(corners[0][0], corners[0][1])
	p1 := cap.at(corners[1][0], corners[1][1])
	p2 := cap.at(corners[2][0], corners[2][1])
	normal := lfVCross(lfVSub(p1, p0), lfVSub(p2, p0))
	if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
		return false
	}
	d := lfVDot(normal, p0)
	if lfVDot(normal, cap.at(corners[3][0], corners[3][1])).Cmp(d) != 0 {
		return false
	}
	radius := lfAdd(lfMul(m, circleX), b)
	if radius.Sign() <= 0 {
		return false
	}
	a := lfSub(lfMul(normal[0], circleX), d)
	y := lfMul(normal[1], radius)
	z := lfMul(normal[2], radius)
	return lfMul(a, a).Cmp(lfAdd(lfMul(y, y), lfMul(z, z))) > 0
}

func spiralTrimSourceCurrent(trim *loftFacesTrimRecord) bool {
	if trim == nil || trim.source == nil || !pointSourceCurrent(trim.source) ||
		trim.toe == nil || trim.heel == nil || len(trim.maps) != 9 || len(trim.bounds) != 9 ||
		!orderedLoftFacesConeLimits(trim.toe, trim.heel, trim.base) ||
		!trim.base.loftFacesPrimitive || trim.base.loftFacesCap == nil ||
		trim.base.loftFacesCap.source != trim.source ||
		len(trim.base.loftFacesCap.maps) != 9 || len(trim.base.loftFacesCap.bounds) != 9 ||
		len(trim.stationGroup) != len(trim.source.sourceStation) ||
		len(trim.base.loftFacesCap.stationGroup) != len(trim.source.sourceStation) ||
		len(trim.source.profile.Outer.Segments) != 6 || len(trim.source.source.entities) != 6 ||
		len(trim.source.profile.Holes) != 0 {
		return false
	}
	for _, entry := range trim.source.source.profile.Outer {
		if entry.Partial {
			return false
		}
	}
	segments := trim.source.profile.Outer.Segments
	entities := trim.source.source.entities
	for _, group := range [...]int{0, 2} {
		if _, ok := segments[group].(fitSplineSeg); !ok {
			return false
		}
		if _, ok := entities[group].(*sketch.FitSpline); !ok {
			return false
		}
	}
	for _, group := range [...]int{1, 4} {
		if _, ok := segments[group].(arcSeg); !ok {
			return false
		}
		if _, ok := entities[group].(*sketch.Arc); !ok {
			return false
		}
	}
	for _, group := range [...]int{3, 5} {
		if _, ok := segments[group].(lineSeg); !ok {
			return false
		}
		if _, ok := entities[group].(*sketch.Line); !ok {
			return false
		}
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	for _, mapped := range trim.maps {
		if lfPositiveDenominator(mapped, box) != nil {
			return false
		}
	}
	return true
}

func spiralTrimCapGroupsCurrent(p facetedPayload, trim *loftFacesTrimRecord) bool {
	if trim.toeCapGroup < 0 || trim.heelCapGroup < 0 ||
		trim.toeCapGroup == trim.heelCapGroup ||
		trim.toeCapGroup >= len(p.groups) || trim.heelCapGroup >= len(p.groups) ||
		len(p.faceOf) != len(p.src) {
		return false
	}
	toeSeen, heelSeen := false, false
	for _, group := range p.src {
		toeSeen = toeSeen || group == trim.toeCapGroup
		heelSeen = heelSeen || group == trim.heelCapGroup
	}
	return toeSeen && heelSeen
}

// The whole ruled source stays in positive Y and above zero signed radius
// on the blank's root meridian. The opposite root-circle arc passes negative
// Y, while a root of the squared implicit with negative signed radius would
// not be a physical cone crossing.
func spiralSourcePhysicalSector(trim *loftFacesTrimRecord, m, b *big.Rat) bool {
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	corners := lfBoxCorners(box)
	y := loftFaceRatVec{lfZero(), lfOne(), lfZero()}
	x := loftFaceRatVec{lfOne(), lfZero(), lfZero()}
	for _, mapped := range trim.maps {
		if lfScalarRange(mapped, y, corners).lo.Sign() <= 0 {
			return false
		}
		xRange := lfScalarRange(mapped, x, corners)
		r0, r1 := lfAdd(lfMul(m, xRange.lo), b), lfAdd(lfMul(m, xRange.hi), b)
		if r0.Sign() <= 0 || r1.Sign() <= 0 {
			return false
		}
	}
	return true
}

// The four non-flank source edges cannot create another root crossing.
// Each authenticated source contour cell lies within its cap's certified
// source and projective departure from the chord joining exact mapped
// stations. Band interpolation is convex in the two caps, so the entire
// ruled cell lies in the four-corner box enlarged by that departure.
func spiralOtherSides(ctx context.Context, trim *loftFacesTrimRecord, m, b *big.Rat) bool {
	source := trim.source
	n := len(source.sourceStation)
	if n == 0 || len(trim.stationGroup) != n {
		return false
	}
	var seen [6]int
	for band := 0; band < 8; band++ {
		lo, hi := trim.bounds[band], trim.bounds[band+1]
		if len(lo.sourceDeparture) != n || len(hi.sourceDeparture) != n ||
			len(lo.projectiveDeparture) != n || len(hi.projectiveDeparture) != n {
			return false
		}
		for j, station := range source.sourceStation {
			if j%64 == 0 && ctx.Err() != nil {
				return false
			}
			group := trim.stationGroup[j]
			if group < 0 || group >= len(seen) {
				return false
			}
			if group == 0 || group == 2 {
				continue
			}
			seen[group]++
			departure := max(
				proofbound.AbsSumUpper(lo.sourceDeparture[j], lo.projectiveDeparture[j]),
				proofbound.AbsSumUpper(hi.sourceDeparture[j], hi.projectiveDeparture[j]))
			if proofbound.IsNonFinite(departure) || departure < 0 {
				return false
			}
			next := source.sourceStation[(j+1)%n]
			if !spiralCellSide(trim.maps[band], trim.maps[band+1], station, next,
				lfF(departure), m, b, group == 1, lfZero(), lfOne(), 8) {
				return false
			}
		}
	}
	return seen[1] > 0 && seen[3] > 0 && seen[4] > 0 && seen[5] > 0
}

func spiralCellSide(first, second loftFaceMap, a, z Point2, departure, m, b *big.Rat,
	outside bool, tLo, tHi *big.Rat, depth int) bool {
	firstA := first.at(lfF(a.U), lfF(a.V))
	firstZ := first.at(lfF(z.U), lfF(z.V))
	secondA := second.at(lfF(a.U), lfF(a.V))
	secondZ := second.at(lfF(z.U), lfF(z.V))
	blend := func(x, y loftFaceRatVec, t *big.Rat) loftFaceRatVec {
		return lfVAdd(lfVScale(x, lfSub(lfOne(), t)), lfVScale(y, t))
	}
	corners := [4]loftFaceRatVec{
		blend(firstA, secondA, tLo), blend(firstZ, secondZ, tLo),
		blend(firstA, secondA, tHi), blend(firstZ, secondZ, tHi),
	}
	var box [3]loftFaceInterval
	for axis := range box {
		minimum, maximum := corners[0][axis], corners[0][axis]
		for _, corner := range corners[1:] {
			if corner[axis].Cmp(minimum) < 0 {
				minimum = corner[axis]
			}
			if corner[axis].Cmp(maximum) > 0 {
				maximum = corner[axis]
			}
		}
		box[axis] = loftFaceInterval{lo: lfSub(minimum, departure), hi: lfAdd(maximum, departure)}
	}
	r0, r1 := lfAdd(lfMul(m, box[0].lo), b), lfAdd(lfMul(m, box[0].hi), b)
	if r0.Cmp(r1) > 0 {
		r0, r1 = r1, r0
	}
	radius := loftFaceInterval{lo: r0, hi: r1}
	minimumF := lfSub(lfAdd(loftIntervalSquareMin(box[1]), loftIntervalSquareMin(box[2])),
		loftIntervalSquareMax(radius))
	maximumF := lfSub(lfAdd(loftIntervalSquareMax(box[1]), loftIntervalSquareMax(box[2])),
		loftIntervalSquareMin(radius))
	if outside && minimumF.Sign() > 0 || !outside && maximumF.Sign() < 0 {
		return true
	}
	if depth == 0 {
		return false
	}
	mid := lfMul(lfAdd(tLo, tHi), big.NewRat(1, 2))
	return spiralCellSide(first, second, a, z, departure, m, b, outside, tLo, mid, depth-1) &&
		spiralCellSide(first, second, a, z, departure, m, b, outside, mid, tHi, depth-1)
}

// On the blank root, r=m*x+b. The tool cone generator is x+k*r=apex,
// so their intersection has an exact rational axial coordinate.
func spiralRootToolX(m, b *big.Rat, cone *pointConeLimit) (*big.Rat, bool) {
	if cone == nil || cone.apex == nil || cone.slope == nil || cone.slope.Sign() <= 0 {
		return nil, false
	}
	denominator := lfAdd(lfOne(), lfMul(cone.slope, m))
	if denominator.Sign() <= 0 {
		return nil, false
	}
	return lfDiv(lfSub(cone.apex, lfMul(cone.slope, b)), denominator), true
}

// The exact straight blank cap meridian and tool cone differ affinely.
// Endpoint signs and the largest absolute endpoint residual bound their
// separation over the entire finite blank cap generator.
func spiralToolOuterGap(cone *pointConeLimit, first, last Point2, sign int) (*big.Rat, bool) {
	residual := func(point Point2) *big.Rat {
		return lfSub(lfAdd(lfF(point.U), lfMul(cone.slope, lfF(point.V))), cone.apex)
	}
	a, z := residual(first), residual(last)
	if a.Sign() != sign || z.Sign() != sign {
		return nil, false
	}
	a, z = lfAbs(a), lfAbs(z)
	if a.Cmp(z) < 0 {
		return z, true
	}
	return a, true
}

func spiralBinom(n, k int) int64 {
	if k < 0 || k > n {
		return 0
	}
	if k > n-k {
		k = n - k
	}
	out := int64(1)
	for i := 1; i <= k; i++ {
		out = out * int64(n-k+i) / int64(i)
	}
	return out
}

func spiralAdd(a, b []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = lfAdd(a[i], b[i])
	}
	return out
}

func spiralSub(a, b []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = lfSub(a[i], b[i])
	}
	return out
}

func spiralScale(a []*big.Rat, k *big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a))
	for i := range a {
		out[i] = lfMul(a[i], k)
	}
	return out
}

// Products of Bernstein polynomials retain exact rational coefficients.
// The largest degree used here is 23, whose binomial weights fit int64.
func spiralMul(a, b []*big.Rat) []*big.Rat {
	n, m := len(a)-1, len(b)-1
	out := make([]*big.Rat, n+m+1)
	for k := range out {
		out[k] = lfZero()
		for i := max(0, k-m); i <= min(n, k); i++ {
			j := k - i
			weight := big.NewRat(spiralBinom(n, i)*spiralBinom(m, j), spiralBinom(n+m, k))
			out[k] = lfAdd(out[k], lfMul(weight, lfMul(a[i], b[j])))
		}
	}
	return out
}

func spiralDeriv(a []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(a)-1)
	for i := range out {
		out[i] = lfMul(big.NewRat(int64(len(a)-1), 1), lfSub(a[i+1], a[i]))
	}
	return out
}

// These cap-X coefficients are negative on the admitted first spline span.
// Thus endpoint evaluations enclose X for a rational crossing interval.
func spiralCapXDecreases(m loftFaceMap, span freeform.BezierSpan) bool {
	g, h := spiralCapBern(m, span)
	numerator := spiralSub(spiralMul(spiralDeriv(h[0]), g),
		spiralMul(h[0], spiralDeriv(g)))
	for _, coefficient := range numerator {
		if coefficient.Sign() >= 0 {
			return false
		}
	}
	return true
}

func spiralBezierPoint(span freeform.BezierSpan, s *big.Rat) [2]*big.Rat {
	var u, v [4]*big.Rat
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	for width := len(span); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			u[i] = lfAdd(lfMul(lfSub(lfOne(), s), u[i]), lfMul(s, u[i+1]))
			v[i] = lfAdd(lfMul(lfSub(lfOne(), s), v[i]), lfMul(s, v[i+1]))
		}
	}
	return [2]*big.Rat{u[0], v[0]}
}

func spiralCapXRange(m loftFaceMap, span freeform.BezierSpan,
	crossing spiralRootCrossing) loftFaceInterval {
	left, right := spiralBezierPoint(span, crossing.lo), spiralBezierPoint(span, crossing.hi)
	return loftFaceInterval{
		lo: m.at(right[0], right[1])[0],
		hi: m.at(left[0], left[1])[0],
	}
}

func spiralCapBern(m loftFaceMap, span freeform.BezierSpan) ([]*big.Rat, [3][]*big.Rat) {
	g := make([]*big.Rat, len(span))
	var h [3][]*big.Rat
	for axis := range h {
		h[axis] = make([]*big.Rat, len(span))
	}
	for i, p := range span {
		g[i] = lfAdd(lfAdd(m.g0, lfMul(m.gu, p.U)), lfMul(m.gv, p.V))
		for axis := range h {
			h[axis][i] = lfAdd(lfMul(m.constant[axis], g[i]),
				lfAdd(lfAdd(m.q0[axis], lfMul(m.qu[axis], p.U)), lfMul(m.qv[axis], p.V)))
		}
	}
	return g, h
}

// Q=D²(y²+z²−(m*x+b)²) after rational cap interpolation, where D=g0*g1>0.
// Each returned row is a degree-12 Bernstein polynomial in flank s; rows
// 0,1,2 are the quadratic Bernstein coefficients in band t.
func spiralRootPolynomial(first, second loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	g0, h0 := spiralCapBern(first, span)
	g1, h1 := spiralCapBern(second, span)
	d := spiralMul(g0, g1)
	var a, z [3][]*big.Rat
	for axis := range a {
		a[axis] = spiralMul(h0[axis], g1)
		z[axis] = spiralMul(h1[axis], g0)
	}
	al := spiralAdd(spiralScale(a[0], m), spiralScale(d, b))
	zl := spiralAdd(spiralScale(z[0], m), spiralScale(d, b))
	dy, dz, dl := spiralSub(z[1], a[1]), spiralSub(z[2], a[2]), spiralSub(zl, al)
	q0 := spiralSub(spiralAdd(spiralMul(a[1], a[1]), spiralMul(a[2], a[2])), spiralMul(al, al))
	q1 := spiralScale(spiralSub(spiralAdd(spiralMul(a[1], dy), spiralMul(a[2], dz)),
		spiralMul(al, dl)), big.NewRat(2, 1))
	q2 := spiralSub(spiralAdd(spiralMul(dy, dy), spiralMul(dz, dz)), spiralMul(dl, dl))
	return [3][]*big.Rat{q0, spiralAdd(q0, spiralScale(q1, big.NewRat(1, 2))),
		spiralAdd(spiralAdd(q0, q1), q2)}
}

func spiralAllPositive(rows [3][]*big.Rat) bool {
	for _, row := range rows {
		for _, value := range row {
			if value.Sign() <= 0 {
				return false
			}
		}
	}
	return true
}

func spiralCrossingSigns(q [3][]*big.Rat) bool {
	for _, row := range q {
		if row[0].Sign() >= 0 || row[len(row)-1].Sign() <= 0 {
			return false
		}
	}
	var derivative [3][]*big.Rat
	for i := range q {
		derivative[i] = spiralDeriv(q[i])
	}
	return spiralAllPositive(derivative)
}

// Along Q=0, dX/dt has the sign of X_t*Q_s−X_s*Q_t because Q_s>0.
// With X=(A+t*B)/D and D independent of t, the numerator after multiplying
// by D² is J=B*D*Q_s−(H_s*D−H*D_s)*Q_t, degree (23,2).
func spiralRootXOrder(first, second loftFaceMap, span freeform.BezierSpan,
	q [3][]*big.Rat) [3][]*big.Rat {
	g0, h0 := spiralCapBern(first, span)
	g1, h1 := spiralCapBern(second, span)
	d := spiralMul(g0, g1)
	a, z := spiralMul(h0[0], g1), spiralMul(h1[0], g0)
	b := spiralSub(z, a)
	bd := spiralMul(b, d)
	u0 := spiralSub(spiralMul(spiralDeriv(a), d), spiralMul(a, spiralDeriv(d)))
	u1 := spiralSub(spiralMul(spiralDeriv(b), d), spiralMul(b, spiralDeriv(d)))
	q0 := q[0]
	q1 := spiralScale(spiralSub(q[1], q[0]), big.NewRat(2, 1))
	q2 := spiralAdd(spiralSub(q[2], spiralScale(q[1], big.NewRat(2, 1))), q[0])
	j0 := spiralSub(spiralMul(bd, spiralDeriv(q0)), spiralMul(u0, q1))
	j1 := spiralSub(spiralMul(bd, spiralDeriv(q1)),
		spiralAdd(spiralScale(spiralMul(u0, q2), big.NewRat(2, 1)), spiralMul(u1, q1)))
	j2 := spiralSub(spiralMul(bd, spiralDeriv(q2)),
		spiralScale(spiralMul(u1, q2), big.NewRat(2, 1)))
	return [3][]*big.Rat{j0, spiralAdd(j0, spiralScale(j1, big.NewRat(1, 2))),
		spiralAdd(spiralAdd(j0, j1), j2)}
}

func spiralBernValue(coeff []*big.Rat, parameter *big.Rat) *big.Rat {
	work := append([]*big.Rat(nil), coeff...)
	for width := len(work); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			work[i] = lfAdd(lfMul(lfSub(lfOne(), parameter), work[i]),
				lfMul(parameter, work[i+1]))
		}
	}
	return work[0]
}

// The 64 dyadic bisections give a rational enclosure narrower than 2^-64
// of one source cubic. The cap builder still has to charge its mapped width.
func spiralIsolateRoot(coeff []*big.Rat) spiralRootCrossing {
	return spiralIsolateRootN(coeff, 64)
}

func spiralIsolateRootN(coeff []*big.Rat, steps int) spiralRootCrossing {
	lo, hi := lfZero(), lfOne()
	for i := 0; i < steps; i++ {
		mid := lfMul(lfAdd(lo, hi), big.NewRat(1, 2))
		if spiralBernValue(coeff, mid).Sign() < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return spiralRootCrossing{span: 0, lo: lo, hi: hi}
}

func spiralBernSplit(a []*big.Rat, at *big.Rat) ([]*big.Rat, []*big.Rat) {
	n := len(a) - 1
	left, right := make([]*big.Rat, n+1), make([]*big.Rat, n+1)
	work := append([]*big.Rat(nil), a...)
	for level := 0; level <= n; level++ {
		left[level], right[n-level] = work[0], work[n-level]
		for i := 0; i < n-level; i++ {
			work[i] = lfAdd(lfMul(lfSub(lfOne(), at), work[i]), lfMul(at, work[i+1]))
		}
	}
	return left, right
}

func spiralBernRestrict(a []*big.Rat, lo, hi *big.Rat) []*big.Rat {
	_, right := spiralBernSplit(a, lo)
	left, _ := spiralBernSplit(right, lfDiv(lfSub(hi, lo), lfSub(lfOne(), lo)))
	return left
}

func spiralRootAtBandT(q [3][]*big.Rat, t *big.Rat) []*big.Rat {
	oneMinus := lfSub(lfOne(), t)
	a := lfMul(oneMinus, oneMinus)
	b := lfMul(big.NewRat(2, 1), lfMul(t, oneMinus))
	c := lfMul(t, t)
	out := make([]*big.Rat, len(q[0]))
	for i := range out {
		out[i] = lfAdd(lfAdd(lfMul(a, q[0][i]), lfMul(b, q[1][i])), lfMul(c, q[2][i]))
	}
	return out
}

func spiralBandXRange(first, second loftFaceMap, span freeform.BezierSpan,
	root spiralRootCrossing, t *big.Rat) loftFaceInterval {
	a, z := spiralCapXRange(first, span, root), spiralCapXRange(second, span, root)
	oneMinus := lfSub(lfOne(), t)
	return loftFaceInterval{
		lo: lfAdd(lfMul(oneMinus, a.lo), lfMul(t, z.lo)),
		hi: lfAdd(lfMul(oneMinus, a.hi), lfMul(t, z.hi)),
	}
}

func spiralIsolateJunction(ctx context.Context, maps []loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat, caps [9]spiralRootCrossing, targetX *big.Rat) (spiralJunction, bool) {
	band := -1
	for i := 0; i < 8; i++ {
		left, right := spiralCapXRange(maps[i], span, caps[i]),
			spiralCapXRange(maps[i+1], span, caps[i+1])
		if left.hi.Cmp(targetX) < 0 && right.lo.Cmp(targetX) > 0 {
			if band >= 0 {
				return spiralJunction{}, false
			}
			band = i
		}
	}
	if band < 0 {
		return spiralJunction{}, false
	}
	q := spiralRootPolynomial(maps[band], maps[band+1], span, m, b)
	tLo, tHi := lfZero(), lfOne()
	for step := 0; step < 48; step++ {
		if step%8 == 0 && ctx.Err() != nil {
			return spiralJunction{}, false
		}
		mid := lfMul(lfAdd(tLo, tHi), big.NewRat(1, 2))
		fixed := spiralRootAtBandT(q, mid)
		root := spiralIsolateRootN(fixed, 64)
		x := spiralBandXRange(maps[band], maps[band+1], span, root, mid)
		switch {
		case x.hi.Cmp(targetX) < 0:
			tLo = mid
		case x.lo.Cmp(targetX) > 0:
			tHi = mid
		default:
			root = spiralIsolateRootN(fixed, 128)
			x = spiralBandXRange(maps[band], maps[band+1], span, root, mid)
			if x.hi.Cmp(targetX) < 0 {
				tLo = mid
			} else if x.lo.Cmp(targetX) > 0 {
				tHi = mid
			} else {
				return spiralJunction{}, false
			}
		}
	}
	leftRoot := spiralIsolateRootN(spiralRootAtBandT(q, tLo), 64)
	rightRoot := spiralIsolateRootN(spiralRootAtBandT(q, tHi), 64)
	sLo, sHi := leftRoot.lo, leftRoot.hi
	if rightRoot.lo.Cmp(sLo) < 0 {
		sLo = rightRoot.lo
	}
	if rightRoot.hi.Cmp(sHi) > 0 {
		sHi = rightRoot.hi
	}
	margin := big.NewRat(1, 1<<40)
	sLo, sHi = lfSub(sLo, margin), lfAdd(sHi, margin)
	if sLo.Sign() <= 0 || sHi.Cmp(lfOne()) >= 0 ||
		!spiralJunctionRootBracket(q, sLo, sHi, tLo, tHi) {
		return spiralJunction{}, false
	}
	box, ok := spiralJunctionBox(maps[band], maps[band+1], span, sLo, sHi, tLo, tHi)
	if !ok || box[0].lo.Cmp(targetX) > 0 || box[0].hi.Cmp(targetX) < 0 {
		return spiralJunction{}, false
	}
	box[0] = loftFaceInterval{lo: targetX, hi: targetX}
	return spiralJunction{band: band, sLo: sLo, sHi: sHi, tLo: tLo, tHi: tHi, box: box}, true
}

// spiralRootTube encloses the one true flank/root crossing branch over a
// caller-specified exact X strip. J>0 orders its two endpoint cuts in band
// time. Bernstein signs enclose every intermediate source parameter before
// the rational cap maps enclose the entire resulting ruled strip.
func spiralRootTube(ctx context.Context, proof *spiralFirstJoinSource, side int,
	xLo, xHi *big.Rat) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	if ctx == nil {
		return empty, false, nil
	}
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	if proof == nil || side < 0 || side >= len(proof.flank) ||
		xLo == nil || xHi == nil || xLo.Cmp(xHi) >= 0 {
		return empty, false, nil
	}
	span := proof.flank[side]
	low, ok := spiralIsolateJunction(ctx, proof.tooth.maps, span,
		proof.rootM, proof.rootB, proof.crossings[side], xLo)
	if !ok {
		return empty, false, ctx.Err()
	}
	high, ok := spiralIsolateJunction(ctx, proof.tooth.maps, span,
		proof.rootM, proof.rootB, proof.crossings[side], xHi)
	if !ok || low.band != high.band || low.tHi.Cmp(high.tLo) >= 0 {
		return empty, false, ctx.Err()
	}
	band := low.band
	q := spiralRootPolynomial(proof.tooth.maps[band], proof.tooth.maps[band+1],
		span, proof.rootM, proof.rootB)
	sLo, sHi := lfZero(), lfOne()
	if !spiralJunctionRootBracket(q, sLo, sHi, low.tLo, high.tHi) {
		return empty, false, ctx.Err()
	}
	for step := 0; step < 64; step++ {
		if step%8 == 0 && ctx.Err() != nil {
			return empty, false, ctx.Err()
		}
		mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
		if spiralRootTSign(q, mid, low.tLo, high.tHi, -1) {
			sLo = mid
			continue
		}
		if spiralRootTSign(q, mid, low.tLo, high.tHi, 1) {
			sHi = mid
			continue
		}
		break
	}
	box, ok := spiralJunctionBox(proof.tooth.maps[band], proof.tooth.maps[band+1],
		span, sLo, sHi, low.tLo, high.tHi)
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	if !ok || box[0].lo.Cmp(xLo) > 0 || box[0].hi.Cmp(xHi) < 0 {
		return empty, false, nil
	}
	box[0] = loftFaceInterval{lo: xLo, hi: xHi}
	return box, true, nil
}

func spiralRootTSign(q [3][]*big.Rat, s, tLo, tHi *big.Rat, sign int) bool {
	curve := []*big.Rat{spiralBernValue(q[0], s), spiralBernValue(q[1], s),
		spiralBernValue(q[2], s)}
	for _, coefficient := range spiralBernRestrict(curve, tLo, tHi) {
		if coefficient.Sign() != sign {
			return false
		}
	}
	return true
}

func spiralJunctionRootBracket(q [3][]*big.Rat, sLo, sHi, tLo, tHi *big.Rat) bool {
	for _, entry := range []struct {
		s    *big.Rat
		sign int
	}{{sLo, -1}, {sHi, 1}} {
		if !spiralRootTSign(q, entry.s, tLo, tHi, entry.sign) {
			return false
		}
	}
	return true
}

// A linear-fractional cap coordinate has its extrema at corners of a source
// rectangle when its denominator is positive. The restricted cubic's
// Bernstein controls enclose the true source U,V over the s interval.
func spiralJunctionBox(first, second loftFaceMap, span freeform.BezierSpan,
	sLo, sHi, tLo, tHi *big.Rat) ([3]loftFaceInterval, bool) {
	var out [3]loftFaceInterval
	u, v := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	u, v = spiralBernRestrict(u, sLo, sHi), spiralBernRestrict(v, sLo, sHi)
	uMin, uMax, vMin, vMax := u[0], u[0], v[0], v[0]
	for i := range u {
		if u[i].Cmp(uMin) < 0 {
			uMin = u[i]
		}
		if u[i].Cmp(uMax) > 0 {
			uMax = u[i]
		}
		if v[i].Cmp(vMin) < 0 {
			vMin = v[i]
		}
		if v[i].Cmp(vMax) > 0 {
			vMax = v[i]
		}
	}
	corners := [4][2]*big.Rat{{uMin, vMin}, {uMin, vMax}, {uMax, vMin}, {uMax, vMax}}
	var caps [2][3]loftFaceInterval
	for i, mapped := range []loftFaceMap{first, second} {
		for j, corner := range corners {
			if mapped.denominator(corner[0], corner[1]).Sign() <= 0 {
				return out, false
			}
			point := mapped.at(corner[0], corner[1])
			for axis := range out {
				if j == 0 || point[axis].Cmp(caps[i][axis].lo) < 0 {
					caps[i][axis].lo = point[axis]
				}
				if j == 0 || point[axis].Cmp(caps[i][axis].hi) > 0 {
					caps[i][axis].hi = point[axis]
				}
			}
		}
	}
	interpolate := func(a, z, t *big.Rat) *big.Rat {
		return lfAdd(lfMul(lfSub(lfOne(), t), a), lfMul(t, z))
	}
	for axis := range out {
		lo0 := interpolate(caps[0][axis].lo, caps[1][axis].lo, tLo)
		lo1 := interpolate(caps[0][axis].lo, caps[1][axis].lo, tHi)
		hi0 := interpolate(caps[0][axis].hi, caps[1][axis].hi, tLo)
		hi1 := interpolate(caps[0][axis].hi, caps[1][axis].hi, tHi)
		if lo0.Cmp(lo1) > 0 {
			lo0 = lo1
		}
		if hi0.Cmp(hi1) < 0 {
			hi0 = hi1
		}
		out[axis] = loftFaceInterval{lo: lo0, hi: hi0}
	}
	return out, true
}
