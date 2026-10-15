package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// spiralBlankSource is the legacy six-side view of one authenticated Revolve
// section. The generic certificate owns the source walk, faces, and cone gap;
// these named corners only map the older spiral join's fixed cap roles.
type spiralBlankSource struct {
	Section           revolveSectionCertificate
	ToeAxis, HeelAxis planePoint
	ToeOuter, ToeRoot planePoint
	Ded, HeelOuter    planePoint
	EdgeIndex         [6]int
	RootFace          *topo.Face
}

func exactSpiralBlank(blank *Body) (*spiralBlankSource, bool) {
	for _, section := range revolveRootCandidates(blank) {
		source, ok := spiralSixSideBlank(section)
		if ok {
			return source, true
		}
	}
	return nil, false
}

func spiralSixSideBlank(section revolveSectionCertificate) (*spiralBlankSource, bool) {
	if len(section.Spans) != 6 || section.AxisSpan < 0 ||
		section.AxisSpan >= len(section.Spans) ||
		section.RootSpan < 0 || section.RootSpan >= len(section.Spans) {
		return nil, false
	}
	result := &spiralBlankSource{Section: section,
		RootFace: section.Spans[section.RootSpan].Face}
	if result.RootFace == nil {
		return nil, false
	}
	axis := section.Spans[section.AxisSpan]
	result.ToeAxis, result.HeelAxis = axis.Start, axis.End
	if result.ToeAxis.U > result.HeelAxis.U {
		result.ToeAxis, result.HeelAxis = result.HeelAxis, result.ToeAxis
	}
	var interior []planePoint
	for _, span := range section.Spans {
		if len(span.SourceEdges) != 1 {
			return nil, false
		}
		point := span.Start
		if point.V == 0 {
			continue
		}
		switch point.U {
		case result.ToeAxis.U:
			if result.ToeOuter != (planePoint{}) {
				return nil, false
			}
			result.ToeOuter = point
		case result.HeelAxis.U:
			if result.HeelOuter != (planePoint{}) {
				return nil, false
			}
			result.HeelOuter = point
		default:
			interior = append(interior, point)
		}
	}
	if len(interior) != 2 || result.ToeOuter == (planePoint{}) ||
		result.HeelOuter == (planePoint{}) {
		return nil, false
	}
	if interior[0].U > interior[1].U {
		interior[0], interior[1] = interior[1], interior[0]
	}
	result.ToeRoot, result.Ded = interior[0], interior[1]
	if !(result.ToeRoot.U < result.ToeAxis.U &&
		result.ToeAxis.U < result.Ded.U && result.Ded.U < result.HeelAxis.U) {
		return nil, false
	}
	cycle := [6]planePoint{result.ToeAxis, result.HeelAxis, result.HeelOuter,
		result.Ded, result.ToeRoot, result.ToeOuter}
	var seen [6]bool
	for _, span := range section.Spans {
		matched := false
		for edge := range cycle {
			start, end := cycle[edge], cycle[(edge+1)%len(cycle)]
			forward := span.Start == start && span.End == end
			reverse := span.Start == end && span.End == start
			if !forward && !reverse {
				continue
			}
			if seen[edge] || edge == 3 && span.Face != result.RootFace {
				return nil, false
			}
			seen[edge] = true
			result.EdgeIndex[edge] = span.SourceEdges[0]
			matched = true
			break
		}
		if !matched {
			return nil, false
		}
	}
	return result, seen[3]
}

// A bounded box around one true flank/root/tool triple junction. The true
// point has X=the corresponding exact ToeX or HeelX; s and t remain enclosed
// separately because their algebraic roots need not be rational.
type spiralJunction struct {
	Band     int
	SLo, SHi *big.Rat
	TLo, THi *big.Rat
	Box      [3]loftFaceInterval
}

// A first curved tooth join may use this only while both input bodies remain
// the original unplaced blank and cone-trimmed LoftFaces tooth. The matcher
// below rechecks the live Sketch source and the exact finite blank meridian.
type spiralFirstJoinSource struct {
	Blank      *spiralBlankSource
	Tooth      *loftFacesTrimRecord
	RootM      *big.Rat
	RootB      *big.Rat
	ToeX       *big.Rat
	HeelX      *big.Rat
	ToeOffset  *big.Rat
	HeelOffset *big.Rat
	ToeCapGap  *big.Rat
	HeelCapGap *big.Rat
	Flank      [2]freeform.BezierSpan // root-containing cubic of groups 0 and 2
	Crossings  [2][9]loftface.RootCrossing
	Junctions  [2][2]spiralJunction // flank 0/2, then toe/heel
	HeelSeam   *spiralHeelRootSeam
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
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim == nil || p.Xform != r3.Identity() {
		return nil, false, nil
	}
	trim := p.LoftFacesTrim
	if !spiralTrimSourceCurrent(trim) || !facetedtopology.CapGroupsPresent(p.Src, p.FaceOf, len(p.Groups),
		trim.ToeCapGroup, trim.HeelCapGroup) {
		return nil, false, nil
	}
	m, b := section.Section.MeridianEquation()
	if !spiralSourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	if !spiralOtherSides(ctx, trim, m, b) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		return nil, false, nil
	}
	toeX, ok := spiralRootToolX(m, b, trim.Toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := spiralRootToolX(m, b, trim.Heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		lfAdd(lfMul(m, toeX), b).Sign() <= 0 ||
		lfAdd(lfMul(m, heelX), b).Sign() <= 0 {
		return nil, false, nil
	}
	toeGap, ok := spiralToolOuterGap(trim.Toe, section.ToeRoot, section.ToeOuter, 1)
	if !ok {
		return nil, false, nil
	}
	heelGap, ok := spiralToolOuterGap(trim.Heel, section.Ded, section.HeelOuter, -1)
	if !ok {
		return nil, false, nil
	}
	proof := &spiralFirstJoinSource{Blank: section, Tooth: trim, RootM: m, RootB: b,
		ToeX: toeX, HeelX: heelX,
		ToeOffset:  lfSub(toeX, lfF(section.ToeRoot.U)),
		HeelOffset: lfSub(heelX, lfF(section.Ded.U)),
		ToeCapGap:  toeGap, HeelCapGap: heelGap}
	// The real finite root lies between the tool-clipped ends, with a small
	// overhang at each cap. The later cap proof charges both exact offsets.
	if proof.ToeOffset.Sign() >= 0 || proof.HeelOffset.Sign() <= 0 {
		return nil, false, nil
	}
	if !spiralOriginalCapsAvoidToolRootCircles(trim, m, b, toeX, heelX) &&
		!spiralTangentCapTouchesHeelCircle(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	for side, group := range [...]int{0, 2} {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		fit, ok := trim.Source.Profile.Outer.Segments[group].(fitSplineSeg)
		if !ok {
			return nil, false, nil
		}
		work := freeform.NewFreeformWork()
		work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		spans, err := splinebezier.FitSplineBezierSpans(fit, work)
		if err != nil {
			return nil, false, err
		}
		if len(spans) != 14 {
			return nil, false, nil
		}
		proof.Flank[side] = spans[0]
		for band := range 8 {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			for spanIndex, span := range spans {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				q := loftface.MeridianQRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
				if spanIndex == 0 {
					if !polynomial.BernsteinRowsCross(q) ||
						!loftface.CapXDecreases(trim.Maps[band], span) ||
						!loftface.CapXDecreases(trim.Maps[band+1], span) ||
						!polynomial.BernsteinRowsPositive(loftface.MeridianJRows(trim.Maps[band], trim.Maps[band+1], span, q)) {
						return nil, false, nil
					}
					if band == 0 {
						proof.Crossings[side][0] = loftface.IsolateRoot(q[0])
					}
					proof.Crossings[side][band+1] = loftface.IsolateRoot(q[2])
				} else if !polynomial.BernsteinRowsPositive(q) {
					return nil, false, nil
				}
			}
		}
		toeRange := loftface.CapXRange(trim.Maps[0], spans[0], proof.Crossings[side][0])
		heelRange := loftface.CapXRange(trim.Maps[8], spans[0], proof.Crossings[side][8])
		if toeRange.Hi.Cmp(toeX) >= 0 || heelRange.Lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
		for end, targetX := range []*big.Rat{toeX, heelX} {
			junction, ok := spiralIsolateJunction(ctx, trim.Maps, spans[0], m, b,
				proof.Crossings[side], targetX)
			if !ok {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
				return nil, false, nil
			}
			proof.Junctions[side][end] = junction
		}
	}
	// The embedded source solid's boundary meets each tool/root circle only
	// at these two flank junctions. All other contour groups stay on one side.
	// The whole original cap 0 is below toeX. The whole cap-8 plane either
	// misses its heel tool/root circle (the strict branch), or, for a
	// cone-tangent end with a matching record, touches that circle at one
	// point with the circle on the source side of the plane
	// (spiralTangentCapTouchesHeelCircle). Either way neither end-cap
	// interior can create an extra contact, even though cap 8 spans the
	// heelX coordinate.
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
	proof.HeelSeam = seam
	return proof, true, nil
}

// The toe cap lies wholly before the toe tool/root circle in X. The heel
// cap may span heelX, so its full supporting plane must miss the entire
// heel tool/root circle. For A+B*cos(theta)+C*sin(theta), A²>B²+C² is an
// exact rational separation proof for every angle, including cap interiors.
func spiralOriginalCapsAvoidToolRootCircles(trim *loftFacesTrimRecord,
	m, b, toeX, heelX *big.Rat) bool {
	box, err := lfSourceBox(trim.Source)
	if err != nil || len(trim.Maps) != 9 {
		return false
	}
	for _, end := range [...]int{0, 8} {
		if loftface.PositiveDenominator(trim.Maps[end], box) != nil {
			return false
		}
	}
	return spiralToeCapBeforeToeCircle(trim, box, toeX) &&
		loftface.CapPlaneAvoidsRootCircle(trim.Maps[8], loftface.BoxCorners(box), m, b, heelX)
}

// spiralToeCapBeforeToeCircle reports whether the whole original cap 0 lies
// below toeX. The caller has proved cap 0's denominator positive over box.
func spiralToeCapBeforeToeCircle(trim *loftFacesTrimRecord, box [4]*big.Rat, toeX *big.Rat) bool {
	xAxis := loftFaceRatVec{lfOne(), lfZero(), lfZero()}
	return loftface.ScalarRange(trim.Maps[0], xAxis, loftface.BoxCorners(box)).Hi.Cmp(toeX) < 0
}

// spiralTangentCapTouchesHeelCircle is the cone-tangent branch of the heel
// half of spiralOriginalCapsAvoidToolRootCircles. It runs only when the trim
// record carries a cone-tangent end that matches the heel cone and the final
// cap map exactly. The cap-8 plane then touches the heel tool/root circle at
// one point instead of missing it, which loftface.CapPlaneTouchesRootCircle
// decides exactly, with the circle on the side of the preceding cap.
func spiralTangentCapTouchesHeelCircle(trim *loftFacesTrimRecord, m, b, toeX, heelX *big.Rat) bool {
	if trim == nil || trim.Tangent == nil || !spiralTangentRecordCurrent(trim) || len(trim.Maps) != 9 {
		return false
	}
	box, err := lfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	for _, end := range [...]int{0, 7, 8} {
		if loftface.PositiveDenominator(trim.Maps[end], box) != nil {
			return false
		}
	}
	return spiralToeCapBeforeToeCircle(trim, box, toeX) &&
		loftface.CapPlaneTouchesRootCircle(trim.Maps[8], loftface.BoxCorners(box), m, b, heelX, trim.Maps[7])
}

// spiralTangentRecordCurrent reports whether the cone-tangent record of trim
// is present on both the trim and its base cap record, equal on both by
// exact value, built for exactly the heel cone's apex and slope, and
// reproduces the recorded final cap map. Every comparison is an exact Cmp of
// immutable values, never a pointer or a flag. It is false when either
// record is missing, so the tangent branches run only for a matching record;
// a trim with no tangent record on either side is not tangent, and the
// strict branches decide it.
func spiralTangentRecordCurrent(trim *loftFacesTrimRecord) bool {
	if trim == nil || trim.Tangent == nil || trim.Heel == nil || len(trim.Maps) == 0 ||
		trim.Base.LoftFacesCap == nil || trim.Base.LoftFacesCap.Tangent == nil {
		return false
	}
	end := trim.Tangent
	return end.Equal(*trim.Base.LoftFacesCap.Tangent) &&
		end.MatchesCone(trim.Heel.Apex, trim.Heel.Slope) &&
		loftface.MapEqual(end.Map(), trim.Maps[len(trim.Maps)-1])
}

func spiralTrimSourceCurrent(trim *loftFacesTrimRecord) bool {
	if trim == nil || trim.Source == nil || !pointSourceCurrent(trim.Source) ||
		trim.Toe == nil || trim.Heel == nil || len(trim.Maps) != 9 || len(trim.Bounds) != 9 ||
		!orderedLoftFacesConeLimits(trim.Toe, trim.Heel, trim.Base) ||
		!trim.Base.LoftFacesPrimitive || trim.Base.LoftFacesCap == nil ||
		trim.Base.LoftFacesCap.Source != trim.Source ||
		len(trim.Base.LoftFacesCap.Maps) != 9 || len(trim.Base.LoftFacesCap.Bounds) != 9 ||
		len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.Base.LoftFacesCap.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.Source.Profile.Outer.Segments) != 6 || len(trim.Source.Source.Entities) != 6 ||
		len(trim.Source.Profile.Holes) != 0 {
		return false
	}
	// A tangent record on either side must be the same one, current for the
	// heel cone and the final map. No record on either side is the strict case.
	if (trim.Tangent != nil || trim.Base.LoftFacesCap.Tangent != nil) && !spiralTangentRecordCurrent(trim) {
		return false
	}
	for _, entry := range trim.Source.Source.Profile.Outer {
		if entry.Partial {
			return false
		}
	}
	segments := trim.Source.Profile.Outer.Segments
	entities := trim.Source.Source.Entities
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
	box, err := lfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	for _, mapped := range trim.Maps {
		if loftface.PositiveDenominator(mapped, box) != nil {
			return false
		}
	}
	return true
}

// The whole ruled source stays in positive Y and above zero signed radius
// on the blank's root meridian. The opposite root-circle arc passes negative
// Y, while a root of the squared implicit with negative signed radius would
// not be a physical cone crossing.
func spiralSourcePhysicalSector(trim *loftFacesTrimRecord, m, b *big.Rat) bool {
	box, err := lfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	corners := loftface.BoxCorners(box)
	y := loftFaceRatVec{lfZero(), lfOne(), lfZero()}
	x := loftFaceRatVec{lfOne(), lfZero(), lfZero()}
	for _, mapped := range trim.Maps {
		if loftface.ScalarRange(mapped, y, corners).Lo.Sign() <= 0 {
			return false
		}
		xRange := loftface.ScalarRange(mapped, x, corners)
		r0, r1 := lfAdd(lfMul(m, xRange.Lo), b), lfAdd(lfMul(m, xRange.Hi), b)
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
	source := trim.Source
	n := len(source.SourceStation)
	if n == 0 || len(trim.StationGroup) != n {
		return false
	}
	var seen [6]int
	for band := range 8 {
		lo, hi := trim.Bounds[band], trim.Bounds[band+1]
		if len(lo.SourceDeparture) != n || len(hi.SourceDeparture) != n ||
			len(lo.ProjectiveDeparture) != n || len(hi.ProjectiveDeparture) != n {
			return false
		}
		for j, station := range source.SourceStation {
			if j%64 == 0 && ctx.Err() != nil {
				return false
			}
			group := trim.StationGroup[j]
			if group < 0 || group >= len(seen) {
				return false
			}
			if group == 0 || group == 2 {
				continue
			}
			seen[group]++
			departure := max(
				proofbound.AbsSumUpper(lo.SourceDeparture[j], lo.ProjectiveDeparture[j]),
				proofbound.AbsSumUpper(hi.SourceDeparture[j], hi.ProjectiveDeparture[j]))
			if proofbound.IsNonFinite(departure) || departure < 0 {
				return false
			}
			next := source.SourceStation[(j+1)%n]
			if !spiralCellSide(trim.Maps[band], trim.Maps[band+1], station, next,
				lfF(departure), m, b, group == 1, lfZero(), lfOne(), 8) {
				return false
			}
		}
	}
	return seen[1] > 0 && seen[3] > 0 && seen[4] > 0 && seen[5] > 0
}

func spiralCellSide(first, second loftFaceMap, a, z planePoint, departure, m, b *big.Rat,
	outside bool, tLo, tHi *big.Rat, depth int) bool {
	firstA := first.At(lfF(a.U), lfF(a.V))
	firstZ := first.At(lfF(z.U), lfF(z.V))
	secondA := second.At(lfF(a.U), lfF(a.V))
	secondZ := second.At(lfF(z.U), lfF(z.V))
	blend := func(x, y loftFaceRatVec, t *big.Rat) loftFaceRatVec {
		return loftface.VAdd(loftface.VScale(x, lfSub(lfOne(), t)), loftface.VScale(y, t))
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
		box[axis] = loftFaceInterval{Lo: lfSub(minimum, departure), Hi: lfAdd(maximum, departure)}
	}
	r0, r1 := lfAdd(lfMul(m, box[0].Lo), b), lfAdd(lfMul(m, box[0].Hi), b)
	if r0.Cmp(r1) > 0 {
		r0, r1 = r1, r0
	}
	radius := loftFaceInterval{Lo: r0, Hi: r1}
	minimumF := lfSub(lfAdd(loftface.IntervalSquareMin(box[1]), loftface.IntervalSquareMin(box[2])),
		loftface.IntervalSquareMax(radius))
	maximumF := lfSub(lfAdd(loftface.IntervalSquareMax(box[1]), loftface.IntervalSquareMax(box[2])),
		loftface.IntervalSquareMin(radius))
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
	if cone == nil || cone.Apex == nil || cone.Slope == nil || cone.Slope.Sign() <= 0 {
		return nil, false
	}
	denominator := lfAdd(lfOne(), lfMul(cone.Slope, m))
	if denominator.Sign() <= 0 {
		return nil, false
	}
	return lfDiv(lfSub(cone.Apex, lfMul(cone.Slope, b)), denominator), true
}

// The exact straight blank cap meridian and tool cone differ affinely.
// Endpoint signs and the largest absolute endpoint residual bound their
// separation over the entire finite blank cap generator.
func spiralToolOuterGap(cone *pointConeLimit, first, last planePoint, sign int) (*big.Rat, bool) {
	residual := func(point planePoint) *big.Rat {
		return lfSub(lfAdd(lfF(point.U), lfMul(cone.Slope, lfF(point.V))), cone.Apex)
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

func spiralIsolateJunction(ctx context.Context, maps []loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat, caps [9]loftface.RootCrossing, targetX *big.Rat) (spiralJunction, bool) {
	band := -1
	for i := range 8 {
		left, right := loftface.CapXRange(maps[i], span, caps[i]),
			loftface.CapXRange(maps[i+1], span, caps[i+1])
		if left.Hi.Cmp(targetX) < 0 && right.Lo.Cmp(targetX) > 0 {
			if band >= 0 {
				return spiralJunction{}, false
			}
			band = i
		}
	}
	if band < 0 {
		return spiralJunction{}, false
	}
	q := loftface.MeridianQRows(maps[band], maps[band+1], span, m, b)
	tLo, tHi := lfZero(), lfOne()
	for step := range 48 {
		if step%8 == 0 && ctx.Err() != nil {
			return spiralJunction{}, false
		}
		mid := lfMul(lfAdd(tLo, tHi), big.NewRat(1, 2))
		fixed := polynomial.BernsteinRowsAt(q, mid)
		root := loftface.IsolateRootN(fixed, 64)
		x := loftface.BandXRange(maps[band], maps[band+1], span, root, mid)
		switch {
		case x.Hi.Cmp(targetX) < 0:
			tLo = mid
		case x.Lo.Cmp(targetX) > 0:
			tHi = mid
		default:
			root = loftface.IsolateRootN(fixed, 128)
			x = loftface.BandXRange(maps[band], maps[band+1], span, root, mid)
			if x.Hi.Cmp(targetX) < 0 {
				tLo = mid
			} else if x.Lo.Cmp(targetX) > 0 {
				tHi = mid
			} else {
				return spiralJunction{}, false
			}
		}
	}
	leftRoot := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, tLo), 64)
	rightRoot := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, tHi), 64)
	sLo, sHi := leftRoot.Lo, leftRoot.Hi
	if rightRoot.Lo.Cmp(sLo) < 0 {
		sLo = rightRoot.Lo
	}
	if rightRoot.Hi.Cmp(sHi) > 0 {
		sHi = rightRoot.Hi
	}
	margin := big.NewRat(1, 1<<40)
	sLo, sHi = lfSub(sLo, margin), lfAdd(sHi, margin)
	if sLo.Sign() <= 0 || sHi.Cmp(lfOne()) >= 0 ||
		!spiralJunctionRootBracket(q, sLo, sHi, tLo, tHi) {
		return spiralJunction{}, false
	}
	box, ok := loftface.JunctionBox(maps[band], maps[band+1], span, sLo, sHi, tLo, tHi)
	if !ok || box[0].Lo.Cmp(targetX) > 0 || box[0].Hi.Cmp(targetX) < 0 {
		return spiralJunction{}, false
	}
	box[0] = loftFaceInterval{Lo: targetX, Hi: targetX}
	return spiralJunction{Band: band, SLo: sLo, SHi: sHi, TLo: tLo, THi: tHi, Box: box}, true
}

func spiralJunctionRootBracket(q [3][]*big.Rat, sLo, sHi, tLo, tHi *big.Rat) bool {
	for _, entry := range []struct {
		s    *big.Rat
		sign int
	}{{sLo, -1}, {sHi, 1}} {
		if !polynomial.BernsteinRowsSignAt(q, entry.s, tLo, tHi, entry.sign) {
			return false
		}
	}
	return true
}
