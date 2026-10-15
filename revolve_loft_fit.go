package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// certifyRevolveFitBoundary asks Sketch for exact contacts on two complete
// fitted curves. The remaining whole entities need strict side certificates;
// the source walk must cross from inside to outside at each fitted curve.
func certifyRevolveFitBoundary(record *pointSectionRecord,
	section revolveSectionCertificate, base facetedPayload) (revolveSectorBoundary, bool) {
	var empty revolveSectorBoundary
	if !pointSourceCurrent(record) ||
		len(record.profile.Outer.Segments) < 4 || len(record.profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := revolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.ConeSection()
	src := record.source
	result := revolveSectorBoundary{InsideSegment: -1, OutsideWitness: -1,
		FitRoute: true, Sides: make([]sketch.ConeCurveSide, len(src.entities)),
		SourceRevision: src.revision}
	sides := make([]sketch.ConeSide, len(src.entities))
	fitCount := 0
	for i, entity := range src.entities {
		if src.profile.Outer[i].Partial {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.profile.Outer.Segments[i].(lineSeg); !ok {
				return empty, false
			}
		case *sketch.FitSpline:
			segment, ok := record.profile.Outer.Segments[i].(fitSplineSeg)
			if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
				(segment.TStart == 1 && segment.TEnd == 0)) {
				return empty, false
			}
			cert, err := src.sketch.CertifyConeFitCrossing(typed, cone)
			if err == nil {
				if fitCount >= len(result.Contacts) || cert.Spline() != typed ||
					cert.Cone() != cone || cert.SketchRevision() != src.revision ||
					cert.IsStale() || cert.BeforeSide() == cert.AfterSide() ||
					cert.BeforeSide() == 0 || cert.AfterSide() == 0 {
					return empty, false
				}
				lo, hi := cert.TBounds()
				if !(0 < lo && lo <= hi && hi < 1) {
					return empty, false
				}
				derivativeLo, derivativeHi, ok := cert.TangentBounds(0, 1)
				if !ok || !(derivativeLo[0] > 0 || derivativeHi[0] < 0) {
					return empty, false
				}
				result.Contacts[fitCount] = revolveSectorContact{Segment: i, Fit: cert}
				fitCount++
				continue
			}
		case *sketch.Arc:
			if _, ok := record.profile.Outer.Segments[i].(arcSeg); !ok {
				return empty, false
			}
		default:
			return empty, false
		}
		cert, err := src.sketch.CertifyConeCurveSide(entity, cone)
		if err != nil || cert.Entity() != entity || cert.Cone() != cone ||
			cert.IsStale() || cert.ImplicitMargin() == nil ||
			cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
			return empty, false
		}
		sides[i] = cert.Side()
		switch sides[i] {
		case sketch.ConeInside:
			if _, isArc := entity.(*sketch.Arc); isArc {
				if result.InsideSegment >= 0 {
					return empty, false
				}
				result.InsideSegment = i
			}
		case sketch.ConeOutside:
			result.OutsideWitness = i
		default:
			return empty, false
		}
		result.Sides[i] = cert
	}
	if fitCount != 2 || result.InsideSegment < 0 || result.OutsideWitness < 0 {
		return empty, false
	}
	for i, side := range sides {
		next := sides[(i+1)%len(sides)]
		if side != 0 {
			if next != 0 && next != side {
				return empty, false
			}
			continue
		}
		k := 0
		if result.Contacts[1].Segment == i {
			k = 1
		} else if result.Contacts[0].Segment != i {
			return empty, false
		}
		previous := sides[(i+len(sides)-1)%len(sides)]
		if previous == 0 || next == 0 || previous == next {
			return empty, false
		}
		segment, ok := record.profile.Outer.Segments[i].(fitSplineSeg)
		if !ok {
			return empty, false
		}
		startSide, endSide := result.Contacts[k].Fit.BeforeSide(), result.Contacts[k].Fit.AfterSide()
		if segment.TStart == 1 {
			startSide, endSide = endSide, startSide
		}
		if startSide != previous || endSide != next {
			return empty, false
		}
	}
	return result, true
}

// revolveFitMeridianDifference bounds F_blank-F_cone and its parameter
// derivative over one certified fitted-spline range. The blank's recorded
// meridian is R(x)=m*x+b, while Sketch certifies
// F_cone=y*y+z*z-s*s*(x-originX)^2. Both forms use exact rationals of their
// stored binary64 inputs; a positive derivative margin can therefore prove
// that the recorded meridian has the same single crossing as Sketch's cone.
func revolveFitMeridianDifference(section revolveSectionCertificate, cone sketch.ConeSection,
	frame r3.Frame, pointLo, pointHi, tangentLo, tangentHi [2]float64,
) (*big.Rat, *big.Rat, [3]proofbound.RatInterval, bool) {
	var empty [3]proofbound.RatInterval
	if cone.Axis != (r3.Vec{X: 1}) ||
		cone.Apex.Y != 0 || cone.Apex.Z != 0 || cone.Slope <= 0 ||
		math.IsNaN(cone.Slope) || math.IsInf(cone.Slope, 0) {
		return nil, nil, empty, false
	}
	point, ok := proofbound.FrameBoxWorld(frame, frame.Origin(), pointLo, pointHi)
	if !ok {
		return nil, nil, empty, false
	}
	tangent, ok := proofbound.FrameBoxWorld(frame, r3.Vec{}, tangentLo, tangentHi)
	if !ok {
		return nil, nil, empty, false
	}
	m, b := section.MeridianEquation()
	x := point[0]
	rootRadius := proofbound.IntervalAdd(proofbound.IntervalScale(x, m),
		proofbound.PointInterval(b))
	coneX := proofbound.IntervalSub(x,
		proofbound.PointInterval(proofarith.FloatRat(cone.Apex.X)))
	if rootRadius.Lo.Sign() <= 0 || coneX.Lo.Sign() <= 0 {
		return nil, nil, empty, false
	}
	slope := proofarith.FloatRat(cone.Slope)
	// Factoring the difference avoids losing the small public-cone versus
	// recorded-meridian gap when their two squared radii nearly cancel.
	deltaSlope := new(big.Rat).Sub(slope, m)
	slopeOrigin := new(big.Rat).Mul(slope, proofarith.FloatRat(cone.Apex.X))
	gap := proofbound.IntervalSub(proofbound.IntervalScale(x, deltaSlope),
		proofbound.PointInterval(new(big.Rat).Add(slopeOrigin, b)))
	sum := proofbound.IntervalAdd(proofbound.IntervalScale(x,
		new(big.Rat).Add(slope, m)),
		proofbound.PointInterval(new(big.Rat).Sub(b, slopeOrigin)))
	difference := proofbound.IntervalMul(gap, sum)
	coefficient := proofbound.IntervalAdd(
		proofbound.IntervalScale(proofbound.IntervalScale(coneX, slope), deltaSlope),
		proofbound.IntervalScale(gap, m))
	derivative := proofbound.IntervalScale(
		proofbound.IntervalMul(coefficient, tangent[0]), big.NewRat(2, 1))
	return proofbound.IntervalAbsUpper(difference), proofbound.IntervalAbsUpper(derivative), point, true
}

// revolveFitSeatPoint encloses the unique crossing with the blank's recorded
// meridian. Sketch proves the root and strict sides for the reported cone;
// the interval calculation here proves the recorded meridian has one root in
// a slightly wider parameter interval and no roots outside it.
func revolveFitSeatPoint(src *pointSectionSource, cert sketch.ConeFitCrossing,
	section revolveSectionCertificate) (r3.Vec, float64, [2]float64, bool) {
	if src == nil || cert.Spline() == nil || cert.IsStale() ||
		cert.SketchRevision() != src.revision || cert.Cone() != section.ConeSection() {
		return r3.Vec{}, 0, [2]float64{}, false
	}
	rootLo, rootHi := cert.TBounds()
	if !(0 < rootLo && rootLo <= rootHi && rootHi < 1) ||
		cert.BeforeSide() == cert.AfterSide() || cert.BeforeSide() == 0 || cert.AfterSide() == 0 {
		return r3.Vec{}, 0, [2]float64{}, false
	}
	globalLo, globalHi, ok := cert.PointBounds(0, 1)
	if !ok {
		return r3.Vec{}, 0, [2]float64{}, false
	}
	globalTangentLo, globalTangentHi, ok := cert.TangentBounds(0, 1)
	if !ok {
		return r3.Vec{}, 0, [2]float64{}, false
	}
	maxDifference, _, _, ok := revolveFitMeridianDifference(section, cert.Cone(), src.frame,
		globalLo, globalHi, globalTangentLo, globalTangentHi)
	if !ok {
		return r3.Vec{}, 0, [2]float64{}, false
	}
	width := rootHi - rootLo
	if width == 0 {
		width = math.Nextafter(rootHi, 1) - rootHi
	}
	for range 48 {
		lo := math.Nextafter(max(0, rootLo-width), 0)
		hi := math.Nextafter(min(1, rootHi+width), 1)
		width *= 2
		if !(0 < lo && lo < rootLo && rootHi < hi && hi < 1) {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		before, after := cert.MarginBefore(lo), cert.MarginAfter(hi)
		if before == nil || after == nil ||
			before.Cmp(maxDifference) <= 0 || after.Cmp(maxDifference) <= 0 {
			continue
		}
		pointLo, pointHi, pointOK := cert.PointBounds(lo, hi)
		tangentLo, tangentHi, tangentOK := cert.TangentBounds(lo, hi)
		if !pointOK || !tangentOK {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		_, derivativeDifference, box, boxOK := revolveFitMeridianDifference(
			section, cert.Cone(), src.frame, pointLo, pointHi, tangentLo, tangentHi)
		if !boxOK {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		derivativeSign, derivativeMin := cert.DerivativeBound(lo, hi)
		if derivativeSign == 0 || derivativeMin == nil ||
			derivativeMin.Cmp(derivativeDifference) <= 0 {
			continue
		}
		if derivativeSign > 0 &&
			(cert.BeforeSide() != sketch.ConeInside || cert.AfterSide() != sketch.ConeOutside) ||
			derivativeSign < 0 &&
				(cert.BeforeSide() != sketch.ConeOutside || cert.AfterSide() != sketch.ConeInside) {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		point, bound, stationOK := proofbound.BoxMidStation(box)
		if !stationOK {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		return point, bound, [2]float64{lo, hi}, true
	}
	return r3.Vec{}, 0, [2]float64{}, false
}

// revolveFitStationEdge selects the one recorded fan cell whose exact source
// endpoints lie strictly on opposite sides of the meridian-root interval.
// Loft's stationRound encloses each held local coordinate against the same
// rational fitted curve Sketch certified. Strict local-U monotonicity turns
// that coordinate order into the source parameter order; a contact near a
// station or a second qualifying cell is refused.
func revolveFitStationEdge(base facetedPayload, groups []int, source *pointSectionSource,
	entity int, cert sketch.ConeFitCrossing, root [2]float64) (int, bool) {
	record := base.pointSection
	if record == nil || record.source != source || source == nil ||
		len(record.sourceStation) != len(groups) || entity < 0 ||
		entity >= len(record.profile.Outer.Segments) || entity >= len(source.entities) ||
		source.entities[entity] != cert.Spline() || cert.IsStale() ||
		proofbound.IsNonFinite(record.sourceStationRound) || record.sourceStationRound < 0 {
		return -1, false
	}
	segment, ok := record.profile.Outer.Segments[entity].(fitSplineSeg)
	if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
		(segment.TStart == 1 && segment.TEnd == 0)) ||
		!(0 < root[0] && root[0] < root[1] && root[1] < 1) {
		return -1, false
	}
	lower, upper, ok := cert.PointBounds(root[0], root[1])
	if !ok || !proofbound.FiniteVec(r3.Vec{X: lower[0], Y: upper[0]}) || lower[0] > upper[0] {
		return -1, false
	}
	tangentLo, tangentHi, ok := cert.TangentBounds(0, 1)
	if !ok || !(tangentLo[0] > 0 || tangentHi[0] < 0) {
		return -1, false
	}
	increasing := tangentLo[0] > 0
	if segment.TStart == 1 {
		increasing = !increasing
	}
	rootLo, rootHi := proofarith.FloatRat(lower[0]), proofarith.FloatRat(upper[0])
	round := proofarith.FloatRat(record.sourceStationRound)
	selected := -1
	for i, group := range groups {
		if group != entity {
			continue
		}
		a, b := record.sourceStation[i].U, record.sourceStation[(i+1)%len(groups)].U
		if math.IsNaN(a) || math.IsInf(a, 0) || math.IsNaN(b) || math.IsInf(b, 0) {
			return -1, false
		}
		aLo := new(big.Rat).Sub(proofarith.FloatRat(a), round)
		aHi := new(big.Rat).Add(proofarith.FloatRat(a), round)
		bLo := new(big.Rat).Sub(proofarith.FloatRat(b), round)
		bHi := new(big.Rat).Add(proofarith.FloatRat(b), round)
		brackets := increasing && aHi.Cmp(rootLo) < 0 && bLo.Cmp(rootHi) > 0 ||
			!increasing && aLo.Cmp(rootHi) > 0 && bHi.Cmp(rootLo) < 0
		if brackets {
			if selected >= 0 {
				return -1, false
			}
			selected = i
		}
	}
	return selected, selected >= 0
}
