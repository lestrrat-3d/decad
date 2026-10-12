package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

func certifyBevelFitBoundary(record *pointSectionRecord,
	blank *bevelBlankSource, base facetedPayload) (*bevelToothBoundary, bool) {
	if !pointSourceCurrent(record) || blank == nil || len(record.profile.Outer.Segments) != 6 {
		return nil, false
	}
	implicitCost, ok := bevelImplicitDisplacement(blank, base)
	if !ok {
		return nil, false
	}
	cone := blank.coneSection()
	src := record.source
	boundary := &bevelToothBoundary{insideArc: -1, fitRoute: true}
	lineIndex := [2]int{-1, -1}
	lineCount, fitCount, arcCount, tipArc := 0, 0, 0, -1
	for i, entity := range src.entities {
		if src.profile.Outer[i].Partial {
			return nil, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.profile.Outer.Segments[i].(lineSeg); !ok || lineCount >= 2 {
				return nil, false
			}
			cert, err := src.sketch.CertifyConeCurveSide(typed, cone)
			if err != nil || cert.Entity() != entity || cert.Cone() != cone ||
				cert.IsStale() || cert.Side() != sketch.ConeInside ||
				cert.ImplicitMargin() == nil ||
				cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
				return nil, false
			}
			lineIndex[lineCount], boundary.sides[i] = i, cert
			lineCount++
		case *sketch.FitSpline:
			segment, ok := record.profile.Outer.Segments[i].(fitSplineSeg)
			if !ok || fitCount >= 2 ||
				!((segment.TStart == 0 && segment.TEnd == 1) ||
					(segment.TStart == 1 && segment.TEnd == 0)) {
				return nil, false
			}
			cert, err := src.sketch.CertifyConeFitCrossing(typed, cone)
			if err != nil || cert.Spline() != typed || cert.Cone() != cone ||
				cert.SketchRevision() != src.revision || cert.IsStale() ||
				cert.BeforeSide() == cert.AfterSide() || cert.BeforeSide() == 0 ||
				cert.AfterSide() == 0 {
				return nil, false
			}
			lo, hi := cert.TBounds()
			if !(0 < lo && lo <= hi && hi < 1) {
				return nil, false
			}
			// The fan stores source-entity labels, not station parameters.
			// A strict local-U derivative lets the cap path locate the root
			// within this entity's ordered chord chain.
			derivativeLo, derivativeHi, ok := cert.TangentBounds(0, 1)
			if !ok || !(derivativeLo[0] > 0 || derivativeHi[0] < 0) {
				return nil, false
			}
			boundary.fitIndex[fitCount], boundary.fitCross[fitCount] = i, cert
			fitCount++
		case *sketch.Arc:
			if _, ok := record.profile.Outer.Segments[i].(arcSeg); !ok {
				return nil, false
			}
			cert, err := src.sketch.CertifyConeCurveSide(typed, cone)
			if err != nil || cert.Entity() != entity || cert.Cone() != cone ||
				cert.IsStale() || cert.ImplicitMargin() == nil ||
				cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
				return nil, false
			}
			switch cert.Side() {
			case sketch.ConeInside:
				if boundary.insideArc >= 0 {
					return nil, false
				}
				boundary.insideArc = i
			case sketch.ConeOutside:
				if tipArc >= 0 {
					return nil, false
				}
				tipArc = i
			default:
				return nil, false
			}
			boundary.sides[i] = cert
			arcCount++
		default:
			return nil, false
		}
	}
	if lineCount != 2 || fitCount != 2 || arcCount != 2 ||
		boundary.insideArc < 0 || tipArc < 0 ||
		lineIndex[0] != (boundary.insideArc+5)%6 &&
			lineIndex[1] != (boundary.insideArc+5)%6 ||
		lineIndex[0] != (boundary.insideArc+1)%6 &&
			lineIndex[1] != (boundary.insideArc+1)%6 {
		return nil, false
	}
	beforeTip, afterTip := (tipArc+5)%6, (tipArc+1)%6
	if boundary.fitIndex[0] != beforeTip && boundary.fitIndex[1] != beforeTip ||
		boundary.fitIndex[0] != afterTip && boundary.fitIndex[1] != afterTip {
		return nil, false
	}
	for k, i := range boundary.fitIndex {
		segment, ok := record.profile.Outer.Segments[i].(fitSplineSeg)
		if !ok {
			return nil, false
		}
		cert := boundary.fitCross[k]
		startSide, endSide := cert.BeforeSide(), cert.AfterSide()
		if segment.TStart == 1 {
			startSide, endSide = endSide, startSide
		}
		if i == beforeTip &&
			(startSide != sketch.ConeInside || endSide != sketch.ConeOutside) ||
			i == afterTip &&
				(startSide != sketch.ConeOutside || endSide != sketch.ConeInside) {
			return nil, false
		}
	}
	return boundary, true
}

// bevelFitWorldBox maps Sketch's outward-rounded local curve bounds through
// the exact binary64 coefficients of its recorded frame. It does not use a
// rounded world point to decide which side of a cone contains the curve.
func bevelFitWorldBox(frame r3.Frame, origin r3.Vec,
	lower, upper [2]float64) ([3]proofbound.RatInterval, bool) {
	var world [3]proofbound.RatInterval
	if !proofbound.FiniteVec(r3.Vec{X: lower[0], Y: lower[1], Z: upper[0]}) ||
		!proofbound.FiniteVec(r3.Vec{X: upper[1]}) ||
		lower[0] > upper[0] || lower[1] > upper[1] {
		return world, false
	}
	u := proofbound.Interval(proofarith.FloatRat(lower[0]), proofarith.FloatRat(upper[0]))
	v := proofbound.Interval(proofarith.FloatRat(lower[1]), proofarith.FloatRat(upper[1]))
	for i, axis := range [3][3]float64{
		{origin.X, frame.U().X, frame.V().X},
		{origin.Y, frame.U().Y, frame.V().Y},
		{origin.Z, frame.U().Z, frame.V().Z},
	} {
		world[i] = proofbound.IntervalAdd(
			proofbound.PointInterval(proofarith.FloatRat(axis[0])),
			proofbound.IntervalAdd(
				proofbound.IntervalScale(u, proofarith.FloatRat(axis[1])),
				proofbound.IntervalScale(v, proofarith.FloatRat(axis[2]))))
	}
	return world, true
}

// bevelFitMeridianDifference bounds F_blank-F_cone and its parameter
// derivative over one certified fitted-spline range. The blank's recorded
// meridian is R(x)=m*x+b, while Sketch certifies
// F_cone=y*y+z*z-s*s*(x-originX)^2. Both forms use exact rationals of their
// stored binary64 inputs; a positive derivative margin can therefore prove
// that the recorded meridian has the same single crossing as Sketch's cone.
func bevelFitMeridianDifference(blank *bevelBlankSource, cone sketch.ConeSection,
	frame r3.Frame, pointLo, pointHi, tangentLo, tangentHi [2]float64,
) (*big.Rat, *big.Rat, [3]proofbound.RatInterval, bool) {
	var empty [3]proofbound.RatInterval
	if blank == nil || cone.Axis != (r3.Vec{X: 1}) ||
		cone.Apex.Y != 0 || cone.Apex.Z != 0 || cone.Slope <= 0 ||
		math.IsNaN(cone.Slope) || math.IsInf(cone.Slope, 0) {
		return nil, nil, empty, false
	}
	point, ok := bevelFitWorldBox(frame, frame.Origin(), pointLo, pointHi)
	if !ok {
		return nil, nil, empty, false
	}
	tangent, ok := bevelFitWorldBox(frame, r3.Vec{}, tangentLo, tangentHi)
	if !ok {
		return nil, nil, empty, false
	}
	m, b := blank.meridianEquation()
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

// bevelFitWorldStation chooses a held mesh station inside a proven world box
// and returns a spatial allowance containing every exact point in that box.
func bevelFitWorldStation(box [3]proofbound.RatInterval) (r3.Vec, float64, bool) {
	var held [3]float64
	maxError := 0.0
	for i, interval := range box {
		if interval.Lo == nil || interval.Hi == nil || interval.Lo.Cmp(interval.Hi) > 0 {
			return r3.Vec{}, 0, false
		}
		mid := new(big.Rat).Quo(new(big.Rat).Add(interval.Lo, interval.Hi), big.NewRat(2, 1))
		held[i], _ = mid.Float64()
		if math.IsNaN(held[i]) || math.IsInf(held[i], 0) {
			return r3.Vec{}, 0, false
		}
		maxError = max(maxError, proofbound.IntervalFloatError(interval, held[i]))
	}
	bound := proofbound.Radius3D(maxError)
	return r3.Vec{X: held[0], Y: held[1], Z: held[2]},
		bound, !proofbound.IsNonFinite(bound)
}

// bevelFitSeatPoint encloses the unique crossing with the blank's recorded
// meridian. Sketch proves the root and strict sides for the reported cone;
// the interval calculation here proves the recorded meridian has one root in
// a slightly wider parameter interval and no roots outside it.
func bevelFitSeatPoint(src *pointSectionSource, cert sketch.ConeFitCrossing,
	blank *bevelBlankSource) (r3.Vec, float64, [2]float64, bool) {
	if src == nil || blank == nil || cert.Spline() == nil || cert.IsStale() ||
		cert.SketchRevision() != src.revision || cert.Cone() != blank.coneSection() {
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
	maxDifference, _, _, ok := bevelFitMeridianDifference(blank, cert.Cone(), src.frame,
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
		_, derivativeDifference, box, boxOK := bevelFitMeridianDifference(
			blank, cert.Cone(), src.frame, pointLo, pointHi, tangentLo, tangentHi)
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
		point, bound, stationOK := bevelFitWorldStation(box)
		if !stationOK {
			return r3.Vec{}, 0, [2]float64{}, false
		}
		return point, bound, [2]float64{lo, hi}, true
	}
	return r3.Vec{}, 0, [2]float64{}, false
}

// bevelFitStationEdge selects the one recorded fan cell whose exact source
// endpoints lie strictly on opposite sides of the meridian-root interval.
// Loft's stationRound encloses each held local coordinate against the same
// rational fitted curve Sketch certified. Strict local-U monotonicity turns
// that coordinate order into the source parameter order; a contact near a
// station or a second qualifying cell is refused.
func bevelFitStationEdge(base facetedPayload, groups []int, source *pointSectionSource,
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
