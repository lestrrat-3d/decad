package revolveproof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// revolveFitMeridianDifference bounds F_blank-F_cone and its parameter
// derivative over one certified fitted-spline range. The blank's recorded
// meridian is R(x)=m*x+b, while Sketch certifies
// F_cone=y*y+z*z-s*s*(x-originX)^2. Both forms use exact rationals of their
// stored binary64 inputs; a positive derivative margin can therefore prove
// that the recorded meridian has the same single crossing as Sketch's cone.
// A general certificate takes fitMeridianDifferenceAbout.
func revolveFitMeridianDifference(section facetedrecord.RevolveSectionCertificate, cone sketch.ConeSection,
	frame r3.Frame, pointLo, pointHi, tangentLo, tangentHi [2]float64,
) (*big.Rat, *big.Rat, [3]proofbound.RatInterval, bool) {
	if section.General != nil {
		return fitMeridianDifferenceAbout(section, cone, frame, pointLo, pointHi, tangentLo, tangentHi)
	}
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
func revolveFitSeatPoint(src *facetedrecord.PointSectionSource, cert sketch.ConeFitCrossing,
	section facetedrecord.RevolveSectionCertificate) (r3.Vec, float64, [2]float64, bool) {
	if src == nil || cert.Spline() == nil || cert.IsStale() ||
		cert.SketchRevision() != src.Revision || cert.Cone() != section.ConeSection() {
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
	maxDifference, _, _, ok := revolveFitMeridianDifference(section, cert.Cone(), src.Frame,
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
			section, cert.Cone(), src.Frame, pointLo, pointHi, tangentLo, tangentHi)
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
func revolveFitStationEdge(base facetedrecord.Payload, groups []int, source *facetedrecord.PointSectionSource,
	entity int, cert sketch.ConeFitCrossing, root [2]float64) (int, bool) {
	record := base.PointSection
	if record == nil || record.Source != source || source == nil ||
		len(record.SourceStation) != len(groups) || entity < 0 ||
		entity >= len(record.Profile.Outer.Segments) || entity >= len(source.Entities) ||
		source.Entities[entity] != cert.Spline() || cert.IsStale() ||
		proofbound.IsNonFinite(record.SourceStationRound) || record.SourceStationRound < 0 {
		return -1, false
	}
	segment, ok := record.Profile.Outer.Segments[entity].(sectionrecord.FitSplineSeg)
	edge := source.Profile.Outer[entity]
	if !ok || !FitRangeMatchesEdge(segment, edge) ||
		!(0 < root[0] && root[0] < root[1] && root[1] < 1) ||
		!(min(segment.TStart, segment.TEnd) < root[0] && root[1] < max(segment.TStart, segment.TEnd)) {
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
	if segment.TStart > segment.TEnd {
		increasing = !increasing
	}
	rootLo, rootHi := proofarith.FloatRat(lower[0]), proofarith.FloatRat(upper[0])
	round := proofarith.FloatRat(record.SourceStationRound)
	selected := -1
	for i, group := range groups {
		if group != entity {
			continue
		}
		a, b := record.SourceStation[i].U, record.SourceStation[(i+1)%len(groups)].U
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
