package revolveproof

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// CertifyRevolveFitBoundary asks Sketch for exact contacts on two complete
// fitted curves. The remaining whole entities need strict side certificates;
// the source walk must cross from inside to outside at each fitted curve.
func CertifyRevolveFitBoundary(record *facetedrecord.PointSectionRecord,
	section facetedrecord.RevolveSectionCertificate, base facetedrecord.Payload) (facetedrecord.RevolveSectorBoundary, bool) {
	var empty facetedrecord.RevolveSectorBoundary
	if !facetedrecord.PointSourceCurrent(record) ||
		len(record.Profile.Outer.Segments) < 4 || len(record.Profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := RevolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.ConeSection()
	src := record.Source
	result := facetedrecord.RevolveSectorBoundary{InsideSegment: -1, OutsideWitness: -1,
		FitRoute: true, Sides: make([]sketch.ConeCurveSide, len(src.Entities)),
		SourceRevision: src.Revision}
	sides := make([]sketch.ConeSide, len(src.Entities))
	fitCount := 0
	for i, entity := range src.Entities {
		edge := src.Profile.Outer[i]
		_, isFit := entity.(*sketch.FitSpline)
		_, isCircle := entity.(*sketch.Circle)
		if edge.Partial && ((!isFit && !isCircle) || !edge.TExact) {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.Profile.Outer.Segments[i].(sectionrecord.LineSeg); !ok {
				return empty, false
			}
		case *sketch.FitSpline:
			segment, ok := record.Profile.Outer.Segments[i].(sectionrecord.FitSplineSeg)
			if !ok || !FitRangeMatchesEdge(segment, edge) {
				return empty, false
			}
			cert, err := src.Sketch.CertifyConeFitCrossing(typed, cone)
			if err == nil {
				if fitCount >= len(result.Contacts) || cert.Spline() != typed ||
					cert.Cone() != cone || cert.SketchRevision() != src.Revision ||
					cert.IsStale() || cert.BeforeSide() == cert.AfterSide() ||
					cert.BeforeSide() == 0 || cert.AfterSide() == 0 {
					return empty, false
				}
				lo, hi := cert.TBounds()
				// The crossing must lie strictly inside the fragment the
				// profile keeps, so the Before and After sides bound it.
				if !(0 < lo && lo <= hi && hi < 1) ||
					!(min(segment.TStart, segment.TEnd) < lo && hi < max(segment.TStart, segment.TEnd)) {
					return empty, false
				}
				derivativeLo, derivativeHi, ok := cert.TangentBounds(0, 1)
				if !ok || !(derivativeLo[0] > 0 || derivativeHi[0] < 0) {
					return empty, false
				}
				result.Contacts[fitCount] = facetedrecord.RevolveSectorContact{Segment: i, Fit: cert}
				fitCount++
				continue
			}
		case *sketch.Arc:
			if _, ok := record.Profile.Outer.Segments[i].(sectionrecord.ArcSeg); !ok {
				return empty, false
			}
		case *sketch.Circle:
			// Only a Partial fragment reaches here; a whole circle would
			// straddle the cone and has no side to prove.
			segment, ok := record.Profile.Outer.Segments[i].(sectionrecord.CircleSeg)
			if !ok {
				return empty, false
			}
			side, margin, ok := CircleFragmentSide(src.Frame, typed, edge, segment, cone)
			if !ok || margin.Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
				return empty, false
			}
			sides[i] = side
			switch side {
			case sketch.ConeInside:
				// The root fragment plays the part of the inside arc.
				if result.InsideSegment >= 0 {
					return empty, false
				}
				result.InsideSegment = i
			case sketch.ConeOutside:
				result.OutsideWitness = i
			}
			continue
		default:
			return empty, false
		}
		cert, err := src.Sketch.CertifyConeCurveSide(entity, cone)
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
		segment, ok := record.Profile.Outer.Segments[i].(sectionrecord.FitSplineSeg)
		if !ok {
			return empty, false
		}
		startSide, endSide := result.Contacts[k].Fit.BeforeSide(), result.Contacts[k].Fit.AfterSide()
		if segment.TStart > segment.TEnd {
			startSide, endSide = endSide, startSide
		}
		if startSide != previous || endSide != next {
			return empty, false
		}
	}
	return result, true
}

// CertifyRevolveConnectorBoundary checks two whole-line cone crossings and
// strict Sketch sides for the remaining source entities.
func CertifyRevolveConnectorBoundary(record *facetedrecord.PointSectionRecord,
	section facetedrecord.RevolveSectionCertificate, base facetedrecord.Payload) (facetedrecord.RevolveSectorBoundary, bool) {
	var empty facetedrecord.RevolveSectorBoundary
	if !facetedrecord.PointSourceCurrent(record) ||
		len(record.Profile.Outer.Segments) < 5 || len(record.Profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := RevolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.ConeSection()
	src := record.Source
	result := facetedrecord.RevolveSectorBoundary{InsideSegment: -1, OutsideWitness: -1,
		Sides: make([]sketch.ConeCurveSide, len(src.Entities)), SourceRevision: src.Revision}
	count := 0
	for i, entity := range src.Entities {
		// Sketch's approximate Partial has no admissible exact endpoint. A
		// certified partial needs a separate restricted-entity side proof.
		if src.Profile.Outer[i].Partial {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.Profile.Outer.Segments[i].(sectionrecord.LineSeg); !ok {
				return empty, false
			}
			cert, err := src.Sketch.CertifyConeLineCrossing(typed, cone)
			if err == nil {
				if count >= len(result.Contacts) || cert.Line() != typed ||
					cert.Cone() != cone || cert.SketchRevision() != src.Revision ||
					cert.IsStale() || !cert.TExact() {
					return empty, false
				}
				lo, hi := cert.TBounds()
				point := cert.Point()
				if !(0 < lo && lo < hi && hi < 1) ||
					!proofbound.FiniteVec(r3.Vec{X: cert.T(), Y: point[0], Z: point[1]}) {
					return empty, false
				}
				result.Contacts[count] = facetedrecord.RevolveSectorContact{Segment: i, Line: cert}
				count++
				continue
			}
		case *sketch.FitSpline:
			if _, ok := record.Profile.Outer.Segments[i].(sectionrecord.FitSplineSeg); !ok {
				return empty, false
			}
		case *sketch.Arc:
			if _, ok := record.Profile.Outer.Segments[i].(sectionrecord.ArcSeg); !ok {
				return empty, false
			}
		default:
			return empty, false
		}
		cert, err := src.Sketch.CertifyConeCurveSide(entity, cone)
		if err != nil || cert.Entity() != entity || cert.Cone() != cone || cert.IsStale() ||
			cert.ImplicitMargin() == nil ||
			cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
			return empty, false
		}
		switch cert.Side() {
		case sketch.ConeInside:
			if _, isArc := entity.(*sketch.Arc); !isArc || result.InsideSegment >= 0 {
				return empty, false
			}
			result.InsideSegment = i
		case sketch.ConeOutside:
			result.OutsideWitness = i
		default:
			return empty, false
		}
		result.Sides[i] = cert
	}
	if count != 2 || result.InsideSegment < 0 || result.OutsideWitness < 0 {
		return empty, false
	}
	left := (result.InsideSegment + len(src.Entities) - 1) % len(src.Entities)
	right := (result.InsideSegment + 1) % len(src.Entities)
	if (result.Contacts[0].Segment != left || result.Contacts[1].Segment != right) &&
		(result.Contacts[0].Segment != right || result.Contacts[1].Segment != left) {
		return empty, false
	}
	return result, true
}

// FitRangeMatchesEdge reports whether a recorded fitted-spline segment walks
// exactly the span Sketch reports for edge. A whole edge records 0..1 or 1..0.
// A Partial edge is admitted only when Sketch certifies its bounds as the true
// parameters (TExact), and the record must carry those same bounds in the
// edge's walk direction.
func FitRangeMatchesEdge(segment sectionrecord.FitSplineSeg, edge sketch.BoundaryEdge) bool {
	if !edge.Partial {
		return (segment.TStart == 0 && segment.TEnd == 1) || (segment.TStart == 1 && segment.TEnd == 0)
	}
	if !edge.TExact || !(0 <= edge.TStart && edge.TStart < edge.TEnd && edge.TEnd <= 1) {
		return false
	}
	if edge.Reversed {
		return segment.TStart == edge.TEnd && segment.TEnd == edge.TStart
	}
	return segment.TStart == edge.TStart && segment.TEnd == edge.TEnd
}
