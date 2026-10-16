package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/sketch"
)

// certifyRevolveFitBoundary asks Sketch for exact contacts on two complete
// fitted curves. The remaining whole entities need strict side certificates;
// the source walk must cross from inside to outside at each fitted curve.
func certifyRevolveFitBoundary(record *pointSectionRecord,
	section revolveSectionCertificate, base facetedPayload) (revolveSectorBoundary, bool) {
	var empty revolveSectorBoundary
	if !pointSourceCurrent(record) ||
		len(record.Profile.Outer.Segments) < 4 || len(record.Profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := revolveproof.RevolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.ConeSection()
	src := record.Source
	result := revolveSectorBoundary{InsideSegment: -1, OutsideWitness: -1,
		FitRoute: true, Sides: make([]sketch.ConeCurveSide, len(src.Entities)),
		SourceRevision: src.Revision}
	sides := make([]sketch.ConeSide, len(src.Entities))
	fitCount := 0
	for i, entity := range src.Entities {
		if src.Profile.Outer[i].Partial {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.Profile.Outer.Segments[i].(lineSeg); !ok {
				return empty, false
			}
		case *sketch.FitSpline:
			segment, ok := record.Profile.Outer.Segments[i].(fitSplineSeg)
			if !ok || !((segment.TStart == 0 && segment.TEnd == 1) ||
				(segment.TStart == 1 && segment.TEnd == 0)) {
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
			if _, ok := record.Profile.Outer.Segments[i].(arcSeg); !ok {
				return empty, false
			}
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
		segment, ok := record.Profile.Outer.Segments[i].(fitSplineSeg)
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
