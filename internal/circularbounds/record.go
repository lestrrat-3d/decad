package circularbounds

import "github.com/lestrrat-3d/decad/internal/sectionrecord"

// RecordSegment transfers the recorded fields used by circular proofs.
func RecordSegment(segment sectionrecord.CurveSegment) CurveSegment {
	switch segment := segment.(type) {
	case sectionrecord.CircleSeg:
		return CircleSeg{
			Center: RecordPoint(segment.Center), Radius: segment.Radius, CCW: segment.CCW,
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	case sectionrecord.ArcSeg:
		return ArcSeg{
			Center: RecordPoint(segment.Center), Start: RecordPoint(segment.Start), End: RecordPoint(segment.End),
			TStart: segment.TStart, TEnd: segment.TEnd,
		}
	default:
		return nil
	}
}

// RecordPoint transfers a recorded plane-local point to a circular proof.
func RecordPoint(point sectionrecord.Point2) Point2 { return Point2{U: point.U, V: point.V} }
