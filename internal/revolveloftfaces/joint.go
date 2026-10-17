package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/sketch"
)

// A joint source vertex is the live Sketch point shared by two adjacent
// complete entities. Record coordinates and fitted Bezier controls must
// describe that same point exactly; proximity of sampled geometry is not a
// substitute for Sketch topology.
type revolveLoftFacesJointVertex struct {
	point                  *sketch.Point
	fitSegment, arcSegment int
	fitSpan                freeform.BezierSpan
	fitAtEnd               bool
	coordinate             [2]*big.Rat
}

func revolveLoftFacesFitArcVertex(trim *loftFacesTrimRecord,
	fitIndex, arcIndex int, fitAtEnd bool) (revolveLoftFacesJointVertex, bool) {
	if trim == nil || trim.Source == nil || trim.Source.Source == nil {
		return revolveLoftFacesJointVertex{}, false
	}
	segments := trim.Source.Profile.Outer.Segments
	entities := trim.Source.Source.Entities
	n := len(segments)
	if n < 2 || len(entities) != n || fitIndex < 0 || arcIndex < 0 ||
		fitIndex >= n || arcIndex >= n || fitIndex == arcIndex {
		return revolveLoftFacesJointVertex{}, false
	}
	if fitAtEnd && (fitIndex+1)%n != arcIndex ||
		!fitAtEnd && (arcIndex+1)%n != fitIndex {
		return revolveLoftFacesJointVertex{}, false
	}
	fitRecord, fitOK := segments[fitIndex].(fitSplineSeg)
	arcRecord, arcOK := segments[arcIndex].(arcSeg)
	fitEntity, liveFit := entities[fitIndex].(*sketch.FitSpline)
	arcEntity, liveArc := entities[arcIndex].(*sketch.Arc)
	if !fitOK || !arcOK || !liveFit || !liveArc ||
		len(fitRecord.Fit) < 2 || len(fitEntity.Fit) != len(fitRecord.Fit) ||
		!((fitRecord.TStart == 0 && fitRecord.TEnd == 1) ||
			(fitRecord.TStart == 1 && fitRecord.TEnd == 0)) ||
		!((arcRecord.TStart == 0 && arcRecord.TEnd == 1) ||
			(arcRecord.TStart == 1 && arcRecord.TEnd == 0)) {
		return revolveLoftFacesJointVertex{}, false
	}
	fitAtHigh := fitRecord.TStart == 1
	if fitAtEnd {
		fitAtHigh = fitRecord.TEnd == 1
	}
	fitPointIndex := 0
	if fitAtHigh {
		fitPointIndex = len(fitEntity.Fit) - 1
	}
	arcAtNaturalStart := arcRecord.TEnd == 0
	if fitAtEnd {
		arcAtNaturalStart = arcRecord.TStart == 0
	}
	arcPoint, recordedArc := arcEntity.End, arcRecord.End
	if arcAtNaturalStart {
		arcPoint, recordedArc = arcEntity.Start, arcRecord.Start
	}
	point, recordedFit := fitEntity.Fit[fitPointIndex], fitRecord.Fit[fitPointIndex]
	if point == nil || point != arcPoint || point.IsStale() ||
		recordedFit != recordedArc || recordedFit.U != point.X() ||
		recordedFit.V != point.Y() {
		return revolveLoftFacesJointVertex{}, false
	}
	spans, ok := RevolveLoftFacesStraightOrFitSpans(trim, fitIndex, fitRecord)
	if !ok || len(spans) == 0 {
		return revolveLoftFacesJointVertex{}, false
	}
	span := spans[0]
	sourceAtEnd := fitAtHigh
	if sourceAtEnd {
		span = spans[len(spans)-1]
	}
	parameter := new(big.Rat)
	if sourceAtEnd {
		parameter = big.NewRat(1, 1)
	}
	location := loftface.BezierPoint(span, parameter)
	if location[0].Cmp(proofarith.FloatRat(recordedFit.U)) != 0 ||
		location[1].Cmp(proofarith.FloatRat(recordedFit.V)) != 0 {
		return revolveLoftFacesJointVertex{}, false
	}
	return revolveLoftFacesJointVertex{point: point, fitSegment: fitIndex,
		arcSegment: arcIndex, fitSpan: span, fitAtEnd: sourceAtEnd,
		coordinate: [2]*big.Rat{location[0], location[1]}}, true
}
