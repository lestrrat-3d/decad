package decad

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

func fitSplineBezierSpans(segment FitSplineSeg, work *freeform.FreeformWork) ([]freeform.BezierSpan, error) {
	return splinebezier.FitSplineBezierSpans(segment, work)
}

func isFitSplineSeg(segment CurveSegment) bool { return splinebezier.IsFitSplineSeg(segment) }

func fitCoords(points []Point2) [][2]float64 { return splinebezier.FitCoords(points) }
