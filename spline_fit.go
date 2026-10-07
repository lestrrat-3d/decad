package decad

import (
	"github.com/lestrrat-3d/decad/internal/curveconvert"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func fitSplineBezierSpans(segment FitSplineSeg, work *freeform.FreeformWork) ([]survey2d.BezierSpan, error) {
	return curveconvert.FitSplineBezierSpans(segment, work)
}

func isFitSplineSeg(segment CurveSegment) bool { return curveconvert.IsFitSplineSeg(segment) }

func fitCoords(points []Point2) [][2]float64 { return curveconvert.FitCoords(points) }
