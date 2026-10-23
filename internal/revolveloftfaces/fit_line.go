package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// A fit/line joint binds both whole Sketch entities to the same live point.
// The two source spans must give that point identical exact meridian values.
type revolveLoftFacesFitLineVertex struct {
	Point                   *sketch.Point
	FitSegment, LineSegment int
	FitSpan, LineSpan       freeform.BezierSpan
	LineAtEnd               bool
	Coordinate              [2]*big.Rat
}

type RevolveLoftFacesFitLineJoint struct {
	Vertex revolveLoftFacesFitLineVertex
	Band   int
	Roots  []loftface.EndpointRoot
}
