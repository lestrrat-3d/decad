package revolveloftfaces

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
)

// FitSideCrossing describes the exact source crossing before root binds live faces.
type FitSideCrossing struct {
	Segment int
	Span    freeform.BezierSpan
	Index   int
	First   int
	Last    int
	Caps    []loftface.RootCrossing
	X       []loftface.Interval
}
