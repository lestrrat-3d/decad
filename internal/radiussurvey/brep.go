package radiussurvey

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// BrepFilletRadius is one concave fillet band's proven tube radius.
type BrepFilletRadius struct {
	Radius float64
	Bound  float64
}

// Brep reads concave circular swept faces and filling fillet bands. A section
// displacement makes a recorded wall radius undecidable.
func Brep(walls []sectionrecord.CurveSegment, fillets []BrepFilletRadius,
	sectionDelta float64) reportvocab.ScalarSurvey {
	if sectionDelta != 0 {
		return reportvocab.ScalarSurvey{}
	}
	agg := survey2d.MinAggregate()
	for _, fillet := range fillets {
		agg.Take(fillet.Radius, fillet.Bound)
	}
	work := freeform.NewFreeformWork()
	for _, wall := range walls {
		w, err := boundarywalk.WalkOf(wall, work)
		if err != nil {
			return reportvocab.ScalarSurvey{}
		}
		if w.IsCircular() && w.Th1 < w.Th0 {
			agg.Take(w.Radius, w.RadiusBound)
		}
	}
	return Resolve(agg)
}
