package momentregion

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Translate restores the recorded profile origin after anchor-local integration.
func (s State) Translate(anchor sectionrecord.Point2, order freeform.MomentIntegralOrder) {
	if !*s.ExactDead {
		*s.Exact = momentline.TranslateExactMoments(*s.Exact, momentline.Point{U: anchor.U, V: anchor.V}, order)
	}
	if order == freeform.MomentAreaOrder {
		return
	}
	area := proofbound.MeasuredScalar(*s.Fields[0].Value, *s.Fields[0].Bound)
	mu := proofbound.MeasuredScalar(*s.Fields[1].Value, *s.Fields[1].Bound)
	mv := proofbound.MeasuredScalar(*s.Fields[2].Value, *s.Fields[2].Bound)
	if order >= freeform.MomentSecondOrder {
		two := proofbound.ExactScalar(2)
		anchorU := proofbound.ExactScalar(anchor.U)
		anchorV := proofbound.ExactScalar(anchor.V)
		muu := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(*s.Fields[3].Value, *s.Fields[3].Bound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.BoundedMul(two, anchorU), mu),
				proofbound.BoundedMul(proofbound.BoundedMul(anchorU, anchorU), area),
			),
		)
		muv := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(*s.Fields[4].Value, *s.Fields[4].Bound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(anchorV, mu),
				proofbound.BoundedAdd(
					proofbound.BoundedMul(anchorU, mv),
					proofbound.BoundedMul(proofbound.BoundedMul(anchorU, anchorV), area),
				),
			),
		)
		mvv := proofbound.BoundedAdd(
			proofbound.MeasuredScalar(*s.Fields[5].Value, *s.Fields[5].Bound),
			proofbound.BoundedAdd(
				proofbound.BoundedMul(proofbound.BoundedMul(two, anchorV), mv),
				proofbound.BoundedMul(proofbound.BoundedMul(anchorV, anchorV), area),
			),
		)
		*s.Fields[3].Value, *s.Fields[3].Bound = muu.Value, muu.Bound
		*s.Fields[4].Value, *s.Fields[4].Bound = muv.Value, muv.Bound
		*s.Fields[5].Value, *s.Fields[5].Bound = mvv.Value, mvv.Bound
	}
	mu = proofbound.BoundedAdd(mu, proofbound.BoundedMul(proofbound.ExactScalar(anchor.U), area))
	mv = proofbound.BoundedAdd(mv, proofbound.BoundedMul(proofbound.ExactScalar(anchor.V), area))
	*s.Fields[1].Value, *s.Fields[1].Bound = mu.Value, mu.Bound
	*s.Fields[2].Value, *s.Fields[2].Bound = mv.Value, mv.Bound
	*s.CoordUpper = math.Max(*s.CoordUpper, proofbound.AbsSumUpper(anchor.U, anchor.V))
}
