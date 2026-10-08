package momentregion

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Field is one held integral and its outward error bound.
type Field struct{ Value, Bound *float64 }

// State writes directly into the region accumulator's stored fields.
// The view avoids copying a rational accumulator between segment additions.
type State struct {
	CoordUpper *float64
	Fields     [6]Field
	Exact      *freeform.ExactMoments
	ExactDead  *bool
	Third      *[4]proofbound.RatInterval
	ThirdDead  *bool
}

// RequirePositiveArea rejects a region with no proven positive net area.
func (s State) RequirePositiveArea() error {
	if !*s.ExactDead && s.Exact.Complete() {
		if s.Exact.Area.Sign() > 0 {
			return nil
		}
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, decaderr.ErrDegenerate)
	}
	if *s.Fields[0].Value <= 0 {
		return fmt.Errorf(`%w: the recorded region encloses no positive net area`, decaderr.ErrDegenerate)
	}
	return nil
}

// AddThird folds one segment's third-order enclosure in walk order.
func (s State) AddThird(terms [4]proofbound.RatInterval, ok bool) {
	if *s.ThirdDead {
		return
	}
	if !ok {
		*s.ThirdDead = true
		*s.Third = [4]proofbound.RatInterval{}
		return
	}
	if s.Third[0].Lo == nil {
		*s.Third = terms
		return
	}
	for i := range s.Third {
		s.Third[i] = proofbound.IntervalAdd(s.Third[i], terms[i])
	}
}

// ThirdMoments returns the third-order sum when every term had an enclosure.
func (s State) ThirdMoments() ([4]proofbound.RatInterval, bool) {
	if *s.ThirdDead || s.Third[0].Lo == nil {
		return [4]proofbound.RatInterval{}, false
	}
	return *s.Third, true
}

// IsFinite checks the held fields needed for the requested integral order.
func (s State) IsFinite(order freeform.MomentIntegralOrder) bool {
	switch order {
	case freeform.MomentAreaOrder:
		return freeform.FiniteMomentValues(*s.Fields[0].Value)
	case freeform.MomentFirstOrder:
		return freeform.FiniteMomentValues(*s.Fields[0].Value, *s.Fields[1].Value, *s.Fields[2].Value)
	default:
		return freeform.FiniteMomentValues(*s.Fields[0].Value, *s.Fields[1].Value, *s.Fields[2].Value,
			*s.Fields[3].Value, *s.Fields[4].Value, *s.Fields[5].Value)
	}
}

// NewExactMoments initializes all six rational fields.
func NewExactMoments() freeform.ExactMoments {
	return freeform.ExactMoments{
		Area: new(big.Rat),
		Mu:   new(big.Rat),
		Mv:   new(big.Rat),
		Muu:  new(big.Rat),
		Muv:  new(big.Rat),
		Mvv:  new(big.Rat),
	}
}

// AddExact folds a segment's exact rational contribution into the region.
func (s State) AddExact(exact freeform.ExactMoments) {
	if *s.ExactDead {
		return
	}
	if !exact.Complete() {
		s.DropExact()
		return
	}
	if !s.Exact.Complete() {
		*s.Exact = NewExactMoments()
	}
	running := s.Exact.Fields()
	for i, term := range exact.Fields() {
		running[i].Add(running[i], term)
	}
}

// ExactChord is a junction's closing chord where both of its ends are exact
// rationals the record denotes (a line's lerp at its recorded parameter, a
// free-form chain's end control point), in plane coordinates.
type ExactChord struct{ From, To freeform.RatPoint }

// ChargeJunction accounts for one straight closing chord, where two
// consecutive walks of a loop end and start at denoted points that need not
// coincide (docs/evaluator-design.md §4). gap is a proven upper bound on the
// chord's length. anchorReach bounds the distance of every chord point from
// the walk anchor the six held fields are taken about, and originReach the
// same about the plane origin the third-order sum is kept about.
//
// Every field is a boundary integral ∮F over one form shared by every segment
// kind, and each form's integrand is a monomial of degree k in the coordinates
// with a coefficient of at most 1: ½(u dv − v du) for the area (k = 1),
// ½u² dv and −½v² du for the first moments (k = 2), ⅓u³ dv, ½u²v dv and
// −⅓v³ du for the second (k = 3), and the dv form for the third (k = 4). So
// the chord adds at most gap·reach^k to a field, and each held field is
// widened by that much.
//
// The rational accumulator states no bound. Where chord is non-nil it adds
// the chord's exact integral instead, so a record whose every junction has
// exact ends keeps an exact integral of the closed loop. A nil chord retires
// it. The third-order enclosure takes the exact chord the same way, and is
// widened by gap·originReach⁴ without one.
func (s State) ChargeJunction(gap, anchorReach, originReach float64, chord *ExactChord, anchor sectionrecord.Point2, order freeform.MomentIntegralOrder) {
	charge := gap
	for k := 1; k <= 3; k++ {
		charge = proofbound.ProductUpper(charge, anchorReach)
		for _, field := range fieldsOfDegree(s.Fields, k, order) {
			*field.Bound = proofbound.AbsSumUpper(*field.Bound, charge)
		}
	}
	if chord == nil {
		s.DropExact()
	} else {
		au, av := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
		if au == nil || av == nil {
			s.DropExact()
		} else {
			s.AddExact(momentline.ExactChordMoments(
				new(big.Rat).Sub(chord.From.U, au), new(big.Rat).Sub(chord.From.V, av),
				new(big.Rat).Sub(chord.To.U, au), new(big.Rat).Sub(chord.To.V, av),
				order,
			))
		}
	}
	if order < freeform.MomentThirdOrder || *s.ThirdDead || s.Third[0].Lo == nil {
		return
	}
	if chord != nil {
		exact := freeform.PolyThirdMoments(
			polynomial.RatPoly{chord.From.U, new(big.Rat).Sub(chord.To.U, chord.From.U)},
			polynomial.RatPoly{chord.From.V, new(big.Rat).Sub(chord.To.V, chord.From.V)},
		)
		var terms [4]proofbound.RatInterval
		for i, value := range exact {
			terms[i] = proofbound.PointInterval(value)
		}
		s.AddThird(terms, true)
		return
	}
	third := gap
	for range 4 {
		third = proofbound.ProductUpper(third, originReach)
	}
	widen := proofarith.FloatRat(third)
	if widen == nil {
		*s.ThirdDead = true
		*s.Third = [4]proofbound.RatInterval{}
		return
	}
	for i := range s.Third {
		s.Third[i] = proofbound.IntervalWiden(s.Third[i], widen)
	}
}

// fieldsOfDegree returns the held fields whose boundary form has degree k:
// the area (k = 1), the first moments (k = 2) and the second moments (k = 3),
// the last only where order asks for them.
func fieldsOfDegree(fields [6]Field, k int, order freeform.MomentIntegralOrder) []Field {
	switch k {
	case 1:
		return fields[:1]
	case 2:
		if order < freeform.MomentFirstOrder {
			return nil
		}
		return fields[1:3]
	default:
		if order < freeform.MomentSecondOrder {
			return nil
		}
		return fields[3:6]
	}
}

// DropExact retires the region-level rational accumulator.
func (s State) DropExact() {
	*s.ExactDead = true
	*s.Exact = freeform.ExactMoments{}
}

// PublishExact rounds each complete region-level rational only once.
func (s State) PublishExact() {
	if *s.ExactDead || !s.Exact.Complete() {
		return
	}
	exact := s.Exact.Fields()
	for i, field := range s.Fields {
		held, _ := exact[i].Float64()
		if proofbound.IsNonFinite(held) {
			continue
		}
		*field.Value = held
		*field.Bound = proofarith.RationalFloatError(exact[i], held)
	}
}
