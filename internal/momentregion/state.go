package momentregion

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
