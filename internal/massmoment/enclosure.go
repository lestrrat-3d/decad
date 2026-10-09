package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// momentInterval reads a held section moment and its absolute bound as a rational interval.
func momentInterval(value proofbound.BoundedScalar) (proofbound.RatInterval, error) {
	if proofbound.IsNonFinite(value.Value) || proofbound.IsNonFinite(value.Bound) || value.Bound < 0 {
		return proofbound.RatInterval{}, fmt.Errorf("%w: section moment has no finite enclosure", decaderr.ErrNotFinite)
	}
	held, bound := proofarith.FloatRat(value.Value), proofarith.FloatRat(value.Bound)
	return proofbound.IntervalOwned(new(big.Rat).Sub(held, bound), new(big.Rat).Add(held, bound)), nil
}

// SectionInputs carries the recorded second-order moments and their exact rational form when available.
type SectionInputs struct {
	ExactAvailable bool
	Exact          [6]*big.Rat
	Bounded        [6]proofbound.BoundedScalar
}

// SectionInputsOf reads the held and exact second-order values recorded by
// the section moment evaluator.
func SectionInputsOf(ig momentinput.Integrals) SectionInputs {
	input := SectionInputs{Bounded: [6]proofbound.BoundedScalar{
		{Value: ig.Area, Bound: ig.AreaBound}, {Value: ig.Mu, Bound: ig.MuBound},
		{Value: ig.Mv, Bound: ig.MvBound}, {Value: ig.Muu, Bound: ig.MuuBound},
		{Value: ig.Muv, Bound: ig.MuvBound}, {Value: ig.Mvv, Bound: ig.MvvBound},
	}}
	if !ig.ExactDead && ig.Exact.Complete() {
		input.ExactAvailable = true
		input.Exact = ig.Exact.Fields()
	}
	return input
}

// SectionIntervals keeps exact moments as points and widens held moments by their individual bounds.
func SectionIntervals(input SectionInputs) ([6]proofbound.RatInterval, error) {
	section := [6]proofbound.RatInterval{}
	if input.ExactAvailable {
		for i, value := range input.Exact {
			section[i] = proofbound.PointInterval(value)
		}
		return section, nil
	}
	for i, value := range input.Bounded {
		var err error
		section[i], err = momentInterval(value)
		if err != nil {
			return section, err
		}
	}
	return section, nil
}
