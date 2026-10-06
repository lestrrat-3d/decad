package freeform

import (
	"math"
	"math/big"
)

type MomentIntegralOrder uint8

// The orders are cumulative: each integrates everything the lower ones do.
// MomentThirdOrder adds regionIntegrals.third, the section moments a revolve's
// volume second moments need (docs/dynamic-mass-design.md §2.1): the
// cylindrical Jacobian contributes one radius and a transverse second moment
// two more.
const (
	MomentAreaOrder MomentIntegralOrder = iota
	MomentFirstOrder
	MomentSecondOrder
	MomentThirdOrder
)

func FiniteMomentValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

type ExactMoments struct {
	Area *big.Rat
	Mu   *big.Rat
	Mv   *big.Rat
	Muu  *big.Rat
	Muv  *big.Rat
	Mvv  *big.Rat
}

// complete reports whether every moment has an exact rational. A contributor
// that could not build one leaves a nil field, which is the signal to retire
// the region-level accumulator rather than publish a partial sum.
func (m ExactMoments) Complete() bool {
	return m.Area != nil && m.Mu != nil && m.Mv != nil &&
		m.Muu != nil && m.Muv != nil && m.Mvv != nil
}

func (m ExactMoments) Fields() [6]*big.Rat {
	return [6]*big.Rat{m.Area, m.Mu, m.Mv, m.Muu, m.Muv, m.Mvv}
}
