package extent

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// MagnitudeInBounded also reports the rounding committed by unit conversion.
func MagnitudeInBounded(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, float64, error) {
	m, err := sectionrecord.MagnitudeIn(v, kind, unit, what)
	if err != nil {
		return 0, 0, err
	}
	return m, ConversionRound(v, unit, m), nil
}

// DisplacementIn accepts a signed offset and reports its conversion rounding.
// The zero Value denotes no offset.
func DisplacementIn(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, float64, error) {
	if v == (units.Value{}) {
		return 0, 0, nil
	}
	if v.Kind() != kind {
		return 0, 0, fmt.Errorf(`%w: %s must be a %s, got %s`, decaderr.ErrUnitKind, what, kind, v.Kind())
	}
	m, err := v.In(unit)
	if err != nil {
		return 0, 0, fmt.Errorf(`%w: %s is not representable: %s`, decaderr.ErrNotFinite, what, err)
	}
	return m, ConversionRound(v, unit, m), nil
}

// ConversionRound compares the held float with the exact rational rescale of
// the same float inputs and unit factors.
func ConversionRound(v units.Value, unit units.Unit, held float64) float64 {
	exact := ExactConversion(v, unit)
	if exact == nil {
		return math.Inf(1)
	}
	return proofarith.RationalFloatError(exact, held)
}

// ExactConversion is the rational value of v in unit. It returns nil for
// non-finite operands or a zero target factor.
func ExactConversion(v units.Value, unit units.Unit) *big.Rat {
	mag := proofarith.FloatRat(v.Mag())
	from := proofarith.FloatRat(v.Unit().Factor())
	to := proofarith.FloatRat(unit.Factor())
	if mag == nil || from == nil || to == nil || to.Sign() == 0 {
		return nil
	}
	return new(big.Rat).Quo(new(big.Rat).Mul(mag, from), to)
}
