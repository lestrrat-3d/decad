package dynamics

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/units"
)

// frictionCoefficient keeps the exact interval used by the response proof.
// The nominal value only proposes impulses; it does not certify a cone.
type frictionCoefficient struct {
	nominal      units.Value
	lower, upper *big.Rat
}

func exactFrictionCoefficient(value units.Value) frictionCoefficient {
	base := exactBase(value)
	return frictionCoefficient{nominal: value, lower: base, upper: new(big.Rat).Set(base)}
}

// mixBodyFriction bounds the geometric mean of two held body coefficients.
// Its endpoints are adjacent representable floats unless the root is exact.
// It rejects a positive mean outside the nonzero finite float64 range.
func mixBodyFriction(a, b units.Value) (frictionCoefficient, bool) {
	product := new(big.Rat).Mul(exactBase(a), exactBase(b))
	if product.Sign() == 0 {
		return exactFrictionCoefficient(units.Scalar(0)), true
	}
	minimum := ratFloat(math.SmallestNonzeroFloat64)
	maximum := ratFloat(math.MaxFloat64)
	if product.Cmp(new(big.Rat).Mul(minimum, minimum)) < 0 ||
		product.Cmp(new(big.Rat).Mul(maximum, maximum)) > 0 {
		return frictionCoefficient{}, false
	}
	root := new(big.Float).SetPrec(256).SetRat(product)
	root.Sqrt(root)
	nominal, _ := root.Float64()
	value := units.Scalar(nominal)
	square := func(x float64) int {
		r := ratFloat(x)
		return new(big.Rat).Mul(r, r).Cmp(product)
	}
	lower, upper := nominal, nominal
	for square(lower) > 0 {
		lower = math.Nextafter(lower, 0)
	}
	for square(upper) < 0 {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	return frictionCoefficient{nominal: value, lower: ratFloat(lower), upper: ratFloat(upper)}, true
}
