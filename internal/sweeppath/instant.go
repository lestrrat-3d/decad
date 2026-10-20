package sweeppath

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"
)

// Instant names the dyadic fraction f of a sweep of the given duration,
// rounding the fraction and the elapsed time to the nearest floats and
// charging the rounding to the elapsed time's bound.
func Instant(f, duration *big.Rat) report.SweepInstant {
	t := new(big.Rat).Mul(f, duration)
	value := RatFloatNearest(t)
	bound := proofarith.RationalFloatError(t, value)
	return report.SweepInstant{
		Fraction: units.Scalar(RatFloatNearest(f)),
		Elapsed: report.Measurement{Value: units.Seconds(value), Bound: units.Seconds(bound),
			Exactness: measurement.ExactnessOf(bound)},
	}
}
