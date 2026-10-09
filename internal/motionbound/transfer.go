package motionbound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// TransferredOverlap is motion §5.1's collision transfer. An overlap measured
// at the float pose is a collision at the ideal pose only when its proven
// lower end, Value − Bound rounded down, strictly exceeds the volume the
// mover's boundary can sweep between the two poses. The published volume
// carries that allowance in its Bound, so Value − Bound stays a proven lower
// bound on the ideal overlap. An unmeasured overlap never transfers.
func TransferredOverlap(volume measurement.Measurement, measured bool, allowance float64) (measurement.Measurement, bool) {
	if !measured || proofbound.IsNonFinite(allowance) {
		return measurement.Measurement{}, false
	}
	value, bound := proofarith.FloatRat(volume.Value.Base()), proofarith.FloatRat(volume.Bound.Base())
	if value == nil || bound == nil {
		return measurement.Measurement{}, false
	}
	lower := proofbound.RatFloatDown(new(big.Rat).Sub(value, bound))
	if !(lower > allowance) {
		return measurement.Measurement{}, false
	}
	if allowance == 0 {
		return volume, true
	}
	return measurement.Measurement{
		Value:     volume.Value,
		Exactness: measurement.Approximate,
		Bound:     units.CubicMillimeters(proofbound.AbsSumUpper(volume.Bound.Base(), allowance)),
	}, true
}

// LowerBoundMeasurement publishes a proven lower distance as an approximate
// measurement whose value is rounded down and whose bound is zero.
func LowerBoundMeasurement(lowest *big.Rat) *measurement.Measurement {
	if lowest == nil {
		return nil
	}
	return &measurement.Measurement{
		Value:     units.Millimeters(proofbound.RatFloatDown(lowest)),
		Exactness: measurement.Approximate,
		Bound:     units.Millimeters(0),
	}
}
