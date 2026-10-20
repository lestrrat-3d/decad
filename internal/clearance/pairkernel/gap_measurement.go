package pairkernel

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// GapMeasurement publishes the interval [res.Lo, res.Hi] as its midpoint with a
// bound covering the half width and the arithmetic's own error. The reading is
// Exact only when the kernel proved the interval exact and the bound is zero.
func GapMeasurement(res Result) measurement.Measurement {
	midpoint := proofbound.BoundedMul(proofbound.BoundedAdd(proofbound.ExactScalar(res.Lo), proofbound.ExactScalar(res.Hi)), proofbound.ExactScalar(0.5))
	halfWidth := proofbound.BoundedMul(proofbound.BoundedSub(proofbound.ExactScalar(res.Hi), proofbound.ExactScalar(res.Lo)), proofbound.ExactScalar(0.5))
	// The midpoint error and the half-width's own error both widen the
	// published interval. The same bound feeds appendClearance's tolerance gate.
	bound := proofbound.AbsSumUpper(halfWidth.Value, halfWidth.Bound, midpoint.Bound)
	gap := measurement.Measurement{
		Value:     units.Millimeters(midpoint.Value),
		Exactness: measurement.Approximate,
		Bound:     units.Millimeters(bound),
	}
	if res.Exact && bound == 0 {
		gap.Exactness = measurement.Exact
	}
	return gap
}
