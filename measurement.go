package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
)

// Exactness reports whether a measured value is exact or bounded.
type Exactness = measurement.Exactness

const (
	// Exact marks a proved exactly representable result.
	Exact = measurement.Exact
	// Approximate marks a result with a proved absolute error bound.
	Approximate = measurement.Approximate
)

// Measurement is a scalar value with its exactness and absolute error bound.
type Measurement = measurement.Measurement

// VecMeasurement is a position or direction with its exactness and bound.
type VecMeasurement = measurement.VecMeasurement

// Box is an axis-aligned bound with its own exactness and error bound.
type Box = measurement.Box

// validateAnalyticBodyMeasurements is evalPrismContext's last gate before a
// prism commits: every one of the four checks below is finiteness only, so a
// +Inf bound reads the same ErrNotFinite as a NaN value would, even though
// +Inf is this codebase's own stated UNDERIVABLE-bound convention
// (proofbound.WalkEndBound's doc comment) rather than a non-finite input.
//
// That mismatch cannot actually reach a free-form prism's build. Every
// mechanism capable of publishing +Inf into one of these four measurements
// already refuses earlier, with its own Table R sentinel, before this runs: a
// length bracket that cannot be enclosed is R15 inside freeform.FreeformArcLength, a
// directional extreme past float64 range is R18 inside
// boundaryExtremesBoundedContext (both consumed building Area/Bounds), and a
// walk endpoint bound that cannot be derived is the "no span" case
// freeformEndpointBounds already turned into ErrDegenerate before a
// survey2d.SegmentWalk exists to fold into a Vertex at all (freeformVertexAllow,
// extrude.go). So a body that reaches this call has already had every
// free-form-derived bound proven finite; a +Inf or NaN reaching here names a
// genuinely non-finite INPUT elsewhere, which is exactly what ErrNotFinite is
// for.
func validateAnalyticBodyMeasurements(body *Body) error {
	finiteMeasurement := func(m Measurement) bool {
		return finiteMeasurementValues(m.Value.Base(), m.Bound.Base())
	}
	finiteVecMeasurement := func(m VecMeasurement) bool {
		return finiteMeasurementValues(
			m.Value.X, m.Value.Y, m.Value.Z, m.Bound.Base(),
		)
	}
	finiteBox := func(b Box) bool {
		return finiteMeasurementValues(
			b.Min.X, b.Min.Y, b.Min.Z,
			b.Max.X, b.Max.Y, b.Max.Z,
			b.Bound.Base(),
		)
	}
	checks := []struct {
		name string
		ok   bool
	}{
		{name: "volume", ok: finiteMeasurement(body.volume)},
		{name: "area", ok: finiteMeasurement(body.area)},
		{name: "centroid", ok: finiteVecMeasurement(body.centroid)},
		{name: "bounds", ok: finiteBox(body.bounds)},
	}
	for _, check := range checks {
		if !check.ok {
			return fmt.Errorf(`%w: the analytic body's %s measurement is not finite`, ErrNotFinite, check.name)
		}
	}
	return nil
}

func finiteMeasurementValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}
