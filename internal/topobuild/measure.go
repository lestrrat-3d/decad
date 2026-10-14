package topobuild

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ValidateAnalyticMeasurements is an analytic build's last gate before its
// body commits: every one of the four checks below is finiteness only, so a
// +Inf bound reads the same decaderr.ErrNotFinite as a NaN value would, even though
// +Inf is this codebase's own stated UNDERIVABLE-bound convention
// (proofbound.WalkEndBound's doc comment) rather than a non-finite input.
//
// That mismatch cannot actually reach a free-form prism's build. Every
// mechanism capable of publishing +Inf into one of these four measurements
// already refuses earlier, with its own Table R sentinel, before this runs: a
// length bracket that cannot be enclosed is R15 inside freeform.FreeformArcLength, a
// directional extreme past float64 range is R18 inside
// prismextent.ProfileBoundaryExtremes (both consumed building Area/Bounds), and a
// walk endpoint bound that cannot be derived is the "no span" case
// freeformEndpointBounds already turned into decaderr.ErrDegenerate before a
// survey2d.SegmentWalk exists to fold into a Vertex at all (boundarywalk.JunctionVertexAt,
// root extrude.go). So a body that reaches this call has already had every
// free-form-derived bound proven finite; a +Inf or NaN reaching here names a
// genuinely non-finite INPUT elsewhere, which is exactly what decaderr.ErrNotFinite is
// for.
func ValidateAnalyticMeasurements(body *topo.Body) error {
	finiteMeasurement := func(m measurement.Measurement) bool {
		return measurement.FiniteValues(m.Value.Base(), m.Bound.Base())
	}
	finiteVecMeasurement := func(m measurement.VecMeasurement) bool {
		return measurement.FiniteValues(
			m.Value.X, m.Value.Y, m.Value.Z, m.Bound.Base(),
		)
	}
	finiteBox := func(b measurement.Box) bool {
		return measurement.FiniteValues(
			b.Min.X, b.Min.Y, b.Min.Z,
			b.Max.X, b.Max.Y, b.Max.Z,
			b.Bound.Base(),
		)
	}
	checks := []struct {
		name string
		ok   bool
	}{
		{name: "volume", ok: finiteMeasurement(body.Volume)},
		{name: "area", ok: finiteMeasurement(body.Area)},
		{name: "centroid", ok: finiteVecMeasurement(body.Centroid)},
		{name: "bounds", ok: finiteBox(body.Bounds)},
	}
	for _, check := range checks {
		if !check.ok {
			return fmt.Errorf(`%w: the analytic body's %s measurement is not finite`, decaderr.ErrNotFinite, check.name)
		}
	}
	return nil
}
