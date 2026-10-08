package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
// survey2d.SegmentWalk exists to fold into a Vertex at all (junctionVertexAt,
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

// chargePrismMap widens every reading a plane-coordinate build published —
// the volume, the body's and each face's area, each edge's length — to
// cover the image under the map the prism family denotes through:
// L = B·[U V N], the frame's held U, V and N and the placement's held basis
// read as exact rationals (massmoment.PrismRotation, the leaves
// proofbound.ExactFrameLiftRound compares every lifted vertex against).
// Every prism-family build integrates its readings in plane coordinates,
// and r3 keeps L orthonormal only to rounding (docs/evaluator-design.md §5).
// The centroid and the box need no charge: an affine map carries a centroid
// to its image's centroid, and the box reads the linear functional g·L over
// the plane-coordinate body. An exactly orthonormal L changes nothing.
func chargePrismMap(body *Body, frame r3.Frame, xform r3.Transform) error {
	l, err := massmoment.PrismRotation(frame, xform)
	if err != nil {
		return err
	}
	c, err := massmoment.MapChargeOf(l)
	if err != nil {
		return err
	}
	chargeBodyMap(body, c)
	return nil
}

// chargeBodyMap is chargePrismMap's widening for an already-read charge.
func chargeBodyMap(body *Body, c massmoment.MapCharge) {
	if c == (massmoment.MapCharge{}) {
		return
	}
	if body.solid {
		v := c.VolumeOf(proofbound.MeasuredScalar(body.volume.Value.Base(), body.volume.Bound.Base()))
		body.volume.Bound = units.CubicMillimeters(v.Bound)
		body.volume.Exactness = exactnessOf(v.Bound)
	}
	a := c.AreaOf(proofbound.MeasuredScalar(body.area.Value.Base(), body.area.Bound.Base()))
	body.area.Bound = units.SquareMillimeters(a.Bound)
	body.area.Exactness = exactnessOf(a.Bound)
	for _, f := range body.Faces() {
		f.areaBound = c.AreaOf(proofbound.MeasuredScalar(f.area, f.areaBound)).Bound
	}
	for _, e := range body.Edges() {
		e.lengthBound = c.LengthBound(e.length, e.lengthBound)
	}
}

// chargePlacement widens a face-copy body's readings for its placement's own
// departure from orthonormal: the copy denotes the source face carried
// through the placement's held basis read exactly, and its area and lengths
// are the source's.
func chargePlacement(body *Body, xform r3.Transform) error {
	l, err := massmoment.PlacementRotation(xform)
	if err != nil {
		return err
	}
	c, err := massmoment.MapChargeOf(l)
	if err != nil {
		return err
	}
	chargeBodyMap(body, c)
	return nil
}
