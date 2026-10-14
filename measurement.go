package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Exactness reports whether a measured value is exact or bounded.
type Exactness int

const (
	// Exact marks a proved exactly representable result.
	Exact Exactness = iota
	// Approximate marks a result with a proved absolute error bound.
	Approximate
)

// String renders the exactness for diagnostics.
func (e Exactness) String() string {
	switch e {
	case Exact:
		return "Exact"
	case Approximate:
		return "Approximate"
	default:
		return fmt.Sprintf("Exactness(%d)", int(e))
	}
}

// Measurement is a scalar value with its exactness and absolute error bound.
type Measurement struct {
	Value     units.Value
	Exactness Exactness
	Bound     units.Value
}

// VecMeasurement is a position or direction with its exactness and bound.
type VecMeasurement struct {
	Value     r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// Box is an axis-aligned bound with its own exactness and error bound.
type Box struct {
	Min, Max  r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// ratIntervalMeasurement publishes an exact length interval [lo, hi] as its
// nearest midpoint float and an outward bound covering both ends.
func ratIntervalMeasurement(lo, hi *big.Rat) (Measurement, bool) {
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	value := sweeppath.RatFloatNearest(mid)
	if !measurement.FiniteValues(value) {
		return Measurement{}, false
	}
	held := proofarith.FloatRat(value)
	spread := proofbound.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(lo, held)), new(big.Rat).Abs(new(big.Rat).Sub(hi, held)))
	bound := proofbound.RatFloatUp(spread)
	if !measurement.FiniteValues(value, bound) {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessFromBound(bound)}, true
}

func measurementFromInternal(m measurement.Measurement) Measurement {
	return Measurement{Value: m.Value, Exactness: Exactness(m.Exactness), Bound: m.Bound}
}

func measurementToInternal(m Measurement) measurement.Measurement {
	return measurement.Measurement{Value: m.Value, Exactness: measurement.Exactness(m.Exactness), Bound: m.Bound}
}

func measurementPtrFromInternal(m *measurement.Measurement) *Measurement {
	if m == nil {
		return nil
	}
	result := measurementFromInternal(*m)
	return &result
}

func measurementPtrToInternal(m *Measurement) *measurement.Measurement {
	if m == nil {
		return nil
	}
	result := measurementToInternal(*m)
	return &result
}

func vecMeasurementToInternal(m VecMeasurement) measurement.VecMeasurement {
	return measurement.VecMeasurement{Value: m.Value, Exactness: measurement.Exactness(m.Exactness), Bound: m.Bound}
}

func vecMeasurementFromInternal(m measurement.VecMeasurement) VecMeasurement {
	return VecMeasurement{Value: m.Value, Exactness: Exactness(m.Exactness), Bound: m.Bound}
}

func vecMeasurementPtrFromInternal(m *measurement.VecMeasurement) *VecMeasurement {
	if m == nil {
		return nil
	}
	result := vecMeasurementFromInternal(*m)
	return &result
}

func vecMeasurementPtrToInternal(m *VecMeasurement) *measurement.VecMeasurement {
	if m == nil {
		return nil
	}
	result := vecMeasurementToInternal(*m)
	return &result
}

func boxPtrFromInternal(b *measurement.Box) *Box {
	if b == nil {
		return nil
	}
	result := Box{Min: b.Min, Max: b.Max, Exactness: Exactness(b.Exactness), Bound: b.Bound}
	return &result
}

func boxToInternal(b Box) measurement.Box {
	return measurement.Box{Min: b.Min, Max: b.Max, Exactness: measurement.Exactness(b.Exactness), Bound: b.Bound}
}

func boxFromInternal(b measurement.Box) Box {
	return Box{Min: b.Min, Max: b.Max, Exactness: Exactness(b.Exactness), Bound: b.Bound}
}

func boxPtrToInternal(b *Box) *measurement.Box {
	if b == nil {
		return nil
	}
	result := boxToInternal(*b)
	return &result
}

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
// prismextent.ProfileBoundaryExtremes (both consumed building Area/Bounds), and a
// walk endpoint bound that cannot be derived is the "no span" case
// freeformEndpointBounds already turned into ErrDegenerate before a
// survey2d.SegmentWalk exists to fold into a Vertex at all (boundarywalk.JunctionVertexAt,
// extrude.go). So a body that reaches this call has already had every
// free-form-derived bound proven finite; a +Inf or NaN reaching here names a
// genuinely non-finite INPUT elsewhere, which is exactly what ErrNotFinite is
// for.
func validateAnalyticBodyMeasurements(body *Body) error {
	finiteMeasurement := func(m Measurement) bool {
		return measurement.FiniteValues(m.Value.Base(), m.Bound.Base())
	}
	finiteVecMeasurement := func(m VecMeasurement) bool {
		return measurement.FiniteValues(
			m.Value.X, m.Value.Y, m.Value.Z, m.Bound.Base(),
		)
	}
	finiteBox := func(b Box) bool {
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
		{name: "volume", ok: finiteMeasurement(measurementFromInternal(body.n.Volume))},
		{name: "area", ok: finiteMeasurement(measurementFromInternal(body.n.Area))},
		{name: "centroid", ok: finiteVecMeasurement(vecMeasurementFromInternal(body.n.Centroid))},
		{name: "bounds", ok: finiteBox(boxFromInternal(body.n.Bounds))},
	}
	for _, check := range checks {
		if !check.ok {
			return fmt.Errorf(`%w: the analytic body's %s measurement is not finite`, ErrNotFinite, check.name)
		}
	}
	return nil
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
	if body.n.Solid {
		v := c.VolumeOf(proofbound.MeasuredScalar(body.n.Volume.Value.Base(), body.n.Volume.Bound.Base()))
		body.n.Volume.Bound = units.CubicMillimeters(v.Bound)
		body.n.Volume.Exactness = measurement.ExactnessOf(v.Bound)
	}
	a := c.AreaOf(proofbound.MeasuredScalar(body.n.Area.Value.Base(), body.n.Area.Bound.Base()))
	body.n.Area.Bound = units.SquareMillimeters(a.Bound)
	body.n.Area.Exactness = measurement.ExactnessOf(a.Bound)
	for _, f := range body.Faces() {
		f.n.AreaBound = c.AreaOf(proofbound.MeasuredScalar(f.n.Area, f.n.AreaBound)).Bound
	}
	for _, e := range body.Edges() {
		e.n.LengthBound = c.LengthBound(e.n.Length, e.n.LengthBound)
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
