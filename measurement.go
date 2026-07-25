package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Exactness reports how far a computed result can be trusted: whether the
// number IS the truth, or a bounded numerical approximation whose error the
// accompanying Bound holds (docs/api-design.md §5.3). Every measurement
// the API returns carries one, from the first commit — that is what makes an
// exact-kernel future monotonic rather than breaking: callers already branch
// on Approximate, and under an exact evaluator that branch stops being taken.
type Exactness int

const (
	// Exact marks a proved exactly representable result: the number is the
	// truth, and the Bound beside it is zero.
	Exact Exactness = iota
	// Approximate marks a bounded numerical or tessellation-derived result:
	// the Bound beside it holds the absolute error the evaluator proves.
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

// Measurement is a scalar quantity plus how far it can be trusted. Value is a
// typed quantity — never a naked float (docs/api-design.md §5.1) — and Bound
// is the absolute error bound the evaluator proves, of the same Kind as
// Value: the error bound on a volume is a volume. Bound is zero when the
// measurement is Exact.
type Measurement struct {
	Value     units.Value
	Exactness Exactness
	Bound     units.Value
}

// VecMeasurement is a computed coordinate or direction, with how far it can
// be trusted. It is what the API returns wherever the evaluator computes a
// vector: a computed vector never crosses the API as a bare r3.Vec.
//
// Value is a position — a length in millimetres, the docs/api-design.md §5.2
// carve-out — or a unit direction, which is dimensionless and carries no unit
// at all. Bound follows Value: it is the radius of the ball around Value the
// true vector is proven to lie in, so its Kind is Length for a position and
// Dimensionless for a direction (the deviation of a unit vector from the true
// unit vector). Bound is zero when the measurement is Exact.
type VecMeasurement struct {
	Value     r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// Box is an axis-aligned bounding box, as tight as the evaluator can prove —
// and it reports its own exactness: a box around a curved body produced by a
// tessellation-backed boolean is bounded by faceted faces and is therefore
// not tight, and says so. Min and Max are positions, lengths in millimetres
// (docs/api-design.md §5.2); Bound is the absolute error bound on them, of
// Kind Length, zero when the box is Exact.
type Box struct {
	Min, Max  r3.Vec
	Exactness Exactness
	Bound     units.Value
}

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
