package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
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
