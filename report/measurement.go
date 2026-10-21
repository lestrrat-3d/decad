package report

import (
	"fmt"

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
type Measurement = struct {
	Value     units.Value
	Exactness Exactness
	Bound     units.Value
}

// VecMeasurement is a position or direction with its exactness and bound.
type VecMeasurement = struct {
	Value     r3.Vec
	Exactness Exactness
	Bound     units.Value
}

// Box is an axis-aligned bound with its own exactness and error bound.
type Box = struct {
	Min, Max  r3.Vec
	Exactness Exactness
	Bound     units.Value
}
