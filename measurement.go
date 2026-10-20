package decad

import (
	"github.com/lestrrat-3d/decad/report"
)

// Exactness reports whether a measured value is exact or bounded.
type Exactness = report.Exactness

const (
	Exact       = report.Exact
	Approximate = report.Approximate
)

// Measurement is a scalar value with its exactness and absolute error bound.
type Measurement = report.Measurement

// VecMeasurement is a position or direction with its exactness and bound.
type VecMeasurement = report.VecMeasurement

// Box is an axis-aligned bound with its own exactness and error bound.
type Box = report.Box
