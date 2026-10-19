// Package pair holds result types shared by box and planar contact proofs.
package pair

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// Relation is the proven relation of two admitted occupied sets.
type Relation uint8

const (
	Undecided Relation = iota
	Separated
	Touching
	Overlapping
)

// Reason identifies a missing gap or manifold certificate.
type Reason uint8

const (
	NoReason Reason = iota
	NoGapProof
	AmbiguousFeature
	PointTooCoarse
	PayloadUnsupported
	NoNormalProof
)

// ScalarReading encloses a length in millimetres.
type ScalarReading struct {
	ValueMM, BoundMM float64
}

// Measurement publishes the reading as a length, Exact only for a zero bound.
func (r ScalarReading) Measurement() measurement.Measurement {
	return measurement.Measurement{Value: units.Millimeters(r.ValueMM),
		Bound: units.Millimeters(r.BoundMM), Exactness: measurement.ExactnessOf(r.BoundMM)}
}
