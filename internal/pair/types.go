// Package pair holds result types shared by box and planar contact proofs.
package pair

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
