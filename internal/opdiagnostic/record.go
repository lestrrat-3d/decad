// Package opdiagnostic stores bounded, immutable summaries from modeling calls.
package opdiagnostic

// Code names a diagnostic reason without depending on the public root package.
type Code int

const (
	CodeUnknown Code = iota
	CodeGapShiftApplied
	CodeChainAllowanceApplied
	CodeStitchMergeApplied
	CodeEmbeddingReachApplied
)

// ChangeMechanism names the accepted geometric adjustment.
type ChangeMechanism int

const (
	ChangeUnknown ChangeMechanism = iota
	ChangeGapShift
	ChangeStitchMerge
	ChangeEmbeddingReach
)

// Record is the private plain-data form of one operation diagnostic.
type Record struct {
	Operation          int
	PublicOperation    int
	Code               Code
	Operand            int
	Change             ChangeMechanism
	DisplacementBound  float64
	AllowedReach       float64
	TouchedBound       float64
	PairChordLimit     float64
	ChainAllowance     *float64
	AutomaticAllowance bool
}
