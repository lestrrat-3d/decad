package decad

import "github.com/lestrrat-3d/decad/internal/reportvocab"

const (
	tokenNotEvaluated = "not_evaluated"
	tokenUndecided    = "undecided"
)

// ScalarOutcome is a whole-body scalar survey's result.
type ScalarOutcome = reportvocab.ScalarOutcome

// Coverage is an undercut survey's face-membership result.
type Coverage = reportvocab.Coverage

// Assessment is a verdict against a stated survey requirement.
type Assessment = reportvocab.Assessment

// ToleranceState is a reading's verdict against the relative tolerance gate.
type ToleranceState = reportvocab.ToleranceState

// ValidityOutcome is a body's held-boundary validity verdict.
type ValidityOutcome = reportvocab.ValidityOutcome

const (
	ScalarNotEvaluated     = reportvocab.ScalarNotEvaluated
	ScalarNotRequested     = reportvocab.ScalarNotRequested
	ScalarUnavailable      = reportvocab.ScalarUnavailable
	ScalarUndecided        = reportvocab.ScalarUndecided
	ScalarAbsent           = reportvocab.ScalarAbsent
	ScalarMeasured         = reportvocab.ScalarMeasured
	CoverageNotEvaluated   = reportvocab.CoverageNotEvaluated
	CoverageNotRequested   = reportvocab.CoverageNotRequested
	CoverageUnavailable    = reportvocab.CoverageUnavailable
	CoverageUndecided      = reportvocab.CoverageUndecided
	CoveragePartial        = reportvocab.CoveragePartial
	CoverageComplete       = reportvocab.CoverageComplete
	AssessmentNotEvaluated = reportvocab.AssessmentNotEvaluated
	AssessmentMet          = reportvocab.AssessmentMet
	AssessmentViolated     = reportvocab.AssessmentViolated
	AssessmentUndecided    = reportvocab.AssessmentUndecided
	ToleranceNotEvaluated  = reportvocab.ToleranceNotEvaluated
	ToleranceSatisfied     = reportvocab.ToleranceSatisfied
	ToleranceExceeded      = reportvocab.ToleranceExceeded
	ToleranceUndecided     = reportvocab.ToleranceUndecided
	ValidityNotEvaluated   = reportvocab.ValidityNotEvaluated
	ValidityValid          = reportvocab.ValidityValid
	ValidityInvalid        = reportvocab.ValidityInvalid
	ValidityUndecided      = reportvocab.ValidityUndecided
)

// WallRequest records the effective wall-thickness settings.
type WallRequest = reportvocab.WallRequest

// UndercutRequest records the effective pull direction.
type UndercutRequest = reportvocab.UndercutRequest

// VerifyRequest records one verification call's effective settings.
type VerifyRequest = reportvocab.VerifyRequest

// ToleranceResult is one reading's tolerance verdict.
type ToleranceResult = reportvocab.ToleranceResult

// ScalarReading combines a scalar measurement and tolerance verdict.
type ScalarReading = reportvocab.ScalarReading

// VectorReading combines a vector measurement and tolerance verdict.
type VectorReading = reportvocab.VectorReading

// BoundsReading combines a box measurement and tolerance verdict.
type BoundsReading = reportvocab.BoundsReading

// WallResult records one body's wall survey.
type WallResult = reportvocab.WallResult[*Body, JointCell]

// UndercutResult records one body's undercut survey.
type UndercutResult = reportvocab.UndercutResult[*Body, *Face, JointCell]

// ConcaveRadiusResult records one body's concave-radius survey.
type ConcaveRadiusResult = reportvocab.ConcaveRadiusResult[*Body, JointCell]

// ValidityResult records one body's held-boundary verdict.
type ValidityResult = reportvocab.ValidityResult[*Body, JointCell]

// HeldTopology records a body's lump and void counts.
type HeldTopology = reportvocab.HeldTopology

// RegionReadings groups a solid body's volume and centroid.
type RegionReadings = reportvocab.RegionReadings

// BodyReport records one body's verification results.
type BodyReport = reportvocab.BodyReport[*Body, *Face, JointCell]

// Report records one verification call's results.
type Report = reportvocab.Report[*Body, *Face, JointCell]
