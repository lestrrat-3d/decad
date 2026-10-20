package decad

import (
	"github.com/lestrrat-3d/decad/report"
)

// ScalarOutcome is a whole-body scalar survey's result.
type ScalarOutcome = report.ScalarOutcome

// Coverage is an undercut survey's face-membership result.
type Coverage = report.Coverage

// Assessment is a verdict against a stated survey requirement.
type Assessment = report.Assessment

// ToleranceState is a reading's verdict against the relative tolerance gate.
type ToleranceState = report.ToleranceState

// ValidityOutcome is a body's held-boundary validity verdict.
type ValidityOutcome = report.ValidityOutcome

const (
	ScalarNotEvaluated = report.ScalarNotEvaluated
	ScalarNotRequested = report.ScalarNotRequested
	ScalarUnavailable  = report.ScalarUnavailable
	ScalarUndecided    = report.ScalarUndecided
	ScalarAbsent       = report.ScalarAbsent
	ScalarMeasured     = report.ScalarMeasured
)

const (
	CoverageNotEvaluated = report.CoverageNotEvaluated
	CoverageNotRequested = report.CoverageNotRequested
	CoverageUnavailable  = report.CoverageUnavailable
	CoverageUndecided    = report.CoverageUndecided
	CoveragePartial      = report.CoveragePartial
	CoverageComplete     = report.CoverageComplete
)

const (
	AssessmentNotEvaluated = report.AssessmentNotEvaluated
	AssessmentMet          = report.AssessmentMet
	AssessmentViolated     = report.AssessmentViolated
	AssessmentUndecided    = report.AssessmentUndecided
)

const (
	ToleranceNotEvaluated = report.ToleranceNotEvaluated
	ToleranceSatisfied    = report.ToleranceSatisfied
	ToleranceExceeded     = report.ToleranceExceeded
	ToleranceUndecided    = report.ToleranceUndecided
)

const (
	ValidityNotEvaluated = report.ValidityNotEvaluated
	ValidityValid        = report.ValidityValid
	ValidityInvalid      = report.ValidityInvalid
	ValidityUndecided    = report.ValidityUndecided
)

// WallRequest records the effective wall-thickness settings.
type WallRequest = report.WallRequest

// UndercutRequest records the effective pull direction.
type UndercutRequest = report.UndercutRequest

// VerifyRequest records one verification call's effective settings.
type VerifyRequest = report.VerifyRequest

// ToleranceResult is one reading's tolerance verdict.
type ToleranceResult = report.ToleranceResult

// ScalarReading combines a scalar measurement and tolerance verdict.
type ScalarReading = report.ScalarReading

// VectorReading combines a vector measurement and tolerance verdict.
type VectorReading = report.VectorReading

// BoundsReading combines a box measurement and tolerance verdict.
type BoundsReading = report.BoundsReading

// WallResult records one body's wall survey.
type WallResult = report.WallResultOf[*Body, JointCell]

// UndercutResult records one body's undercut survey.
type UndercutResult = report.UndercutResultOf[*Body, *Face, JointCell]

// ConcaveRadiusResult records one body's concave-radius survey.
type ConcaveRadiusResult = report.ConcaveRadiusResultOf[*Body, JointCell]

// ValidityResult records one body's held-boundary verdict.
type ValidityResult = report.ValidityResultOf[*Body, JointCell]

// HeldTopology records a body's lump and void counts.
type HeldTopology = report.HeldTopology

// RegionReadings groups a solid body's volume and centroid.
type RegionReadings = report.RegionReadings

// BodyReport records one body's verification results.
type BodyReport = report.BodyReportOf[*Body, *Face, JointCell]

// Report records one verification call's results.
type Report = report.ReportOf[*Body, *Face, JointCell]
