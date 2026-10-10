package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
type WallRequest struct {
	Minimum        units.Value
	DraftAllowance units.Value
}

// UndercutRequest records the effective pull direction.
type UndercutRequest struct {
	PullDirection r3.Vec
}

// VerifyRequest records one verification call's effective settings.
type VerifyRequest struct {
	RelativeTolerance units.Value
	Wall              *WallRequest
	Undercut          *UndercutRequest
	ConcaveRadius     bool
	Clearances        bool
}

// ToleranceResult is one reading's tolerance verdict.
type ToleranceResult = reportvocab.ToleranceResult

// ScalarReading combines a scalar measurement and tolerance verdict.
type ScalarReading struct {
	Measurement
	Tolerance ToleranceResult
}

// VectorReading combines a vector measurement and tolerance verdict.
type VectorReading struct {
	VecMeasurement
	Tolerance ToleranceResult
}

// BoundsReading combines a box measurement and tolerance verdict.
type BoundsReading struct {
	Box
	Tolerance ToleranceResult
}

func scalarReadingFromInternal(in reportvocab.ScalarReading) ScalarReading {
	return ScalarReading{Measurement: measurementFromInternal(in.Measurement), Tolerance: in.Tolerance}
}

func scalarReadingToInternal(in ScalarReading) reportvocab.ScalarReading {
	return reportvocab.ScalarReading{Measurement: measurementToInternal(in.Measurement), Tolerance: in.Tolerance}
}

func scalarReadingPtrFromInternal(in *reportvocab.ScalarReading) *ScalarReading {
	if in == nil {
		return nil
	}
	out := scalarReadingFromInternal(*in)
	return &out
}

func scalarReadingPtrToInternal(in *ScalarReading) *reportvocab.ScalarReading {
	if in == nil {
		return nil
	}
	out := scalarReadingToInternal(*in)
	return &out
}

func vectorReadingFromInternal(in reportvocab.VectorReading) VectorReading {
	return VectorReading{VecMeasurement: vecMeasurementFromInternal(in.VecMeasurement), Tolerance: in.Tolerance}
}

func vectorReadingToInternal(in VectorReading) reportvocab.VectorReading {
	return reportvocab.VectorReading{
		VecMeasurement: vecMeasurementToInternal(in.VecMeasurement), Tolerance: in.Tolerance,
	}
}

func boundsReadingFromInternal(in reportvocab.BoundsReading) BoundsReading {
	return BoundsReading{Box: boxFromInternal(in.Box), Tolerance: in.Tolerance}
}

func boundsReadingToInternal(in BoundsReading) reportvocab.BoundsReading {
	return reportvocab.BoundsReading{Box: boxToInternal(in.Box), Tolerance: in.Tolerance}
}

// WallResult records one body's wall survey.
type WallResult struct {
	Request     *WallRequest
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic
}

// UndercutResult records one body's undercut survey.
type UndercutResult struct {
	Request     *UndercutRequest
	Coverage    Coverage
	Faces       []*Face
	Assessment  Assessment
	Diagnostics []Diagnostic
}

// ConcaveRadiusResult records one body's concave-radius survey.
type ConcaveRadiusResult struct {
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Diagnostics []Diagnostic
}

// ValidityResult records one body's held-boundary verdict.
type ValidityResult struct {
	Outcome     ValidityOutcome
	Diagnostics []Diagnostic
}

func validityResultFromInternal(in reportvocab.ValidityResult[*Body, JointCell]) ValidityResult {
	return ValidityResult{Outcome: in.Outcome, Diagnostics: diagnosticsFromInternal(in.Diagnostics)}
}

func validityResultToInternal(in ValidityResult) reportvocab.ValidityResult[*Body, JointCell] {
	return reportvocab.ValidityResult[*Body, JointCell]{
		Outcome: in.Outcome, Diagnostics: diagnosticsToInternal(in.Diagnostics),
	}
}

func concaveRadiusResultFromInternal(in reportvocab.ConcaveRadiusResult[*Body, JointCell]) ConcaveRadiusResult {
	return ConcaveRadiusResult{
		Outcome: in.Outcome, Minimum: scalarReadingPtrFromInternal(in.Minimum),
		Diagnostics: diagnosticsFromInternal(in.Diagnostics),
	}
}

// HeldTopology records a body's lump and void counts.
type HeldTopology = reportvocab.HeldTopology

// RegionReadings groups a solid body's volume and centroid.
type RegionReadings struct {
	Volume   ScalarReading
	Centroid VectorReading
}

func regionReadingsFromInternal(in *reportvocab.RegionReadings) *RegionReadings {
	if in == nil {
		return nil
	}
	return &RegionReadings{
		Volume: scalarReadingFromInternal(in.Volume), Centroid: vectorReadingFromInternal(in.Centroid),
	}
}

func regionReadingsToInternal(in *RegionReadings) *reportvocab.RegionReadings {
	if in == nil {
		return nil
	}
	return &reportvocab.RegionReadings{
		Volume: scalarReadingToInternal(in.Volume), Centroid: vectorReadingToInternal(in.Centroid),
	}
}

// BodyReport records one body's verification results.
type BodyReport struct {
	Body          *Body
	Status        Status
	Validity      ValidityResult
	Topology      HeldTopology
	Area          ScalarReading
	Bounds        BoundsReading
	Region        *RegionReadings
	Wall          WallResult
	Undercut      UndercutResult
	ConcaveRadius ConcaveRadiusResult
	Diagnostics   []Diagnostic
}

// Report records one verification call's results.
type Report struct {
	Request       VerifyRequest
	Bodies        []*BodyReport
	Interferences []Interference
	Clearances    []Clearance
	Diagnostics   []Diagnostic
	Status        Status
}

// Passed reports whether the whole report is Sound for its effective request.
func (r *Report) Passed() bool { return r != nil && r.Status == Sound }

// ForBody returns the report's existing body row by exact pointer identity.
func (r *Report) ForBody(body *Body) (*BodyReport, error) {
	if r == nil || body == nil {
		return nil, fmt.Errorf("%w: a nil report or body names no lookup", ErrDegenerate)
	}
	for _, row := range r.Bodies {
		if row.Body == body {
			return row, nil
		}
	}
	return nil, ErrBodyReportNotFound
}
