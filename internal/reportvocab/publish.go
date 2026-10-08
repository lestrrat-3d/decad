package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// SurveyReason distinguishes a staged payload from an undecided proof.
type SurveyReason int

const (
	SurveyUndecided SurveyReason = iota
	SurveyFacetedUnsupported
	SurveyPayloadStaged
)

// ScalarSurvey is a wall or concave-radius reading before publication.
type ScalarSurvey struct {
	Reading *float64
	Bound   float64
	OK      bool
	Reason  SurveyReason
}

// UndercutSurvey carries the faces already proven to oppose the pull.
type UndercutSurvey[FaceT comparable] struct {
	Faces     []FaceT
	OK        bool
	Undecided bool
	Reason    SurveyReason
}

// SurveyPublication carries the three survey results and their local diagnostics.
type SurveyPublication[BodyT comparable, FaceT comparable, CellT any] struct {
	Wall                ScalarSurvey
	WallDiagnostics     []Diagnostic[BodyT, CellT]
	Undercut            UndercutSurvey[FaceT]
	UndercutDiagnostics []Diagnostic[BodyT, CellT]
	Radius              ScalarSurvey
	RadiusDiagnostics   []Diagnostic[BodyT, CellT]
}

// BodyPublication contains the already decided readings of one body.
type BodyPublication[BodyT comparable, FaceT comparable, CellT any] struct {
	Body                BodyT
	Solid               bool
	Sheet               bool
	Faces               []FaceT
	Status              Status
	Validity            ValidityResult[BodyT, CellT]
	Topology            HeldTopology
	Area                ScalarReading
	Bounds              BoundsReading
	Region              *RegionReadings
	Request             VerifyRequest
	Surveys             SurveyPublication[BodyT, FaceT, CellT]
	WallTolerance       ToleranceResult
	WallToleranceDiag   *Diagnostic[BodyT, CellT]
	RadiusTolerance     ToleranceResult
	RadiusToleranceDiag *Diagnostic[BodyT, CellT]
	CoreDiagnostics     []Diagnostic[BodyT, CellT]
}

// PublishBody flattens diagnostics in validity, core, wall, undercut, radius order.
func PublishBody[BodyT comparable, FaceT comparable, CellT any](in BodyPublication[BodyT, FaceT, CellT]) *BodyReport[BodyT, FaceT, CellT] {
	wall := PublishWall(in.Body, in.Solid, in.Sheet, in.Surveys.Wall, in.Surveys.WallDiagnostics,
		in.Request, in.Validity.Outcome, in.WallTolerance, in.WallToleranceDiag)
	undercut := PublishUndercut(in.Body, in.Solid, in.Sheet, in.Faces, in.Surveys.Undercut,
		in.Surveys.UndercutDiagnostics, in.Request, in.Validity.Outcome)
	radius := PublishRadius(in.Body, in.Solid, in.Sheet, in.Surveys.Radius, in.Surveys.RadiusDiagnostics,
		in.Request, in.Validity.Outcome, in.RadiusTolerance, in.RadiusToleranceDiag)
	var region *RegionReadings
	if in.Validity.Outcome == ValidityValid && in.Solid {
		region = in.Region
	}
	var diags []Diagnostic[BodyT, CellT]
	diags = append(diags, in.Validity.Diagnostics...)
	diags = append(diags, in.CoreDiagnostics...)
	diags = append(diags, wall.Diagnostics...)
	diags = append(diags, undercut.Diagnostics...)
	diags = append(diags, radius.Diagnostics...)
	return &BodyReport[BodyT, FaceT, CellT]{
		Body: in.Body, Status: in.Status, Validity: in.Validity, Topology: in.Topology,
		Area: in.Area, Bounds: in.Bounds, Region: region, Wall: wall,
		Undercut: undercut, ConcaveRadius: radius, Diagnostics: diags,
	}
}

// PublishReport assembles an already decided document report.
func PublishReport[BodyT comparable, FaceT any, CellT any](req VerifyRequest,
	bodies []*BodyReport[BodyT, FaceT, CellT], interferences []Interference[BodyT],
	clearances []Clearance[BodyT], diagnostics []Diagnostic[BodyT, CellT], status Status,
) *Report[BodyT, FaceT, CellT] {
	return &Report[BodyT, FaceT, CellT]{
		Request: req, Bodies: bodies, Interferences: interferences,
		Clearances: clearances, Diagnostics: diagnostics, Status: status,
	}
}

// ValidityEvidence distinguishes a proven sheet audit from a solid boundary audit.
type ValidityEvidence struct {
	Sheet         bool
	SheetProven   bool
	SheetViolated bool
	Clean         bool
	Built         bool
	Solid         bool
}

// PublishValidity maps the appropriate body's audit onto one verdict.
func PublishValidity[BodyT comparable, CellT any](body BodyT, ev ValidityEvidence) ValidityResult[BodyT, CellT] {
	undecided := ValidityResult[BodyT, CellT]{
		Outcome: ValidityUndecided,
		Diagnostics: []Diagnostic[BodyT, CellT]{{
			Code: DiagUndecidedValidity, Status: Suspect, Body: body, Reading: ReadingNone,
			Message: "the held boundary's validity is not decisive beyond its own proven bound",
		}},
	}
	if ev.Sheet {
		switch {
		case ev.SheetProven:
			return ValidityResult[BodyT, CellT]{Outcome: ValidityValid}
		case ev.SheetViolated:
			return ValidityResult[BodyT, CellT]{Outcome: ValidityInvalid,
				Diagnostics: []Diagnostic[BodyT, CellT]{{
					Code: DiagInvalidBody, Status: Unsound, Body: body, Reading: ReadingNone,
					Message: "the held boundary is proven not a valid sheet",
				}}}
		default:
			return undecided
		}
	}
	switch {
	case !ev.Clean:
		return ValidityResult[BodyT, CellT]{Outcome: ValidityInvalid,
			Diagnostics: []Diagnostic[BodyT, CellT]{{
				Code: DiagInvalidBody, Status: Unsound, Body: body, Reading: ReadingNone,
				Message: "the held boundary is proven not a valid solid",
			}}}
	case ev.Built && ev.Solid:
		return ValidityResult[BodyT, CellT]{Outcome: ValidityValid}
	default:
		return undecided
	}
}

// SurveyPrerequisiteDiagnostic identifies a requested survey blocked by validity or body kind.
func SurveyPrerequisiteDiagnostic[BodyT comparable, CellT any](body BodyT, sheet bool,
	survey SurveyKind) Diagnostic[BodyT, CellT] {
	msg := "the requested survey needs a proven solid, and this body's validity is not valid"
	switch {
	case survey == SurveyWall && sheet:
		msg = "the requested survey needs a proven solid, and this body is a sheet with no material for a wall question"
	case survey == SurveyConcaveRadius && sheet:
		msg = "the requested survey needs a proven solid, and this body is a sheet with no material for a concave question"
	}
	return Diagnostic[BodyT, CellT]{
		Code: DiagSurveyPrerequisite, Status: Suspect, Body: body,
		Survey: survey, Reading: ReadingNone, Message: msg,
	}
}

// PublishWall maps the wall survey and its tolerance verdict onto one result.
func PublishWall[BodyT comparable, CellT any](body BodyT, solid, sheet bool, out ScalarSurvey,
	diagnostics []Diagnostic[BodyT, CellT], req VerifyRequest, validity ValidityOutcome,
	tolerance ToleranceResult, toleranceDiag *Diagnostic[BodyT, CellT],
) WallResult[BodyT, CellT] {
	if req.Wall == nil {
		return WallResult[BodyT, CellT]{Outcome: ScalarNotRequested, Assessment: AssessmentNotEvaluated}
	}
	if validity != ValidityValid || !solid {
		return WallResult[BodyT, CellT]{
			Request: req.Wall, Outcome: ScalarUnavailable, Assessment: AssessmentUndecided,
			Diagnostics: []Diagnostic[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, SurveyWall)},
		}
	}
	res := WallResult[BodyT, CellT]{Request: req.Wall}
	switch {
	case !out.OK:
		res.Assessment = AssessmentUndecided
		res.Outcome = unavailableOrUndecided(out.Reason)
	case out.Reading == nil:
		res.Outcome = ScalarAbsent
		res.Assessment = AssessmentMet
	default:
		res.Outcome = ScalarMeasured
		res.Minimum = &ScalarReading{Measurement: LengthMeasurement(*out.Reading, out.Bound), Tolerance: tolerance}
		switch IntervalVerdict(*out.Reading, out.Bound, req.Wall.Minimum.Base()) {
		case -1:
			res.Assessment = AssessmentViolated
		case 1:
			res.Assessment = AssessmentMet
		default:
			res.Assessment = AssessmentUndecided
		}
	}
	res.Diagnostics = appendToleranceDiag(diagnostics, toleranceDiag)
	return res
}

func unavailableOrUndecided(reason SurveyReason) ScalarOutcome {
	if reason == SurveyFacetedUnsupported || reason == SurveyPayloadStaged {
		return ScalarUnavailable
	}
	return ScalarUndecided
}

func unavailableOrUndecidedCoverage(reason SurveyReason) Coverage {
	if reason == SurveyFacetedUnsupported || reason == SurveyPayloadStaged {
		return CoverageUnavailable
	}
	return CoverageUndecided
}

func appendToleranceDiag[BodyT comparable, CellT any](survey []Diagnostic[BodyT, CellT],
	toleranceDiag *Diagnostic[BodyT, CellT]) []Diagnostic[BodyT, CellT] {
	var diags []Diagnostic[BodyT, CellT]
	diags = append(diags, survey...)
	if toleranceDiag != nil {
		diags = append(diags, *toleranceDiag)
	}
	return diags
}

// OrderedFaces returns confirmed faces in topology order, preserving nil versus empty.
func OrderedFaces[FaceT comparable](all, confirmed []FaceT) []FaceT {
	if confirmed == nil {
		return nil
	}
	want := make(map[FaceT]struct{}, len(confirmed))
	for _, face := range confirmed {
		want[face] = struct{}{}
	}
	ordered := make([]FaceT, 0, len(confirmed))
	for _, face := range all {
		if _, ok := want[face]; ok {
			ordered = append(ordered, face)
		}
	}
	return ordered
}

// PublishUndercut maps proven opposing faces and survey coverage onto one result.
func PublishUndercut[BodyT comparable, FaceT comparable, CellT any](body BodyT, solid, sheet bool,
	all []FaceT, out UndercutSurvey[FaceT], diagnostics []Diagnostic[BodyT, CellT],
	req VerifyRequest, validity ValidityOutcome,
) UndercutResult[BodyT, FaceT, CellT] {
	if req.Undercut == nil {
		return UndercutResult[BodyT, FaceT, CellT]{Coverage: CoverageNotRequested, Assessment: AssessmentNotEvaluated}
	}
	if validity != ValidityValid || (!solid && !sheet) {
		return UndercutResult[BodyT, FaceT, CellT]{
			Request: req.Undercut, Coverage: CoverageUnavailable, Assessment: AssessmentUndecided,
			Diagnostics: []Diagnostic[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, SurveyUndercut)},
		}
	}
	res := UndercutResult[BodyT, FaceT, CellT]{
		Request: req.Undercut, Faces: OrderedFaces(all, out.Faces), Diagnostics: diagnostics,
	}
	switch {
	case !out.OK:
		res.Assessment = AssessmentUndecided
		res.Coverage = unavailableOrUndecidedCoverage(out.Reason)
	case out.Undecided:
		if len(out.Faces) > 0 {
			res.Coverage = CoveragePartial
			res.Assessment = AssessmentViolated
		} else {
			res.Coverage = CoverageUndecided
			res.Assessment = AssessmentUndecided
		}
	default:
		res.Coverage = CoverageComplete
		res.Assessment = AssessmentMet
		if len(out.Faces) > 0 {
			res.Assessment = AssessmentViolated
		}
	}
	return res
}

// PublishRadius maps the concave-radius survey and its tolerance verdict onto one result.
func PublishRadius[BodyT comparable, CellT any](body BodyT, solid, sheet bool, out ScalarSurvey,
	diagnostics []Diagnostic[BodyT, CellT], req VerifyRequest, validity ValidityOutcome,
	tolerance ToleranceResult, toleranceDiag *Diagnostic[BodyT, CellT],
) ConcaveRadiusResult[BodyT, CellT] {
	if !req.ConcaveRadius {
		return ConcaveRadiusResult[BodyT, CellT]{Outcome: ScalarNotRequested}
	}
	if validity != ValidityValid || !solid {
		return ConcaveRadiusResult[BodyT, CellT]{
			Outcome:     ScalarUnavailable,
			Diagnostics: []Diagnostic[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, SurveyConcaveRadius)},
		}
	}
	res := ConcaveRadiusResult[BodyT, CellT]{}
	switch {
	case !out.OK:
		res.Outcome = unavailableOrUndecided(out.Reason)
	case out.Reading == nil:
		res.Outcome = ScalarAbsent
	default:
		res.Outcome = ScalarMeasured
		res.Minimum = &ScalarReading{Measurement: LengthMeasurement(*out.Reading, out.Bound), Tolerance: tolerance}
	}
	res.Diagnostics = appendToleranceDiag(diagnostics, toleranceDiag)
	return res
}

// LengthMeasurement publishes a survey's held millimetres with its proven bound.
func LengthMeasurement(mm, bound float64) measurement.Measurement {
	exactness := measurement.Exact
	if bound != 0 {
		exactness = measurement.Approximate
	}
	return measurement.Measurement{
		Value: units.Millimeters(mm), Exactness: exactness, Bound: units.Millimeters(bound),
	}
}

// IntervalVerdict compares a proven interval with a requested minimum.
func IntervalVerdict(value, bound, tool float64) int {
	if value+bound < tool {
		return -1
	}
	if value-bound >= tool {
		return 1
	}
	return 0
}
