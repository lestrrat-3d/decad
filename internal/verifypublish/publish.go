package verifypublish

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/report"
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
	WallDiagnostics     []report.DiagnosticOf[BodyT, CellT]
	Undercut            UndercutSurvey[FaceT]
	UndercutDiagnostics []report.DiagnosticOf[BodyT, CellT]
	Radius              ScalarSurvey
	RadiusDiagnostics   []report.DiagnosticOf[BodyT, CellT]
}

// BodyPublication contains the already decided readings of one body.
type BodyPublication[BodyT comparable, FaceT comparable, CellT any] struct {
	Body                BodyT
	Solid               bool
	Sheet               bool
	Faces               []FaceT
	Status              report.Status
	Validity            report.ValidityResultOf[BodyT, CellT]
	Topology            report.HeldTopology
	Area                report.ScalarReading
	Bounds              report.BoundsReading
	Region              *report.RegionReadings
	Request             report.VerifyRequest
	Surveys             SurveyPublication[BodyT, FaceT, CellT]
	WallTolerance       report.ToleranceResult
	WallToleranceDiag   *report.DiagnosticOf[BodyT, CellT]
	RadiusTolerance     report.ToleranceResult
	RadiusToleranceDiag *report.DiagnosticOf[BodyT, CellT]
	CoreDiagnostics     []report.DiagnosticOf[BodyT, CellT]
}

// PublishBody flattens diagnostics in validity, core, wall, undercut, radius order.
func PublishBody[BodyT comparable, FaceT comparable, CellT any](in BodyPublication[BodyT, FaceT, CellT]) *report.BodyReportOf[BodyT, FaceT, CellT] {
	wall := PublishWall(in.Body, in.Solid, in.Sheet, in.Surveys.Wall, in.Surveys.WallDiagnostics,
		in.Request, in.Validity.Outcome, in.WallTolerance, in.WallToleranceDiag)
	undercut := PublishUndercut(in.Body, in.Solid, in.Sheet, in.Faces, in.Surveys.Undercut,
		in.Surveys.UndercutDiagnostics, in.Request, in.Validity.Outcome)
	radius := PublishRadius(in.Body, in.Solid, in.Sheet, in.Surveys.Radius, in.Surveys.RadiusDiagnostics,
		in.Request, in.Validity.Outcome, in.RadiusTolerance, in.RadiusToleranceDiag)
	var region *report.RegionReadings
	if in.Validity.Outcome == report.ValidityValid && in.Solid {
		region = in.Region
	}
	var diags []report.DiagnosticOf[BodyT, CellT]
	diags = append(diags, in.Validity.Diagnostics...)
	diags = append(diags, in.CoreDiagnostics...)
	diags = append(diags, wall.Diagnostics...)
	diags = append(diags, undercut.Diagnostics...)
	diags = append(diags, radius.Diagnostics...)
	return &report.BodyReportOf[BodyT, FaceT, CellT]{
		Body: in.Body, Status: in.Status, Validity: in.Validity, Topology: in.Topology,
		Area: in.Area, Bounds: in.Bounds, Region: region, Wall: wall,
		Undercut: undercut, ConcaveRadius: radius, Diagnostics: diags,
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
func PublishValidity[BodyT comparable, CellT any](body BodyT, ev ValidityEvidence) report.ValidityResultOf[BodyT, CellT] {
	undecided := report.ValidityResultOf[BodyT, CellT]{
		Outcome: report.ValidityUndecided,
		Diagnostics: []report.DiagnosticOf[BodyT, CellT]{{
			Code: report.DiagUndecidedValidity, Status: report.Suspect, Body: body, Reading: report.ReadingNone,
			Message: "the held boundary's validity is not decisive beyond its own proven bound",
		}},
	}
	if ev.Sheet {
		switch {
		case ev.SheetProven:
			return report.ValidityResultOf[BodyT, CellT]{Outcome: report.ValidityValid}
		case ev.SheetViolated:
			return report.ValidityResultOf[BodyT, CellT]{Outcome: report.ValidityInvalid,
				Diagnostics: []report.DiagnosticOf[BodyT, CellT]{{
					Code: report.DiagInvalidBody, Status: report.Unsound, Body: body, Reading: report.ReadingNone,
					Message: "the held boundary is proven not a valid sheet",
				}}}
		default:
			return undecided
		}
	}
	switch {
	case !ev.Clean:
		return report.ValidityResultOf[BodyT, CellT]{Outcome: report.ValidityInvalid,
			Diagnostics: []report.DiagnosticOf[BodyT, CellT]{{
				Code: report.DiagInvalidBody, Status: report.Unsound, Body: body, Reading: report.ReadingNone,
				Message: "the held boundary is proven not a valid solid",
			}}}
	case ev.Built && ev.Solid:
		return report.ValidityResultOf[BodyT, CellT]{Outcome: report.ValidityValid}
	default:
		return undecided
	}
}

// SurveyPrerequisiteDiagnostic identifies a requested survey blocked by validity or body kind.
func SurveyPrerequisiteDiagnostic[BodyT comparable, CellT any](body BodyT, sheet bool,
	survey report.SurveyKind) report.DiagnosticOf[BodyT, CellT] {
	msg := "the requested survey needs a proven solid, and this body's validity is not valid"
	switch {
	case survey == report.SurveyWall && sheet:
		msg = "the requested survey needs a proven solid, and this body is a sheet with no material for a wall question"
	case survey == report.SurveyConcaveRadius && sheet:
		msg = "the requested survey needs a proven solid, and this body is a sheet with no material for a concave question"
	}
	return report.DiagnosticOf[BodyT, CellT]{
		Code: report.DiagSurveyPrerequisite, Status: report.Suspect, Body: body,
		Survey: survey, Reading: report.ReadingNone, Message: msg,
	}
}

// PublishWall maps the wall survey and its tolerance verdict onto one result.
func PublishWall[BodyT comparable, CellT any](body BodyT, solid, sheet bool, out ScalarSurvey,
	diagnostics []report.DiagnosticOf[BodyT, CellT], req report.VerifyRequest, validity report.ValidityOutcome,
	tolerance report.ToleranceResult, toleranceDiag *report.DiagnosticOf[BodyT, CellT],
) report.WallResultOf[BodyT, CellT] {
	if req.Wall == nil {
		return report.WallResultOf[BodyT, CellT]{Outcome: report.ScalarNotRequested, Assessment: report.AssessmentNotEvaluated}
	}
	if validity != report.ValidityValid || !solid {
		return report.WallResultOf[BodyT, CellT]{
			Request: req.Wall, Outcome: report.ScalarUnavailable, Assessment: report.AssessmentUndecided,
			Diagnostics: []report.DiagnosticOf[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, report.SurveyWall)},
		}
	}
	res := report.WallResultOf[BodyT, CellT]{Request: req.Wall}
	switch {
	case !out.OK:
		res.Assessment = report.AssessmentUndecided
		res.Outcome = unavailableOrUndecided(out.Reason)
	case out.Reading == nil:
		res.Outcome = report.ScalarAbsent
		res.Assessment = report.AssessmentMet
	default:
		res.Outcome = report.ScalarMeasured
		res.Minimum = &report.ScalarReading{Measurement: LengthMeasurement(*out.Reading, out.Bound), Tolerance: tolerance}
		switch IntervalVerdict(*out.Reading, out.Bound, req.Wall.Minimum.Base()) {
		case -1:
			res.Assessment = report.AssessmentViolated
		case 1:
			res.Assessment = report.AssessmentMet
		default:
			res.Assessment = report.AssessmentUndecided
		}
	}
	res.Diagnostics = appendToleranceDiag(diagnostics, toleranceDiag)
	return res
}

func unavailableOrUndecided(reason SurveyReason) report.ScalarOutcome {
	if reason == SurveyFacetedUnsupported || reason == SurveyPayloadStaged {
		return report.ScalarUnavailable
	}
	return report.ScalarUndecided
}

func unavailableOrUndecidedCoverage(reason SurveyReason) report.Coverage {
	if reason == SurveyFacetedUnsupported || reason == SurveyPayloadStaged {
		return report.CoverageUnavailable
	}
	return report.CoverageUndecided
}

func appendToleranceDiag[BodyT comparable, CellT any](survey []report.DiagnosticOf[BodyT, CellT],
	toleranceDiag *report.DiagnosticOf[BodyT, CellT]) []report.DiagnosticOf[BodyT, CellT] {
	var diags []report.DiagnosticOf[BodyT, CellT]
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
	all []FaceT, out UndercutSurvey[FaceT], diagnostics []report.DiagnosticOf[BodyT, CellT],
	req report.VerifyRequest, validity report.ValidityOutcome,
) report.UndercutResultOf[BodyT, FaceT, CellT] {
	if req.Undercut == nil {
		return report.UndercutResultOf[BodyT, FaceT, CellT]{Coverage: report.CoverageNotRequested, Assessment: report.AssessmentNotEvaluated}
	}
	if validity != report.ValidityValid || (!solid && !sheet) {
		return report.UndercutResultOf[BodyT, FaceT, CellT]{
			Request: req.Undercut, Coverage: report.CoverageUnavailable, Assessment: report.AssessmentUndecided,
			Diagnostics: []report.DiagnosticOf[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, report.SurveyUndercut)},
		}
	}
	res := report.UndercutResultOf[BodyT, FaceT, CellT]{
		Request: req.Undercut, Faces: OrderedFaces(all, out.Faces), Diagnostics: diagnostics,
	}
	switch {
	case !out.OK:
		res.Assessment = report.AssessmentUndecided
		res.Coverage = unavailableOrUndecidedCoverage(out.Reason)
	case out.Undecided:
		if len(out.Faces) > 0 {
			res.Coverage = report.CoveragePartial
			res.Assessment = report.AssessmentViolated
		} else {
			res.Coverage = report.CoverageUndecided
			res.Assessment = report.AssessmentUndecided
		}
	default:
		res.Coverage = report.CoverageComplete
		res.Assessment = report.AssessmentMet
		if len(out.Faces) > 0 {
			res.Assessment = report.AssessmentViolated
		}
	}
	return res
}

// PublishRadius maps the concave-radius survey and its tolerance verdict onto one result.
func PublishRadius[BodyT comparable, CellT any](body BodyT, solid, sheet bool, out ScalarSurvey,
	diagnostics []report.DiagnosticOf[BodyT, CellT], req report.VerifyRequest, validity report.ValidityOutcome,
	tolerance report.ToleranceResult, toleranceDiag *report.DiagnosticOf[BodyT, CellT],
) report.ConcaveRadiusResultOf[BodyT, CellT] {
	if !req.ConcaveRadius {
		return report.ConcaveRadiusResultOf[BodyT, CellT]{Outcome: report.ScalarNotRequested}
	}
	if validity != report.ValidityValid || !solid {
		return report.ConcaveRadiusResultOf[BodyT, CellT]{
			Outcome:     report.ScalarUnavailable,
			Diagnostics: []report.DiagnosticOf[BodyT, CellT]{SurveyPrerequisiteDiagnostic[BodyT, CellT](body, sheet, report.SurveyConcaveRadius)},
		}
	}
	res := report.ConcaveRadiusResultOf[BodyT, CellT]{}
	switch {
	case !out.OK:
		res.Outcome = unavailableOrUndecided(out.Reason)
	case out.Reading == nil:
		res.Outcome = report.ScalarAbsent
	default:
		res.Outcome = report.ScalarMeasured
		res.Minimum = &report.ScalarReading{Measurement: LengthMeasurement(*out.Reading, out.Bound), Tolerance: tolerance}
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
