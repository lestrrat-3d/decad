package verifypublish

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/units"
)

// WallDiagnostics records an unanswered wall survey or a proven comparison
// with the requested tool. The numeric tolerance gate runs separately.
func WallDiagnostics[BodyT comparable, CellT any](body BodyT, payload any, out ScalarSurvey,
	tool units.Value, toolMM float64,
) []report.DiagnosticOf[BodyT, CellT] {
	if !out.OK {
		return []report.DiagnosticOf[BodyT, CellT]{surveyRefusalDiagnostic[BodyT, CellT](
			body, payload, report.SurveyWall, out.Reason, report.DiagUndecidedWall,
			"the wall survey could neither answer nor prove no wall exists",
			"facetedPayload wall survey support is not implemented; use an analytic body or wait for faceted wall support",
			"wall")}
	}
	if out.Reading == nil {
		return nil
	}
	m := LengthMeasurement(*out.Reading, out.Bound)
	var code report.DiagnosticCode
	var status report.Status
	var message string
	switch IntervalVerdict(*out.Reading, out.Bound, toolMM) {
	case -1:
		code, status = report.DiagWallTooThin, report.Violating
		message = "the minimum wall thickness is proven below the tool"
	case 0:
		code, status = report.DiagUndecidedWall, report.Suspect
		message = "the minimum wall thickness interval straddles the tool"
	default:
		return nil
	}
	return []report.DiagnosticOf[BodyT, CellT]{{
		Code: code, Status: status, Body: body, Survey: report.SurveyWall,
		Reading: report.ReadingWall, Observed: &m, Required: &tool, Message: message,
	}}
}

// UndercutDiagnostics records confirmed opposing faces and incomplete pull
// coverage as separate findings, in that order.
func UndercutDiagnostics[BodyT comparable, FaceT comparable, CellT any](body BodyT,
	payload any, out UndercutSurvey[FaceT],
) []report.DiagnosticOf[BodyT, CellT] {
	var diags []report.DiagnosticOf[BodyT, CellT]
	if out.OK && len(out.Faces) > 0 {
		diags = append(diags, report.DiagnosticOf[BodyT, CellT]{
			Code: report.DiagUndercut, Status: report.Violating, Body: body,
			Survey: report.SurveyUndercut, Reading: report.ReadingNone,
			Message: "a face is a proven undercut against the pull",
		})
	}
	if !out.OK || out.Undecided {
		diags = append(diags, surveyRefusalDiagnostic[BodyT, CellT](
			body, payload, report.SurveyUndercut, out.Reason, report.DiagUndecidedUndercut,
			"the pull survey could neither prove nor exclude an undercut",
			"facetedPayload pull survey support is not implemented; use an analytic body or wait for faceted undercut support",
			"pull"))
	}
	return diags
}

// RadiusDiagnostics records a concave-radius survey that could not answer.
func RadiusDiagnostics[BodyT comparable, CellT any](body BodyT, payload any,
	out ScalarSurvey, ok bool,
) []report.DiagnosticOf[BodyT, CellT] {
	if ok && out.OK {
		return nil
	}
	return []report.DiagnosticOf[BodyT, CellT]{surveyRefusalDiagnostic[BodyT, CellT](
		body, payload, report.SurveyConcaveRadius, out.Reason, report.DiagUndecidedMinRadius,
		"the concave-radius survey could neither measure nor exclude a concave feature",
		"facetedPayload concave-radius survey support is not implemented; use an analytic body or wait for faceted radius support",
		"concave-radius")}
}

// surveyRefusalDiagnostic distinguishes an undecided proof from a payload
// whose survey has not been implemented.
func surveyRefusalDiagnostic[BodyT comparable, CellT any](body BodyT, payload any,
	survey report.SurveyKind, reason SurveyReason, undecidedCode report.DiagnosticCode,
	undecidedMessage, facetedMessage, surveyNoun string,
) report.DiagnosticOf[BodyT, CellT] {
	code, message := undecidedCode, undecidedMessage
	switch reason {
	case SurveyFacetedUnsupported:
		code, message = report.DiagUnsupportedSurveyPayload, facetedMessage
	case SurveyPayloadStaged:
		class := payloadClassName(payload)
		code = report.DiagUnsupportedSurveyPayload
		message = fmt.Sprintf(
			"%s %s survey support is not implemented; use an analytic body or wait for wider %s survey support",
			class, surveyNoun, class)
	}
	return report.DiagnosticOf[BodyT, CellT]{
		Code: code, Status: report.Suspect, Body: body, Survey: survey,
		Reading: report.ReadingNone, Message: message,
	}
}

// payloadClassName names a payload class in a staged-survey message: the class
// the payload states through SurveyClass, else its Go type name.
func payloadClassName(payload any) string {
	if named, ok := payload.(interface{ SurveyClass() string }); ok {
		return named.SurveyClass()
	}
	name := fmt.Sprintf("%T", payload)
	if i := strings.LastIndexByte(name, '.'); i >= 0 {
		return name[i+1:]
	}
	return name
}
