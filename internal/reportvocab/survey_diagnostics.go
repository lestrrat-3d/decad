package reportvocab

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/units"
)

// WallDiagnostics records an unanswered wall survey or a proven comparison
// with the requested tool. The numeric tolerance gate runs separately.
func WallDiagnostics[BodyT comparable, CellT any](body BodyT, payload any, out ScalarSurvey,
	tool units.Value, toolMM float64,
) []Diagnostic[BodyT, CellT] {
	if !out.OK {
		return []Diagnostic[BodyT, CellT]{surveyRefusalDiagnostic[BodyT, CellT](
			body, payload, SurveyWall, out.Reason, DiagUndecidedWall,
			"the wall survey could neither answer nor prove no wall exists",
			"facetedPayload wall survey support is not implemented; use an analytic body or wait for faceted wall support",
			"wall")}
	}
	if out.Reading == nil {
		return nil
	}
	m := LengthMeasurement(*out.Reading, out.Bound)
	var code DiagnosticCode
	var status Status
	var message string
	switch IntervalVerdict(*out.Reading, out.Bound, toolMM) {
	case -1:
		code, status = DiagWallTooThin, Violating
		message = "the minimum wall thickness is proven below the tool"
	case 0:
		code, status = DiagUndecidedWall, Suspect
		message = "the minimum wall thickness interval straddles the tool"
	default:
		return nil
	}
	return []Diagnostic[BodyT, CellT]{{
		Code: code, Status: status, Body: body, Survey: SurveyWall,
		Reading: ReadingWall, Observed: &m, Required: &tool, Message: message,
	}}
}

// UndercutDiagnostics records confirmed opposing faces and incomplete pull
// coverage as separate findings, in that order.
func UndercutDiagnostics[BodyT comparable, FaceT comparable, CellT any](body BodyT,
	payload any, out UndercutSurvey[FaceT],
) []Diagnostic[BodyT, CellT] {
	var diags []Diagnostic[BodyT, CellT]
	if out.OK && len(out.Faces) > 0 {
		diags = append(diags, Diagnostic[BodyT, CellT]{
			Code: DiagUndercut, Status: Violating, Body: body,
			Survey: SurveyUndercut, Reading: ReadingNone,
			Message: "a face is a proven undercut against the pull",
		})
	}
	if !out.OK || out.Undecided {
		diags = append(diags, surveyRefusalDiagnostic[BodyT, CellT](
			body, payload, SurveyUndercut, out.Reason, DiagUndecidedUndercut,
			"the pull survey could neither prove nor exclude an undercut",
			"facetedPayload pull survey support is not implemented; use an analytic body or wait for faceted undercut support",
			"pull"))
	}
	return diags
}

// RadiusDiagnostics records a concave-radius survey that could not answer.
func RadiusDiagnostics[BodyT comparable, CellT any](body BodyT, payload any,
	out ScalarSurvey, ok bool,
) []Diagnostic[BodyT, CellT] {
	if ok && out.OK {
		return nil
	}
	return []Diagnostic[BodyT, CellT]{surveyRefusalDiagnostic[BodyT, CellT](
		body, payload, SurveyConcaveRadius, out.Reason, DiagUndecidedMinRadius,
		"the concave-radius survey could neither measure nor exclude a concave feature",
		"facetedPayload concave-radius survey support is not implemented; use an analytic body or wait for faceted radius support",
		"concave-radius")}
}

// surveyRefusalDiagnostic distinguishes an undecided proof from a payload
// whose survey has not been implemented.
func surveyRefusalDiagnostic[BodyT comparable, CellT any](body BodyT, payload any,
	survey SurveyKind, reason SurveyReason, undecidedCode DiagnosticCode,
	undecidedMessage, facetedMessage, surveyNoun string,
) Diagnostic[BodyT, CellT] {
	code, message := undecidedCode, undecidedMessage
	switch reason {
	case SurveyFacetedUnsupported:
		code, message = DiagUnsupportedSurveyPayload, facetedMessage
	case SurveyPayloadStaged:
		class := payloadClassName(payload)
		code = DiagUnsupportedSurveyPayload
		message = fmt.Sprintf(
			"%s %s survey support is not implemented; use an analytic body or wait for wider %s survey support",
			class, surveyNoun, class)
	}
	return Diagnostic[BodyT, CellT]{
		Code: code, Status: Suspect, Body: body, Survey: survey,
		Reading: ReadingNone, Message: message,
	}
}

func payloadClassName(payload any) string {
	name := fmt.Sprintf("%T", payload)
	if i := strings.LastIndexByte(name, '.'); i >= 0 {
		return name[i+1:]
	}
	return name
}
