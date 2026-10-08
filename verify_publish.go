package decad

import "github.com/lestrrat-3d/decad/internal/reportvocab"

// bodyPublishInput carries the already decided body readings and raw surveys.
type bodyPublishInput struct {
	Body                *Body
	Status              Status
	Validity            ValidityResult
	Topology            HeldTopology
	Area                ScalarReading
	Bounds              BoundsReading
	Region              *RegionReadings
	Request             VerifyRequest
	Surveys             surveyResults
	WallTolerance       ToleranceResult
	WallToleranceDiag   *Diagnostic
	RadiusTolerance     ToleranceResult
	RadiusToleranceDiag *Diagnostic
	CoreDiagnostics     []Diagnostic
}

// publishBodyResult adapts live topology and private surveys to the report vocabulary.
func publishBodyResult(in bodyPublishInput) *BodyReport {
	kind := in.Body.Kind()
	var faces []*Face
	if in.Request.Undercut != nil && in.Validity.Outcome == ValidityValid &&
		(kind == BodySolid || kind == BodySheet) {
		faces = in.Body.Faces()
	}
	return reportvocab.PublishBody(reportvocab.BodyPublication[*Body, *Face, JointCell]{
		Body:                in.Body,
		Solid:               kind == BodySolid,
		Sheet:               kind == BodySheet,
		Faces:               faces,
		Status:              in.Status,
		Validity:            in.Validity,
		Topology:            in.Topology,
		Area:                in.Area,
		Bounds:              in.Bounds,
		Region:              in.Region,
		Request:             in.Request,
		Surveys:             surveyPublication(in.Surveys),
		WallTolerance:       in.WallTolerance,
		WallToleranceDiag:   in.WallToleranceDiag,
		RadiusTolerance:     in.RadiusTolerance,
		RadiusToleranceDiag: in.RadiusToleranceDiag,
		CoreDiagnostics:     in.CoreDiagnostics,
	})
}

// surveyPublication maps the private producer outcomes without changing their readings.
func surveyPublication(in surveyResults) reportvocab.SurveyPublication[*Body, *Face, JointCell] {
	return reportvocab.SurveyPublication[*Body, *Face, JointCell]{
		Wall: reportvocab.ScalarSurvey{
			Reading: in.Wall.reading, Bound: in.Wall.bound, OK: in.Wall.ok, Reason: in.Wall.reason,
		},
		WallDiagnostics: in.WallDiagnostics,
		Undercut: reportvocab.UndercutSurvey[*Face]{
			Faces: in.Undercut.faces, OK: in.Undercut.ok,
			Undecided: in.Undercut.undecided, Reason: in.Undercut.reason,
		},
		UndercutDiagnostics: in.UndercutDiagnostics,
		Radius: reportvocab.ScalarSurvey{
			Reading: in.Radius.reading, Bound: in.Radius.bound, OK: in.Radius.ok, Reason: in.Radius.reason,
		},
		RadiusDiagnostics: in.RadiusDiagnostics,
	}
}

// publishReport builds the document report from its already decided rows.
func publishReport(req VerifyRequest, bodies []*BodyReport, interferences []Interference,
	clearances []Clearance, diagnostics []Diagnostic, status Status) *Report {
	return reportvocab.PublishReport(req, bodies, interferences, clearances, diagnostics, status)
}

// validityEvidence contains the solid or sheet audit facts for one body.
type validityEvidence struct {
	Kind  BodyKind
	Clean bool
	Built bool
	Solid bool
	Sheet sheetAuditOutcome
}

// publishValidityResult adapts the body kind and sheet audit to the shared verdict.
func publishValidityResult(body *Body, ev validityEvidence) ValidityResult {
	return reportvocab.PublishValidity[*Body, JointCell](body, reportvocab.ValidityEvidence{
		Sheet:         ev.Kind == BodySheet,
		SheetProven:   ev.Sheet == sheetAuditProven,
		SheetViolated: ev.Sheet == sheetAuditViolated,
		Clean:         ev.Clean,
		Built:         ev.Built,
		Solid:         ev.Solid,
	})
}

// publishUndercutResult is the in-package entry used by the sheet survey test.
func publishUndercutResult(body *Body, surveys surveyResults, req VerifyRequest,
	validity ValidityOutcome) UndercutResult {
	kind := body.Kind()
	var faces []*Face
	if req.Undercut != nil && validity == ValidityValid &&
		(kind == BodySolid || kind == BodySheet) {
		faces = body.Faces()
	}
	return reportvocab.PublishUndercut[*Body, *Face, JointCell](
		body, kind == BodySolid, kind == BodySheet, faces,
		surveyPublication(surveys).Undercut, surveys.UndercutDiagnostics, req, validity,
	)
}
