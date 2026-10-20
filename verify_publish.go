package decad

import (
	"github.com/lestrrat-3d/decad/internal/boundaryaudit"
	"github.com/lestrrat-3d/decad/internal/verifypublish"
)

func encodedVerifyRequest(req VerifyRequest) VerifyRequest {
	encoded := VerifyRequest{
		RelativeTolerance: req.RelativeTolerance,
		ConcaveRadius:     req.ConcaveRadius,
		Clearances:        req.Clearances,
	}
	if req.Wall != nil {
		encoded.Wall = &WallRequest{
			Minimum: req.Wall.Minimum, DraftAllowance: req.Wall.DraftAllowance,
		}
	}
	if req.Undercut != nil {
		encoded.Undercut = &UndercutRequest{PullDirection: req.Undercut.PullDirection}
	}
	return encoded
}

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
	published := verifypublish.PublishBody(verifypublish.BodyPublication[*Body, *Face, JointCell]{
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
		Request:             encodedVerifyRequest(in.Request),
		Surveys:             surveyPublication(in.Surveys),
		WallTolerance:       in.WallTolerance,
		WallToleranceDiag:   in.WallToleranceDiag,
		RadiusTolerance:     in.RadiusTolerance,
		RadiusToleranceDiag: in.RadiusToleranceDiag,
		CoreDiagnostics:     in.CoreDiagnostics,
	})
	wall := WallResult{
		Request: in.Request.Wall, Outcome: published.Wall.Outcome,
		Minimum: published.Wall.Minimum, Assessment: published.Wall.Assessment,
		Diagnostics: published.Wall.Diagnostics,
	}
	undercut := UndercutResult{
		Request: in.Request.Undercut, Coverage: published.Undercut.Coverage,
		Faces: published.Undercut.Faces, Assessment: published.Undercut.Assessment,
		Diagnostics: published.Undercut.Diagnostics,
	}
	return &BodyReport{
		Body: published.Body, Status: published.Status, Validity: published.Validity,
		Topology: published.Topology, Area: published.Area,
		Bounds: published.Bounds,
		Region: published.Region, Wall: wall, Undercut: undercut,
		ConcaveRadius: published.ConcaveRadius,
		Diagnostics:   published.Diagnostics,
	}
}

// surveyPublication maps the private producer outcomes without changing their readings.
func surveyPublication(in surveyResults) verifypublish.SurveyPublication[*Body, *Face, JointCell] {
	return verifypublish.SurveyPublication[*Body, *Face, JointCell]{
		Wall: verifypublish.ScalarSurvey{
			Reading: in.Wall.reading, Bound: in.Wall.bound, OK: in.Wall.ok, Reason: in.Wall.reason,
		},
		WallDiagnostics: in.WallDiagnostics,
		Undercut: verifypublish.UndercutSurvey[*Face]{
			Faces: in.Undercut.faces, OK: in.Undercut.ok,
			Undecided: in.Undercut.undecided, Reason: in.Undercut.reason,
		},
		UndercutDiagnostics: in.UndercutDiagnostics,
		Radius: verifypublish.ScalarSurvey{
			Reading: in.Radius.reading, Bound: in.Radius.bound, OK: in.Radius.ok, Reason: in.Radius.reason,
		},
		RadiusDiagnostics: in.RadiusDiagnostics,
	}
}

// publishReport builds the document report from its already decided rows.
func publishReport(req VerifyRequest, bodies []*BodyReport, interferences []Interference,
	clearances []Clearance, diagnostics []Diagnostic, status Status) *Report {
	return &Report{
		Request: req, Bodies: bodies, Interferences: interferences,
		Clearances: clearances, Diagnostics: diagnostics, Status: status,
	}
}

// validityEvidence contains the solid or sheet audit facts for one body.
type validityEvidence struct {
	Kind  BodyKind
	Clean bool
	Built bool
	Solid bool
	Sheet boundaryaudit.SheetAuditOutcome
}

// publishValidityResult adapts the body kind and sheet audit to the shared verdict.
func publishValidityResult(body *Body, ev validityEvidence) ValidityResult {
	return verifypublish.PublishValidity[*Body, JointCell](body, verifypublish.ValidityEvidence{
		Sheet:         ev.Kind == BodySheet,
		SheetProven:   ev.Sheet == boundaryaudit.SheetAuditProven,
		SheetViolated: ev.Sheet == boundaryaudit.SheetAuditViolated,
		Clean:         ev.Clean,
		Built:         ev.Built,
		Solid:         ev.Solid,
	})
}
