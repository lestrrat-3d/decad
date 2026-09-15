package decad

// This file is Verify's publication assembler: it turns the private survey
// outcomes (survey.go) and certified readings verifyBody has already decided
// into the result records verify_result.go declares, and it is the
// publication code the final Verify uses — never a stand-in for it. It fills
// Body, Status, Area, Bounds, Wall, Undercut, ConcaveRadius and Diagnostics
// on the returned bodyResult; Validity, Topology and Region stay their zero
// value until PR 4 extends it. projectLegacyBodyReport is the temporary
// bridge proposal §16 allows while the exported shapes still compile: PR 5
// deletes it and nothing else of this assembler.

// bodyPublishInput bundles publishBodyResult's inputs: the per-body facts
// verifyBody has already computed — the certified core readings, the raw
// survey outcomes, the effective request, and each reading's tolerance
// verdict — so the assembler itself only maps them onto the result
// vocabulary (proposal §3/§6/§8). Data flows one way into it: a real private
// survey outcome and a real certified reading, never a value read back off
// an already-built legacy BodyReport.
type bodyPublishInput struct {
	Body            *Body
	Status          Status
	Area            scalarReading
	Bounds          boundsReading
	Request         verifyRequest
	Surveys         surveyResults
	WallTolerance   toleranceResult
	RadiusTolerance toleranceResult
	Diagnostics     []Diagnostic
}

// publishBodyResult builds one bodyResult from in (proposal §16: the
// publication assembler consuming actual private survey outcomes and
// certified readings).
func publishBodyResult(in bodyPublishInput) *bodyResult {
	return &bodyResult{
		Body:          in.Body,
		Status:        in.Status,
		Area:          in.Area,
		Bounds:        in.Bounds,
		Wall:          publishWallResult(in.Surveys, in.Request, in.WallTolerance),
		Undercut:      publishUndercutResult(in.Surveys, in.Request),
		ConcaveRadius: publishConcaveRadiusResult(in.Surveys, in.Request, in.RadiusTolerance),
		Diagnostics:   in.Diagnostics,
	}
}

// publishWallResult maps one body's wall survey outcome onto the private
// result vocabulary (proposal §6, both tables): the effective request alone
// decides ScalarNotRequested; otherwise the producer's own ok/reason decide
// Unavailable versus Undecided, a nil reading is the proven Absent, and a
// non-nil reading is Measured — its assessment taken from the SAME interval
// comparison (intervalVerdict) the survey diagnostic itself used, so a
// straddling proven interval and a met/violated one agree with the
// diagnostic that already fired for it.
func publishWallResult(surveys surveyResults, req verifyRequest, tolerance toleranceResult) wallResult {
	if req.Wall == nil {
		return wallResult{Outcome: scalarNotRequested, Assessment: assessmentNotEvaluated}
	}
	res := wallResult{Request: req.Wall}
	out := surveys.Wall
	switch {
	case !out.ok:
		res.Assessment = assessmentUndecided
		if out.reason == surveyFacetedUnsupported {
			res.Outcome = scalarUnavailable
		} else {
			res.Outcome = scalarUndecided
		}
	case out.reading == nil:
		// No wall exists below the requested minimum (proposal §6).
		res.Outcome = scalarAbsent
		res.Assessment = assessmentMet
	default:
		res.Outcome = scalarMeasured
		m := lengthMeasurement(*out.reading, out.bound)
		res.Minimum = &scalarReading{Measurement: m, Tolerance: tolerance}
		switch intervalVerdict(*out.reading, out.bound, req.Wall.Minimum.Base()) {
		case -1:
			res.Assessment = assessmentViolated
		case 1:
			res.Assessment = assessmentMet
		default:
			res.Assessment = assessmentUndecided
		}
	}
	return res
}

// publishUndercutResult maps one body's undercut survey outcome onto the
// private result vocabulary (proposal §7's coverage table): the effective
// request alone decides CoverageNotRequested; otherwise the producer's own
// ok/reason/undecided decide Unavailable, Undecided, Partial or Complete.
// Faces carries the producer's own face list unchanged — every face in it is
// CONFIRMED to oppose the pull, and its nil-versus-empty shape is exactly the
// producer's own (nil for an unrecoverable or entirely undecided survey,
// otherwise the producer's own listing) so CoverageUndecided is published
// rather than a claimed Partial when no face is confirmed. Diagnostics is
// the pull survey's own subset of runSurveys' findings (surveys.go), routed
// here rather than recomputed.
func publishUndercutResult(surveys surveyResults, req verifyRequest) undercutResult {
	if req.Undercut == nil {
		return undercutResult{Coverage: coverageNotRequested, Assessment: assessmentNotEvaluated}
	}
	out := surveys.Undercut
	res := undercutResult{
		Request:     req.Undercut,
		Faces:       out.faces,
		Diagnostics: surveys.UndercutDiagnostics,
	}
	switch {
	case !out.ok:
		res.Assessment = assessmentUndecided
		if out.reason == surveyFacetedUnsupported {
			res.Coverage = coverageUnavailable
		} else {
			res.Coverage = coverageUndecided
		}
	case out.undecided:
		if len(out.faces) > 0 {
			res.Coverage = coveragePartial
			res.Assessment = assessmentViolated
		} else {
			res.Coverage = coverageUndecided
			res.Assessment = assessmentUndecided
		}
	default:
		res.Coverage = coverageComplete
		res.Assessment = assessmentMet
		if len(out.faces) > 0 {
			res.Assessment = assessmentViolated
		}
	}
	return res
}

// publishConcaveRadiusResult maps one body's concave-radius survey outcome
// onto the private result vocabulary (proposal §6): ScalarNotRequested,
// Unavailable, Undecided, Absent, or Measured with its tolerance verdict. It
// carries no assessment — Verify accepts no radius requirement.
func publishConcaveRadiusResult(surveys surveyResults, req verifyRequest, tolerance toleranceResult) concaveRadiusResult {
	if !req.ConcaveRadius {
		return concaveRadiusResult{Outcome: scalarNotRequested}
	}
	out := surveys.Radius
	var res concaveRadiusResult
	switch {
	case !out.ok:
		if out.reason == surveyFacetedUnsupported {
			res.Outcome = scalarUnavailable
		} else {
			res.Outcome = scalarUndecided
		}
	case out.reading == nil:
		res.Outcome = scalarAbsent
	default:
		res.Outcome = scalarMeasured
		m := lengthMeasurement(*out.reading, out.bound)
		res.Minimum = &scalarReading{Measurement: m, Tolerance: tolerance}
	}
	return res
}

// projectLegacyBodyReport copies one bodyResult's PR-1 fields onto the
// still-exported legacy BodyReport so the whole repository keeps compiling
// and every existing test keeps passing (proposal §16's temporary private
// bridge). PR 5 deletes this function and nothing else of the assembler.
func projectLegacyBodyReport(res *bodyResult, br *BodyReport) {
	br.Area = res.Area.Measurement
	br.Bounds = res.Bounds.Box
	br.MinWallThickness = nil
	if res.Wall.Minimum != nil {
		m := res.Wall.Minimum.Measurement
		br.MinWallThickness = &m
	}
	br.MinRadius = nil
	if res.ConcaveRadius.Minimum != nil {
		m := res.ConcaveRadius.Minimum.Measurement
		br.MinRadius = &m
	}
	br.Undercuts = res.Undercut.Faces
}
