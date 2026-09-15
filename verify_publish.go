package decad

// This file is Verify's publication assembler: it turns the private survey
// outcomes (survey.go) and certified readings verifyBody has already decided
// into the result records verify_result.go declares, and it is the
// publication code the final Verify uses — never a stand-in for it. It fills
// every field of the returned bodyResult: Body, Status, Validity, Topology,
// Area, Bounds, Region, Wall, Undercut, ConcaveRadius and Diagnostics.
// projectLegacyBodyReport is the temporary bridge proposal §16 allows while
// the exported shapes still compile: PR 5 deletes it and nothing else of
// this assembler.

// bodyPublishInput bundles publishBodyResult's inputs: the per-body facts
// verifyBody has already computed — the validity verdict, the held topology
// counts, the certified core readings, the raw survey outcomes, the
// effective request, and each reading's tolerance verdict — so the
// assembler itself only maps them onto the result vocabulary (proposal
// §3/§6/§8/§9). Data flows one way into it: a real private survey outcome
// and a real certified reading, never a value read back off an
// already-built legacy BodyReport. Region carries the body's Volume and
// Centroid readings when the caller's own validity switch proved a solid;
// publishBodyResult decides on its own whether to publish it, so a
// non-valid in.Region is simply never read (proposal §9). CoreDiagnostics is
// the other flattening group (proposal §10) the assembler does not derive
// from a survey outcome: the area/bounds/volume/centroid tolerance
// findings, always SurveyNone. WallToleranceDiag and RadiusToleranceDiag are
// that survey's own reading's precision finding, kept apart from Core
// because §10 flattens each with its OWN survey's diagnostics rather than
// with the core group.
type bodyPublishInput struct {
	Body                *Body
	Status              Status
	Validity            validityResult
	Topology            heldTopology
	Area                scalarReading
	Bounds              boundsReading
	Region              *regionReadings
	Request             verifyRequest
	Surveys             surveyResults
	WallTolerance       toleranceResult
	WallToleranceDiag   *Diagnostic
	RadiusTolerance     toleranceResult
	RadiusToleranceDiag *Diagnostic
	CoreDiagnostics     []Diagnostic
}

// publishBodyResult builds one bodyResult from in (proposal §16: the
// publication assembler consuming actual private survey outcomes and
// certified readings). It decides Region's presence itself — non-nil
// exactly when in.Validity.Outcome is validityValid (proposal §9) — rather
// than trusting in.Region's own zero-or-not state, so a caller cannot
// publish a region reading beside a non-valid verdict. Wall, Undercut and
// ConcaveRadius likewise block on a non-valid in.Validity.Outcome before
// looking at in.Surveys at all, publishing Unavailable plus
// DiagSurveyPrerequisite for a requested survey the body's validity does
// not permit running (proposal §9). bodyResult.Diagnostics is the
// deterministic flattening proposal §10 requires: validity diagnostics,
// core reading diagnostics, wall diagnostics, undercut diagnostics, and
// concave-radius diagnostics, in that order, each finding assembled exactly
// once here even though the same finding is also reachable through its own
// result's local Diagnostics slice.
func publishBodyResult(in bodyPublishInput) *bodyResult {
	wall := publishWallResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.WallTolerance, in.WallToleranceDiag)
	undercut := publishUndercutResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome)
	radius := publishConcaveRadiusResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.RadiusTolerance, in.RadiusToleranceDiag)

	var region *regionReadings
	if in.Validity.Outcome == validityValid {
		region = in.Region
	}

	var diags []Diagnostic
	diags = append(diags, in.Validity.Diagnostics...)
	diags = append(diags, in.CoreDiagnostics...)
	diags = append(diags, wall.Diagnostics...)
	diags = append(diags, undercut.Diagnostics...)
	diags = append(diags, radius.Diagnostics...)

	return &bodyResult{
		Body:          in.Body,
		Status:        in.Status,
		Validity:      in.Validity,
		Topology:      in.Topology,
		Area:          in.Area,
		Bounds:        in.Bounds,
		Region:        region,
		Wall:          wall,
		Undercut:      undercut,
		ConcaveRadius: radius,
		Diagnostics:   diags,
	}
}

// publishValidityResult maps the held-boundary audit's three-way outcome —
// clean is auditBoundary's own verdict, built is whether an evaluator
// feature produced the body, solid is the body's own proven-solid bit — onto
// validityResult (proposal §9): a failed audit is a concrete invalid-solid
// proof, a built and proven-solid body is the entailed positive conclusion
// of watertightness, manifoldness and no self-intersection for that proof,
// and every other combination is undecided — the current evidence cannot
// decide validity. It carries the body's one underlying validity diagnostic,
// at most one, so publishBodyResult's flattening never repeats it.
func publishValidityResult(body *Body, clean, built, solid bool) validityResult {
	switch {
	case !clean:
		return validityResult{
			Outcome: validityInvalid,
			Diagnostics: []Diagnostic{{
				Code:    DiagInvalidBody,
				Status:  Unsound,
				Body:    body,
				Reading: ReadingNone,
				Message: "the held boundary is proven not a valid solid",
			}},
		}
	case built && solid:
		return validityResult{Outcome: validityValid}
	default:
		return validityResult{
			Outcome: validityUndecided,
			Diagnostics: []Diagnostic{{
				Code:    DiagUndecidedValidity,
				Status:  Suspect,
				Body:    body,
				Reading: ReadingNone,
				Message: "the held boundary's validity is not decisive beyond its own proven bound",
			}},
		}
	}
}

// surveyPrerequisiteDiagnostic builds the one local diagnostic a requested
// survey publishes when the body's validity is not validityValid (proposal
// §9): the survey needs a proven solid, and this body did not prove one. It
// never repeats the body's own underlying validity diagnostic — that finding
// stays in validityResult.Diagnostics alone.
func surveyPrerequisiteDiagnostic(body *Body, survey SurveyKind) Diagnostic {
	return Diagnostic{
		Code:    DiagSurveyPrerequisite,
		Status:  Suspect,
		Body:    body,
		Survey:  survey,
		Reading: ReadingNone,
		Message: "the requested survey needs a proven solid, and this body's validity is not valid",
	}
}

// publishWallResult maps one body's wall survey outcome onto the private
// result vocabulary (proposal §6, both tables): the effective request alone
// decides ScalarNotRequested; a non-valid validity decides ScalarUnavailable
// before the survey outcome is even consulted (proposal §9), since a wall
// survey never runs on a body that did not prove a solid; otherwise the
// producer's own ok/reason decide Unavailable versus Undecided, a nil
// reading is the proven Absent, and a non-nil reading is Measured — its
// assessment taken from the SAME interval comparison (intervalVerdict) the
// survey diagnostic itself used, so a straddling proven interval and a
// met/violated one agree with the diagnostic that already fired for it.
// Diagnostics is the wall group of proposal §10's flattening: the survey's
// own findings (or the single prerequisite finding when validity blocked
// it), plus the wall reading's own precision finding when it fired, in that
// order.
func publishWallResult(body *Body, surveys surveyResults, req verifyRequest, validity validityOutcome, tolerance toleranceResult, toleranceDiag *Diagnostic) wallResult {
	if req.Wall == nil {
		return wallResult{Outcome: scalarNotRequested, Assessment: assessmentNotEvaluated}
	}
	if validity != validityValid {
		return wallResult{
			Request:     req.Wall,
			Outcome:     scalarUnavailable,
			Assessment:  assessmentUndecided,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyWall)},
		}
	}
	res := wallResult{Request: req.Wall}
	out := surveys.Wall
	switch {
	case !out.ok:
		res.Assessment = assessmentUndecided
		res.Outcome = unavailableOrUndecided(out.reason)
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
	res.Diagnostics = appendToleranceDiag(surveys.WallDiagnostics, toleranceDiag)
	return res
}

// unavailableOrUndecided maps a survey's own !ok reason onto ScalarUnavailable
// or ScalarUndecided (proposal §6's ScalarUnavailable row, §16's "Map an
// explicit unsupported payload dispatch to Unavailable"): a payload with no
// implemented survey — faceted, or a payload class the dispatch does not name
// at all — is Unavailable; every other unresolved proof stays Undecided.
func unavailableOrUndecided(reason surveyReason) scalarOutcome {
	switch reason {
	case surveyFacetedUnsupported, surveyPayloadStaged:
		return scalarUnavailable
	default:
		return scalarUndecided
	}
}

// unavailableOrUndecidedCoverage is unavailableOrUndecided's coverageState
// counterpart, for the undercut survey.
func unavailableOrUndecidedCoverage(reason surveyReason) coverageState {
	switch reason {
	case surveyFacetedUnsupported, surveyPayloadStaged:
		return coverageUnavailable
	default:
		return coverageUndecided
	}
}

// appendToleranceDiag builds one result's Diagnostics: the survey's own
// findings, plus its reading's own precision finding when one fired,
// appended last (proposal §10). It always returns a fresh slice so no
// result's Diagnostics aliases surveyResults' own slice.
func appendToleranceDiag(survey []Diagnostic, toleranceDiag *Diagnostic) []Diagnostic {
	var diags []Diagnostic
	diags = append(diags, survey...)
	if toleranceDiag != nil {
		diags = append(diags, *toleranceDiag)
	}
	return diags
}

// publishUndercutResult maps one body's undercut survey outcome onto the
// private result vocabulary (proposal §7's coverage table): the effective
// request alone decides CoverageNotRequested; a non-valid validity decides
// CoverageUnavailable before the survey outcome is even consulted (proposal
// §9), since the pull survey never runs on a body that did not prove a
// solid; otherwise the producer's own ok/reason/undecided decide
// Unavailable, Undecided, Partial or Complete. Faces carries the producer's
// own face list unchanged — every face in it is CONFIRMED to oppose the
// pull, and its nil-versus-empty shape is exactly the producer's own (nil
// for an unrecoverable or entirely undecided survey, otherwise the
// producer's own listing) so CoverageUndecided is published rather than a
// claimed Partial when no face is confirmed. Diagnostics is the pull
// survey's own subset of runSurveys' findings (surveys.go), routed here
// rather than recomputed — or the single prerequisite finding when validity
// blocked the survey from running at all.
func publishUndercutResult(body *Body, surveys surveyResults, req verifyRequest, validity validityOutcome) undercutResult {
	if req.Undercut == nil {
		return undercutResult{Coverage: coverageNotRequested, Assessment: assessmentNotEvaluated}
	}
	if validity != validityValid {
		return undercutResult{
			Request:     req.Undercut,
			Coverage:    coverageUnavailable,
			Assessment:  assessmentUndecided,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyUndercut)},
		}
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
		res.Coverage = unavailableOrUndecidedCoverage(out.reason)
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
// Unavailable, Undecided, Absent, or Measured with its tolerance verdict. A
// non-valid validity decides Unavailable before the survey outcome is even
// consulted (proposal §9), since the radius survey never runs on a body that
// did not prove a solid. It carries no assessment — Verify accepts no radius
// requirement. Diagnostics is the concave-radius group of proposal §10's
// flattening: the survey's own findings, plus the radius reading's own
// precision finding when it fired — or the single prerequisite finding when
// validity blocked the survey from running at all.
func publishConcaveRadiusResult(body *Body, surveys surveyResults, req verifyRequest, validity validityOutcome, tolerance toleranceResult, toleranceDiag *Diagnostic) concaveRadiusResult {
	if !req.ConcaveRadius {
		return concaveRadiusResult{Outcome: scalarNotRequested}
	}
	if validity != validityValid {
		return concaveRadiusResult{
			Outcome:     scalarUnavailable,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyConcaveRadius)},
		}
	}
	out := surveys.Radius
	var res concaveRadiusResult
	switch {
	case !out.ok:
		res.Outcome = unavailableOrUndecided(out.reason)
	case out.reading == nil:
		res.Outcome = scalarAbsent
	default:
		res.Outcome = scalarMeasured
		m := lengthMeasurement(*out.reading, out.bound)
		res.Minimum = &scalarReading{Measurement: m, Tolerance: tolerance}
	}
	res.Diagnostics = appendToleranceDiag(surveys.RadiusDiagnostics, toleranceDiag)
	return res
}

// projectLegacyBodyReport copies one bodyResult's fields onto the
// still-exported legacy BodyReport so the whole repository keeps compiling
// and every existing test keeps passing (proposal §16's temporary private
// bridge). Solid, Watertight and Manifold all collapse to the one bit the
// legacy shape never distinguished further — Validity.Outcome ==
// validityValid; SelfIntersecting stays false on every outcome, exactly as
// the current audit never asserts one true. PR 5 deletes this function and
// nothing else of the assembler.
func projectLegacyBodyReport(res *bodyResult, br *BodyReport) {
	br.Area = res.Area.Measurement
	br.Bounds = res.Bounds.Box
	br.Lumps = res.Topology.Lumps
	br.Voids = res.Topology.Voids

	valid := res.Validity.Outcome == validityValid
	br.Solid = valid
	br.Watertight = valid
	br.Manifold = valid
	br.SelfIntersecting = false

	br.Volume = nil
	br.Centroid = nil
	if res.Region != nil {
		vol := res.Region.Volume.Measurement
		cen := res.Region.Centroid.VecMeasurement
		br.Volume = &vol
		br.Centroid = &cen
	}

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
