package decad

// This file is Verify's publication assembler: it turns the private survey
// outcomes (survey.go) and certified readings verifyBody has already decided
// into the Report and BodyReport values verify_result.go declares, and it is
// the publication code the final Verify uses. It fills every field of the
// returned BodyReport: Body, Status, Validity, Topology, Area, Bounds,
// Region, Wall, Undercut, ConcaveRadius and Diagnostics; publishReport
// assembles the returned Report from the bodies, pair rows and diagnostics
// Verify has already computed, so the report is built once and never
// mutated afterward.

// bodyPublishInput bundles publishBodyResult's inputs: the per-body facts
// verifyBody has already computed — the validity verdict, the held topology
// counts, the certified core readings, the raw survey outcomes, the
// effective request, and each reading's tolerance verdict — so the
// assembler itself only maps them onto the result vocabulary (proposal
// §3/§6/§8/§9). Data flows one way into it: a real private survey outcome
// and a real certified reading, never a value read back off an
// already-built BodyReport. Region carries the body's Volume and Centroid
// readings when the caller's own validity switch proved a solid;
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

// publishBodyResult builds one BodyReport from in (proposal §16: the
// publication assembler consuming actual private survey outcomes and
// certified readings). It decides Region's presence itself — non-nil
// exactly when in.Validity.Outcome is ValidityValid (proposal §9) — rather
// than trusting in.Region's own zero-or-not state, so a caller cannot
// publish a region reading beside a non-valid verdict. Wall, Undercut and
// ConcaveRadius likewise block on a non-valid in.Validity.Outcome before
// looking at in.Surveys at all, publishing Unavailable plus
// DiagSurveyPrerequisite for a requested survey the body's validity does
// not permit running (proposal §9). BodyReport.Diagnostics is the
// deterministic flattening proposal §10 requires: validity diagnostics,
// core reading diagnostics, wall diagnostics, undercut diagnostics, and
// concave-radius diagnostics, in that order, each finding assembled exactly
// once here even though the same finding is also reachable through its own
// result's local Diagnostics slice.
func publishBodyResult(in bodyPublishInput) *BodyReport {
	wall := publishWallResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.WallTolerance, in.WallToleranceDiag)
	undercut := publishUndercutResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome)
	radius := publishConcaveRadiusResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.RadiusTolerance, in.RadiusToleranceDiag)

	var region *RegionReadings
	if in.Validity.Outcome == ValidityValid {
		region = in.Region
	}

	var diags []Diagnostic
	diags = append(diags, in.Validity.Diagnostics...)
	diags = append(diags, in.CoreDiagnostics...)
	diags = append(diags, wall.Diagnostics...)
	diags = append(diags, undercut.Diagnostics...)
	diags = append(diags, radius.Diagnostics...)

	return &BodyReport{
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

// publishReport builds the *Report Verify returns from the pieces it has
// already assembled: the effective request, the body reports in
// Document.Bodies() order, the proven interferences and clearances, the
// flattened diagnostic inventory — body diagnostics in body order, then
// pair diagnostics in pair order (proposal §10) — and the aggregated
// Status. It is the one place a returned Report is constructed, so no slice
// is appended to or mutated after this call (proposal §10).
func publishReport(req VerifyRequest, bodies []*BodyReport, interferences []Interference, clearances []Clearance, diagnostics []Diagnostic, status Status) *Report {
	return &Report{
		Request:       req,
		Bodies:        bodies,
		Interferences: interferences,
		Clearances:    clearances,
		Diagnostics:   diagnostics,
		Status:        status,
	}
}

// publishValidityResult maps the held-boundary audit's three-way outcome —
// kind is the body's own BodyKind, clean is auditBoundary's own verdict,
// built is whether an evaluator feature produced the body, solid is the
// body's own proven-solid bit — onto ValidityResult (proposal §9): a failed
// audit is a concrete invalid-solid proof, a built and proven-solid body is
// the entailed positive conclusion of watertightness, manifoldness and no
// self-intersection for that proof, and every other combination is
// undecided — the current evidence cannot decide validity. It carries the
// body's one underlying validity diagnostic, at most one, so
// publishBodyResult's flattening never repeats it.
//
// A BodySheet body takes its own first arm, decided on kind alone and
// BEFORE clean is read at all: auditBoundary's closed-body audit — every
// edge bounds exactly two faces — does not apply to a sheet, whose free
// edges are its ordinary shape, not the watertightness failure they would be
// on a solid, so clean's verdict on a sheet says nothing this function may
// act on. This is docs/surface-design.md §9.1's holding fix: the
// manifold-with-boundary audit that DOES apply to a sheet lands in a later
// increment and replaces this arm outright; until then, a sheet reads
// ValidityUndecided rather than proven invalid, on the same diagnostic the
// default arm below publishes for an undecided solid.
func publishValidityResult(body *Body, kind BodyKind, clean, built, solid bool) ValidityResult {
	switch {
	case kind == BodySheet:
		return ValidityResult{
			Outcome: ValidityUndecided,
			Diagnostics: []Diagnostic{{
				Code:    DiagUndecidedValidity,
				Status:  Suspect,
				Body:    body,
				Reading: ReadingNone,
				Message: "the held boundary's validity is not decisive beyond its own proven bound",
			}},
		}
	case !clean:
		return ValidityResult{
			Outcome: ValidityInvalid,
			Diagnostics: []Diagnostic{{
				Code:    DiagInvalidBody,
				Status:  Unsound,
				Body:    body,
				Reading: ReadingNone,
				Message: "the held boundary is proven not a valid solid",
			}},
		}
	case built && solid:
		return ValidityResult{Outcome: ValidityValid}
	default:
		return ValidityResult{
			Outcome: ValidityUndecided,
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
// survey publishes when the body's validity is not ValidityValid (proposal
// §9): the survey needs a proven solid, and this body did not prove one. It
// never repeats the body's own underlying validity diagnostic — that finding
// stays in ValidityResult.Diagnostics alone.
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

// publishWallResult maps one body's wall survey outcome onto the public
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
func publishWallResult(body *Body, surveys surveyResults, req VerifyRequest, validity ValidityOutcome, tolerance ToleranceResult, toleranceDiag *Diagnostic) WallResult {
	if req.Wall == nil {
		return WallResult{Outcome: ScalarNotRequested, Assessment: AssessmentNotEvaluated}
	}
	if validity != ValidityValid {
		return WallResult{
			Request:     req.Wall,
			Outcome:     ScalarUnavailable,
			Assessment:  AssessmentUndecided,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyWall)},
		}
	}
	res := WallResult{Request: req.Wall}
	out := surveys.Wall
	switch {
	case !out.ok:
		res.Assessment = AssessmentUndecided
		res.Outcome = unavailableOrUndecided(out.reason)
	case out.reading == nil:
		// No wall exists below the requested minimum (proposal §6).
		res.Outcome = ScalarAbsent
		res.Assessment = AssessmentMet
	default:
		res.Outcome = ScalarMeasured
		m := lengthMeasurement(*out.reading, out.bound)
		res.Minimum = &ScalarReading{Measurement: m, Tolerance: tolerance}
		switch intervalVerdict(*out.reading, out.bound, req.Wall.Minimum.Base()) {
		case -1:
			res.Assessment = AssessmentViolated
		case 1:
			res.Assessment = AssessmentMet
		default:
			res.Assessment = AssessmentUndecided
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
func unavailableOrUndecided(reason surveyReason) ScalarOutcome {
	switch reason {
	case surveyFacetedUnsupported, surveyPayloadStaged:
		return ScalarUnavailable
	default:
		return ScalarUndecided
	}
}

// unavailableOrUndecidedCoverage is unavailableOrUndecided's Coverage
// counterpart, for the undercut survey.
func unavailableOrUndecidedCoverage(reason surveyReason) Coverage {
	switch reason {
	case surveyFacetedUnsupported, surveyPayloadStaged:
		return CoverageUnavailable
	default:
		return CoverageUndecided
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

// orderFacesConfirmed reorders confirmed — every face is already CONFIRMED
// to oppose the pull, and each appears in it once — into body.Faces() order
// (proposal §11), without touching which faces are confirmed or how the
// producer proved them. confirmed's own nil-versus-empty shape survives
// unchanged: nil comes back nil, an empty non-nil listing comes back empty
// non-nil.
func orderFacesConfirmed(body *Body, confirmed []*Face) []*Face {
	if confirmed == nil {
		return nil
	}
	want := make(map[*Face]struct{}, len(confirmed))
	for _, f := range confirmed {
		want[f] = struct{}{}
	}
	ordered := make([]*Face, 0, len(confirmed))
	for _, f := range body.Faces() {
		if _, ok := want[f]; ok {
			ordered = append(ordered, f)
		}
	}
	return ordered
}

// publishUndercutResult maps one body's undercut survey outcome onto the
// public result vocabulary (proposal §7's coverage table): the effective
// request alone decides CoverageNotRequested; a non-valid validity decides
// CoverageUnavailable before the survey outcome is even consulted (proposal
// §9), since the pull survey never runs on a body that did not prove a
// solid; otherwise the producer's own ok/reason/undecided decide
// Unavailable, Undecided, Partial or Complete. Faces carries the producer's
// own confirmed faces reordered into body.Faces() order (proposal §11) —
// every face in it is CONFIRMED to oppose the pull, and its nil-versus-empty
// shape is exactly the producer's own (nil for an unrecoverable or entirely
// undecided survey, otherwise the producer's own listing) so
// CoverageUndecided is published rather than a claimed Partial when no face
// is confirmed. Diagnostics is the pull survey's own subset of runSurveys'
// findings (surveys.go), routed here rather than recomputed — or the single
// prerequisite finding when validity blocked the survey from running at
// all.
func publishUndercutResult(body *Body, surveys surveyResults, req VerifyRequest, validity ValidityOutcome) UndercutResult {
	if req.Undercut == nil {
		return UndercutResult{Coverage: CoverageNotRequested, Assessment: AssessmentNotEvaluated}
	}
	if validity != ValidityValid {
		return UndercutResult{
			Request:     req.Undercut,
			Coverage:    CoverageUnavailable,
			Assessment:  AssessmentUndecided,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyUndercut)},
		}
	}
	out := surveys.Undercut
	res := UndercutResult{
		Request:     req.Undercut,
		Faces:       orderFacesConfirmed(body, out.faces),
		Diagnostics: surveys.UndercutDiagnostics,
	}
	switch {
	case !out.ok:
		res.Assessment = AssessmentUndecided
		res.Coverage = unavailableOrUndecidedCoverage(out.reason)
	case out.undecided:
		if len(out.faces) > 0 {
			res.Coverage = CoveragePartial
			res.Assessment = AssessmentViolated
		} else {
			res.Coverage = CoverageUndecided
			res.Assessment = AssessmentUndecided
		}
	default:
		res.Coverage = CoverageComplete
		res.Assessment = AssessmentMet
		if len(out.faces) > 0 {
			res.Assessment = AssessmentViolated
		}
	}
	return res
}

// publishConcaveRadiusResult maps one body's concave-radius survey outcome
// onto the public result vocabulary (proposal §6): ScalarNotRequested,
// Unavailable, Undecided, Absent, or Measured with its tolerance verdict. A
// non-valid validity decides Unavailable before the survey outcome is even
// consulted (proposal §9), since the radius survey never runs on a body that
// did not prove a solid. It carries no assessment — Verify accepts no radius
// requirement. Diagnostics is the concave-radius group of proposal §10's
// flattening: the survey's own findings, plus the radius reading's own
// precision finding when it fired — or the single prerequisite finding when
// validity blocked the survey from running at all.
func publishConcaveRadiusResult(body *Body, surveys surveyResults, req VerifyRequest, validity ValidityOutcome, tolerance ToleranceResult, toleranceDiag *Diagnostic) ConcaveRadiusResult {
	if !req.ConcaveRadius {
		return ConcaveRadiusResult{Outcome: ScalarNotRequested}
	}
	if validity != ValidityValid {
		return ConcaveRadiusResult{
			Outcome:     ScalarUnavailable,
			Diagnostics: []Diagnostic{surveyPrerequisiteDiagnostic(body, SurveyConcaveRadius)},
		}
	}
	out := surveys.Radius
	var res ConcaveRadiusResult
	switch {
	case !out.ok:
		res.Outcome = unavailableOrUndecided(out.reason)
	case out.reading == nil:
		res.Outcome = ScalarAbsent
	default:
		res.Outcome = ScalarMeasured
		m := lengthMeasurement(*out.reading, out.bound)
		res.Minimum = &ScalarReading{Measurement: m, Tolerance: tolerance}
	}
	res.Diagnostics = appendToleranceDiag(surveys.RadiusDiagnostics, toleranceDiag)
	return res
}
