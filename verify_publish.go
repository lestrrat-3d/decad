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
// certified readings). It decides Region's presence itself — non-nil exactly
// when in.Validity.Outcome is ValidityValid AND in.Body.Kind() is BodySolid
// (proposal §9, docs/surface-design.md §9.1) — rather than trusting
// in.Region's own zero-or-not state, so a caller cannot publish a region
// reading beside a non-valid verdict or a sheet. Wall, Undercut and
// ConcaveRadius likewise block on a non-valid in.Validity.Outcome or a
// non-solid kind before looking at in.Surveys at all, publishing Unavailable
// plus DiagSurveyPrerequisite for a requested survey the body does not permit
// running. BodyReport.Diagnostics is the
// deterministic flattening proposal §10 requires: validity diagnostics,
// core reading diagnostics, wall diagnostics, undercut diagnostics, and
// concave-radius diagnostics, in that order, each finding assembled exactly
// once here even though the same finding is also reachable through its own
// result's local Diagnostics slice.
func publishBodyResult(in bodyPublishInput) *BodyReport {
	wall := publishWallResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.WallTolerance, in.WallToleranceDiag)
	undercut := publishUndercutResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome)
	radius := publishConcaveRadiusResult(in.Body, in.Surveys, in.Request, in.Validity.Outcome, in.RadiusTolerance, in.RadiusToleranceDiag)

	// Region carries the two region quantities exactly when validity is
	// proven valid AND the body is a solid (verification §1): a sound sheet's
	// Validity can be ValidityValid too, but it encloses no region
	// (docs/surface-design.md §9.1), so the kind conjunct is enforced here,
	// independently of whatever in.Region carries.
	var region *RegionReadings
	if in.Validity.Outcome == ValidityValid && in.Body.Kind() == BodySolid {
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

// validityEvidence bundles publishValidityResult's inputs, in the house style
// of bodyPublishInput: Kind is the body's own BodyKind; Clean is
// auditBoundary's verdict and Built/Solid are the evaluator-built and
// proven-solid bits, filled for a BodySolid; Sheet is auditSheetBoundary's
// verdict, filled for a BodySheet. verifyBody's kind switch fills exactly one
// of Clean or Sheet — the other stays its zero value and publishValidityResult
// never reads it.
type validityEvidence struct {
	Kind  BodyKind
	Clean bool
	Built bool
	Solid bool
	Sheet sheetAuditOutcome
}

// publishValidityResult maps ev onto ValidityResult (proposal §9). It carries
// the body's one underlying validity diagnostic, at most one, so
// publishBodyResult's flattening never repeats it.
//
// Two audits share the word "manifold" here and decide different questions.
// This function's BodySheet arm rests on `auditSheetBoundary`
// (docs/surface-design.md §9.1) — the BODY-LEVEL audit this PR adds, over the
// held topology (`Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge`), which
// decides Validity.Outcome for a sheet BODY. It is not
// `docs/tessellation-design.md` §1.2's manifold-with-boundary audit, which
// runs over a TESSELLATED sheet MESH and decides whether that mesh is well
// formed; that audit says nothing about body validity and this arm does not
// consult it.
//
// A BodySolid runs the pre-existing three-way logic: a failed auditBoundary
// audit (!ev.Clean) is a concrete invalid-solid proof, a built and
// proven-solid body (ev.Built && ev.Solid) is the entailed positive
// conclusion of watertightness, manifoldness and no self-intersection for
// that proof, and every other combination is undecided — the current
// evidence cannot decide validity.
func publishValidityResult(body *Body, ev validityEvidence) ValidityResult {
	undecided := ValidityResult{
		Outcome: ValidityUndecided,
		Diagnostics: []Diagnostic{{
			Code:    DiagUndecidedValidity,
			Status:  Suspect,
			Body:    body,
			Reading: ReadingNone,
			Message: "the held boundary's validity is not decisive beyond its own proven bound",
		}},
	}
	if ev.Kind == BodySheet {
		switch ev.Sheet {
		case sheetAuditProven:
			return ValidityResult{Outcome: ValidityValid}
		case sheetAuditViolated:
			return ValidityResult{
				Outcome: ValidityInvalid,
				Diagnostics: []Diagnostic{{
					Code:    DiagInvalidBody,
					Status:  Unsound,
					Body:    body,
					Reading: ReadingNone,
					Message: "the held boundary is proven not a valid sheet",
				}},
			}
		default: // sheetAuditUndecided
			return undecided
		}
	}
	switch {
	case !ev.Clean:
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
	case ev.Built && ev.Solid:
		return ValidityResult{Outcome: ValidityValid}
	default:
		return undecided
	}
}

// surveyPrerequisiteDiagnostic builds the one local diagnostic a requested
// survey publishes when the body cannot supply the survey's prerequisite
// (proposal §9, docs/surface-design.md §9.1): a non-solid validity, or a
// sheet body, which has no material for a wall, pull or concave question to
// be about even when its own boundary is proven sound. The message names
// which cause applies; it never repeats the body's own underlying validity
// diagnostic, which stays in ValidityResult.Diagnostics alone.
func surveyPrerequisiteDiagnostic(body *Body, survey SurveyKind) Diagnostic {
	msg := "the requested survey needs a proven solid, and this body's validity is not valid"
	if body.Kind() == BodySheet {
		msg = "the requested survey needs a proven solid, and this body is a sheet with no material for a wall, pull or concave question"
	}
	return Diagnostic{
		Code:    DiagSurveyPrerequisite,
		Status:  Suspect,
		Body:    body,
		Survey:  survey,
		Reading: ReadingNone,
		Message: msg,
	}
}

// publishWallResult maps one body's wall survey outcome onto the public
// result vocabulary (proposal §6, both tables): the effective request alone
// decides ScalarNotRequested; a non-valid validity, or a non-solid kind,
// decides ScalarUnavailable before the survey outcome is even consulted
// (proposal §9, docs/surface-design.md §9.1), since a wall survey never runs
// on a body that did not prove a solid — a sound sheet's ValidityValid does
// not by itself admit the survey; otherwise the
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
	if validity != ValidityValid || body.Kind() != BodySolid {
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
// request alone decides CoverageNotRequested; a non-valid validity, or a
// non-solid kind, decides CoverageUnavailable before the survey outcome is
// even consulted (proposal §9, docs/surface-design.md §9.1), since the pull
// survey never runs on a body that did not prove a solid; otherwise the
// producer's own ok/reason/undecided decide
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
	if validity != ValidityValid || body.Kind() != BodySolid {
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
// non-valid validity, or a non-solid kind, decides Unavailable before the
// survey outcome is even consulted (proposal §9, docs/surface-design.md
// §9.1), since the radius survey never runs on a body that did not prove a
// solid. It carries no assessment — Verify accepts no radius
// requirement. Diagnostics is the concave-radius group of proposal §10's
// flattening: the survey's own findings, plus the radius reading's own
// precision finding when it fired — or the single prerequisite finding when
// validity blocked the survey from running at all.
func publishConcaveRadiusResult(body *Body, surveys surveyResults, req VerifyRequest, validity ValidityOutcome, tolerance ToleranceResult, toleranceDiag *Diagnostic) ConcaveRadiusResult {
	if !req.ConcaveRadius {
		return ConcaveRadiusResult{Outcome: ScalarNotRequested}
	}
	if validity != ValidityValid || body.Kind() != BodySolid {
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
