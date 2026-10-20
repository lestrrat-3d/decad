package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/decad/internal/verifyoption"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the Verify of docs/evaluator-design.md §10/§11 and
// docs/verification-design.md: one non-mutating call returning a rich report
// with a Status at both levels, aggregated by a fixed severity precedence,
// and one bit — Passed() — an agent gates on. The wall, undercut and
// concave-radius questions are answered outright by the analytic surveys of
// survey.go on the bodies this evaluator builds; the staging rule of
// evaluator §11 still governs everything else — an option this evaluator
// cannot ANSWER is accepted, its parameters validated, and the
// asked-but-unanswered question reads Suspect, never a silent pass.
//
// Two sibling files carry the rest, each with its own doc comment: report.go
// the vocabulary the report is written in, and verify_tolerance.go the gate
// that decides which readings are trustworthy. The proof of the reference
// diameter that gate is anchored on is internal/clearance/pairkernel/gate.go.

// VerifyOption configures Verify.
type VerifyOption interface {
	verifyOption()
}

// WallOption parameterizes WithMinWallThickness.
type WallOption interface {
	wallOption()
}

type verifyOptionValue struct{ encoded verifyoption.Token }
type wallOptionValue struct{ encoded verifyoption.WallToken }

func (verifyOptionValue) verifyOption() {}
func (wallOptionValue) wallOption()     {}

type verifyConfig = verifyoption.Config

// WithTolerance sets the relative tolerance gate of verification §2.
func WithTolerance(rel units.Value) VerifyOption {
	return verifyOptionValue{verifyoption.WithTolerance(rel)}
}

// WithMinWallThickness asks whether a wall is thinner than minimum.
func WithMinWallThickness(minimum units.Value, opts ...WallOption) VerifyOption {
	encoded := make([]verifyoption.WallToken, len(opts))
	for i, opt := range opts {
		if value, ok := opt.(wallOptionValue); ok {
			encoded[i] = value.encoded
		}
	}
	return verifyOptionValue{verifyoption.WithMinWallThickness(minimum, encoded...)}
}

// WithDraftAllowance sets the angle separating walls from edges.
func WithDraftAllowance(a units.Value) WallOption {
	return wallOptionValue{verifyoption.WithDraftAllowance(a)}
}

// WithPullDirection asks for undercuts against the stated direction.
func WithPullDirection(v r3.Vec) VerifyOption {
	return verifyOptionValue{verifyoption.WithPullDirection(v)}
}

// WithConcaveRadius asks for the tightest concave radius.
func WithConcaveRadius() VerifyOption {
	return verifyOptionValue{verifyoption.WithConcaveRadius()}
}

// WithClearances asks for minimum gaps between disjoint pairs.
func WithClearances() VerifyOption {
	return verifyOptionValue{verifyoption.WithClearances()}
}

func resolveVerifyOptions(opts []VerifyOption) (verifyConfig, error) {
	encoded := make([]verifyoption.Token, len(opts))
	for i, opt := range opts {
		if opt == nil {
			continue
		}
		value, ok := opt.(verifyOptionValue)
		if !ok {
			return verifyConfig{}, fmt.Errorf("%w: the verification option is not a decad option (%T)", ErrDegenerate, opt)
		}
		encoded[i] = value.encoded
	}
	return verifyoption.Resolve(encoded)
}

func effectiveVerifyRequest(cfg verifyConfig) VerifyRequest {
	encoded := verifyoption.EffectiveRequest(cfg)
	req := VerifyRequest{
		RelativeTolerance: encoded.RelativeTolerance,
		ConcaveRadius:     encoded.ConcaveRadius,
		Clearances:        encoded.Clearances,
	}
	if encoded.Wall != nil {
		req.Wall = &WallRequest{Minimum: encoded.Wall.Minimum, DraftAllowance: encoded.Wall.DraftAllowance}
	}
	if encoded.Undercut != nil {
		req.Undercut = &UndercutRequest{PullDirection: encoded.Undercut.PullDirection}
	}
	return req
}

// Verify is one non-mutating call over the live model: solidity and boundary
// validity per body, every quantity judged by the tolerance gate, and the
// pairwise partition — proven overlap, proven disjointness, or Suspect
// (verification §1/§6). It mirrors sketch.Verify (core §10).
//
// Verify checks interference even when no options are passed. For each pair of
// proven-valid bodies that cheaper proofs do not settle, its mesh fallback
// checks every pair of operand facet boxes before pruning exact predicates. A
// pair holding a sheet operand never reaches that fallback: it is decided by
// box separation, or, when the boxes meet, the sheet decision procedure of
// docs/surface-design.md §9.3. One pair's
// work can therefore grow with the two facet counts multiplied together, and
// total work also grows with the number of unresolved body pairs. Verify
// proves those pairs on up to GOMAXPROCS goroutines (at most 12) and builds
// the report in pair order, so the report and any returned error match a
// one-pair-at-a-time walk (docs/interference-design.md §2). Large-model
// callers should pass a context with a deadline chosen from representative
// inputs. For a document with live bodies, after document and option
// validation, cancellation returns ctx.Err() and a nil report; validation
// errors take precedence even when ctx is already canceled. An empty document
// retains its Sound result even when the context is already canceled. The
// document remains unchanged. Verify reads the live set once, as it stands
// when the call begins, so a concurrent Remove never changes the bodies one
// call reports on.
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	cfg, err := resolveVerifyOptions(opts)
	if err != nil {
		return nil, err
	}
	// The effective request is recorded once per call, even for an empty
	// document (proposal §5), and every body's WallResult.Request /
	// UndercutResult.Request shares these same pointers.
	req := effectiveVerifyRequest(cfg)

	// report accumulates the pieces publishReport assembles into the
	// returned *Report at the very end; Interferences is always computed —
	// the Interfering rung reads it (verification §1) — and the read-only
	// proof never consumes an operand or exposes a transient intersection
	// through the document.
	report := &Report{Interferences: []Interference{}}
	if cfg.Clearances {
		report.Clearances = []Clearance{}
	}

	undecided := false // some asked question or pair this evaluator cannot decide

	// pairBodies holds every proven-valid body — solid or sheet alike — in
	// Document.Bodies() order (interference design §2): a body whose own
	// validity is undecided or invalid never reaches pair work at all, which
	// is what keeps §9.3's sheet pair rule from ever running on a sheet that
	// is not itself proven sound.
	var pairBodies []*BodyReport
	for _, b := range d.liveBodies() {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		br, err := verifyBody(ctx, b, cfg, req)
		if err != nil {
			return nil, err
		}
		report.Bodies = append(report.Bodies, br)
		report.Diagnostics = append(report.Diagnostics, br.Diagnostics...)
		if br.Status == Suspect {
			undecided = true
		}
		if br.Status != Unsound && br.Validity.Outcome == ValidityValid {
			pairBodies = append(pairBodies, br)
		}
	}

	// The stable pair partition over proven-valid bodies (interference design
	// §2): box separation first, then the analytic relation, strict
	// containment or equality, and finally read-only intersection
	// measurement. Only a proven positive bounded volume emits an
	// Interference row. Expected empty, contact, staging, or coarse outcomes
	// stay Suspect and name themselves in the slice; invariant failures
	// return from Verify. A pair holding a sheet operand takes none of this —
	// see proveVerifyPair (verify_pairs.go) and docs/surface-design.md §9.3.
	// The pairs box separation does not finish are proven on a bounded
	// worker pool and folded back in pair order.
	jobs, err := verifyPairJobs(ctx, pairBodies, cfg)
	if err != nil {
		return nil, err
	}
	var geomCache *pairkernel.Cache
	if cfg.Clearances {
		geomCache = &pairkernel.Cache{}
	}
	// Each body is meshed once, at one chord, for every pair that reaches the
	// read-only mesh intersection (interference design §5.3).
	meshes, err := newVerifyMeshCache(ctx, jobs)
	if err != nil {
		return nil, err
	}
	// A parallel pair already occupies one worker, so its contact batches use
	// that worker. The single-worker path keeps the caller's contact setting.
	parallelCtx := meshbool.WithContactWorkers(ctx, 1)
	workers := orderedwork.Workers(ctx, meshbool.DefaultContactWorkers())
	outcomes, err := orderedwork.Run(ctx, parallelCtx, jobs, workers, func(ctx context.Context, job verifyPairJob) (verifyPairOutcome, error) {
		return proveVerifyPair(ctx, job, cfg, geomCache, meshes)
	})
	if err != nil {
		return nil, err
	}
	for _, o := range outcomes {
		report.Interferences = append(report.Interferences, o.interferences...)
		report.Clearances = append(report.Clearances, o.clearances...)
		report.Diagnostics = append(report.Diagnostics, o.diagnostics...)
		undecided = undecided || o.undecided
	}

	report.Status = aggregateStatus(report, undecided)
	return publishReport(req, report.Bodies, report.Interferences, report.Clearances, report.Diagnostics, report.Status), nil
}

// appendClearance records the proven-disjoint pair's Clearance row and returns
// a DiagMeasurementBeyondTolerance (Reading ReadingGap) when the gap's Bound
// exceeds the pair-relative tolerance, else nil (verification §1.1/§6).
func appendClearance(report *Report, a, b *Body, res pairkernel.Result, rel float64) *Diagnostic {
	gap := measurementFromInternal(pairkernel.GapMeasurement(res))
	report.Clearances = append(report.Clearances, Clearance{A: a, B: b, Gap: gap})
	pairGate := pairToleranceInputs{diameter: res.Diam}
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, rel, pairGate.lengthReference)
	if pass {
		return nil
	}
	obs := gap
	d := Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Pair:     &DiagnosticPair{A: a, B: b},
		Reading:  ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		d.Required = tolerance.RequiredThreshold(rel*ref, gap.Value)
	}
	return &d
}

// pairDiagNone builds a pair diagnostic that names no bounded reading
// (verification §1.1): Reading ReadingNone, every Observed* and Required nil.
func pairDiagNone(a, b *Body, code DiagnosticCode, msg string) Diagnostic {
	return Diagnostic{
		Code:    code,
		Status:  Suspect,
		Pair:    &DiagnosticPair{A: a, B: b},
		Reading: ReadingNone,
		Message: msg,
	}
}

// undecidedPairDiag converts reportvocab.UndecidedPairReason's code and
// message for a pair whose overlap volume the evaluator could not measure into
// the public pair diagnostic.
func undecidedPairDiag(a, b *Body, verdict reportvocab.PairVerdict, outcome reportvocab.InterferenceOutcome) Diagnostic {
	code, msg := reportvocab.UndecidedPairReason(&a.n, &b.n, verdict, outcome)
	return pairDiagNone(a, b, DiagnosticCode(code), msg)
}

// verifyBody audits one body and assembles its BodyReport through
// verify_publish.go's publishBodyResult. A feature-built body is valid by
// construction, and the proof is the construction (evaluator §10): the
// structural audit is an invariant check, cheap, and its verdict is decided,
// not sampled. Exposed at package scope so an in-package test can construct
// a body directly and assert on the returned BodyReport (task-list §4
// item 4's validity fixtures).
func verifyBody(ctx context.Context, b *Body, cfg verifyConfig, req VerifyRequest) (*BodyReport, error) {
	area := b.n.Area
	bounds := b.n.Bounds

	// The held topology's lump and void counts (proposal §9): descriptive
	// data every body carries, using the current count definitions,
	// independent of the validity verdict decided below.
	topology := HeldTopology{Lumps: len(b.n.Lumps)}
	for _, s := range b.Shells() {
		if s.IsVoid() {
			topology.Voids++
		}
	}

	// The held boundary's structural audit. A BodySolid runs
	// reportvocab.AuditSolidBoundary: every edge bounds exactly two faces,
	// every face carries at least one loop of at least one edge. A BodySheet
	// runs AuditSheetBoundary instead (docs/surface-design.md §9.1). This evaluator's boundary is exact, so a
	// defect is proven — Unsound — and a clean audit on a feature-built body
	// is proven validity. publishValidityResult (verify_publish.go) maps the
	// evidence onto ValidityResult and carries the one diagnostic that
	// explains it (proposal §9).
	validity := publishValidityResult(b, bodyValidityEvidence(ctx, b))
	haveRegion := validity.Outcome == ValidityValid && b.Kind() == BodySolid

	var vol Measurement
	var cen VecMeasurement
	status := Sound
	switch validity.Outcome {
	case ValidityInvalid:
		status = Unsound
	case ValidityValid:
		vol = measurementFromInternal(b.n.Volume)
		cen = vecMeasurementFromInternal(b.n.Centroid)
	default:
		// A body this evaluator did not build (unreachable through the
		// public API) has a validity the audit alone cannot prove:
		// undecided, Suspect — never a fabricated pass.
		status = Suspect
	}

	// A proven-invalid body emits one DiagInvalidBody and no region-quantity
	// diagnostics, because §1 gives it no region quantity to gate. The gate
	// still runs, discarded, to surface a cancellation observed while lazily
	// loading a reference. Its area and bounds stay available as boundary
	// data, but publishBodyResult never gates them — their Tolerance is
	// ToleranceNotEvaluated (proposal §9) since no Tolerance is passed here.
	// A requested survey on this body publishes Unavailable plus
	// DiagSurveyPrerequisite; publishBodyResult decides that from Validity
	// alone, without a Surveys record.
	if validity.Outcome == ValidityInvalid {
		if _, _, err := bodyReadingDiagnostics(ctx, b, bodyReadingSet{
			Area: area, Bounds: bounds,
		}, cfg.Rel); err != nil {
			return nil, err
		}
		return publishBodyResult(bodyPublishInput{
			Body:     b,
			Status:   status,
			Validity: validity,
			Topology: topology,
			Area:     ScalarReading{Measurement: measurementFromInternal(area)},
			Bounds:   BoundsReading{Box: boxFromInternal(bounds)},
			Request:  req,
		}), nil
	}

	// The asked opt-in surveys (evaluator §10, verification §6): answered
	// outright on this evaluator's analytic bodies — validity is decided
	// first, so only a proven solid or a proven, pull-asked sheet reaches
	// runSurveys at all. A survey requested on an undecided-validity body
	// never reaches it; publishBodyResult publishes its Unavailable outcome
	// and DiagSurveyPrerequisite from Validity alone (proposal §9). A survey
	// runSurveys itself cannot decide (a payload no shipped feature builds)
	// leaves the asked question undecided, and a stated spec proven to fail
	// is Violating. Each non-Sound survey outcome names itself in the slice.
	surveysAsked := cfg.Wall != nil || cfg.Pull != nil || cfg.ConcaveRadius
	// surveysRunnable widens haveRegion by exactly one case: a proven sheet
	// with a pull requested. The undercut survey answers over a sheet's own
	// walls (docs/surface-design.md §2.3, §9.1), and runSurveys itself
	// gates the wall and concave-radius blocks back down to a solid, so
	// admitting a sheet here never lets those two surveys run on one.
	surveysRunnable := haveRegion ||
		(validity.Outcome == ValidityValid && b.Kind() == BodySheet && cfg.Pull != nil)
	violating, suspect := false, false
	var surveys surveyResults
	switch {
	case surveysRunnable && surveysAsked:
		var surveyDiags []Diagnostic
		var err error
		surveys, surveyDiags, err = runSurveys(proofbound.NewWorkBudget(ctx), b, cfg)
		if err != nil {
			return nil, err
		}
		for _, d := range surveyDiags {
			switch d.Status {
			case Violating:
				violating = true
			case Suspect:
				suspect = true
			}
		}
		// The wall and concave-radius prerequisite refusals are emitted by
		// reportvocab.PublishWall/PublishRadius, not by runSurveys, so
		// they never appear in surveyDiags above; a sheet that asked either
		// one still owes this body a Suspect for it.
		if b.Kind() == BodySheet && (cfg.Wall != nil || cfg.ConcaveRadius) {
			suspect = true
		}
	case validity.Outcome == ValidityValid && surveysAsked:
		// surveysRunnable is false here only because the body is a proven
		// sheet that asked no pull — a wall and/or concave-radius question
		// with no material to be about (docs/surface-design.md §9.1).
		// runSurveys never runs; reportvocab.PublishWall and
		// PublishRadius each publish Unavailable plus their own
		// DiagSurveyPrerequisite from validity and kind alone, so this
		// body's Status must already carry that Suspect verdict.
		suspect = true
	}

	var wallReading, radiusReading *Measurement
	if surveys.Wall.reading != nil {
		m := reportvocab.LengthMeasurement(*surveys.Wall.reading, surveys.Wall.bound)
		wallReading = measurementPtrFromInternal(&m)
	}
	if surveys.Radius.reading != nil {
		m := reportvocab.LengthMeasurement(*surveys.Radius.reading, surveys.Radius.bound)
		radiusReading = measurementPtrFromInternal(&m)
	}

	var volPtr *Measurement
	var cenPtr *VecMeasurement
	if haveRegion {
		volPtr, cenPtr = &vol, &cen
	}
	readings := bodyReadingSet{
		Area:     area,
		Bounds:   bounds,
		Volume:   measurementPtrToInternal(volPtr),
		Centroid: vecMeasurementPtrToInternal(cenPtr),
		Wall:     measurementPtrToInternal(wallReading),
		Radius:   measurementPtrToInternal(radiusReading),
	}
	verdicts, diagSet, err := bodyReadingDiagnostics(ctx, b, readings, cfg.Rel)
	if err != nil {
		return nil, err
	}
	if len(diagSet.Core) > 0 || diagSet.Wall != nil || diagSet.Radius != nil {
		suspect = true
	}

	// Worst wins at the body level: Violating > Suspect > Sound
	// (verification §6).
	if suspect {
		status = Suspect
	}
	if violating {
		status = Violating
	}

	// Region groups the two proven-solid quantities alongside their own
	// tolerance verdicts (proposal §9); it stays nil on every other validity
	// outcome, and publishBodyResult enforces that on its own regardless of
	// what is passed here.
	var region *RegionReadings
	if haveRegion {
		region = &RegionReadings{
			Volume:   ScalarReading{Measurement: vol, Tolerance: toleranceResultFromInternal(verdicts.Volume)},
			Centroid: VectorReading{VecMeasurement: cen, Tolerance: toleranceResultFromInternal(verdicts.Centroid)},
		}
	}

	return publishBodyResult(bodyPublishInput{
		Body:                b,
		Status:              status,
		Validity:            validity,
		Topology:            topology,
		Area:                ScalarReading{Measurement: measurementFromInternal(area), Tolerance: toleranceResultFromInternal(verdicts.Area)},
		Bounds:              BoundsReading{Box: boxFromInternal(bounds), Tolerance: toleranceResultFromInternal(verdicts.Bounds)},
		Region:              region,
		Request:             req,
		Surveys:             surveys,
		WallTolerance:       toleranceResultFromInternal(verdicts.Wall),
		WallToleranceDiag:   diagnosticPtrFromInternal(diagSet.Wall),
		RadiusTolerance:     toleranceResultFromInternal(verdicts.Radius),
		RadiusToleranceDiag: diagnosticPtrFromInternal(diagSet.Radius),
		CoreDiagnostics:     diagnosticsFromInternal(diagSet.Core),
	}), nil
}

// bodyValidityEvidence gathers the evidence publishValidityResult decides one
// body's validity from: the sheet audit for a BodySheet, the closed-skin audit
// plus the build and solidity facts otherwise. Verify and VerifyMotion share
// it so the two never judge one body two different ways. It reads the faceted
// image's audit facts itself because internal/facetedrecord imports
// internal/reportvocab.
func bodyValidityEvidence(ctx context.Context, b *Body) validityEvidence {
	evidence := validityEvidence{Kind: b.Kind()}
	switch b.Kind() {
	case BodySheet:
		evidence.Sheet = reportvocab.AuditSheetBoundary(ctx, &b.n)
	default:
		fp, faceted := b.n.Payload.(facetedPayload)
		evidence.Clean = reportvocab.AuditSolidBoundary(&b.n, faceted && fp.ContactAudited && len(fp.Tris) > 0)
		evidence.Built = b.n.Payload != nil
		evidence.Solid = b.n.Solid
	}
	return evidence
}

// aggregateStatus is the worst-wins precedence of verification §6.
func aggregateStatus(r *Report, undecided bool) Status {
	for _, br := range r.Bodies {
		if br.Status == Unsound {
			return Unsound
		}
	}
	if len(r.Interferences) > 0 {
		return Interfering
	}
	// A sheet proven to cross a solid's boundary (DiagSheetSolidCrossing,
	// docs/surface-design.md §9.3) contributes the Interfering rung with no
	// Interference row at all: a sheet encloses no region, so there is no
	// overlap volume to report. The scan keeps the invariant that the
	// report's Status is the worst Diagnostic.Status in the slice even for
	// this volume-less case, immediately after the volume-bearing check
	// above so worst-wins order is unchanged.
	for _, d := range r.Diagnostics {
		if d.Status == Interfering {
			return Interfering
		}
	}
	for _, br := range r.Bodies {
		if br.Status == Violating {
			return Violating
		}
	}
	for _, br := range r.Bodies {
		if br.Status == Suspect {
			return Suspect
		}
	}
	if undecided {
		return Suspect
	}
	return Sound
}
