package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/decad/internal/topo"
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
// Three sibling files carry the rest, each with its own doc comment:
// report.go the vocabulary the report is written in, verify_tolerance.go the
// gate that decides which readings are trustworthy, and verify_gate.go the
// proof of the reference diameter that gate is anchored on.

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
	var geomCache *bodyGeomCache
	if cfg.Clearances {
		geomCache = &bodyGeomCache{}
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

func pairGapMeasurement(res pairResult) Measurement {
	midpoint := proofbound.BoundedMul(proofbound.BoundedAdd(proofbound.ExactScalar(res.lo), proofbound.ExactScalar(res.hi)), proofbound.ExactScalar(0.5))
	halfWidth := proofbound.BoundedMul(proofbound.BoundedSub(proofbound.ExactScalar(res.hi), proofbound.ExactScalar(res.lo)), proofbound.ExactScalar(0.5))
	// The midpoint error and the half-width's own error both widen the
	// published interval. The same bound feeds appendClearance's tolerance gate.
	bound := proofbound.AbsSumUpper(halfWidth.Value, halfWidth.Bound, midpoint.Bound)
	gap := Measurement{
		Value:     units.Millimeters(midpoint.Value),
		Exactness: Approximate,
		Bound:     units.Millimeters(bound),
	}
	if res.exact && bound == 0 {
		gap.Exactness = Exact
	}
	return gap
}

// appendClearance records the proven-disjoint pair's Clearance row and returns
// a DiagMeasurementBeyondTolerance (Reading ReadingGap) when the gap's Bound
// exceeds the pair-relative tolerance, else nil (verification §1.1/§6).
func appendClearance(report *Report, a, b *Body, res pairResult, rel float64) *Diagnostic {
	gap := pairGapMeasurement(res)
	report.Clearances = append(report.Clearances, Clearance{A: a, B: b, Gap: gap})
	pairGate := pairToleranceInputs{diameter: res.diam}
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

// undecidedPairDiag picks the diagnostic for a pair whose overlap volume the
// evaluator could not measure (verification §1.1): payload, contact, and
// in-pipeline limits each keep their own code and action; only an overlap with
// an otherwise undecided measurement is DiagUndecidedInterference; an
// unresolved partition is DiagUndecidedPair. Verify emits only this
// cause-specific diagnostic. A pair holding a sheet operand never reaches
// this function: it is resolved earlier,
// by box separation or the sheet decision procedure (sheetSolidPair,
// clearance.go), and takes DiagUnsupportedPairSheet or DiagSheetSolidCrossing
// instead (docs/surface-design.md §9.3).
func undecidedPairDiag(a, b *Body, verdict reportvocab.PairVerdict, outcome reportvocab.InterferenceOutcome) Diagnostic {
	switch {
	case outcome == reportvocab.InterferenceUnsupportedPayloadFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the first operand cannot enter the read-only intersection at the chord tolerance this check derives from the pair's own size, which no option sets: its tessellation refused that tolerance, or its held facets where the pair meets are coarser than it; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach")
	case outcome == reportvocab.InterferenceUnsupportedPayloadSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the second operand cannot enter the read-only intersection at the chord tolerance this check derives from the pair's own size, which no option sets: its tessellation refused that tolerance, or its held facets where the pair meets are coarser than it; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach")
	case outcome == reportvocab.InterferenceUnsupportedVolumeProofFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the first operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof")
	case outcome == reportvocab.InterferenceUnsupportedVolumeProofSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the second operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof")
	case outcome == reportvocab.InterferenceUnsupportedContact:
		if sharesFacePlane(a, b) {
			return pairDiagNone(a, b, DiagUnsupportedPairContact,
				"the pair reaches a contact or near-contact that the read-only intersection cannot classify, and the operands share a face plane; offset the operands so no face plane is shared, or adjust the geometry to create clear separation or deeper overlap, or wait for contact support")
		}
		return pairDiagNone(a, b, DiagUnsupportedPairContact,
			"the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support")
	case outcome == reportvocab.InterferenceUnsupportedPipeline:
		return pairDiagNone(a, b, DiagUnsupportedPairPipeline,
			"both operands tessellate, but later read-only intersection geometry exceeds the boolean pipeline's reach; simplify the boolean geometry or wait for pipeline support")
	case outcome == reportvocab.InterferenceUndecided && verdict == reportvocab.PairOverlapping:
		return pairDiagNone(a, b, DiagUndecidedInterference,
			"the pair is proven to overlap but the overlap volume is unmeasured")
	default:
		return pairDiagNone(a, b, DiagUndecidedPair,
			"the disjoint/overlap partition proof resolved neither way")
	}
}

// facePlaneKey is a's canonicalized (normal, signed offset) key for one planar
// face, built so two coplanar faces key equal regardless of which way each
// one's normal happens to face.
type facePlaneKey struct {
	nx, ny, nz, off float64
}

// canonicalFacePlaneKey builds p's facePlaneKey: the normal's sign is fixed by
// its first nonzero component being positive (flipping the offset along with
// it), so a pair of opposite-facing coplanar faces produces the same key, and
// every component is passed through zeroSign so -0.0 and 0.0 never key
// differently.
func canonicalFacePlaneKey(p surfacegeom.Plane) facePlaneKey {
	n := p.Frame.N()
	off := n.Dot(p.Frame.Origin())
	flip := n.X < 0 || (n.X == 0 && (n.Y < 0 || (n.Y == 0 && n.Z < 0)))
	if flip {
		n = n.Scale(-1)
		off = -off
	}
	return facePlaneKey{zeroSign(n.X), zeroSign(n.Y), zeroSign(n.Z), zeroSign(off)}
}

// zeroSign normalizes a negative zero to a positive zero so it never keys a
// map entry differently from 0.0.
func zeroSign(f float64) float64 {
	if f == 0 {
		return 0
	}
	return f
}

// sharesFacePlane reports whether a and b have a planar face whose infinite
// plane coincides — exact float comparison on the canonicalized normal and
// offset, reject-only in spirit: it can only fail to name a shared plane it
// cannot exactly confirm, never invent one that is not there. It is a
// diagnostic aid, not a proof: it never changes a Diagnostic's Code, Status,
// Reading, or Pair, only whether its Message names the cause.
func sharesFacePlane(a, b *Body) bool {
	keys := make(map[facePlaneKey]struct{})
	for _, f := range a.Faces() {
		if p, ok := f.n.Surface.(surfacegeom.Plane); ok {
			keys[canonicalFacePlaneKey(p)] = struct{}{}
		}
	}
	for _, f := range b.Faces() {
		p, ok := f.n.Surface.(surfacegeom.Plane)
		if !ok {
			continue
		}
		if _, found := keys[canonicalFacePlaneKey(p)]; found {
			return true
		}
	}
	return false
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

	// The held boundary's structural audit. A BodySolid runs auditBoundary:
	// every edge bounds exactly two faces, every face carries at least one
	// loop of at least one edge. A BodySheet runs auditSheetBoundary instead
	// (docs/surface-design.md §9.1). This evaluator's boundary is exact, so a
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
// it so the two never judge one body two different ways.
func bodyValidityEvidence(ctx context.Context, b *Body) validityEvidence {
	evidence := validityEvidence{Kind: b.Kind()}
	switch b.Kind() {
	case BodySheet:
		evidence.Sheet = auditSheetBoundary(ctx, b)
	default:
		evidence.Clean = auditBoundary(b)
		evidence.Built = b.n.Payload != nil
		evidence.Solid = b.n.Solid
	}
	return evidence
}

// auditBoundary checks the structural invariants of the held boundary:
// every edge bounds exactly two faces (manifold and watertight at once, for
// a closed skin), and every face carries at least one loop with at least one
// edge.
func auditBoundary(b *Body) bool {
	faces := b.Faces()
	if len(faces) == 0 {
		return false
	}
	for _, f := range faces {
		loops := f.Loops()
		if len(loops) == 0 {
			// A closed surface of revolution bounds a solid with no edges
			// at all. A faceted one-face image may have the same topology
			// only after its embedded closed mesh passed the build audit.
			switch f.n.Surface.Kind() {
			case surfacegeom.KindSphere, surfacegeom.KindTorus:
				continue
			case surfacegeom.KindFaceted:
				fp, ok := b.n.Payload.(facetedPayload)
				if ok && fp.ContactAudited && len(faces) == 1 && len(fp.Tris) > 0 && len(b.Edges()) == 0 {
					continue
				}
				return false
			default:
				return false
			}
		}
		for _, l := range loops {
			if len(l.Edges()) == 0 {
				return false
			}
		}
	}
	for _, e := range b.Edges() {
		if len(e.n.Faces) != 2 {
			return false
		}
	}
	return true
}

// auditSheetBoundary decides Table V's sheet validity audit
// (docs/surface-design.md §9.1): three structural legs read off the recorded
// topology, plus non-self-intersection admitted BY CONSTRUCTION alone. It is
// deliberately NOT §6.4's closure audit (`loftmesh.LoftCrossingAudit`) with closure
// dropped: that audit runs over a triangulated, all-planar face set sharing
// one exact vertex table, which a surface-extruded wall's `Plane`,
// `Cylinder` or `NURBSSurface` geometry — rimmed by `Arc3`, `Circle3` or
// `NURBSCurve` segments — cannot supply without first being chorded. Admitting
// a sheet because its chord mesh audits clean would be an admission gate
// resting on an approximation, which CLAUDE.md's reject-only rule forbids
// outright.
//
// Leg 1 — every face has at least one loop, and every loop has at least one
// coedge. Unlike auditBoundary's closed-solid audit, there is NO sphere/torus
// exemption: that allowance is a fact about a closed surface of revolution
// bounding a solid, and does not carry to a sheet, whose free edges are
// already the expected shape (docs/surface-design.md §2.2).
//
// Leg 2 — every edge is adjacent to one or two faces. Three or more is
// non-manifold, a defect on either body kind.
//
// Leg 3 — every edge adjacent to two faces is traversed by exactly one
// forward and one backward coedge, counted over every face's every loop's
// every coedge use. A coedge-use count that disagrees with the edge's own
// adjacent-face count is also a violation: it means some face's loop walks an
// edge Faces() does not know about, or fails to walk one it does.
//
// Leg 4 — non-self-intersection, decided by topo.ProvesSimple on the
// body's own payload. See topo.SimpleProver and each payload's ProvesSimple
// method for the admission rule, one payload at a time.
func auditSheetBoundary(ctx context.Context, b *Body) reportvocab.SheetAuditOutcome {
	faces := b.Faces()
	if len(faces) == 0 {
		return reportvocab.SheetAuditViolated
	}
	for _, f := range faces {
		loops := f.Loops()
		if len(loops) == 0 {
			return reportvocab.SheetAuditViolated
		}
		for _, l := range loops {
			if len(l.Edges()) == 0 {
				return reportvocab.SheetAuditViolated
			}
		}
	}

	for _, e := range b.Edges() {
		if len(e.n.Faces) == 0 || len(e.n.Faces) > 2 {
			return reportvocab.SheetAuditViolated
		}
	}

	uses := map[*Edge]int{}
	forward := map[*Edge]int{}
	backward := map[*Edge]int{}
	for _, f := range faces {
		for _, l := range f.Loops() {
			for _, ce := range l.CoEdges() {
				e := ce.Edge()
				uses[e]++
				if ce.IsForward() {
					forward[e]++
				} else {
					backward[e]++
				}
			}
		}
	}
	for _, e := range b.Edges() {
		if uses[e] != len(e.n.Faces) {
			return reportvocab.SheetAuditViolated
		}
		if len(e.n.Faces) == 2 && (forward[e] != 1 || backward[e] != 1) {
			return reportvocab.SheetAuditViolated
		}
	}

	if topo.ProvesSimple(ctx, b.n.Payload) {
		return reportvocab.SheetAuditProven
	}
	return reportvocab.SheetAuditUndecided
}

// boxesDisjoint reports whether the two bounds-inflated boxes have disjoint
// interiors. A body's interior lies within its box's interior, so disjoint
// box interiors prove the bodies cannot share volume — touching boxes
// included (evaluator §10).
func boxesDisjoint(a, b Box) bool {
	return measurement.BoxesDisjoint(boxToInternal(a), boxToInternal(b))
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
