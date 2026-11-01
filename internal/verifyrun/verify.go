package verifyrun

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/boundaryaudit"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/decad/internal/verifypublish"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/decad/internal/verifyoption"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Run verifies bodies, which the caller lists in document order, and returns
// the report over their topology nodes (agent-docs/verification-design.md). The
// caller owns the options and the document; Run reads only the nodes.
func Run(ctx context.Context, h Hooks, bodies []*topo.Body, cfg verifyConfig) (*Report, error) {
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
	var bodyReports []*BodyReport
	if workers := bodyCheckWorkers(ctx, bodies, cfg); workers > 1 {
		var err error
		bodyReports, err = orderedwork.Run(ctx, meshbool.WithContactWorkers(ctx, 1), bodies, workers,
			func(ctx context.Context, b *topo.Body) (*BodyReport, error) {
				return verifyBody(ctx, h.Draft, b, cfg, req)
			})
		if err != nil {
			return nil, err
		}
	}
	for i, b := range bodies {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var br *BodyReport
		if bodyReports != nil {
			br = bodyReports[i]
		} else {
			var err error
			br, err = verifyBody(ctx, h.Draft, b, cfg, req)
			if err != nil {
				return nil, err
			}
		}
		report.Bodies = append(report.Bodies, br)
		report.Diagnostics = append(report.Diagnostics, br.Diagnostics...)
		if br.Status == vocab.Suspect {
			undecided = true
		}
		if br.Status != vocab.Unsound && br.Validity.Outcome == vocab.ValidityValid {
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
	// see proveVerifyPair (internal/verifyrun/pairs.go) and agent-docs/surface-design.md §9.3.
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
	meshes, err := newVerifyMeshCache(ctx, h, jobs)
	if err != nil {
		return nil, err
	}
	// A parallel pair already occupies one worker, so its contact batches use
	// that worker. The single-worker path keeps the caller's contact setting.
	parallelCtx := meshbool.WithContactWorkers(ctx, 1)
	workers := orderedwork.Workers(ctx, meshbool.DefaultContactWorkers())
	outcomes, err := orderedwork.Run(ctx, parallelCtx, jobs, workers, func(ctx context.Context, job verifyPairJob) (verifyPairOutcome, error) {
		return proveVerifyPair(ctx, job, cfg, geomCache, meshes, h.Mesher, h.Alloc, h.History)
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

// bodyCheckWorkers limits the pool to the measured prism wall-survey path.
// Default checks and unavailable surveys can finish before pool setup does.
// Keep mixed documents serial so this gate also bounds which kernels overlap.
func bodyCheckWorkers(ctx context.Context, bodies []*topo.Body, cfg verifyConfig) int {
	if cfg.Wall == nil || len(bodies) < 2 {
		return 1
	}
	workers := min(orderedwork.Workers(ctx, orderedwork.EachWorkers(len(bodies), 2)), orderedwork.MaxEachWorkers)
	if workers < 2 {
		return 1
	}
	for _, b := range bodies {
		if ctx.Err() != nil {
			return 1
		}
		p, ok := b.Payload.(prismrecord.Payload)
		if !ok || b.Kind != topo.BodySolid || !b.Solid || p.SectionDelta != 0 || !analyticWallLoop(ctx, p.Profile.Outer) {
			return 1
		}
		for _, hole := range p.Profile.Holes {
			if !analyticWallLoop(ctx, hole) {
				return 1
			}
		}
	}
	return workers
}

func analyticWallLoop(ctx context.Context, loop sectionrecord.LoopRecord) bool {
	if len(loop.Segments) == 0 {
		return false
	}
	for i, segment := range loop.Segments {
		if i%proofbound.WorkPollInterval == 0 && ctx.Err() != nil {
			return false
		}
		switch segment.(type) {
		case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
		default:
			return false
		}
	}
	return true
}

func effectiveVerifyRequest(cfg verifyConfig) vocab.VerifyRequest {
	encoded := verifyoption.EffectiveRequest(cfg)
	req := vocab.VerifyRequest{
		RelativeTolerance: encoded.RelativeTolerance,
		ConcaveRadius:     encoded.ConcaveRadius,
		Clearances:        encoded.Clearances,
	}
	if encoded.Wall != nil {
		req.Wall = &vocab.WallRequest{Minimum: encoded.Wall.Minimum, DraftAllowance: encoded.Wall.DraftAllowance}
	}
	if encoded.Undercut != nil {
		req.Undercut = &vocab.UndercutRequest{PullDirection: encoded.Undercut.PullDirection}
	}
	return req
}

// appendClearance records the proven-disjoint pair's Clearance row and returns
// a DiagMeasurementBeyondTolerance (Reading ReadingGap) when the gap's Bound
// exceeds the pair-relative tolerance, else nil (verification §1.1/§6).
func appendClearance(report *Report, a, b *topo.Body, res pairkernel.Result, rel float64) *Diagnostic {
	gap := pairkernel.GapMeasurement(res)
	report.Clearances = append(report.Clearances, Clearance{A: a, B: b, Gap: gap})
	pairGate := PairToleranceInputs{Diameter: res.Diam}
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, rel, pairGate.LengthReference)
	if pass {
		return nil
	}
	obs := gap
	d := Diagnostic{
		Code:     vocab.DiagMeasurementBeyondTolerance,
		Status:   vocab.Suspect,
		Pair:     &DiagnosticPair{A: a, B: b},
		Reading:  vocab.ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		d.Required = tolerance.RequiredThreshold(rel*ref, gap.Value)
	}
	return &d
}

// PairDiagNone builds a pair diagnostic that names no bounded reading
// (verification §1.1): Reading ReadingNone, every Observed* and Required nil.
func PairDiagNone[B comparable](a, b B, code vocab.DiagnosticCode, msg string) vocab.DiagnosticOf[B, vocab.JointCell] {
	return vocab.DiagnosticOf[B, vocab.JointCell]{
		Code:    code,
		Status:  vocab.Suspect,
		Pair:    &vocab.DiagnosticPairOf[B]{A: a, B: b},
		Reading: vocab.ReadingNone,
		Message: msg,
	}
}

// UndecidedPairDiag converts boundaryaudit.UndecidedPairReason's code and
// message for a pair whose overlap volume the evaluator could not measure into
// the public pair diagnostic.
func UndecidedPairDiag[B comparable](a, b B, nodeA, nodeB *topo.Body, verdict boundaryaudit.PairVerdict,
	outcome boundaryaudit.InterferenceOutcome) vocab.DiagnosticOf[B, vocab.JointCell] {
	code, msg := boundaryaudit.UndecidedPairReason(nodeA, nodeB, verdict, outcome)
	return PairDiagNone(a, b, code, msg)
}

// verifyBody audits one body and assembles its BodyReport through
// internal/verifyrun/publish.go's publishBodyResult. A feature-built body is valid by
// construction, and the proof is the construction (evaluator §10): the
// structural audit is an invariant check, cheap, and its verdict is decided,
// not sampled. Exposed at package scope so an in-package test can construct
// a body directly and assert on the returned BodyReport (task-list §4
// item 4's validity fixtures).
func verifyBody(ctx context.Context, mesher verifiedmesh.DraftMesher, b *topo.Body, cfg verifyConfig, req vocab.VerifyRequest) (*BodyReport, error) {
	area := b.Area
	bounds := b.Bounds

	// The held topology's lump and void counts (proposal §9): descriptive
	// data every body carries, using the current count definitions,
	// independent of the validity verdict decided below.
	topology := vocab.HeldTopology{Lumps: len(b.Lumps)}
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			if s.Void {
				topology.Voids++
			}
		}
	}

	// The held boundary's structural audit. A topo.BodySolid runs
	// boundaryaudit.AuditSolidBoundary: every edge bounds exactly two faces,
	// every face carries at least one loop of at least one edge. A topo.BodySheet
	// runs AuditSheetBoundary instead (agent-docs/surface-design.md §9.1). This evaluator's boundary is exact, so a
	// defect is proven — Unsound — and a clean audit on a feature-built body
	// is proven validity. PublishValidityResult (internal/verifyrun/publish.go) maps the
	// evidence onto ValidityResult and carries the one diagnostic that
	// explains it (proposal §9).
	validity := PublishValidityResult(b, BodyValidityEvidence(ctx, b))
	if validity.Outcome == vocab.ValidityValid && b.Kind == topo.BodySolid {
		disproved, err := shellWindingDisproved(ctx, mesher, b)
		if err != nil {
			return nil, err
		}
		if disproved != "" {
			validity = ValidityResult{Outcome: vocab.ValidityInvalid, Diagnostics: []Diagnostic{{
				Code: vocab.DiagInvalidBody, Status: vocab.Unsound, Body: b, Reading: vocab.ReadingNone,
				Message: "the held boundary is proven not a valid solid: " + disproved,
			}}}
		}
	}
	haveRegion := validity.Outcome == vocab.ValidityValid && b.Kind == topo.BodySolid

	var vol vocab.Measurement
	var cen vocab.VecMeasurement
	status := vocab.Sound
	switch validity.Outcome {
	case vocab.ValidityInvalid:
		status = vocab.Unsound
	case vocab.ValidityValid:
		vol = b.Volume
		cen = b.Centroid
	default:
		// A body this evaluator did not build (unreachable through the
		// public API) has a validity the audit alone cannot prove:
		// undecided, Suspect — never a fabricated pass.
		status = vocab.Suspect
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
	if validity.Outcome == vocab.ValidityInvalid {
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
			Area:     vocab.ScalarReading{Measurement: area},
			Bounds:   vocab.BoundsReading{Box: bounds},
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
	// walls (agent-docs/surface-design.md §2.3, §9.1), and runSurveys itself
	// gates the wall and concave-radius blocks back down to a solid, so
	// admitting a sheet here never lets those two surveys run on one.
	surveysRunnable := haveRegion ||
		(validity.Outcome == vocab.ValidityValid && b.Kind == topo.BodySheet && cfg.Pull != nil)
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
			case vocab.Violating:
				violating = true
			case vocab.Suspect:
				suspect = true
			}
		}
		// The wall and concave-radius prerequisite refusals are emitted by
		// verifypublish.PublishWall/PublishRadius, not by runSurveys, so
		// they never appear in surveyDiags above; a sheet that asked either
		// one still owes this body a Suspect for it.
		if b.Kind == topo.BodySheet && (cfg.Wall != nil || cfg.ConcaveRadius) {
			suspect = true
		}
	case validity.Outcome == vocab.ValidityValid && surveysAsked:
		// surveysRunnable is false here only because the body is a proven
		// sheet that asked no pull — a wall and/or concave-radius question
		// with no material to be about (agent-docs/surface-design.md §9.1).
		// runSurveys never runs; verifypublish.PublishWall and
		// PublishRadius each publish Unavailable plus their own
		// DiagSurveyPrerequisite from validity and kind alone, so this
		// body's Status must already carry that Suspect verdict.
		suspect = true
	}

	var wallReading, radiusReading *vocab.Measurement
	if surveys.Wall.reading != nil {
		m := verifypublish.LengthMeasurement(*surveys.Wall.reading, surveys.Wall.bound)
		wallReading = &m
	}
	if surveys.Radius.reading != nil {
		m := verifypublish.LengthMeasurement(*surveys.Radius.reading, surveys.Radius.bound)
		radiusReading = &m
	}

	var volPtr *vocab.Measurement
	var cenPtr *vocab.VecMeasurement
	if haveRegion {
		volPtr, cenPtr = &vol, &cen
	}
	readings := bodyReadingSet{
		Area:     area,
		Bounds:   bounds,
		Volume:   volPtr,
		Centroid: cenPtr,
		Wall:     wallReading,
		Radius:   radiusReading,
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
		status = vocab.Suspect
	}
	if violating {
		status = vocab.Violating
	}

	// Region groups the two proven-solid quantities alongside their own
	// tolerance verdicts (proposal §9); it stays nil on every other validity
	// outcome, and publishBodyResult enforces that on its own regardless of
	// what is passed here.
	var region *vocab.RegionReadings
	if haveRegion {
		region = &vocab.RegionReadings{
			Volume:   vocab.ScalarReading{Measurement: vol, Tolerance: verdicts.Volume},
			Centroid: vocab.VectorReading{VecMeasurement: cen, Tolerance: verdicts.Centroid},
		}
	}

	return publishBodyResult(bodyPublishInput{
		Body:                b,
		Status:              status,
		Validity:            validity,
		Topology:            topology,
		Area:                vocab.ScalarReading{Measurement: area, Tolerance: verdicts.Area},
		Bounds:              vocab.BoundsReading{Box: bounds, Tolerance: verdicts.Bounds},
		Region:              region,
		Request:             req,
		Surveys:             surveys,
		WallTolerance:       verdicts.Wall,
		WallToleranceDiag:   diagSet.Wall,
		RadiusTolerance:     verdicts.Radius,
		RadiusToleranceDiag: diagSet.Radius,
		CoreDiagnostics:     diagSet.Core,
	}), nil
}

// BodyValidityEvidence gathers the evidence PublishValidityResult decides one
// body's validity from: the sheet audit for a topo.BodySheet, the closed-skin audit
// plus the build and solidity facts otherwise. Verify and VerifyMotion share
// it so the two never judge one body two different ways. It reads the faceted
// image's audit facts itself because internal/facetedrecord imports
// internal/boundaryaudit.
func BodyValidityEvidence(ctx context.Context, b *topo.Body) ValidityEvidence {
	evidence := ValidityEvidence{Kind: b.Kind}
	switch b.Kind {
	case topo.BodySheet:
		evidence.Sheet = boundaryaudit.AuditSheetBoundary(ctx, b)
	default:
		fp, faceted := b.Payload.(facetedrecord.Payload)
		evidence.Clean = boundaryaudit.AuditSolidBoundary(b, faceted && fp.ContactAudited && len(fp.Tris) > 0)
		evidence.Built = b.Payload != nil
		evidence.Solid = b.Solid
	}
	return evidence
}

// shellWindingDisproved is agent-docs/tessellation-design.md §4's per-shell
// winding audit read as validity evidence (agent-docs/verification-design.md):
// a solid whose lumps hold two or more closed shells is meshed at its own
// single-body chord at the lowest verification level (the reading needs no
// boundary proof), and a ShellWindingError from that mesh proves its lump
// reading wrong, returned as the refusal's text. The audit only falsifies:
// a body with fewer closed shells, or one whose mesh fails for any other
// reason, is left as the structural audit decided it. Only a context error
// is returned as an error.
func shellWindingDisproved(ctx context.Context, mesher verifiedmesh.DraftMesher, b *topo.Body) (string, error) {
	closed := 0
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			if !s.Open {
				closed++
			}
		}
	}
	if closed < 2 || mesher == nil {
		return "", nil
	}
	share := booloperand.ChordShare(ctx, b)
	chord, _, ok := meshbool.PairChordFrom(share, share)
	if !ok {
		return "", nil
	}
	_, err := mesher.VerifyNone(ctx, b, chord)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return "", ctxErr
	}
	if winding, ok := errors.AsType[*tessellation.ShellWindingError](err); ok {
		return winding.Msg, nil
	}
	return "", nil
}

// aggregateStatus is the worst-wins precedence of verification §6.
func aggregateStatus(r *Report, undecided bool) vocab.Status {
	for _, br := range r.Bodies {
		if br.Status == vocab.Unsound {
			return vocab.Unsound
		}
	}
	if len(r.Interferences) > 0 {
		return vocab.Interfering
	}
	// A sheet proven to cross a solid's boundary (DiagSheetSolidCrossing,
	// agent-docs/surface-design.md §9.3) contributes the Interfering rung with no
	// Interference row at all: a sheet encloses no region, so there is no
	// overlap volume to report. The scan keeps the invariant that the
	// report's Status is the worst Diagnostic.Status in the slice even for
	// this volume-less case, immediately after the volume-bearing check
	// above so worst-wins order is unchanged.
	for _, d := range r.Diagnostics {
		if d.Status == vocab.Interfering {
			return vocab.Interfering
		}
	}
	for _, br := range r.Bodies {
		if br.Status == vocab.Violating {
			return vocab.Violating
		}
	}
	for _, br := range r.Bodies {
		if br.Status == vocab.Suspect {
			return vocab.Suspect
		}
	}
	if undecided {
		return vocab.Suspect
	}
	return vocab.Sound
}
