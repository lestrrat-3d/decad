package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is the Verify of docs/evaluator-design.md §10/§11 and
// docs/verification-design.md: one non-mutating call returning a rich report
// with a Status at both levels, aggregated by a fixed severity precedence,
// and one bit — Trustworthy() — an agent gates on. The wall, undercut and
// minimum-radius questions are answered outright by the analytic surveys of
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
	option.Interface
	verifyOption()
}

type verifyOption struct{ option.Interface }

func (verifyOption) verifyOption() {}

// WallOption parameterizes WithMinWallThickness.
type WallOption interface {
	option.Interface
	wallOption()
}

type wallOption struct{ option.Interface }

func (wallOption) wallOption() {}

type identTolerance struct{}
type identMinWall struct{}
type identPullDirection struct{}
type identMinRadius struct{}
type identClearances struct{}
type identDraftAllowance struct{}

// wallSpec is WithMinWallThickness's recorded question: the tool, and the
// draft allowance drawing the line between a wall and an edge. nilOption
// records a nil WallOption for Verify to reject — an option constructor has
// no error to return.
type wallSpec struct {
	tool      units.Value
	allowance units.Value
	nilOption bool
}

// WithTolerance sets the relative tolerance gate of verification §2: the
// largest error the caller accepts as a fraction of the quantity measured.
// Dimensionless; the default is units.Scalar(1e-3) — three significant
// figures. Exact answers carry a zero proven bound and pass at any
// tolerance.
func WithTolerance(rel units.Value) VerifyOption {
	return verifyOption{option.New(identTolerance{}, rel)}
}

// WithMinWallThickness states the spec that no wall may be thinner than the
// tool that has to cut it (verification §2). The tool is the spec, never the
// probe: the reading is the infimum diameter over the body's spanning
// inscribed balls — material between skins opposing within the draft
// allowance — and the tool enters only where the interval rule decides the
// reading against it (verification §6). A wall proven thinner is Violating.
func WithMinWallThickness(tool units.Value, opts ...WallOption) VerifyOption {
	spec := wallSpec{tool: tool, allowance: units.Degrees(15)}
	for _, o := range opts {
		if o == nil {
			// An option constructor has no error to return; Verify rejects
			// the recorded marker with ErrDegenerate.
			spec.nilOption = true
			continue
		}
		switch o.Ident().(type) {
		case identDraftAllowance:
			if a, ok := option.Get[units.Value](o); ok {
				spec.allowance = a
			}
		}
	}
	return verifyOption{option.New(identMinWall{}, spec)}
}

// WithDraftAllowance sets how much draft opposition tolerates — where the
// wall ends and the edge begins (verification §2). An angle in [0°, 90°);
// the default is units.Degrees(15).
func WithDraftAllowance(a units.Value) WallOption {
	return wallOption{option.New(identDraftAllowance{}, a)}
}

// WithPullDirection states the direction the part must pull along; every
// reported undercut is a proven violation of it (verification §2), decided
// per face from its normal range: a face with a provenly opposing point is
// listed, exactly perpendicular is not opposed (the vertical wall clears),
// and a non-empty listing is Violating. A bounded analytic stand-in widens
// its range by its own proven departure before this comparison.
func WithPullDirection(v r3.Vec) VerifyOption {
	return verifyOption{option.New(identPullDirection{}, v)}
}

// WithMinRadius asks for the tightest concave radius — a measurement, not a
// verdict; the endmill spec lives with the caller (verification §2). On the
// analytic faces convexity and curvature are exact facts, so the survey
// answers outright: the tightest concave principal radius over every face,
// or nil — the proven determination that no concave feature exists.
func WithMinRadius() VerifyOption {
	return verifyOption{option.New(identMinRadius{}, true)}
}

// WithClearances asks for the minimum gap between disjoint pairs — a
// measurement, not a verdict (verification §2): the clearance spec lives
// with the caller, and the gate judges only the measurement's own figures.
// Each proven-disjoint pair gets a row whose Gap the clearance kernel proves
// (docs/clearance-design.md); a gap the kernel cannot prove yields no row
// and reads Suspect — asked and unanswered, never a fabricated number.
func WithClearances() VerifyOption {
	return verifyOption{option.New(identClearances{}, true)}
}

// verifyConfig is the folded option set. toolMM and allowRad carry the wall
// spec resolved to the solver's base units (millimetres, radians).
type verifyConfig struct {
	rel        float64
	wall       *wallSpec
	toolMM     float64
	allowRad   float64
	pull       *r3.Vec
	minRadius  bool
	clearances bool
}

// resolveVerifyOptions folds and validates the options. Every parameter
// error is returned from Verify — never deferred into the report
// (verification §2, core §10).
func resolveVerifyOptions(opts []VerifyOption) (verifyConfig, error) {
	cfg := verifyConfig{rel: 1e-3}
	for _, o := range opts {
		if o == nil {
			return verifyConfig{}, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identTolerance:
			v, ok := option.Get[units.Value](o)
			if !ok {
				return verifyConfig{}, fmt.Errorf(`%w: WithTolerance carries no value`, ErrDegenerate)
			}
			rel, err := magnitudeIn(v, units.Dimensionless, units.One, "tolerance")
			if err != nil {
				return verifyConfig{}, err
			}
			cfg.rel = rel
		case identMinWall:
			spec, ok := option.Get[wallSpec](o)
			if !ok {
				return verifyConfig{}, fmt.Errorf(`%w: WithMinWallThickness carries no spec`, ErrDegenerate)
			}
			if spec.nilOption {
				return verifyConfig{}, fmt.Errorf(`%w: a nil wall option names nothing to apply`, ErrDegenerate)
			}
			tool, err := magnitudeIn(spec.tool, units.Length, units.Millimeter, "wall tool")
			if err != nil {
				return verifyConfig{}, err
			}
			if tool == 0 {
				// No thickness is thinner than zero: a comparison with a
				// single outcome states no spec at all (verification §2).
				return verifyConfig{}, fmt.Errorf(`%w: a zero wall tool poses no question`, ErrDegenerate)
			}
			allow, err := magnitudeIn(spec.allowance, units.Angle, units.Radian, "draft allowance")
			if err != nil {
				return verifyConfig{}, err
			}
			if allow >= math.Pi/2 {
				// At 90° or beyond, skins meeting at a square corner would
				// count as opposing: no longer a question about walls
				// (verification §2). The legal range is [0°, 90°).
				return verifyConfig{}, fmt.Errorf(`%w: a draft allowance must be under 90 degrees, got %s`, ErrDegenerate, spec.allowance)
			}
			cfg.wall = &spec
			cfg.toolMM = tool
			cfg.allowRad = allow
		case identPullDirection:
			v, ok := option.Get[r3.Vec](o)
			if !ok {
				return verifyConfig{}, fmt.Errorf(`%w: WithPullDirection carries no direction`, ErrDegenerate)
			}
			for _, c := range []float64{v.X, v.Y, v.Z} {
				if math.IsNaN(c) || math.IsInf(c, 0) {
					return verifyConfig{}, fmt.Errorf(`%w: a pull direction must be finite, got %v`, ErrNotFinite, v)
				}
			}
			if v.X == 0 && v.Y == 0 && v.Z == 0 {
				return verifyConfig{}, fmt.Errorf(`%w: a zero pull direction poses no direction at all`, ErrDegenerate)
			}
			cfg.pull = &v
		case identMinRadius:
			cfg.minRadius = true
		case identClearances:
			cfg.clearances = true
		}
	}
	return cfg, nil
}

// Verify is one non-mutating call over the live model: solidity and boundary
// validity per body, every quantity judged by the tolerance gate, and the
// pairwise partition — proven overlap, proven disjointness, or Suspect
// (verification §1/§6). It mirrors sketch.Verify (core §10).
//
// Verify checks interference even when no options are passed. For each pair of
// proven solids that cheaper proofs do not settle, its mesh fallback checks
// every pair of operand facet boxes before pruning exact predicates. One pair's
// work can therefore grow with the two facet counts multiplied together, and
// total work also grows with the number of unresolved body pairs. Large-model
// callers should pass a context with a deadline chosen from representative
// inputs. For a document with live bodies, after document and option
// validation, cancellation returns ctx.Err() and a nil report; validation
// errors take precedence even when ctx is already canceled. An empty document
// retains its Sound result even when the context is already canceled. The
// document remains unchanged.
func (d *Document) Verify(ctx context.Context, opts ...VerifyOption) (*Report, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	cfg, err := resolveVerifyOptions(opts)
	if err != nil {
		return nil, err
	}
	report := &Report{Status: Sound}
	// Interferences is always computed — the Interfering rung reads it
	// (verification §1). The read-only proof never consumes an operand or
	// exposes a transient intersection through the document.
	report.Interferences = []Interference{}
	if cfg.clearances {
		report.Clearances = []Clearance{}
	}

	undecided := false // some asked question or pair this evaluator cannot decide

	var solids []*BodyReport
	for _, b := range d.bodies {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		br, bodyDiags, err := verifyBody(ctx, b, cfg)
		if err != nil {
			return nil, err
		}
		report.Bodies = append(report.Bodies, br)
		report.Diagnostics = append(report.Diagnostics, bodyDiags...)
		if br.Status == Suspect {
			undecided = true
		}
		if br.Status != Unsound && br.Solid {
			solids = append(solids, br)
		}
	}

	// The stable pair partition over proven solids (interference design §2):
	// box separation first, then the analytic relation, strict containment or
	// equality, and finally read-only intersection measurement. Only a proven
	// positive bounded volume emits an Interference row. Expected empty,
	// contact, staging, or coarse outcomes stay Suspect and name themselves in
	// the slice; invariant failures return from Verify.
	for i := range solids {
		for j := i + 1; j < len(solids); j++ {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			boxProven := boxesDisjoint(solids[i].Bounds, solids[j].Bounds)
			if boxProven && !cfg.clearances {
				continue
			}
			a, b := solids[i].Body, solids[j].Body
			res, err := clearancePair(ctx, a, b, boxProven)
			if err != nil {
				return nil, err
			}

			// Box separation already proves the partition. The analytic kernel
			// runs only to supply an asked gap; failure to measure that gap is
			// Suspect (DiagUndecidedClearance) but never sends a proven-disjoint
			// pair to intersection.
			if boxProven {
				if res.verdict == pairDisjoint || res.verdict == pairTouching {
					if d := appendClearance(report, a, b, res, cfg.rel); d != nil {
						report.Diagnostics = append(report.Diagnostics, *d)
						undecided = true
					}
				} else {
					report.Diagnostics = append(report.Diagnostics,
						pairDiagNone(a, b, DiagUndecidedClearance,
							"the pair is proven disjoint but the requested clearance gap is unmeasured"))
					undecided = true
				}
				continue
			}

			if res.verdict == pairDisjoint || res.verdict == pairTouching {
				if cfg.clearances {
					if d := appendClearance(report, a, b, res, cfg.rel); d != nil {
						report.Diagnostics = append(report.Diagnostics, *d)
						undecided = true
					}
				}
				continue
			}

			volume, outcome, err := measuredInterference(ctx, a, b, res)
			if err != nil {
				return nil, err
			}
			if outcome != interferenceMeasured {
				diag := undecidedPairDiag(a, b, res.verdict, outcome)
				if legacy, ok := legacyUnsupportedPairDiag(a, b, diag.Code); ok {
					report.Diagnostics = append(report.Diagnostics, legacy)
				}
				report.Diagnostics = append(report.Diagnostics, diag)
				undecided = true
				continue
			}
			report.Interferences = append(report.Interferences, Interference{A: a, B: b, Volume: volume})
			pairD, err := interferencePairDiameter(ctx, a, b)
			if err != nil {
				return nil, err
			}
			obs := volume
			interfere := Diagnostic{
				Code:     DiagInterference,
				Status:   Interfering,
				Pair:     &DiagnosticPair{A: a, B: b},
				Reading:  ReadingOverlapVolume,
				Observed: &obs,
				Message:  "the pair is proven to overlap",
			}
			report.Diagnostics = append(report.Diagnostics, interfere)
			pass, ref, haveRef := interferenceToleranceRef(volume, a, b, pairD, cfg.rel)
			if !pass {
				beyond := Diagnostic{
					Code:     DiagMeasurementBeyondTolerance,
					Status:   Suspect,
					Pair:     &DiagnosticPair{A: a, B: b},
					Reading:  ReadingOverlapVolume,
					Observed: &obs,
					Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", volume.Bound),
				}
				if haveRef {
					beyond.Required = requiredThreshold(cfg.rel*ref, volume.Value)
				}
				report.Diagnostics = append(report.Diagnostics, beyond)
				undecided = true
			}
		}
	}

	report.Status = aggregateStatus(report, undecided)
	return report, nil
}

func pairGapMeasurement(res pairResult) Measurement {
	gap := Measurement{
		Value:     units.Millimeters((res.lo + res.hi) / 2),
		Exactness: Exact,
		Bound:     units.Millimeters((res.hi - res.lo) / 2),
	}
	if !res.exact {
		gap.Exactness = Approximate
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
	pass, ref, haveRef := scalarToleranceRef(gap, rel, pairGate.lengthReference)
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
		d.Required = requiredThreshold(rel*ref, gap.Value)
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

// legacyUnsupportedPairDiag preserves the deprecated broad pair signal for
// callers that still branch on it. The cause-specific diagnostic remains the
// actionable entry and is appended separately by Verify.
func legacyUnsupportedPairDiag(a, b *Body, cause DiagnosticCode) (Diagnostic, bool) {
	switch cause {
	case DiagUnsupportedPairPayload, DiagUnsupportedPairContact, DiagUnsupportedPairPipeline:
		return pairDiagNone(a, b, DiagUnsupportedPair,
			"the pair cannot be decided because a read-only intersection stage is unsupported; inspect the accompanying cause-specific diagnostic for details"), true
	default:
		return Diagnostic{}, false
	}
}

// undecidedPairDiag picks the diagnostic for a pair whose overlap volume the
// evaluator could not measure (verification §1.1): payload, contact, and
// in-pipeline limits each keep their own code and action; only an overlap with
// an otherwise undecided measurement is DiagUndecidedInterference; an
// unresolved partition is DiagUndecidedPair. Verify adds the deprecated broad
// compatibility code alongside the three cause-specific outcomes.
func undecidedPairDiag(a, b *Body, verdict pairVerdict, outcome interferenceOutcome) Diagnostic {
	switch {
	case outcome == interferenceUnsupportedPayloadFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			fmt.Sprintf("the first operand (step %d) tessellates, but its tessellation refused at the chord tolerance this read-only check derives from the pair's own size, which no option sets; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach", a.originStep()))
	case outcome == interferenceUnsupportedPayloadSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			fmt.Sprintf("the second operand (step %d) tessellates, but its tessellation refused at the chord tolerance this read-only check derives from the pair's own size, which no option sets; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach", b.originStep()))
	case outcome == interferenceUnsupportedVolumeProofFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			fmt.Sprintf("the first operand (step %d) tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof", a.originStep()))
	case outcome == interferenceUnsupportedVolumeProofSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			fmt.Sprintf("the second operand (step %d) tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof", b.originStep()))
	case outcome == interferenceUnsupportedContact:
		if sharesFacePlane(a, b) {
			return pairDiagNone(a, b, DiagUnsupportedPairContact,
				"the pair reaches a contact or near-contact that the read-only intersection cannot classify, and the operands share a face plane; offset the operands so no face plane is shared, or adjust the geometry to create clear separation or deeper overlap, or wait for contact support")
		}
		return pairDiagNone(a, b, DiagUnsupportedPairContact,
			"the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support")
	case outcome == interferenceUnsupportedPipeline:
		return pairDiagNone(a, b, DiagUnsupportedPairPipeline,
			"both operands tessellate, but later read-only intersection geometry exceeds the boolean pipeline's reach; simplify the boolean geometry or wait for pipeline support")
	case outcome == interferenceUndecided && verdict == pairOverlapping:
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
func canonicalFacePlaneKey(p Plane) facePlaneKey {
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
		if p, ok := f.Surface().(Plane); ok {
			keys[canonicalFacePlaneKey(p)] = struct{}{}
		}
	}
	for _, f := range b.Faces() {
		p, ok := f.Surface().(Plane)
		if !ok {
			continue
		}
		if _, found := keys[canonicalFacePlaneKey(p)]; found {
			return true
		}
	}
	return false
}

// verifyBody audits one body and assembles its report. A feature-built body
// is valid by construction, and the proof is the construction (evaluator
// §10): the structural audit is an invariant check, cheap, and its verdict
// is decided, not sampled.
func verifyBody(ctx context.Context, b *Body, cfg verifyConfig) (*BodyReport, []Diagnostic, error) {
	br := &BodyReport{
		Body:   b,
		Status: Sound,
		Area:   b.area,
		Bounds: b.bounds,
		Lumps:  len(b.lumps),
	}
	for _, s := range b.Shells() {
		if s.IsVoid() {
			br.Voids++
		}
	}

	// The held boundary's structural audit: every edge bounds exactly two
	// faces, every face carries at least one loop of at least one edge.
	// This evaluator's boundary is exact, so a defect is proven — Unsound —
	// and a clean audit on a feature-built body is proven validity.
	clean := auditBoundary(b)
	built := b.payload != nil
	switch {
	case !clean:
		br.Status = Unsound
	case built && b.solid:
		br.Solid = true
		br.Watertight = true
		br.Manifold = true
		br.SelfIntersecting = false
		vol := b.volume
		cen := b.centroid
		br.Volume = &vol
		br.Centroid = &cen
	default:
		// A body this evaluator did not build (unreachable through the
		// public API) has a validity the audit alone cannot prove:
		// undecided, Suspect — never a fabricated pass.
		br.Status = Suspect
	}

	var diags []Diagnostic

	// A proven-invalid body emits one DiagInvalidBody and no region-quantity
	// diagnostics, because §1 gives it no region quantity to gate. The gate
	// still runs, discarded, to surface a cancellation observed while lazily
	// loading a reference.
	if br.Status == Unsound {
		br.Exactness = bodyExactness(br)
		if _, err := bodyReadingDiagnostics(ctx, br, cfg.rel); err != nil {
			return nil, nil, err
		}
		diags = append(diags, Diagnostic{
			Code:    DiagInvalidBody,
			Status:  Unsound,
			Body:    b,
			Reading: ReadingNone,
			Message: "the held boundary is proven not a valid solid",
		})
		return br, diags, nil
	}

	// An undecided validity (a body this evaluator did not build, unreachable
	// through the public API) is Suspect, and names itself in the slice.
	if br.Status == Suspect {
		diags = append(diags, Diagnostic{
			Code:    DiagUndecidedValidity,
			Status:  Suspect,
			Body:    b,
			Reading: ReadingNone,
			Message: "the held boundary's validity is not decisive beyond its own proven bound",
		})
	}

	// The asked opt-in surveys (evaluator §10, verification §6): answered
	// outright on this evaluator's analytic bodies — validity is decided
	// first, so only a proven solid carries the readings. A survey that
	// cannot decide (a payload no shipped feature builds) leaves the asked
	// question undecided, and a stated spec proven to fail is Violating. Each
	// non-Sound survey outcome names itself in the slice.
	violating, suspect := false, false
	if br.Solid && (cfg.wall != nil || cfg.pull != nil || cfg.minRadius) {
		surveyDiags, err := runSurveys(newWorkBudget(ctx), br, cfg)
		if err != nil {
			return nil, nil, err
		}
		for _, d := range surveyDiags {
			switch d.Status {
			case Violating:
				violating = true
			case Suspect:
				suspect = true
			}
		}
		diags = append(diags, surveyDiags...)
	}

	// Exactness is summary metadata only. The total tolerance gate runs after
	// every requested survey and judges each present bounded result by its own
	// inclusive Bound <= rel*Ref comparison (verification §2/§3/§6); each
	// reading beyond tolerance emits its own DiagMeasurementBeyondTolerance.
	br.Exactness = bodyExactness(br)
	toleranceDiags, err := bodyReadingDiagnostics(ctx, br, cfg.rel)
	if err != nil {
		return nil, nil, err
	}
	if len(toleranceDiags) > 0 {
		suspect = true
		diags = append(diags, toleranceDiags...)
	}

	// Worst wins at the body level: Violating > Suspect > Sound
	// (verification §6).
	if suspect {
		br.Status = Suspect
	}
	if violating {
		br.Status = Violating
	}
	return br, diags, nil
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
			// at all — a full sphere or a full torus needs no boundary
			// loop. Any other loop-less face is a defect.
			switch f.Surface().Kind() {
			case KindSphere, KindTorus:
				continue
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
		if len(e.faces) != 2 {
			return false
		}
	}
	return true
}

// bodyExactness is the weakest link across the quantities the report
// carries (verification §1).
func bodyExactness(br *BodyReport) Exactness {
	worst := max(br.Area.Exactness, br.Bounds.Exactness)
	if br.Volume != nil {
		worst = max(worst, br.Volume.Exactness)
	}
	if br.Centroid != nil {
		worst = max(worst, br.Centroid.Exactness)
	}
	if br.MinWallThickness != nil {
		worst = max(worst, br.MinWallThickness.Exactness)
	}
	if br.MinRadius != nil {
		worst = max(worst, br.MinRadius.Exactness)
	}
	return worst
}

// boxesDisjoint reports whether the two bounds-inflated boxes have disjoint
// interiors. A body's interior lies within its box's interior, so disjoint
// box interiors prove the bodies cannot share volume — touching boxes
// included (evaluator §10).
func boxesDisjoint(a, b Box) bool {
	ia := a.Bound.Base()
	ib := b.Bound.Base()
	overlapping := a.Max.X+ia > b.Min.X-ib && b.Max.X+ib > a.Min.X-ia &&
		a.Max.Y+ia > b.Min.Y-ib && b.Max.Y+ib > a.Min.Y-ia &&
		a.Max.Z+ia > b.Min.Z-ib && b.Max.Z+ib > a.Min.Z-ia
	return !overlapping
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
