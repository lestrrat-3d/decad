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
type identConcaveRadius struct{}
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

// WithMinWallThickness states the spec that no wall may be thinner than
// minimum (verification §2), describing the comparison without assuming a
// tool type. The reading is the infimum diameter over the body's spanning
// inscribed balls — material between skins opposing within the draft
// allowance — and minimum enters only where the interval rule decides the
// reading against it (verification §6). A wall proven thinner is Violating.
func WithMinWallThickness(minimum units.Value, opts ...WallOption) VerifyOption {
	spec := wallSpec{tool: minimum, allowance: units.Degrees(15)}
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
// its range by its own proven departure before this comparison. It also
// answers on a proven-valid surface-extruded prism sheet, reading each
// wall's positive side in place of a solid's outward normal
// (docs/surface-design.md §2.3, §9.1); every other sheet family reads
// CoverageUnavailable.
func WithPullDirection(v r3.Vec) VerifyOption {
	return verifyOption{option.New(identPullDirection{}, v)}
}

// WithConcaveRadius asks for the tightest concave radius — a measurement,
// not a verdict; Verify introduces no radius threshold or machining-access
// assessment of its own (verification §2). On the analytic faces convexity
// and curvature are exact facts, so the survey answers outright: the
// tightest concave principal radius over every face, or nil — the proven
// determination that no concave feature exists.
func WithConcaveRadius() VerifyOption {
	return verifyOption{option.New(identConcaveRadius{}, true)}
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
	rel           float64
	wall          *wallSpec
	toolMM        float64
	allowRad      float64
	pull          *r3.Vec
	concaveRadius bool
	clearances    bool
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
		case identConcaveRadius:
			cfg.concaveRadius = true
		case identClearances:
			cfg.clearances = true
		}
	}
	return cfg, nil
}

// effectiveVerifyRequest resolves cfg into the effective-request record
// (proposal §5): the validated settings this Verify call actually used,
// canonicalized to millimetres and radians, recorded even for an empty
// document. The pull vector is kept exactly as accepted — never normalized —
// so the recorded request shows the actual input the survey consumed.
func effectiveVerifyRequest(cfg verifyConfig) VerifyRequest {
	req := VerifyRequest{
		RelativeTolerance: units.Scalar(cfg.rel),
		ConcaveRadius:     cfg.concaveRadius,
		Clearances:        cfg.clearances,
	}
	if cfg.wall != nil {
		req.Wall = &WallRequest{
			Minimum:        units.Millimeters(cfg.toolMM),
			DraftAllowance: units.Radians(cfg.allowRad),
		}
	}
	if cfg.pull != nil {
		req.Undercut = &UndercutRequest{PullDirection: *cfg.pull}
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
	if cfg.clearances {
		report.Clearances = []Clearance{}
	}

	undecided := false // some asked question or pair this evaluator cannot decide

	// pairBodies holds every proven-valid body — solid or sheet alike — in
	// Document.Bodies() order (interference design §2): a body whose own
	// validity is undecided or invalid never reaches pair work at all, which
	// is what keeps §9.3's sheet pair rule from ever running on a sheet that
	// is not itself proven sound.
	var pairBodies []*BodyReport
	for _, b := range d.bodies {
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
	// see the first arm below and docs/surface-design.md §9.3.
	var geomCache *bodyGeomCache
	if cfg.clearances {
		geomCache = &bodyGeomCache{}
	}
	for i := range pairBodies {
		for j := i + 1; j < len(pairBodies); j++ {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			a, b := pairBodies[i].Body, pairBodies[j].Body
			boxProven := boxesDisjoint(pairBodies[i].Bounds.Box, pairBodies[j].Bounds.Box)

			// A sheet operand encloses no region, so the interference
			// relation §1 decides is not the question for this pair
			// (docs/surface-design.md §9.3). A sheet-sheet pair offers no
			// closed boundary to cast against, so it takes only box
			// separation: separated boxes contribute nothing, and boxes that
			// meet stay DiagUnsupportedPairSheet regardless of
			// WithClearances(). A sheet-against-solid pair is decided by
			// sheetSolidPair (clearance.go) into a proven crossing, a proven
			// containment or separation with a measured gap, or the
			// undecided code when the kernel cannot settle it — run
			// whenever the boxes meet (crossing must always be checked,
			// asked or not), and also when the boxes are separated but a
			// gap was requested, exactly as the solid-solid path below runs
			// clearancePair in that same second case.
			if a.Kind() == BodySheet || b.Kind() == BodySheet {
				if a.Kind() == BodySheet && b.Kind() == BodySheet {
					if boxProven {
						continue
					}
					report.Diagnostics = append(report.Diagnostics,
						pairDiagNone(a, b, DiagUnsupportedPairSheet,
							"both operands are sheet bodies, and neither offers a closed boundary to cast the other against"))
					undecided = true
					continue
				}
				if boxProven && !cfg.clearances {
					continue
				}
				sheet, solid := a, b
				if b.Kind() == BodySheet {
					sheet, solid = b, a
				}
				sres, err := sheetSolidPair(ctx, sheet, solid, boxProven, geomCache)
				if err != nil {
					return nil, err
				}
				switch sres.verdict {
				case sheetSolidCrossing:
					report.Diagnostics = append(report.Diagnostics, Diagnostic{
						Code:    DiagSheetSolidCrossing,
						Status:  Interfering,
						Pair:    &DiagnosticPair{A: a, B: b},
						Reading: ReadingNone,
						Message: "the sheet crosses the solid's boundary; no Interference row is emitted because a sheet encloses no region and there is no overlap volume to report",
					})
				case sheetSolidContained, sheetSolidOutside:
					if cfg.clearances {
						pr := pairResult{lo: sres.lo, hi: sres.hi, exact: sres.exact, diam: sres.diam}
						if d := appendClearance(report, a, b, pr, cfg.rel); d != nil {
							report.Diagnostics = append(report.Diagnostics, *d)
							undecided = true
						}
					}
				case sheetSolidUndecided:
					if boxProven {
						// Box separation already proves the sheet lies
						// outside the solid; only the requested gap itself
						// is unmeasured, the same shape of gap as an
						// unmeasured solid-solid clearance below.
						report.Diagnostics = append(report.Diagnostics,
							pairDiagNone(a, b, DiagUndecidedClearance,
								"the pair is proven disjoint but the requested clearance gap is unmeasured"))
					} else {
						report.Diagnostics = append(report.Diagnostics,
							pairDiagNone(a, b, DiagUnsupportedPairSheet,
								"the clearance kernel could not settle this sheet-against-solid pair"))
					}
					undecided = true
				}
				continue
			}

			if boxProven && !cfg.clearances {
				continue
			}
			res, fast := clearanceAxisBoxes(a, b)
			if !fast {
				var pairErr error
				res, pairErr = clearancePairCached(ctx, a, b, boxProven, geomCache)
				if pairErr != nil {
					return nil, pairErr
				}
			}
			if err := ctx.Err(); err != nil {
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
	return publishReport(req, report.Bodies, report.Interferences, report.Clearances, report.Diagnostics, report.Status), nil
}

func pairGapMeasurement(res pairResult) Measurement {
	midpoint := boundedMul(boundedAdd(exactScalar(res.lo), exactScalar(res.hi)), exactScalar(0.5))
	halfWidth := boundedMul(boundedSub(exactScalar(res.hi), exactScalar(res.lo)), exactScalar(0.5))
	// The midpoint error and the half-width's own error both widen the
	// published interval. The same bound feeds appendClearance's tolerance gate.
	bound := absSumUpper(halfWidth.value, halfWidth.bound, midpoint.bound)
	gap := Measurement{
		Value:     units.Millimeters(midpoint.value),
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

// undecidedPairDiag picks the diagnostic for a pair whose overlap volume the
// evaluator could not measure (verification §1.1): payload, contact, and
// in-pipeline limits each keep their own code and action; only an overlap with
// an otherwise undecided measurement is DiagUndecidedInterference; an
// unresolved partition is DiagUndecidedPair. Verify emits only this
// cause-specific diagnostic — the deprecated broad DiagUnsupportedPair
// constant stays declared for existing callers that still branch on it, but
// no longer appears in a returned report (proposal §10). A pair holding a
// sheet operand never reaches this function at all: it is resolved earlier,
// by box separation or the sheet decision procedure (sheetSolidPair,
// clearance.go), and takes DiagUnsupportedPairSheet or DiagSheetSolidCrossing
// instead (docs/surface-design.md §9.3).
func undecidedPairDiag(a, b *Body, verdict pairVerdict, outcome interferenceOutcome) Diagnostic {
	switch {
	case outcome == interferenceUnsupportedPayloadFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the first operand tessellates, but its tessellation refused at the chord tolerance this read-only check derives from the pair's own size, which no option sets; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach")
	case outcome == interferenceUnsupportedPayloadSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the second operand tessellates, but its tessellation refused at the chord tolerance this read-only check derives from the pair's own size, which no option sets; simplify that operand or reduce the pair's extent, or wait for wider tessellation reach")
	case outcome == interferenceUnsupportedVolumeProofFirst:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the first operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof")
	case outcome == interferenceUnsupportedVolumeProofSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			"the second operand tessellates, but its mesh carries no proof of the volume it and the body it stands for differ by, so no read-only intersection may compose it; keep this body out of overlapping pairs, or wait for its occupied-volume proof")
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

// verifyBody audits one body and assembles its BodyReport through
// verify_publish.go's publishBodyResult. A feature-built body is valid by
// construction, and the proof is the construction (evaluator §10): the
// structural audit is an invariant check, cheap, and its verdict is decided,
// not sampled. Exposed at package scope so an in-package test can construct
// a body directly and assert on the returned BodyReport (task-list §4
// item 4's validity fixtures).
func verifyBody(ctx context.Context, b *Body, cfg verifyConfig, req VerifyRequest) (*BodyReport, error) {
	area := b.area
	bounds := b.bounds

	// The held topology's lump and void counts (proposal §9): descriptive
	// data every body carries, using the current count definitions,
	// independent of the validity verdict decided below.
	topology := HeldTopology{Lumps: len(b.lumps)}
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
	evidence := validityEvidence{Kind: b.Kind()}
	switch b.Kind() {
	case BodySheet:
		evidence.Sheet = auditSheetBoundary(ctx, b)
	default:
		evidence.Clean = auditBoundary(b)
		evidence.Built = b.payload != nil
		evidence.Solid = b.solid
	}
	validity := publishValidityResult(b, evidence)
	haveRegion := validity.Outcome == ValidityValid && b.Kind() == BodySolid

	var vol Measurement
	var cen VecMeasurement
	status := Sound
	switch validity.Outcome {
	case ValidityInvalid:
		status = Unsound
	case ValidityValid:
		vol = b.volume
		cen = b.centroid
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
		if _, _, err := bodyReadingDiagnostics(ctx, b, bodyReadingSet{Area: area, Bounds: bounds}, cfg.rel); err != nil {
			return nil, err
		}
		return publishBodyResult(bodyPublishInput{
			Body:     b,
			Status:   status,
			Validity: validity,
			Topology: topology,
			Area:     ScalarReading{Measurement: area},
			Bounds:   BoundsReading{Box: bounds},
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
	surveysAsked := cfg.wall != nil || cfg.pull != nil || cfg.concaveRadius
	// surveysRunnable widens haveRegion by exactly one case: a proven sheet
	// with a pull requested. The undercut survey answers over a sheet's own
	// walls (docs/surface-design.md §2.3, §9.1), and runSurveys itself
	// gates the wall and concave-radius blocks back down to a solid, so
	// admitting a sheet here never lets those two surveys run on one.
	surveysRunnable := haveRegion ||
		(validity.Outcome == ValidityValid && b.Kind() == BodySheet && cfg.pull != nil)
	violating, suspect := false, false
	var surveys surveyResults
	switch {
	case surveysRunnable && surveysAsked:
		var surveyDiags []Diagnostic
		var err error
		surveys, surveyDiags, err = runSurveys(newWorkBudget(ctx), b, cfg)
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
		// publishWallResult/publishConcaveRadiusResult, not by runSurveys, so
		// they never appear in surveyDiags above; a sheet that asked either
		// one still owes this body a Suspect for it.
		if b.Kind() == BodySheet && (cfg.wall != nil || cfg.concaveRadius) {
			suspect = true
		}
	case validity.Outcome == ValidityValid && surveysAsked:
		// surveysRunnable is false here only because the body is a proven
		// sheet that asked no pull — a wall and/or concave-radius question
		// with no material to be about (docs/surface-design.md §9.1).
		// runSurveys never runs; publishWallResult and
		// publishConcaveRadiusResult each publish Unavailable plus their own
		// DiagSurveyPrerequisite from validity and kind alone, so this
		// body's Status must already carry that Suspect verdict.
		suspect = true
	}

	var wallReading, radiusReading *Measurement
	if surveys.Wall.reading != nil {
		m := lengthMeasurement(*surveys.Wall.reading, surveys.Wall.bound)
		wallReading = &m
	}
	if surveys.Radius.reading != nil {
		m := lengthMeasurement(*surveys.Radius.reading, surveys.Radius.bound)
		radiusReading = &m
	}

	var volPtr *Measurement
	var cenPtr *VecMeasurement
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
	verdicts, diagSet, err := bodyReadingDiagnostics(ctx, b, readings, cfg.rel)
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
			Volume:   ScalarReading{Measurement: vol, Tolerance: verdicts.Volume},
			Centroid: VectorReading{VecMeasurement: cen, Tolerance: verdicts.Centroid},
		}
	}

	return publishBodyResult(bodyPublishInput{
		Body:                b,
		Status:              status,
		Validity:            validity,
		Topology:            topology,
		Area:                ScalarReading{Measurement: area, Tolerance: verdicts.Area},
		Bounds:              BoundsReading{Box: bounds, Tolerance: verdicts.Bounds},
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

// sheetAuditOutcome is auditSheetBoundary's three-way verdict.
type sheetAuditOutcome int

const (
	// sheetAuditProven — every structural leg holds and non-self-intersection
	// is admitted by construction.
	sheetAuditProven sheetAuditOutcome = iota
	// sheetAuditViolated — a structural leg is proven to fail.
	sheetAuditViolated
	// sheetAuditUndecided — every structural leg holds, but this payload does
	// not admit non-self-intersection by construction.
	sheetAuditUndecided
)

// auditSheetBoundary decides Table V's sheet validity audit
// (docs/surface-design.md §9.1): three structural legs read off the recorded
// topology, plus non-self-intersection admitted BY CONSTRUCTION alone. It is
// deliberately NOT §6.4's closure audit (`loftCrossingAudit`) with closure
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
// Leg 4 — non-self-intersection, decided by payloadProvesSimple on the
// body's own payload. See that function's doc comment for the admission rule,
// one payload at a time.
func auditSheetBoundary(ctx context.Context, b *Body) sheetAuditOutcome {
	faces := b.Faces()
	if len(faces) == 0 {
		return sheetAuditViolated
	}
	for _, f := range faces {
		loops := f.Loops()
		if len(loops) == 0 {
			return sheetAuditViolated
		}
		for _, l := range loops {
			if len(l.Edges()) == 0 {
				return sheetAuditViolated
			}
		}
	}

	for _, e := range b.Edges() {
		if len(e.faces) == 0 || len(e.faces) > 2 {
			return sheetAuditViolated
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
		if uses[e] != len(e.faces) {
			return sheetAuditViolated
		}
		if len(e.faces) == 2 && (forward[e] != 1 || backward[e] != 1) {
			return sheetAuditViolated
		}
	}

	if payloadProvesSimple(ctx, b.payload) {
		return sheetAuditProven
	}
	return sheetAuditUndecided
}

// payloadProvesSimple decides, in the one place both callers share, whether
// the body a payload records already proves its own boundary does not
// self-intersect — Table V's leg 4. auditSheetBoundary asks it above; a later
// Stitch increment asks the identical question of an operand's SOURCE
// payload before folding its faces into a curved-solid volume/centroid
// reading, and hoisting the switch here is what keeps the two callers from
// ever answering it two different ways.
//
// A prismPayload with surfaceResult true and sectionDelta zero is proven by
// construction: `sketch` already decided the recorded segments form the
// stated simple closed planar region; evalPrismContext already refuses a
// non-positive height; a simple planar curve crossed with a positive
// interval does not self-intersect; and pp.xform is rigid, so it preserves
// that. A nonzero sectionDelta denotes a set the record is only WITHIN that
// displacement of, so simplicity does not transfer and the answer is
// undecided.
//
// A loftPayload with surfaceResult true and sectionDelta zero admits leg 4 on
// an argument STRONGER than the prism's: the evaluator cannot return a loft
// body at ALL unless docs/loft-design.md §6's crossing audit already passed
// over the complete held triangle set — walls and both caps together — and
// non-self-intersection of a SUBSET (the walls alone, once the caps are
// omitted) follows from non-self-intersection of that superset with no
// further proof needed. A positive sectionDelta means the body denotes a
// curved surface the held chords are only WITHIN that displacement of, so
// simplicity of the chord mesh does not transfer to the curved surface it
// stands for, and the answer is undecided rather than violated — the
// identical reading a nonzero sectionDelta gives a prism, restated here
// because a loft's own displacement is section-plane rather than axial.
//
// A chainLoftPayload records only the held flat ribbon triangles. Its build
// runs the complete crossing audit before committing a body, and placement
// rebuilds and audits that triangle set again. A committed chain loft thus
// already proves leg 4 over the exact sheet Verify reads.
//
// A stitchPayload is proven instead by an explicit build-time audit: Stitch
// runs docs/loft-design.md §6's crossing audit on the OPEN case too
// (docs/surface-design.md §6.3's open-case decision), so a stitchPayload
// whose audit ran and passed (auditClean) admits leg 4 the same as a
// proven-simple prism or loft does, letting a clean stitched sheet read
// ValidityValid instead of being permanently Suspect. A bodyPatchPayload
// admits under Rule P (patch_body.go's bodyPatchPayloadProvesSimple,
// docs/surface-design.md §6.4): Body.Patch itself proves only its own
// chains simple in their own plane (gate 4, §5.2), never the whole
// assembled boundary's non-self-intersection, but where its receiver itself
// admits this same predicate, every new chain is a COMPLETE free-edge chain
// of that receiver's own end, and every new chain's plane is proven — by
// the LEVEL half of the shared-denotation certificate's own identity, never
// a coordinate or a residual — to be exactly one of that receiver's own end
// planes, the patched assembly IS the receiver's own solid boundary and its
// proof carries. Any chain this cannot decide keeps the payload undecided.
//
// A revolvePayload admits leg 4 on the CONSTRUCTION argument
// revolvePayloadProvesSimple states in full — its build runs no crossing
// audit over a triangle set at all, so the proof is a different shape than
// the prism's or loft's, not merely a weaker version of it. Any other
// payload, or a nil one, is undecided.
//
// A sweepPayload admits leg 4 for the ONE-SPAN STRAIGHT reduction alone
// (len(spans) == 0, arc false), on exactly the prismPayload argument above,
// which transfers verbatim: the line reduction IS a prismPayload build. The
// arc-reduced and composite sheets read undecided instead, and deliberately —
// this is not an oversight to lift later without a new proof:
//
//   - the arc reduction is a revolvePayload build whose own sweep is a
//     PARTIAL turn (full == false) — revolvePayloadProvesSimple's own
//     admission needs a full turn, so an arc-reduced sweep never qualifies;
//   - the composite build's own auditCompositeBoundary/auditCompositeVertexLinks
//     (sweep_composite.go) prove a closed two-manifold-with-boundary TOPOLOGY —
//     every edge's face count matches its use count, every vertex link is one
//     cycle or path — which is a combinatorial fact about how the spans sew
//     together, never a geometric claim that the swept walls do not fold back
//     and cross themselves in space. That geometric claim is precisely what
//     this leg (non-self-intersection) asks, and it is exactly the audit this
//     increment relaxes to admit a sheet's free rims in the first place, so it
//     cannot also be read as proving the thing it was relaxed away from.
func payloadProvesSimple(ctx context.Context, p featurePayload) bool {
	switch pp := p.(type) {
	case prismPayload:
		return pp.surfaceResult && pp.sectionDelta == 0
	case loftPayload:
		return pp.surfaceResult && pp.sectionDelta == 0
	case chainLoftPayload:
		return true
	case stitchPayload:
		return pp.auditClean
	case bodyPatchPayload:
		return bodyPatchPayloadProvesSimple(ctx, pp)
	case sweepPayload:
		return len(pp.spans) == 0 && !pp.arc && pp.prism.surfaceResult && pp.prism.sectionDelta == 0
	case revolvePayload:
		return revolvePayloadProvesSimple(ctx, pp)
	case chainPayload:
		// A chain-fed prism earns leg 4 on the profile argument with one word
		// changed (docs/surface-design.md §13.4): `sketch` already proved the
		// recorded walk a simple planar curve — RecordChain's own ch.Valid gate
		// (§13.3) — evalChainExtrudeContext already refuses a non-positive
		// sweep height, and a simple planar curve crossed with a positive
		// interval cannot self-intersect. A chain-fed body is always a sheet
		// by construction (Table G), so surfaceResult carries no separate
		// condition here; sectionDelta is the one term that can still break
		// the argument, on the identical prismPayload reading above.
		return pp.sectionDelta == 0
	case chainSweepPayload:
		// A chain-fed sweep's only reduction reachable today is the one-span
		// straight case (docs/sweep-design.md §15): SweepChain builds the body
		// through the identical evalChainExtrudeContext a plain ExtrudeChain
		// does, over a chainPayload it then wraps unchanged (sweep.go's
		// finishChainSweepBody). So this is the SAME construction argument
		// chainPayload's own arm makes, re-read off the wrapped payload —
		// never a weaker version of it, since chainSweepPayload carries no
		// arc or composite shape of its own yet for the argument to fail to
		// cover.
		return pp.chain.sectionDelta == 0
	case chainRevolvePayload:
		// A chain-fed revolve earns leg 4 on the identical full-turn argument,
		// re-read over the chain's own open walk through the SAME generic
		// reading revolvePayloadProvesSimple already takes off rp.profile and
		// rp.ax — neither of which assumes the walk closes
		// (docs/surface-design.md §13.4).
		return revolvePayloadProvesSimple(ctx, pp.revolve())
	default:
		return false
	}
}

// revolvePayloadProvesSimple decides payloadProvesSimple's revolvePayload
// arm: a revolve's own build runs no crossing audit over a triangle set the
// way a loft's does, so its non-self-intersection proof has to be a
// construction argument over the profile and the resolved axis instead.
//
// THE EMBEDDEDNESS ARGUMENT, repaired. `sketch` already proved the recorded
// profile a simple closed planar region. Every boundary stretch lying ON the
// resolved axis is classified wallAxis (revolve_axis.go's axisFrame.classify)
// and buildRevolveLoop skips it when assembling faces — it sweeps a
// zero-area set and emits no face at all — so the finished face set is the
// revolution of the boundary's OFF-axis part alone. Two distinct off-axis
// boundary points can only map to the same 3D point by sharing the same
// radius and axial position at the same swept angle, and a simple curve
// never repeats a (radius, axial) pair off the axis, so that face set is
// injective off the axis. full == true is what makes the angular fibre
// traverse the sweep exactly once — any other multiple of a turn would cover
// some off-axis point more than once — so both full and the radial condition
// below are load-bearing, and neither alone is enough. This is deliberately
// NOT the claim that the revolution "meets the axis in at most a point":
// semicircleSketch's diameter lies wholly ON the axis, and its sphere sheet
// is a direct counterexample to that stronger claim. The argument above never
// needs it — an on-axis stretch contributes no face to intersect anything
// with in the first place.
//
// THE RADIAL CONDITION IS STRICTER THAN THE BUILD GATE, on purpose.
// resolveAxisSide already refused, at build, any profile whose radial extreme
// does not clear the axis to within tol = 1e-9 * max(1, scale) — it proves
// only that the radial minimum is not below −tol, never that it is
// non-negative. An admission gate resting on that tolerance is exactly what
// CLAUDE.md's reject-only rule forbids outright, so this leg re-decides the
// question at zero instead of inheriting the build's tolerance: it admits
// only when the radial minimum is PROVEN non-negative (rlo − rBound >= 0),
// not merely un-disproven.
//
// NO FREE-FORM REFUSAL IS NEEDED HERE. rejectInteriorContact
// (revolve_axis.go) calls requireAnalyticWalk on every segment before its own
// circularity check, and resolveAxisSide runs on every revolve build
// (revolve.go). So a revolvePayload that built at all already has an
// all-analytic profile and can never carry a free-form face — adding a
// redundant free-form gate here would be dead code guarding a case this
// arm can never reach.
//
// A direct Revolve can carry the strict positive-side proof produced by
// resolveAxisSide's scan of this same profile and axis. Its scan adds a
// dot-product charge that makes the proof at least as strict as this scan.
// Placement keeps the profile and axis, so it keeps that proof; a changed
// profile or axis clears it. Other payloads take the scan below.
//
// The radial extreme is otherwise recomputed against rp.ax, the AXIS THE
// BUILD ALREADY RESOLVED, rather than re-deriving a side or a tolerance: rp.ax's
// own doc comment (axisFrame) states ρ = cross(d, p−a) as its radial
// coordinate with the region already oriented onto its non-negative side, so
// evaluating that same functional's extreme over the recorded profile reads
// the identical quantity resolveAxisSide decided from, at the strict zero
// bound this leg requires instead of the build's ±tol band. A failure to
// measure it (never observed against a payload this evaluator built, since
// the identical scan already succeeded once at build time over the same
// profile) is read as un-proven rather than as an error: this leg only ever
// admits, so withholding admission is always the safe outcome.
//
// boundaryExtremesBoundedContext reads the raw functional nU*u+nV*v over the
// profile boundary and charges every rounding IT commits into rBound. It does
// NOT charge the axis anchor's own offset, roff = nU*aU+nV*aV, which this leg
// still has to subtract to reach the axis-relative ρ axisFrame's own doc
// comment defines: uAxis (this arm's own fixtures) happens to anchor at the
// origin, where aU and aV are both exactly zero and the shift is a no-op, but
// an axis anchored elsewhere makes roff genuinely nonzero, and the multiplies,
// the add and the subtraction that compute it each commit their OWN
// round-to-nearest error on top of whatever aU/aV/dU/dV's own proven bounds
// already are. A bare `rlo -= roff` would leave that error unaccounted, so
// the strict-zero comparison below could read a NEGATIVE true radial minimum
// as non-negative by exactly the rounding it dropped — the identical fault
// this leg exists to close in resolveAxisSide's build-time tolerance, only
// smaller. boundedMul/boundedAdd/boundedSub charge both the operands' own
// proven bounds and this arithmetic's own commissioned rounding (the same
// vocabulary axisFrame.toAxisRhoBound already uses for the identical ρ
// formula at a single point), so the bound handed to admitBelow below
// provably covers every operation between the extremes call and the
// comparison — never a number the accompanying bound does not cover.
func revolvePayloadProvesSimple(ctx context.Context, rp revolvePayload) bool {
	if !rp.full {
		return false
	}
	if ctx.Err() != nil {
		return false
	}
	if rp.radialProof {
		return true
	}
	nU, nV := -rp.ax.dV, rp.ax.dU
	rawLo, _, rawBound, err := boundaryExtremesBoundedContext(ctx, rp.profile, nU, nV, newFreeformWork(), nil)
	if err != nil {
		return false
	}
	offset := boundedAdd(
		boundedMul(measuredScalar(nU, rp.ax.dVBound), measuredScalar(rp.ax.aU, rp.ax.aUBound)),
		boundedMul(measuredScalar(nV, rp.ax.dUBound), measuredScalar(rp.ax.aV, rp.ax.aVBound)),
	)
	rlo := boundedSub(measuredScalar(rawLo, rawBound), offset)
	return admitBelow(rlo, 0) == survReject
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
