package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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

// Status is a verdict, used at both the body and the report level
// (verification §6). Unverified is the reserved zero value and is never
// returned by a successful Verify. Severity among verification results is by
// precedence, worst wins: Unsound > Interfering > Violating > Suspect > Sound.
type Status int

const (
	// Unverified: no verification produced this value. It is reserved so a
	// zero or partially decoded Report fails Trustworthy.
	Unverified Status = iota
	// Sound: every body a proven solid, every stated spec met, every asked
	// absence proven, nothing approximate beyond tolerance.
	Sound
	// Suspect: nothing proven wrong and something not proven right — an
	// answer beyond the caller's tolerance, an asked question undecided, an
	// undecided pair.
	Suspect
	// Violating: a spec a VerifyOption stated is proven to fail.
	Violating
	// Interfering: bodies provenly overlap. A report verdict only — overlap
	// is a property of a pair, so no single body is ever Interfering.
	Interfering
	// Unsound: some body is proven not a valid solid.
	Unsound
)

// String renders the status for diagnostics.
func (s Status) String() string {
	switch s {
	case Unverified:
		return "Unverified"
	case Sound:
		return "Sound"
	case Suspect:
		return "Suspect"
	case Violating:
		return "Violating"
	case Interfering:
		return "Interfering"
	case Unsound:
		return "Unsound"
	default:
		return fmt.Sprintf("Status(%d)", int(s))
	}
}

// ReadingKind names which measured quantity a diagnostic's Observed* form
// carries (verification §1.1) — a named-text enum with a stable String() like
// every other closed set decad owns. Exactly one of Diagnostic.Observed /
// ObservedVec / ObservedBox is non-nil, and this says which: a scalar rides
// Observed, a Centroid ObservedVec, a Bounds box ObservedBox. ReadingNone means
// the reason names no bounded reading and all three are nil.
type ReadingKind int

const (
	// ReadingNone — the reason names no bounded reading; every Observed* is nil.
	ReadingNone ReadingKind = iota
	// ReadingArea — a body's Area (Observed).
	ReadingArea
	// ReadingBounds — a body's Bounds box (ObservedBox).
	ReadingBounds
	// ReadingVolume — a body's Volume (Observed).
	ReadingVolume
	// ReadingCentroid — a body's Centroid (ObservedVec).
	ReadingCentroid
	// ReadingWall — a body's MinWallThickness (Observed).
	ReadingWall
	// ReadingMinRadius — a body's MinRadius (Observed).
	ReadingMinRadius
	// ReadingOverlapVolume — a pair's proven overlap volume (Observed).
	ReadingOverlapVolume
	// ReadingGap — a pair's proven clearance gap (Observed).
	ReadingGap
)

// String renders the pinned lower-snake token — the identity a caller branches
// on and a log prints, never the iota value. An out-of-range value renders
// "reading(<n>)", never a panic (verification §1.1).
func (k ReadingKind) String() string {
	switch k {
	case ReadingNone:
		return "none"
	case ReadingArea:
		return "area"
	case ReadingBounds:
		return "bounds"
	case ReadingVolume:
		return "volume"
	case ReadingCentroid:
		return "centroid"
	case ReadingWall:
		return "wall"
	case ReadingMinRadius:
		return "min_radius"
	case ReadingOverlapVolume:
		return "overlap_volume"
	case ReadingGap:
		return "gap"
	default:
		return fmt.Sprintf("reading(%d)", int(k))
	}
}

// DiagnosticCode is the stable, branchable reason code (verification §1.1) — a
// named-text enum whose stable String() token, never the iota value, is the
// identity a caller and a log share.
type DiagnosticCode int

const (
	// DiagMeasurementBeyondTolerance — a bounded reading's Bound exceeds rel*Ref
	// (§2). Reading names the quantity; its Observed* form carries it, Required
	// is rel*Ref (the largest Bound that would have passed). Body set on a body
	// reading, Pair on a pair reading. Contributes Suspect.
	DiagMeasurementBeyondTolerance DiagnosticCode = iota
	// DiagUndecidedValidity — the held boundary is not decisive beyond its own
	// proven bound (§6). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedValidity
	// DiagInvalidBody — the held boundary is proven not a valid solid (§6).
	// Reading ReadingNone. Contributes Unsound.
	DiagInvalidBody
	// DiagWallTooThin — MinWallThickness's proven interval is below the tool
	// (§6). Reading ReadingWall, Observed the wall reading, Required the tool.
	// Contributes Violating.
	DiagWallTooThin
	// DiagUndercut — a face is a proven undercut against the pull (§6). Reading
	// ReadingNone, every Observed* and Required nil. Contributes Violating.
	DiagUndercut
	// DiagUndecidedWall — the wall survey is undecided: neither answered nor
	// proven absent, OR its proven interval STRADDLES the tool (§6). In the
	// straddle case Reading is ReadingWall with Observed the wall reading and
	// Required the tool; when the survey could not answer at all Reading is
	// ReadingNone and both are nil. Contributes Suspect.
	DiagUndecidedWall
	// DiagUndecidedUndercut — the pull survey could neither prove nor exclude an
	// undercut (§6). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedUndercut
	// DiagUndecidedMinRadius — the concave-radius survey could neither measure
	// nor exclude a concave feature (§6). Reading ReadingNone. Contributes
	// Suspect.
	DiagUndecidedMinRadius
	// DiagInterference — a pair proven to overlap (§1). Reading
	// ReadingOverlapVolume, Observed the overlap volume, Required nil.
	// Contributes Interfering.
	DiagInterference
	// DiagUndecidedPair — a pair the disjoint/overlap PARTITION proof resolved
	// neither way (§1). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedPair
	// DiagUnsupportedPair is the broad compatibility code for a staged pair.
	// Verify emits this alongside one of the cause-specific codes below.
	// Reading ReadingNone. Contributes Suspect.
	//
	// Deprecated: branch on DiagUnsupportedPairPayload,
	// DiagUnsupportedPairContact, or DiagUnsupportedPairPipeline.
	DiagUnsupportedPair
	// DiagUndecidedClearance — a pair PROVEN disjoint (by box or kernel) whose
	// requested WithClearances gap the kernel could not prove: no Clearance row
	// is emitted and the report reads Suspect. Distinct from DiagUndecidedPair
	// (partition unresolved) and the cause-specific unsupported-pair codes
	// (payload, contact, or pipeline) — here the pair is decidedly apart and
	// only the gap is unmeasured. Reading ReadingNone. Contributes Suspect.
	DiagUndecidedClearance
	// DiagUndecidedInterference — a pair PROVEN to overlap whose overlap VOLUME
	// the evaluator cannot bound (§1): the overlap-side mirror of
	// DiagUndecidedClearance. No Interference row is emitted and the report reads
	// Suspect. Reading ReadingNone, Observed and Required nil, Pair set.
	// Contributes Suspect.
	DiagUndecidedInterference
	// DiagUnsupportedPairPayload — one named operand uses a payload the read-only
	// intersection cannot tessellate. Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedPairPayload
	// DiagUnsupportedPairContact — the pair reaches a contact or near-contact the
	// exact boolean policy cannot classify. Reading ReadingNone. Contributes Suspect.
	// Message names a shared face plane between the operands when they have one,
	// since deepening the overlap cannot resolve that cause.
	DiagUnsupportedPairContact
	// DiagUnsupportedPairPipeline — both operands tessellate, but later boolean
	// geometry exceeds the pipeline's supported reach. Reading ReadingNone.
	// Contributes Suspect.
	DiagUnsupportedPairPipeline
	// DiagUnsupportedSurveyPayload — an asked body survey cannot run because
	// its payload class is staged. Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedSurveyPayload
)

// String renders the pinned lower-snake token — the identity a caller branches
// on and a log prints, never the iota value. An out-of-range value renders
// "diagnostic(<n>)", never a panic (verification §1.1).
func (c DiagnosticCode) String() string {
	switch c {
	case DiagMeasurementBeyondTolerance:
		return "measurement_beyond_tolerance"
	case DiagUndecidedValidity:
		return "undecided_validity"
	case DiagInvalidBody:
		return "invalid_body"
	case DiagWallTooThin:
		return "wall_too_thin"
	case DiagUndercut:
		return "undercut"
	case DiagUndecidedWall:
		return "undecided_wall"
	case DiagUndecidedUndercut:
		return "undecided_undercut"
	case DiagUndecidedMinRadius:
		return "undecided_min_radius"
	case DiagInterference:
		return "interference"
	case DiagUndecidedPair:
		return "undecided_pair"
	case DiagUnsupportedPair:
		return "unsupported_pair"
	case DiagUndecidedClearance:
		return "undecided_clearance"
	case DiagUndecidedInterference:
		return "undecided_interference"
	case DiagUnsupportedPairPayload:
		return "unsupported_pair_payload"
	case DiagUnsupportedPairContact:
		return "unsupported_pair_contact"
	case DiagUnsupportedPairPipeline:
		return "unsupported_pair_pipeline"
	case DiagUnsupportedSurveyPayload:
		return "unsupported_survey_payload"
	default:
		return fmt.Sprintf("diagnostic(%d)", int(c))
	}
}

// DiagnosticPair names the two bodies of a pair diagnostic, in the report's own
// stable pair order (interference design §2).
type DiagnosticPair struct{ A, B *Body }

// Diagnostic is one structured, branchable reason a body or a pair is not Sound
// (verification §1.1). It never decides the verdict — Status is still §6's
// worst-wins aggregate — it explains it. Exactly one of Observed / ObservedVec
// / ObservedBox is non-nil, keyed by Reading (all three nil when
// Reading == ReadingNone).
type Diagnostic struct {
	Code        DiagnosticCode  // the stable branch key
	Status      Status          // the rung this reason contributes
	Body        *Body           // the body it concerns; nil for a pair diagnostic
	Pair        *DiagnosticPair // the pair it concerns; nil for a body diagnostic
	Reading     ReadingKind     // which quantity the Observed* form carries; ReadingNone names none
	Observed    *Measurement    // a scalar reading; nil unless Reading names a scalar quantity
	ObservedVec *VecMeasurement // a vector reading (a Centroid); nil unless Reading == ReadingCentroid
	ObservedBox *Box            // a box reading (a Bounds); nil unless Reading == ReadingBounds
	Required    *units.Value    // the threshold the reading was judged against; nil when the reason states none
	Message     string          // human-readable; NEVER the branch key
}

// Interference is a proven pairwise overlap, carrying its bounded overlap
// volume (verification §1, interference design §6). Verification computes it
// without consuming either body or changing the document.
type Interference struct {
	A, B   *Body
	Volume Measurement
}

// Clearance is the minimum gap between a pair of proven-disjoint bodies
// (verification §1). A row exists only for a pair proven disjoint with a
// measured gap: the clearance kernel (docs/clearance-design.md) proves the
// gap as an interval, and Gap reports its midpoint with the interval's half
// width as the proven Bound — Exact exactly when the interval is a point. A
// touching pair's zero is a measured Exact zero carried by a certified
// contact; a pair whose gap the kernel cannot prove yields no row and reads
// Suspect under WithClearances.
type Clearance struct {
	A, B *Body
	Gap  Measurement
}

// Report is what Verify returns: the 3D counterpart of
// sketch.VerificationReport (core §10, verification §1).
type Report struct {
	Bodies        []*BodyReport
	Interferences []Interference
	Clearances    []Clearance
	// Diagnostics contains structured, branchable entries for every reason a
	// report returned by Verify is not Sound (verification §1.1). Staged pair
	// causes also carry the deprecated broad compatibility entry. On such a
	// report it is empty EXACTLY when Status == Sound, and Status is the worst
	// Diagnostic.Status in it — the §6 aggregate, itemized. The zero Report is
	// Unverified and carries no verdict.
	Diagnostics []Diagnostic
	Status      Status
}

// Trustworthy is the single bit to gate on: true only when the whole report
// is Sound (verification §6). A zero Report is Unverified and returns false.
func (r *Report) Trustworthy() bool { return r.Status == Sound }

// BodyReport is one live body's verdict and readings (verification §1).
//
// The validity predicates report the boundary the evaluator holds — exact as
// data; what they prove about the PART is Status's to say. A quantity a body
// does not have is absent — nil, never zero: Area and Bounds are boundary
// properties every body has, so they are unconditional; Volume and Centroid
// are region properties only a proven solid has, so both are non-nil exactly
// when the body is one. The opt-in fields are nil unless their option asks
// AND the body is a proven solid AND (for the two feature measures) the
// feature exists: on this evaluator's analytic bodies the surveys decide
// them outright (survey.go), so a nil MinWallThickness or MinRadius on a
// proven solid inside a Sound report is the PROVEN determination that no
// wall / no concave feature exists, and an empty Undercuts the proven
// all-clear (verification §1/§6).
type BodyReport struct {
	Body   *Body
	Status Status

	Solid            bool
	Watertight       bool
	Manifold         bool
	SelfIntersecting bool
	Lumps            int
	Voids            int

	Area   Measurement
	Bounds Box

	Volume   *Measurement
	Centroid *VecMeasurement

	Exactness Exactness

	MinWallThickness *Measurement
	Undercuts        []*Face
	MinRadius        *Measurement
}

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
			fmt.Sprintf("the first operand (step %d) uses a payload the read-only intersection cannot tessellate; use a tessellatable body type or wait for payload support", a.originStep()))
	case outcome == interferenceUnsupportedPayloadSecond:
		return pairDiagNone(a, b, DiagUnsupportedPairPayload,
			fmt.Sprintf("the second operand (step %d) uses a payload the read-only intersection cannot tessellate; use a tessellatable body type or wait for payload support", b.originStep()))
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

const toleranceEpsilon = 1e-9

// measurementReference forms one scalar result's reference magnitude from
// its non-negative base-unit value. The callback keeps the shared scalar gate
// usable for body readings, clearances, and the future interference volume.
type measurementReference func(value float64) (float64, bool)

// scalarToleranceRef is the one scalar tolerance gate (verification §2), and it
// hands back the reference it formed. Exactness does not enter: a zero Bound
// passes without asking for a diameter, while every nonzero Bound needs a
// finite, non-negative reference. The comparison is deliberately inclusive.
// haveRef reports whether a usable reference was built — false for a zero Bound
// (which passes without one) and for an unusable magnitude — so a caller may
// report rel*ref as a Required threshold only when it exists.
func scalarToleranceRef(m Measurement, rel float64, reference measurementReference) (bool, float64, bool) {
	bound := m.Bound.Base()
	if !usableMagnitude(bound) {
		return false, 0, false
	}
	if bound == 0 {
		return true, 0, false
	}
	value := math.Abs(m.Value.Base())
	if !usableMagnitude(value) {
		return false, 0, false
	}
	ref, ok := reference(value)
	if !ok {
		return false, 0, false
	}
	return withinTolerance(bound, ref, rel), ref, true
}

// boundedToleranceRef applies the same gate to a bounded non-scalar shape such
// as a Box or position VecMeasurement, handing back the reference it formed on
// the same terms as scalarToleranceRef.
func boundedToleranceRef(bound, rel float64, reference func() (float64, bool)) (bool, float64, bool) {
	if !usableMagnitude(bound) {
		return false, 0, false
	}
	if bound == 0 {
		return true, 0, false
	}
	ref, ok := reference()
	if !ok {
		return false, 0, false
	}
	return withinTolerance(bound, ref, rel), ref, true
}

// requiredThreshold builds the Required value a DiagMeasurementBeyondTolerance
// reports: rel*ref, carried in the reading's own unit so its Kind matches the
// reading's Bound (verification §1.1). sample supplies that unit.
func requiredThreshold(relRef float64, sample units.Value) *units.Value {
	v := units.FromBase(relRef, sample.Unit())
	return &v
}

func withinTolerance(bound, ref, rel float64) bool {
	if !usableMagnitude(ref) {
		return false
	}
	if ref == 0 || rel == 0 {
		return bound <= rel*ref
	}
	// Compare the represented ratio directly: multiplying that ratio back by
	// ref can round one ulp below the bound at the inclusive boundary.
	return bound/ref <= rel
}

func usableMagnitude(v float64) bool {
	return v >= 0 && !math.IsNaN(v) && !math.IsInf(v, 0)
}

// bodyToleranceInputs lazily reads one body's intrinsic reference data. The
// laziness is part of the contract: a zero Bound passes even when no usable D
// can be obtained.
type bodyToleranceInputs struct {
	//nolint:containedctx // the reference callbacks have fixed signatures; the gate carries the caller's context through this per-call inputs struct so the diameter build stays cancellable.
	ctx    context.Context
	report *BodyReport
	err    error // a cancellation observed while lazily loading a reference

	diameterLoaded bool
	diameterValue  float64
	diameterOK     bool

	edgeLengthLoaded bool
	edgeLengthValue  float64
	edgeLengthOK     bool
}

func (in *bodyToleranceInputs) diameter() (float64, bool) {
	if !in.diameterLoaded {
		in.diameterValue, in.diameterOK, in.err = bodyGateDiameter(in.ctx, in.report.Body)
		in.diameterLoaded = true
	}
	return in.diameterValue, in.diameterOK
}

func (in *bodyToleranceInputs) edgeLength() (float64, bool) {
	if in.edgeLengthLoaded {
		return in.edgeLengthValue, in.edgeLengthOK
	}
	in.edgeLengthLoaded = true
	in.edgeLengthOK = true
	for _, edge := range in.report.Body.Edges() {
		if edge == nil || !usableMagnitude(edge.length) {
			in.edgeLengthOK = false
			break
		}
		in.edgeLengthValue += edge.length
		if !usableMagnitude(in.edgeLengthValue) {
			in.edgeLengthOK = false
			break
		}
	}
	return in.edgeLengthValue, in.edgeLengthOK
}

func (in *bodyToleranceInputs) areaReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	edgeLength, ok := in.edgeLength()
	if !ok {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter*edgeLength), true
}

func (in *bodyToleranceInputs) volumeReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	area := math.Abs(in.report.Area.Value.Base())
	if !usableMagnitude(area) {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter*area), true
}

func (in *bodyToleranceInputs) lengthReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter), true
}

func (in *bodyToleranceInputs) diameterReference() (float64, bool) {
	return in.diameter()
}

// bodyReadingDiagnostics applies verification §3's complete body-field table,
// emitting one DiagMeasurementBeyondTolerance per present reading that fails —
// never short-circuiting, so a body beyond tolerance on two readings emits two
// (verification §1.1). Body.Edges already deduplicates topology edges, and
// edgeLength reads each held geometric chain directly even when public
// Edge.Length must refuse a curved boolean rim.
func bodyReadingDiagnostics(ctx context.Context, br *BodyReport, rel float64) ([]Diagnostic, error) {
	in := &bodyToleranceInputs{ctx: ctx, report: br}
	diags := in.readingDiagnostics(rel)
	// A cancellation observed while a reference lazily built its geometry is
	// reported to the caller rather than folded into a Suspect verdict.
	if in.err != nil {
		return nil, in.err
	}
	return diags, nil
}

func (in *bodyToleranceInputs) readingDiagnostics(rel float64) []Diagnostic {
	br := in.report
	var diags []Diagnostic

	scalar := func(reading ReadingKind, m Measurement, reference measurementReference) {
		pass, ref, haveRef := scalarToleranceRef(m, rel, reference)
		if pass {
			return
		}
		obs := m
		d := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Body:     br.Body,
			Reading:  reading,
			Observed: &obs,
			Message:  fmt.Sprintf("the %s reading's bound %s is beyond the relative tolerance", reading, m.Bound),
		}
		if haveRef {
			d.Required = requiredThreshold(rel*ref, m.Value)
		}
		diags = append(diags, d)
	}

	scalar(ReadingArea, br.Area, in.areaReference)

	if pass, ref, haveRef := boundedToleranceRef(br.Bounds.Bound.Base(), rel, in.diameterReference); !pass {
		box := br.Bounds
		d := Diagnostic{
			Code:        DiagMeasurementBeyondTolerance,
			Status:      Suspect,
			Body:        br.Body,
			Reading:     ReadingBounds,
			ObservedBox: &box,
			Message:     fmt.Sprintf("the bounds reading's bound %s is beyond the relative tolerance", br.Bounds.Bound),
		}
		if haveRef {
			d.Required = requiredThreshold(rel*ref, br.Bounds.Bound)
		}
		diags = append(diags, d)
	}

	if br.Volume != nil {
		scalar(ReadingVolume, *br.Volume, in.volumeReference)
	}

	if br.Centroid != nil {
		if pass, ref, haveRef := boundedToleranceRef(br.Centroid.Bound.Base(), rel, in.diameterReference); !pass {
			cen := *br.Centroid
			d := Diagnostic{
				Code:        DiagMeasurementBeyondTolerance,
				Status:      Suspect,
				Body:        br.Body,
				Reading:     ReadingCentroid,
				ObservedVec: &cen,
				Message:     fmt.Sprintf("the centroid reading's bound %s is beyond the relative tolerance", br.Centroid.Bound),
			}
			if haveRef {
				d.Required = requiredThreshold(rel*ref, br.Centroid.Bound)
			}
			diags = append(diags, d)
		}
	}

	if br.MinWallThickness != nil {
		scalar(ReadingWall, *br.MinWallThickness, in.lengthReference)
	}
	if br.MinRadius != nil {
		scalar(ReadingMinRadius, *br.MinRadius, in.lengthReference)
	}
	return diags
}

// pairToleranceInputs owns pair-relative references. Clearance uses the
// length reference now; scalarToleranceRef accepts the interference volume
// reference through the same callback path once interference rows land.
type pairToleranceInputs struct {
	diameter float64
}

func (in pairToleranceInputs) lengthReference(value float64) (float64, bool) {
	if !usableMagnitude(in.diameter) {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*in.diameter), true
}

// bodyGateDiameter returns the body's own diameter, never a document scale or
// a bounds-box diagonal — and returns it as a value proven to be at or below
// that diameter, since every arm below publishes through
// pointSetDiameterWithBudget, whose own doc comment owns that rule.
// A Faceted body's cached value covers every held
// payload vertex, including vertices absent from the B-rep boundary loops. The
// analytic carrier model is built through the shared work budget (§7.2), so a
// cancelled Verify observes cancellation during the build instead of waiting for
// the whole model to finish. newBodyGeomBudget's payload switch is the
// clearance kernel's own — it only covers the payloads whose carrier model is
// an exact restatement of the shipped boundary, because clearance.go and
// interference.go trust that model for containment and contact proofs, not
// only for a diameter. A miss there is not necessarily a body with no usable
// diameter: a free-form-walled prismPayload — the one shipped payload whose
// side face the clearance kernel's exact model has no arm for at all — can read
// its diameter through freeformSectionGateDiameter's own witness set of
// analytic vertices and free-form span endpoints instead, tried first because
// its zero sectionDelta would otherwise read as "no arm" below. That arm
// WITHHOLDS its answer on each of the paths its own doc comment lists, and a
// body it withholds from falls through exactly like any other miss. Every miss
// reaches fallbackGateDiameter, which covers the payloads the exact model does
// not (cup, cap-loop chamfer, and a prismPayload whose own sectionDelta is
// nonzero) with a bound that is sound for THIS gate without being eligible
// for that stronger trust, and answers for nothing else — so a free-form-walled
// prismPayload whose own arm declined ends with no gate diameter at all.
//
// A prism with nonzero z0Delta or z1Delta keeps the same carrier model, but
// each held witness can move by axialDelta. The maximum held pair distance can
// therefore overstate the denoted body's diameter by twice that displacement.
// This function shrinks it toward zero before using it as a reference, so the
// result can only tighten the gate. fallbackGateDiameter applies the same
// correction to the prisms it reads, over each one's own displacement.
//
// A loftPayload reads its OWN held vertex-set diameter (pointSetDiameterContext),
// never an envelope: the boundary is a polyhedron, and a convex-hull diameter
// is realized at vertices, so the vertex set's own maximum IS AT OR BELOW the
// body's true diameter — the strongest arm in this function, ahead of the
// exact carrier model that does not yet cover this payload class. For an
// unplaced LineSeg-only loft the claim is the stronger one, IS the true
// diameter, because every held vertex is then exact (docs/loft-design.md
// §5) — which is a claim about that payload's own published delta, never
// about a station's KIND, since a station can be a recorded coordinate and
// still sit off the point the record denotes (docs/loft-design.md §5.2's
// arc-end radial residual); a same-kind circular pairing's own interior stations are held on the
// true recorded curve but are themselves COMPUTED (a10-plan.md Part 3 PR 6),
// so the held maximum is still a chorded (equal-or-fewer, never additional)
// vertex set's own diameter over a set that sits ON the true boundary — at
// or below it, whether or not the body is placed. That weaker claim is what
// this arm always publishes, and it still fails safe: understating a
// diameter can only turn a passing reading into a false Suspect, never a
// false Sound.
//
// This arm's zero-subtraction fast path reads payload.delta == 0 exactly
// (loft_build.go's exact identity-transform-and-no-computed-station
// comparison, never a tolerance) and then reports the shared reader's
// answer UNCHANGED: no subtraction and no rounding of its own. What that
// answer is, is the reader's to state — the largest float64 at or below the
// held diameter, since the reader publishes every witness maximum rounded
// toward zero (pointSetDiameterWithBudget) — so this arm publishes the
// tightest lower bound a float64 can carry on a diameter that is itself
// already at or below the true one. Subtracting a zero allowance on top of
// it would move that reading in exchange for nothing, which is why
// capBlendPayload.extentBoundedAlong's own `outward` helper (capblend.go)
// keeps a zero-displacement candidate untouched too. delta == 0 no longer
// implies the body is unplaced (a10-plan.md Part 3 PR 6): a curved pair
// chorded at ONE station (m = 1, docs/loft-design.md §12's m = 1 case) has
// no interior computed station either, so an UNPLACED body with such a pair
// can reach this same fast path with delta == 0 and an unshrunk reference.
// What makes that sound is the published ZERO and nothing else: at delta == 0
// every held vertex sits exactly at the point the record denotes for it, so
// the vertex set lies ON the true boundary and its maximum cannot exceed the
// true diameter. Being a RECORDED coordinate is not that premise and never
// stands in for it — an untrimmed ArcSeg's t == 1 end is recorded verbatim
// and still sits its own arc-end radial residual off the denoted curve,
// outward as easily as inward (docs/loft-design.md §5.2). Such a build
// publishes a positive delta and takes the shrink below, which is exactly how
// this arm sees the difference.
//
// A PLACED loft's held vertices are no
// longer provably exact (§12 PR 2a): the true diameter can differ from the
// held one by up to 2*delta (each of the two farthest points can sit up to
// delta from its true position), so this arm shrinks the held reading by
// 2*delta before reporting it, understating rather than overstating —
// tightening the gate can only turn a passing reading into a false Suspect,
// never a false Sound, the identical reasoning fallbackGateDiameter already
// carries. That direction is the SUBTRACTION's to lose: 2*delta is exact (a
// power-of-two scaling), so the difference is the one rounding here, and
// round-to-nearest can land it ABOVE the exact d - 2*delta — a reference
// larger than the one proven, which loosens the very gate this arm exists to
// tighten. downRound (spline_length.go, upRound's mirror) steps it back
// toward zero, so the published reference is at or below the exact shrunken
// value for every input rather than only for the ones whose subtraction
// happens to round down. A shrink that collapses to non-positive leaves the
// body with no usable diameter, exactly like any other unusable magnitude
// here.
func bodyGateDiameter(ctx context.Context, body *Body) (float64, bool, error) {
	if body == nil {
		return 0, false, nil
	}
	if payload, ok := body.payload.(facetedPayload); ok {
		return payload.diameter, usableMagnitude(payload.diameter), nil
	}
	if payload, ok := body.payload.(loftPayload); ok {
		d, ok, err := pointSetDiameterContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		if payload.delta == 0 {
			return d, true, nil
		}
		d, ok = lowerDiameterForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	budget := newWorkBudget(ctx)
	geom, ok, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return 0, false, err
	}
	if ok {
		d, ok := pointSetDiameter(geom.supports)
		if !ok {
			return d, false, nil
		}
		if payload, isPrism := body.payload.(prismPayload); isPrism {
			d, ok = lowerDiameterForDisplacement(d, payload.axialDelta())
		}
		return d, ok, nil
	}
	if payload, isPrism := body.payload.(prismPayload); isPrism {
		if d, ok, err := freeformSectionGateDiameter(ctx, payload); ok || err != nil {
			return d, ok, err
		}
	}
	return fallbackGateDiameter(budget, body)
}

// lowerDiameterForDisplacement turns a held witness diameter into a lower
// bound on the denoted body's diameter. Every witness can move by at most the
// supplied displacement, so their pair distance can shrink by twice that
// amount. The subtraction rounds toward zero because this value only tightens
// the tolerance gate when it remains a lower bound.
func lowerDiameterForDisplacement(d, displacement float64) (float64, bool) {
	if displacement == 0 {
		return d, true
	}
	if !usableMagnitude(d) || !usableMagnitude(displacement) {
		return 0, false
	}
	d = downRound(d - 2*displacement)
	return d, d > 0 && usableMagnitude(d)
}

// freeformSectionGateDiameter is bodyGateDiameter's arm for a free-form-walled
// prismPayload (docs/verification-design.md §3): the clearance kernel's exact
// carrier model has no arm for a NURBSSurface side face any more than it has
// one for a displaced section, and gateWitnessPrism gives this payload none
// either, because its own sectionDelta reads zero — the one value that arm
// treats as "the section is already its own denotation, read newBodyGeomBudget
// instead". A free-form wall is not read through either model, so this
// function builds its own reference.
//
// It reports a certified LOWER bound on the body's own diameter: the maximum
// distance over a finite set of points KNOWN TO LIE ON the body — every
// analytic walk's own two endpoints, and every free-form span's own two
// endpoints (docs/spline-design.md §5.1's exact-rational Bézier conversion,
// never the recorded control net, and for a FitSplineSeg never the raw
// recorded Fit points, which are neither the converted chain's own ends nor a
// hull the curve stays inside) — at both cap heights. A Bézier interpolates
// its end control points exactly, so every span endpoint is a real point of
// the curve itself, and every distance the maximum ranges over is therefore
// realized between two real body points.
//
// That is the witness set's own half of the claim, and it is only half: a
// maximum over real body points is at or below the true diameter as a
// QUANTITY, while what this arm publishes is a float64. The other half belongs
// to the shared reader — pointSetDiameterWithBudget computes the winning pair's
// distance over exact rationals and rounds it toward zero — so the published
// number is at or below that maximum too. Composed, the reading can only
// UNDERSTATE the true diameter and never overstate it, exactly the direction
// §3 requires; the displacement subtracted below only widens the
// understatement further.
//
// The displacement subtracted from that maximum composes three terms, none of
// them a certificate claim: the section's own sectionDelta (zero for a
// free-form wall in practice, since the analytic prism-boolean reduction
// never admits one — docs/prism-boolean-design.md's G4 — but read here rather
// than assumed), the payload's own axialDelta, and the widest per-witness
// endpoint bound walkEndBoundAllow reads off whichever walk produced it — an
// analytic walk's recorded-coordinate bound (zero for a whole segment,
// nonzero for a trimmed one) or a free-form span's own conversion rounding.
// Composing the widest witness bound as one uniform displacement, rather than
// a bound per point, is the same convention lowerDiameterForDisplacement's
// other callers already use: every witness pair is presumed to move by up to
// that much, so the subtracted amount is twice the WORST one, never a mix.
//
// This arm PUBLISHES only when its own witness conversion and the shared reader
// both succeed; otherwise it withholds the diameter outright and never
// substitutes a weaker one. This comment owns the complete list of the paths it
// withholds on — docs/verification-design.md §3 states the contract and points
// here rather than keeping a second copy:
//
//   - the profile carries no free-form segment at all, so this arm has nothing
//     to read the exact carrier model or gateWitnessPrism's own
//     displaced-section arm does not already read;
//   - a recorded segment normalizeSegment refuses, or walkOf refuses (an
//     R-table sentinel), so the section never becomes a walk at all;
//   - a free-form walk holds an empty Bézier span, or a span endpoint with no
//     finite float form (point2Of), so no witness can be placed on that curve;
//   - a witness's own endpoint bound cannot be derived (walkEndBoundAllow's
//     +Inf), since an absent bound must never read as a small one — this covers
//     an analytic walk's own two endpoints and a free-form span's alike;
//   - the shared reader declines the witness maximum (pointSetDiameterWithBudget
//     answering ok=false: an empty set, a pair distance that is not a usable
//     magnitude, or a winning pair with no exact rational form);
//   - the displacement subtraction collapses the reading to non-positive
//     (lowerDiameterForDisplacement).
//
// A withheld answer is not rescued downstream. bodyGateDiameter falls through to
// fallbackGateDiameter, whose gateWitnessPrism has no arm for a prismPayload
// whose sectionDelta is zero, so the body ends with NO gate diameter and its
// bounded readings read Suspect. That is the sound direction to fail in — an
// absent reference admits nothing — but it is a real outcome of this arm, not
// one the arm's existence rules out.
//
// Every phase of this arm is cancellable, because neither of its two phases is
// bounded by a work counter of its own: the segment loop polls ctx before each
// segment, and the witness maximum polls it through pointSetDiameterContext,
// the same reader the loftPayload arm above uses. That second poll is the one
// that matters for cost — the witness count grows with the profile's segment
// count (four points per segment, bounded only by recipe_decode.go's own
// MaxSegments ceiling), and the maximum is quadratic in it, so an unpolled scan is by
// far the longest thing a cancelled Verify could be left waiting on here. The
// resulting error is returned AS an error: cancellation is never folded into
// this arm's structural (0, false, nil) answer, which states only that the
// recorded section gives this arm nothing to read.
func freeformSectionGateDiameter(ctx context.Context, pp prismPayload) (float64, bool, error) {
	work := newFreeformWork()
	sawFreeform := false
	ownBound := 0.0
	var pts []r3.Vec

	addWitness := func(u, v float64, bound walkEndBound) bool {
		allow := walkEndBoundAllow(bound)
		if isNonFinite(allow) {
			return false
		}
		ownBound = math.Max(ownBound, allow)
		pts = append(pts, pp.point(u, v, pp.z0), pp.point(u, v, pp.z1))
		return true
	}

	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for _, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
			seg, err := normalizeSegment(seg)
			if err != nil {
				// normalizeSegment reads no ctx and so never observes
				// cancellation; every error it can return is a structural
				// refusal of the recorded segment itself, read here as "no
				// arm" rather than a hard failure.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation — see comment above
			}
			w, err := walkOf(seg, work)
			if err != nil {
				// Likewise walkOf: it reads the record's own freeformWork
				// counter, never ctx, so its error is always a build-time
				// refusal (an R-table sentinel) this arm reads as "no arm"
				// rather than propagates.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation — see comment above
			}
			if w.kind != walkFreeform {
				if !addWitness(w.startU, w.startV, w.startBound) || !addWitness(w.endU, w.endV, w.endBound) {
					return 0, false, nil
				}
				continue
			}
			sawFreeform = true
			for _, span := range w.spans {
				if len(span) == 0 {
					return 0, false, nil
				}
				for _, cp := range [2]ratPoint{span[0], span[len(span)-1]} {
					held, ok := point2Of(cp)
					if !ok {
						return 0, false, nil
					}
					bound := walkEndBound{
						u: rationalFloatError(cp.u, held.U),
						v: rationalFloatError(cp.v, held.V),
					}
					if !addWitness(held.U, held.V, bound) {
						return 0, false, nil
					}
				}
			}
		}
	}
	if !sawFreeform {
		return 0, false, nil
	}

	d, ok, err := pointSetDiameterContext(ctx, pts)
	if err != nil {
		return 0, false, err
	}
	if !ok {
		return 0, false, nil
	}
	displacement := absSumUpper(pp.sectionDelta, pp.axialDelta(), ownBound)
	d, ok = lowerDiameterForDisplacement(d, displacement)
	return d, ok, nil
}

// fallbackGateDiameter is bodyGateDiameter's fallback for a payload whose true
// boundary the clearance kernel's exact carrier model does not cover
// (cupPayload, capBlendPayload, and a prismPayload carrying a section
// displacement — verification design §3's "usable finite, non-negative body
// diameter", never a box diagonal, document scale or zero). A free-form-walled
// prismPayload misses that same exact carrier too, and bodyGateDiameter tries
// freeformSectionGateDiameter for it ahead of this function — but when that arm
// withholds its diameter the body DOES reach here, and this function has no arm
// for it either: its own sectionDelta is zero, the one value gateWitnessPrism
// below reads as "no arm at all" for a prismPayload. Such a body ends with no
// gate diameter and its bounded readings read Suspect.
//
// Each payload earns a witness prism for its own reason, and gateWitnessPrism's
// doc comment states each arm separately rather than pooling them behind one
// justification: a cap-loop chamfer never cuts past the receiver's own recorded
// walls, a cup's shell can ADD material (an outward shell), and a displaced
// section is no containing shape at all — it is the body's own recorded
// boundary, read within a proven displacement of the one it denotes. So what
// each arm proves has to be checked against the geometry it actually returns,
// not against "the receiver" as a stand-in for all three. For the two envelope
// arms the returned prism is a SHAPE that provably contains the true body, so
// the reduction itself can only overstate the true diameter, never understate
// it; the displacement subtracted below is what turns any of the three into the
// lower bound §3 requires.
//
// What this function actually reports, though, is a reading of that geometry,
// taken through the identical witness maximum a shipped prismPayload already
// reads its own diameter through above (addPrismFaces gives two witnesses
// per circular wall — the mid-angle point at mid-height and th0 at z0 —
// which pointSetDiameterWithBudget maxes pairwise, and region2.samples adds
// each cap arc's own th0 and mid-angle). That reader ranges over the body's
// own farthest pair — whose exact distance it then publishes rounded toward
// zero, so even the best case here is the largest float at or below the
// witness maximum — exactly when a
// circular wall's farthest pair lands on one of those three sampled angles
// (th0, mid-angle, th1) — guaranteed for an all-line section (the diameter
// is realized at vertices, all sampled), for a full circle (the two samples
// are antipodal), and for the arc-plus-chord family at or below 180 degrees
// of sweep (the diameter is realized at the arc endpoints) — but NEVER
// guaranteed by a bound on the sweep alone: an outward cup's own four 90
// degree corner arcs already understate this fallback's own output — read
// 64.922642 against that body's true diameter 65, a ratio of 1.0012 (its
// bounding-box diagonal is 68.738635, which the rounded corners keep it
// well inside of) — and a bare arc-plus-chord section peaks at 240 degrees, where the
// only sampled points are th0, the mid-angle, and th1, mutually
// 2R*sin(120 degrees) apart while the wall's true diameter is 2R — a ratio
// of 2/sqrt(3), about 15.5% (docs/verification-design.md §3 works that
// family's own figure). That understatement is not something this fallback
// introduces: the same reader already returns it for an ordinary shipped
// prismPayload built from the same curved section, so this fallback is no
// weaker than the exact path it stands in for, and the repair belongs to
// that shared reader rather than to this construction. The consequence
// stays inside the one direction this gate is free to err in: an
// understated D tightens Ref and can turn a passing reading into a false
// Suspect, never a false Sound (verification design §3). It stays intrinsic
// to the body's own geometry — built from the payload's own frame/xform, the
// same map a shipped prism's diameter is read through — so it carries none
// of the pose-dependence verification design §4 excludes an axis-aligned box
// for.
//
// A witness prism whose payload carries a displacement has the same
// held-witness issue as the exact prism path: each witness sits within that
// displacement of the point the payload denotes, so the held maximum can
// overstate the denoted body's diameter by twice it. The fallback shrinks the
// held witness maximum by that amount before it becomes a lower-bound
// reference, which is why gateWitnessPrism hands back a displacement beside
// the prism to read.
func fallbackGateDiameter(budget *workBudget, body *Body) (float64, bool, error) {
	witness, displacement, ok := gateWitnessPrism(body.payload)
	if !ok {
		return 0, false, nil
	}
	g := &bodyGeom{body: body}
	ok, err := g.addPrismFaces(budget, witness)
	if err != nil || !ok {
		return 0, false, err
	}
	var pts []r3.Vec
	for _, f := range g.faces {
		if err := budget.step(); err != nil {
			return 0, false, err
		}
		pts = append(pts, f.wit...)
	}
	d, ok, err := pointSetDiameterWithBudget(budget, pts)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = lowerDiameterForDisplacement(d, displacement)
	return d, ok, nil
}

// gateWitnessPrism builds the straight prism fallbackGateDiameter reads its
// witnesses off, beside the displacement each of those witnesses can carry
// from the point of the denoted body it stands for. ok is false for every
// payload with no arm here, including a revolvePayload (already exact through
// newBodyGeomBudget, which is why it never reaches this fallback) and an
// analytic-walled prismPayload whose section is its own denotation (the same
// reason). A free-form-walled prismPayload's own section is its denotation
// too, so this switch answers false for it exactly as it does for the analytic
// case: bodyGateDiameter routes a free-form-walled prismPayload through
// freeformSectionGateDiameter before fallbackGateDiameter, and calls this
// function only when that arm has already declined — at which point this false
// answer is what leaves the body with no gate diameter at all.
//
// The three arms read different geometry and earn a witness for different
// reasons.
//
// capBlendPayload reads pl.profile, the receiver's own unrewritten section on
// its unchanged interval: a cap-loop chamfer only ever cuts along a chord
// whose feet sit on the receiver's own recorded walls, so it can never place
// a point beyond the receiver's own extruded envelope. cupPayload reads
// pl.outer, the cup's own outer region — the receiver's unmodified section
// for an INWARD shell, but the wider OFFSET (expanded) region for an OUTWARD
// one, since an outward shell adds material and cupPayloadFor
// (shell_cup.go) always assigns the wider of the two profiles to outer
// regardless of sense. Either way the whole cup body — walls, floor and
// cavity alike — sits inside pl.outer's own full-height prism: the cavity
// never reaches farther than the outer region, the same containment
// cupPayload.extentAlong already relies on. Both are CONTAINING shapes, so
// each can only overstate the true diameter as a shape; both read a section
// that is its own denotation, because every modify op refuses a receiver
// carrying a section displacement (fillet.go's requireExactSection), so the
// only displacement their witnesses carry is the axial one.
//
// A prismPayload whose own sectionDelta is nonzero (docs/prism-boolean-design.md
// §7's re-expressed or cut section — every analytic Union whose merge cut a
// wall, plus any placed prism pair) reads its OWN recorded section, and is not
// a containing shape at all: the denoted section may sit either side of the
// recorded one. It does not need to be. What this gate needs is a lower bound
// on the body's own diameter, and §7 proves every recorded boundary point sits
// within sectionDelta of the section the payload denotes, while each recorded
// level sits within axialDelta of the level it denotes. Those two displacements
// are perpendicular — one moves a coordinate IN the plane, the other moves a
// level ALONG the normal — so their sum is an upper bound on how far a lifted
// witness sits from the denoted body point below it, and lowerDiameterForDisplacement
// turns the held maximum into the lower bound the gate wants. The copy zeroes
// sectionDelta because addPrismFaces (clearance_geom.go) refuses a displaced
// section outright: it builds the clearance kernel's certificate carriers,
// which have to be exact statements about a boundary, and a witness set for a
// diameter is neither a certificate nor a carrier.
func gateWitnessPrism(payload featurePayload) (prismPayload, float64, bool) {
	switch pl := payload.(type) {
	case capBlendPayload:
		witness := prismPayload{
			profile: pl.profile,
			frame:   pl.frame,
			z0:      pl.z0,
			z1:      pl.z1,
			z0Delta: pl.z0Delta,
			z1Delta: pl.z1Delta,
			xform:   pl.xform,
		}
		return witness, witness.axialDelta(), true
	case cupPayload:
		witness := pl.outerPrism()
		witness.profile = pl.outer
		return witness, witness.axialDelta(), true
	case prismPayload:
		if pl.sectionDelta == 0 {
			return prismPayload{}, 0, false
		}
		displacement := absSumUpper(pl.sectionDelta, pl.axialDelta())
		witness := pl
		witness.sectionDelta = 0
		return witness, displacement, true
	default:
		return prismPayload{}, 0, false
	}
}

func pointSetDiameter(points []r3.Vec) (float64, bool) {
	d, ok, _ := pointSetDiameterWithBudget(nil, points)
	return d, ok
}

func pointSetDiameterContext(ctx context.Context, points []r3.Vec) (float64, bool, error) {
	return pointSetDiameterWithBudget(newWorkBudget(ctx), points)
}

// pointSetDiameterWithBudget is the ONE witness-maximum reader every gate
// diameter is published through — bodyGateDiameter's exact carrier arm and its
// loft arm, freeformSectionGateDiameter, fallbackGateDiameter, and the cached
// facetedPayload.diameter buildFacetedBody stores (boolean_body.go). What it
// returns is a CERTIFIED value at or below the exact greatest distance between
// two of the supplied points: the float scan only SELECTS a pair, and the
// published number is that pair's own distance computed over exact rationals
// and rounded toward zero (exactPairDistanceDown).
//
// The float scan cannot publish that number itself. Sub rounds each component
// and Len rounds the norm, so points[i].Sub(points[j]).Len() can land ABOVE
// the pair's exact distance — a 6x6x7 box's corner pair reads
// 11.000000000000002 against an exact sqrt(121) = 11 — while every consumer
// here reads the answer as a LOWER bound on the body's own diameter
// (docs/verification-design.md §3: an understated D tightens the gate into a
// false Suspect at worst, an overstated one loosens it into a false Sound).
// Charging that roundoff as an outward allowance is not open to this function
// either: it publishes one number, not an interval, so the charge has to land
// inside the value, which means rounding the value itself toward zero.
//
// Selecting the pair in floats costs nothing here. Whatever pair the scan
// picks, the published number is a REAL pair distance rounded toward zero and
// is therefore at or below the exact maximum; a near-tie the float rounding
// mis-orders changes how TIGHT the answer is, never whether it is a lower
// bound. So the certification rests on the exact arithmetic alone, and the
// scan is left free to cost what it always did.
//
// ok is false for an empty set, for a pair distance that is not a usable
// magnitude, and for a winning pair whose coordinates have no exact rational
// form — an absent answer, never a substitute one.
func pointSetDiameterWithBudget(budget *workBudget, points []r3.Vec) (float64, bool, error) {
	if len(points) == 0 {
		return 0, false, nil
	}
	best := 0.0
	bestI, bestJ := 0, 0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			if budget != nil {
				if err := budget.step(); err != nil {
					return 0, false, err
				}
			}
			distance := points[i].Sub(points[j]).Len()
			if !usableMagnitude(distance) {
				return 0, false, nil
			}
			if distance > best {
				best, bestI, bestJ = distance, i, j
			}
		}
	}
	if budget != nil {
		if err := budget.err(); err != nil {
			return 0, false, err
		}
	}
	if best == 0 {
		// A single point, or a set whose every pair is coincident: zero is
		// already exact and needs no rounding step.
		return 0, true, nil
	}
	d, ok := exactPairDistanceDown(points[bestI], points[bestJ])
	if !ok {
		return 0, false, nil
	}
	return d, true, nil
}

// exactPairDistanceDown returns the largest float64 at or below the EXACT
// distance between two points. Both coordinates of each axis are float64s and
// so are exact rationals; the difference and its square are exact in that
// arithmetic, and ratSqrtDown (spline_length.go) decides the last step by
// comparing a candidate's exact square against the exact sum rather than by
// trusting the platform's own square root. Nothing in the chain rounds outward,
// so the answer is proven to be at or below the pair's true distance.
//
// ok is false when a coordinate has no rational form (a non-finite one), which
// its callers read as no answer at all.
func exactPairDistanceDown(a, b r3.Vec) (float64, bool) {
	total := new(big.Rat)
	for _, axis := range [3][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		ra, rb := floatRat(axis[0]), floatRat(axis[1])
		if ra == nil || rb == nil {
			return 0, false
		}
		d := ra.Sub(ra, rb)
		total.Add(total, d.Mul(d, d))
	}
	return ratSqrtDown(total), true
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
