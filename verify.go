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
	// DiagUnsupportedPair — a pair this evaluator cannot decide because a
	// payload or a contact is STAGED: a revolve or cup operand, or the boolean's
	// unsupported contact (core §8). Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedPair
	// DiagUndecidedClearance — a pair PROVEN disjoint (by box or kernel) whose
	// requested WithClearances gap the kernel could not prove: no Clearance row
	// is emitted and the report reads Suspect. Distinct from DiagUndecidedPair
	// (partition unresolved) and DiagUnsupportedPair (a staged payload) — here
	// the pair is decidedly apart and only the gap is unmeasured. Reading
	// ReadingNone. Contributes Suspect.
	DiagUndecidedClearance
	// DiagUndecidedInterference — a pair PROVEN to overlap whose overlap VOLUME
	// the evaluator cannot bound (§1): the overlap-side mirror of
	// DiagUndecidedClearance. No Interference row is emitted and the report reads
	// Suspect. Reading ReadingNone, Observed and Required nil, Pair set.
	// Contributes Suspect.
	DiagUndecidedInterference
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
	// Diagnostics is one structured, branchable entry per reason a report
	// returned by Verify is not Sound (verification §1.1). On such a report it
	// is empty EXACTLY when Status == Sound, and Status is the worst
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
// per face from the exact normal range an analytic face sweeps: a face with
// a provenly opposing point is listed, exactly perpendicular is not opposed
// (the vertical wall clears), and a non-empty listing is Violating.
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
// inputs. Cancellation returns ctx.Err() and a nil report; the document remains
// unchanged.
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
				report.Diagnostics = append(report.Diagnostics,
					undecidedPairDiag(a, b, res.verdict, outcome))
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

// undecidedPairDiag picks the diagnostic for a pair whose overlap volume the
// evaluator could not measure (verification §1.1): a proven overlap is
// DiagUndecidedInterference, a staged payload or contact is DiagUnsupportedPair,
// and an unresolved partition is DiagUndecidedPair.
func undecidedPairDiag(a, b *Body, verdict pairVerdict, outcome interferenceOutcome) Diagnostic {
	switch {
	case verdict == pairOverlapping:
		return pairDiagNone(a, b, DiagUndecidedInterference,
			"the pair is proven to overlap but the overlap volume is unmeasured")
	case outcome == interferenceUnsupported:
		return pairDiagNone(a, b, DiagUnsupportedPair,
			"the pair cannot be decided because a payload or a contact is staged")
	default:
		return pairDiagNone(a, b, DiagUndecidedPair,
			"the disjoint/overlap partition proof resolved neither way")
	}
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
		surveyDiags := runSurveys(br, cfg)
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
// a bounds-box diagonal. A Faceted body's cached value covers every held
// payload vertex, including vertices absent from the B-rep boundary loops. The
// analytic carrier model is built through the shared work budget (§7.2), so a
// cancelled Verify observes cancellation during the build instead of waiting for
// the whole model to finish.
func bodyGateDiameter(ctx context.Context, body *Body) (float64, bool, error) {
	if body == nil {
		return 0, false, nil
	}
	if payload, ok := body.payload.(facetedPayload); ok {
		return payload.diameter, usableMagnitude(payload.diameter), nil
	}
	geom, ok, err := newBodyGeomBudget(newWorkBudget(ctx), body)
	if err != nil {
		return 0, false, err
	}
	if !ok {
		return 0, false, nil
	}
	d, ok := pointSetDiameter(geom.supports)
	return d, ok, nil
}

func pointSetDiameter(points []r3.Vec) (float64, bool) {
	if len(points) == 0 {
		return 0, false
	}
	best := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			distance := points[i].Sub(points[j]).Len()
			if !usableMagnitude(distance) {
				return 0, false
			}
			best = math.Max(best, distance)
		}
	}
	return best, true
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
