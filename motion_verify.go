package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyMotion (docs/motion-check-design.md §3, §5-§8):
// validation, the swept-box exclusion, the per-pose pair proof over transient
// placements, the two-sided interval certificate, and the report assembly.
// motion.go owns the vocabulary and motion_bound.go the bounds.
//
// The check evaluates the two endpoints, then bisects on a dyadic grid of the
// path (§6): first for the verdict, until every interval is certified clear,
// colliding at both ends, or no wider than the resolution — a colliding
// interval with a collision-free end is halved toward the contact's onset;
// then for the
// readings, refining the interval holding the smallest certified clearance
// while the whole-path reading fails the tolerance gate or a requested margin
// is neither proven nor disproven, on the same floor. An interval the floor
// leaves uncertified reads IntervalUndecided and makes the report Suspect;
// nothing between evaluated poses is ever assumed clear.

// transientProducer is the reserved producer identity every transient pose
// placement is built under. It never enters the document: a transient body is
// never committed, never published, and its provenance is never read back.
const transientProducer producerID = -1

// motionKind names which Motion variant a motionSpec was read from.
type motionKind int

const (
	motionRevolute motionKind = iota + 1
	motionPrismatic
	motionBetween
)

// motionSpec is a validated Motion read into the fields every pose and bound
// is built from. A Between's parameter runs over the dimensionless fraction
// [0, 1], so its from and to are units.Scalar(0) and units.Scalar(1), and
// between and screw carry its poses and the screw r3 reads off them.
type motionSpec struct {
	motion     Motion
	kind       motionKind
	center     r3.Vec
	axis       r3.Vec
	dir        r3.Vec
	between    Between
	screw      r3.Screw
	from, to   units.Value
	fromP, toP motionParam
	frame      motionFrame
}

// paramKind is the Kind the motion's parameter, and so its resolution, is
// stated in.
func (s motionSpec) paramKind() units.Kind {
	switch s.kind {
	case motionRevolute:
		return units.Angle
	case motionBetween:
		return units.Dimensionless
	default:
		return units.Length
	}
}

// resolveMotion validates m (docs/motion-check-design.md §2, §8): the
// variant's own field refusals, From == To, and both endpoint poses building
// under r3. A nil motion is ErrDegenerate.
func resolveMotion(m Motion) (motionSpec, error) {
	var spec motionSpec
	switch mv := m.(type) {
	case Revolute:
		if err := mv.validate(); err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionRevolute, center: mv.Center, axis: mv.Axis, from: mv.From, to: mv.To}
	case *Revolute:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Prismatic:
		if err := mv.validate(); err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionPrismatic, dir: mv.Dir, from: mv.From, to: mv.To}
	case *Prismatic:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Between:
		sc, err := mv.resolve()
		if err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{kind: motionBetween, between: mv, screw: sc, from: units.Scalar(0), to: units.Scalar(1)}
	case *Between:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case nil:
		return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
	default:
		return motionSpec{}, fmt.Errorf(`%w: a motion of type %T is not one this evaluator checks`, ErrUnsupported, m)
	}
	spec.motion = m
	var okF, okT bool
	spec.fromP, okF = exactMotionParam(spec.from)
	spec.toP, okT = exactMotionParam(spec.to)
	if !okF || !okT {
		return motionSpec{}, fmt.Errorf(`%w: a motion endpoint is not representable`, ErrNotFinite)
	}
	if sameMotionValue(spec.from, spec.to) {
		return motionSpec{}, fmt.Errorf(`%w: From and To are both %s, which names no path`, ErrDegenerate, spec.from)
	}
	for _, end := range []units.Value{spec.from, spec.to} {
		if _, err := m.PoseAt(end); err != nil {
			return motionSpec{}, err
		}
	}
	frame, ok := newMotionFrame(spec)
	if !ok {
		return motionSpec{}, fmt.Errorf(`%w: the motion's axis is not representable`, ErrNotFinite)
	}
	spec.frame = frame
	return spec, nil
}

// defaultResolution is the published default resolution, |To − From|/1024
// carried in From's unit. When that value underflows in From's unit or its
// base unit, it returns the smallest positive resolution in From's unit
// accepted by WithResolution and reports clamped so the check uses that floor.
func (s motionSpec) defaultResolution() (units.Value, bool) {
	var d *big.Rat
	if s.kind == motionPrismatic {
		// The exact base-unit difference survives conversion that could round
		// two distinct endpoints to the same float in From's unit.
		d = new(big.Rat).Sub(s.toP.base, s.fromP.base)
		d.Abs(d)
		d.Quo(d, proofarith.FloatRat(s.from.Unit().Factor()))
	} else {
		from := proofarith.FloatRat(s.from.Mag())
		toMag, err := s.to.In(s.from.Unit())
		to := proofarith.FloatRat(toMag)
		if err != nil || from == nil || to == nil {
			return units.New(0, s.from.Unit()), false
		}
		d = new(big.Rat).Sub(to, from)
		d.Abs(d)
	}
	mag, _ := d.Quo(d, big.NewRat(1024, 1)).Float64()
	base, _ := units.BaseUnit(s.paramKind())
	reported := units.New(mag, s.from.Unit())
	converted, err := reported.In(base)
	if mag > 0 && err == nil && converted > 0 {
		return reported, false
	}
	// A positive exact floor can underflow in From's unit or its base unit.
	// Binary search positive finite float bits for the first value whose base
	// conversion is nonzero, which is also the first WithResolution accepts.
	low, high := uint64(0), math.Float64bits(1)
	for high-low > 1 {
		mid := low + (high-low)/2
		candidate := units.New(math.Float64frombits(mid), s.from.Unit())
		converted, err := candidate.In(base)
		if err != nil || converted == 0 {
			low = mid
			continue
		}
		high = mid
	}
	return units.New(math.Float64frombits(high), s.from.Unit()), true
}

// defaultResolutionParam is |θ(To) − θ(From)|/1024 taken part by part over
// exact rationals: never zero for a validated motion, and exactly one
// dyadic step of depth ten whatever units From and To were stated in.
func (s motionSpec) defaultResolutionParam() motionParam {
	part := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		d.Abs(d)
		return d.Quo(d, big.NewRat(1024, 1))
	}
	return motionParam{turn: part(s.fromP.turn, s.toP.turn), base: part(s.fromP.base, s.toP.base)}
}

// label is the published parameter of the pose at fraction f of the path:
// From and To exactly at the ends, and otherwise the float nearest the exact
// interpolation carried in From's unit — exact itself whenever From and To
// share a unit and the dyadic step is representable. It is a label: every
// bound is built from the exact parameter fromP.lerp(toP, f), and
// poseDeviation charges whatever separates the pose PoseAt builds from this
// label and the ideal pose at f.
func (s motionSpec) label(f *big.Rat) units.Value {
	switch {
	case f.Sign() == 0:
		return s.from
	case f.Cmp(big.NewRat(1, 1)) == 0:
		return s.to
	}
	from := proofarith.FloatRat(s.from.Mag())
	toMag, err := s.to.In(s.from.Unit())
	to := proofarith.FloatRat(toMag)
	if err != nil || from == nil || to == nil {
		// Unreachable for a validated motion, whose endpoints both convert;
		// the exact parameter still governs every bound if it were reached.
		return s.from
	}
	d := new(big.Rat).Sub(to, from)
	d.Mul(d, f)
	mag, _ := d.Add(d, from).Float64()
	return units.New(mag, s.from.Unit())
}

// resolve validates a Between for VerifyMotion (docs/motion-check-design.md
// §2, §8): its own field refusals, then From == To, then the screw r3 reads
// off the relative motion, which must be representable and must not be the
// zero screw — a relative motion with neither angle nor slide names no path,
// since PoseAt is then From at every s.
func (m Between) resolve() (r3.Screw, error) {
	if err := m.validate(); err != nil {
		return r3.Screw{}, err
	}
	if m.From == m.To {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose From equals its To names no path`, ErrDegenerate)
	}
	sc, err := m.screw()
	if err != nil {
		return r3.Screw{}, err
	}
	if sc.Angle.Mag() == 0 && sc.Slide == 0 {
		return r3.Screw{}, fmt.Errorf(`%w: a between whose relative motion is the zero screw names no path`, ErrDegenerate)
	}
	return sc, nil
}

// resolveMotionAs resolves a dereferenced pointer motion while keeping the
// caller's own value as the stated motion the report echoes.
func resolveMotionAs(stated Motion, value Motion) (motionSpec, error) {
	spec, err := resolveMotion(value)
	if err != nil {
		return motionSpec{}, err
	}
	spec.motion = stated
	return spec, nil
}

// resolveMovers validates the moving set (docs/motion-check-design.md §3, §8).
func (d *Document) resolveMovers(moving []*Body) error {
	if len(moving) == 0 {
		return fmt.Errorf(`%w: an empty moving set moves nothing`, ErrDegenerate)
	}
	seen := make(map[*Body]struct{}, len(moving))
	for _, b := range moving {
		if err := d.requireLive(b); err != nil {
			return err
		}
		if _, dup := seen[b]; dup {
			return fmt.Errorf(`%w: a body is listed twice in the moving set`, ErrDegenerate)
		}
		seen[b] = struct{}{}
		if b.payload == nil {
			return fmt.Errorf(`%w: this evaluator cannot move a body it did not build`, ErrUnsupported)
		}
	}
	return nil
}

// VerifyMotion checks whether the rigid set moving, carried along m, meets
// any other live body of d anywhere on the path (docs/motion-check-design.md).
// It never changes the document: each pose re-evaluates a mover's payload
// under the composed motion as a transient body that is never committed, so
// Document.Bodies(), every body's live state and the next producer identity
// are what they were before the call.
//
// Every pair result at a pose is Verify's own row or diagnostic. Between two
// adjacent poses the interval certificate (§5.2) proves the path clear only
// when every (mover, static) pair's two proven lower bounds together exceed
// the farthest any mover point can travel across the interval; an interval it
// cannot certify down to the resolution is IntervalUndecided and the report
// reads Suspect, never Sound. A pose publishes a Collision only for an overlap
// that survives the pose's own deviation from the ideal motion (§5.1).
//
// moving MUST be non-empty, hold live bodies of d, and list no body twice.
// A nil context returns ErrDegenerate before validation. Other validation
// errors precede cancellation; after validation a canceled context returns
// ctx.Err() and no report.
func (d *Document) VerifyMotion(ctx context.Context, moving []*Body, m Motion, opts ...MotionOption) (*MotionReport, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control motion verification`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := d.resolveMovers(moving); err != nil {
		return nil, err
	}
	spec, err := resolveMotion(m)
	if err != nil {
		return nil, err
	}
	cfg, err := resolveMotionOptions(opts, spec)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	run := &motionRun{ctx: ctx, d: d, spec: spec, cfg: cfg, cache: &bodyGeomCache{}}
	run.setup(moving)
	return run.execute()
}

// motionRun is one VerifyMotion call's working state. It lives in the call
// alone, never on the Document.
type motionRun struct {
	ctx       context.Context //nolint:containedctx // motionRun is per-call state and never outlives VerifyMotion.
	d         *Document
	transient *Document // per-call identity counters for uncommitted pose bodies
	spec      motionSpec
	cfg       motionConfig
	cache     *bodyGeomCache
	movers    []motionMover
	statics   []motionStatic
	pairs     [][]motionPair // [mover][static]
	// stretch and stretchEnd are pathAreaUpper's stretch base (§5.1) for a
	// pose before the end and for the end itself: exactly 1 for a Revolute
	// and a Prismatic; basisSigmaUpper of From, and at s = 1 the larger of
	// From's and To's, for a Between.
	stretch, stretchEnd float64
}

type motionMover struct {
	body     *Body
	validity ValidityResult
	rho      float64 // ρ_max; revolute and between only
	r0       float64 // the record radius poseDeviation charges at
	area     float64 // a proven upper bound on the mover's surface area at rest
	sigma    float64 // a proven lower bound on its placement's smallest singular value
}

type motionStatic struct {
	body     *Body
	validity ValidityResult
}

// motionPair is one (mover, static) pair's standing for the whole call.
type motionPair struct {
	// excluded: the swept-box exclusion proved the pair apart over the whole
	// path, with lower a proven lower bound on its distance throughout.
	excluded bool
	lower    float64
	// invalid: an operand is not a proven-valid body; the pair is never
	// evaluated and leaves every interval undecided.
	invalid bool
	// sheet: an operand is a sheet; the pair reads DiagUnsupportedPairSheet
	// at every pose and leaves every interval undecided.
	sheet bool
}

func (p motionPair) evaluated() bool { return !p.excluded && !p.invalid && !p.sheet }

// motionPairPose is one pair's answer at one pose. A gap is a proven interval
// [lo, hi] on the distance between the IDEAL placed mover and the static
// body, η already charged.
type motionPairPose struct {
	hasGap    bool
	lo, hi    float64
	diam      float64
	collision bool
}

// motionPose is one evaluated pose.
type motionPose struct {
	f          *big.Rat
	param      motionParam
	violated   bool // some pair's gap here is proven below the requested minimum
	result     PoseResult
	pairs      [][]motionPairPose
	findings   []Diagnostic // this pose's findings, in report order
	collisions []Collision
}

func (r *motionRun) setup(moving []*Body) {
	// Payload rebuilding mints level and curve denotations. Start above every
	// live identity, then mint only on this copy: successive pose bodies stay
	// distinct from static geometry without changing the caller's document.
	transient := *r.d
	transient.bodies = append([]*Body(nil), r.d.bodies...)
	r.transient = &transient
	isMover := make(map[*Body]struct{}, len(moving))
	for _, b := range moving {
		isMover[b] = struct{}{}
		r.movers = append(r.movers, motionMover{body: b, validity: publishValidityResult(b, bodyValidityEvidence(r.ctx, b))})
	}
	for _, b := range r.d.bodies {
		if _, ok := isMover[b]; ok {
			continue
		}
		r.statics = append(r.statics, motionStatic{body: b, validity: publishValidityResult(b, bodyValidityEvidence(r.ctx, b))})
	}
	r.stretch, r.stretchEnd = 1, 1
	if r.spec.kind == motionBetween {
		r.stretch = basisSigmaUpper(r.spec.between.From)
		r.stretchEnd = math.Max(r.stretch, basisSigmaUpper(r.spec.between.To))
	}
	zero := motionParam{turn: new(big.Rat), base: new(big.Rat)}
	r.pairs = make([][]motionPair, len(r.movers))
	for i := range r.movers {
		mv := &r.movers[i]
		mv.r0 = moverRecordRadius(r.ctx, mv.body)
		mv.area = absSumUpper(mv.body.area.Value.Base(), mv.body.area.Bound.Base())
		mv.sigma = basisSigmaLower(mv.body.payload.transform())
		if r.spec.kind != motionPrismatic {
			mv.rho = moverAxisRadius(mv.body, r.spec.frame)
		}
		// The swept box covers every pose from where the path's box was read:
		// for a Revolute or a Prismatic that is the mover at rest — the
		// parameter 0 — so its travel runs from 0 to the farther endpoint,
		// never merely across [From, To]; for a Between it is the From-placed
		// box at s = 0, From itself, so the travel is the whole path's.
		travel := maxRat(
			moverTravel(r.spec.frame, mv.rho, zero, r.spec.fromP),
			moverTravel(r.spec.frame, mv.rho, zero, r.spec.toP),
		)
		sweptLo, sweptHi, sweptOK := moverSweptBox(mv.body.bounds, r.spec.frame, travel)
		r.pairs[i] = make([]motionPair, len(r.statics))
		for j, st := range r.statics {
			pair := &r.pairs[i][j]
			if mv.validity.Outcome != ValidityValid || st.validity.Outcome != ValidityValid {
				pair.invalid = true
				continue
			}
			if sweptOK {
				if lower, ok := sweptBoxLower(sweptLo, sweptHi, st.body.bounds); ok {
					pair.excluded, pair.lower = true, lower
					continue
				}
			}
			if mv.body.Kind() == BodySheet || st.body.Kind() == BodySheet {
				pair.sheet = true
			}
		}
	}
}

// maxRat is the larger of two optional rationals; a nil (unbounded) operand
// wins.
func maxRat(a, b *big.Rat) *big.Rat {
	if a == nil || b == nil {
		return nil
	}
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// motionSpan is one interval's standing between two adjacent poses.
type motionSpan struct {
	outcome   IntervalOutcome
	clearance *Measurement
}

func (r *motionRun) execute() (*MotionReport, error) {
	var poses []*motionPose
	for _, end := range []struct {
		f  *big.Rat
		at units.Value
	}{{new(big.Rat), r.spec.from}, {big.NewRat(1, 1), r.spec.to}} {
		pose, err := r.evaluatePose(end.f, end.at)
		if err != nil {
			return nil, err
		}
		poses = append(poses, pose)
	}
	spans := []motionSpan{r.intervalVerdict(poses[0], poses[1])}
	for {
		k := r.nextRefinement(poses, spans)
		if k < 0 {
			break
		}
		f := new(big.Rat).Add(poses[k].f, poses[k+1].f)
		f.Quo(f, big.NewRat(2, 1))
		mid, err := r.evaluatePose(f, r.spec.label(f))
		if err != nil {
			return nil, err
		}
		poses = slices.Insert(poses, k+1, mid)
		spans[k] = r.intervalVerdict(poses[k], mid)
		spans = slices.Insert(spans, k+1, r.intervalVerdict(mid, poses[k+2]))
	}
	if err := r.ctx.Err(); err != nil {
		return nil, err
	}
	return r.publish(poses, spans), nil
}

// nextRefinement picks the interval §6 bisects next, or −1 when refinement is
// done. Step 5 comes first: the first interval in traversal order, still wider
// than the resolution, that is either undecided or colliding at one end only —
// halving the latter walks the first collision toward the contact's onset,
// while an interval colliding at both ends says nothing more for being split.
// Step 6
// follows: the interval holding the smallest certified clearance (ties in
// traversal order), while the whole-path reading would fail the tolerance gate
// or a requested margin is neither proven nor disproven by that interval, and
// only while it is wider than the resolution. Every interval narrower than the
// floor stops, so the loop ends.
func (r *motionRun) nextRefinement(poses []*motionPose, spans []motionSpan) int {
	allClear := true
	smallest := -1
	for k, span := range spans {
		startHits, endHits := len(poses[k].collisions) > 0, len(poses[k+1].collisions) > 0
		onset := span.outcome == IntervalColliding && startHits != endHits
		if (span.outcome == IntervalUndecided || onset) && r.wide(poses[k], poses[k+1]) {
			return k
		}
		if span.outcome != IntervalClear {
			allClear = false
		}
		if span.clearance != nil && (smallest < 0 || span.clearance.Value.Base() < spans[smallest].clearance.Value.Base()) {
			smallest = k
		}
	}
	if smallest < 0 || !r.wide(poses[smallest], poses[smallest+1]) {
		return -1
	}
	if allClear {
		if reading, _ := r.pathClearance(poses, spans[smallest].clearance); reading != nil && reading.Tolerance.State != ToleranceSatisfied {
			return smallest
		}
	}
	if r.cfg.minimumMM != nil && !anyViolated(poses) && !r.meetsMinimum(spans[smallest].clearance) {
		return smallest
	}
	return -1
}

// wide reports whether the interval between two poses is wider than the
// resolution.
func (r *motionRun) wide(a, b *motionPose) bool {
	return exceedsResolution(a.param, b.param, r.cfg.resolutionP)
}

// meetsMinimum reports whether a certified interval lower bound proves the
// requested minimum, compared exactly against the minimum as stated.
func (r *motionRun) meetsMinimum(clearance *Measurement) bool {
	if clearance == nil {
		return true
	}
	return proofarith.FloatRat(clearance.Value.Base()).Cmp(r.cfg.minimumMM) >= 0
}

func anyViolated(poses []*motionPose) bool {
	return slices.ContainsFunc(poses, func(p *motionPose) bool { return p.violated })
}

// evaluatePose builds every mover's transient placement at fraction f of the
// path, published as the parameter at, and runs every evaluated pair at it.
// Every bound is built from the exact parameter at f; poseDeviation charges
// whatever separates the pose PoseAt builds from at and the ideal pose at f.
func (r *motionRun) evaluatePose(f *big.Rat, at units.Value) (*motionPose, error) {
	if err := r.ctx.Err(); err != nil {
		return nil, err
	}
	param := r.spec.fromP.lerp(r.spec.toP, f)
	pose, err := r.spec.motion.PoseAt(at)
	if err != nil {
		return nil, err
	}
	ideals := []idealPose{r.spec.frame.at(param)}
	stretch := r.stretch
	if r.spec.kind == motionBetween && f.Cmp(big.NewRat(1, 1)) == 0 {
		// PoseAt(1) returns the stated To, which the exact screw of the read
		// parameters rebuilds only to rounding: the pose is charged against
		// both the ideal end and To itself (§5.1, η_1 = max(η_ideal, η_To)).
		ideals = append(ideals, r.spec.frame.statedEnd())
		stretch = r.stretchEnd
	}
	mp := &motionPose{
		f:     f,
		param: param,
		result: PoseResult{
			At:            at,
			Pose:          pose,
			Interferences: []Interference{},
			Clearances:    []Clearance{},
			Diagnostics:   []Diagnostic{},
		},
		pairs: make([][]motionPairPose, len(r.movers)),
	}
	for _, mv := range r.movers {
		r.validityFindings(mp, mv.validity)
	}
	for _, st := range r.statics {
		r.validityFindings(mp, st.validity)
	}
	for i := range r.movers {
		mp.pairs[i] = make([]motionPairPose, len(r.statics))
		if err := r.evaluateMover(mp, i, pose, ideals, stretch); err != nil {
			return nil, err
		}
	}
	return mp, nil
}

// validityFindings carries a not-proven-valid body's own validity diagnostic
// into the pose (docs/motion-check-design.md §7), with At set.
func (r *motionRun) validityFindings(mp *motionPose, validity ValidityResult) {
	if validity.Outcome == ValidityValid {
		return
	}
	for _, diag := range validity.Diagnostics {
		diag = withAt(diag, mp.result.At)
		mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
		mp.findings = append(mp.findings, diag)
	}
}

func withAt(diag Diagnostic, at units.Value) Diagnostic {
	diag.At = &at
	return diag
}

// evaluateMover places mover i at the pose as a transient body and runs its
// evaluated pairs. η is the largest poseDeviation against every ideal pose
// the claim at this parameter speaks for, and the swept-volume allowance is
// charged at the largest linear factor among them and the pose's stretch base.
func (r *motionRun) evaluateMover(mp *motionPose, i int, pose r3.Transform, ideals []idealPose, stretch float64) error {
	mv := r.movers[i]
	need := false
	for j := range r.statics {
		pair := r.pairs[i][j]
		if pair.sheet {
			diag := withAt(pairDiagNone(mv.body, r.statics[j].body, DiagUnsupportedPairSheet,
				"a sheet operand has no clearance the motion check can certify, so this pair is undecided at every pose"), mp.result.At)
			mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
			mp.findings = append(mp.findings, diag)
		}
		need = need || pair.evaluated()
	}
	if !need {
		return nil
	}
	placement := mv.body.payload.transform()
	composed, err := placement.Then(pose)
	if err != nil {
		return fmt.Errorf(`%w: composing the pose onto a moving body's placement failed: %w`, ErrNotFinite, err)
	}
	transient, err := mv.body.payload.placed(r.ctx, r.transient, transientProducer, composed)
	if err != nil {
		return err
	}
	defer delete(r.cache.entries, transient)
	var eta, linear float64
	for _, ideal := range ideals {
		e, l := poseDeviation(composed, placement, ideal, mv.r0)
		eta, linear = math.Max(eta, e), math.Max(linear, l)
	}
	// The volume an overlap can lose between the float pose and the ideal one
	// (§5.1): every boundary point moves at most η along the straight path
	// between the two images, and the area that path carries is the mover's
	// own, scaled by the path's largest linear stretch.
	allowance := sweptVolumeAllow(eta, pathAreaUpper(mv.area, linear, mv.sigma, stretch))
	for j := range r.statics {
		if !r.pairs[i][j].evaluated() {
			continue
		}
		if err := r.evaluatePair(mp, i, j, transient, eta, allowance); err != nil {
			return err
		}
	}
	return nil
}

// evaluatePair runs Verify's own pair procedure (verify.go) on one (placed
// mover, static) pair at one pose, with the gap always asked: box separation,
// the closed-form axis-box gap or the clearance kernel, then the read-only
// overlap proof.
func (r *motionRun) evaluatePair(mp *motionPose, i, j int, transient *Body, eta, allowance float64) error {
	mover, static := r.movers[i].body, r.statics[j].body
	at := mp.result.At
	boxProven := boxesDisjoint(transient.bounds, static.bounds)
	res, fast := clearanceAxisBoxes(transient, static)
	if !fast {
		var err error
		res, err = clearancePairCached(r.ctx, transient, static, boxProven, r.cache)
		if err != nil {
			return err
		}
	}
	if err := r.ctx.Err(); err != nil {
		return err
	}
	if res.verdict == pairDisjoint || res.verdict == pairTouching {
		r.recordGap(mp, i, j, res, eta)
		return nil
	}
	if boxProven {
		r.poseDiag(mp, withAt(pairDiagNone(mover, static, DiagUndecidedClearance,
			"the pair is proven disjoint at this pose but its gap is unmeasured"), at))
		return nil
	}
	volume, outcome, err := measuredInterference(r.ctx, transient, static, res)
	if err != nil {
		return err
	}
	if outcome != interferenceMeasured && res.verdict != pairOverlapping {
		diag := undecidedPairDiag(transient, static, res.verdict, outcome)
		diag.Pair = &DiagnosticPair{A: mover, B: static}
		r.poseDiag(mp, withAt(diag, at))
		return nil
	}
	published, ok := transferredOverlap(volume, outcome == interferenceMeasured, allowance)
	if !ok {
		msg := "the pair is proven to overlap at the evaluated float pose, but the overlap volume is unmeasured, so it cannot be carried to the ideal pose and no collision is proven at this parameter"
		if outcome == interferenceMeasured {
			msg = fmt.Sprintf("the pair is proven to overlap at the evaluated float pose, but the measured volume %s does not clear the %v mm^3 the pose's deviation from the ideal motion can sweep, so no collision is proven at this parameter", volume.Value, allowance)
		}
		r.poseDiag(mp, withAt(pairDiagNone(mover, static, DiagUndecidedInterference, msg), at))
		return nil
	}
	mp.pairs[i][j].collision = true
	obs := published
	mp.collisions = append(mp.collisions, Collision{At: at, Pose: mp.result.Pose, Moving: mover, Static: static, Volume: published})
	mp.result.Interferences = append(mp.result.Interferences, Interference{A: mover, B: static, Volume: published})
	mp.findings = append(mp.findings, withAt(Diagnostic{
		Code:     DiagMotionCollision,
		Status:   Interfering,
		Pair:     &DiagnosticPair{A: mover, B: static},
		Reading:  ReadingOverlapVolume,
		Observed: &obs,
		Message:  fmt.Sprintf("the moving body overlaps a static body at %s", at),
	}, at))
	pairD, err := interferencePairDiameter(r.ctx, transient, static)
	if err != nil {
		return err
	}
	pass, ref, haveRef := interferenceToleranceRef(published, transient, static, pairD, r.cfg.rel)
	if !pass {
		beyond := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Pair:     &DiagnosticPair{A: mover, B: static},
			Reading:  ReadingOverlapVolume,
			Observed: &obs,
			Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", published.Bound),
		}
		if haveRef {
			beyond.Required = requiredThreshold(r.cfg.rel*ref, published.Value)
		}
		mp.findings = append(mp.findings, withAt(beyond, at))
	}
	return nil
}

// transferredOverlap is §5.1's collision transfer. An overlap measured at the
// float pose is a collision at the ideal pose only when its proven lower end,
// Value − Bound rounded down, strictly exceeds the volume the mover's boundary
// can sweep between the two poses; the published volume then carries that
// allowance in its Bound, so Value − Bound stays a proven lower bound on the
// ideal overlap. An unmeasured overlap never transfers.
func transferredOverlap(volume Measurement, measured bool, allowance float64) (Measurement, bool) {
	if !measured || isNonFinite(allowance) {
		return Measurement{}, false
	}
	value, bound := proofarith.FloatRat(volume.Value.Base()), proofarith.FloatRat(volume.Bound.Base())
	if value == nil || bound == nil {
		return Measurement{}, false
	}
	lower := ratFloatDown(new(big.Rat).Sub(value, bound))
	if !(lower > allowance) {
		return Measurement{}, false
	}
	if allowance == 0 {
		return volume, true
	}
	return Measurement{
		Value:     volume.Value,
		Exactness: Approximate,
		Bound:     units.CubicMillimeters(absSumUpper(volume.Bound.Base(), allowance)),
	}, true
}

// poseDiag records an undecided or unsupported pair finding both on the pose
// and in the report's flat list.
func (r *motionRun) poseDiag(mp *motionPose, diag Diagnostic) {
	mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
	mp.findings = append(mp.findings, diag)
}

// recordGap charges η into the kernel's proven gap interval and publishes the
// pose's Clearance row. The kernel's interval is about the float placement it
// measured; widening it by η on both sides — distance is 1-Lipschitz in a
// displacement of either set — makes it a proven interval on the IDEAL pose,
// which is what the interval certificate consumes. An η that cannot be
// bounded leaves the gap unmeasured.
func (r *motionRun) recordGap(mp *motionPose, i, j int, res pairResult, eta float64) {
	mover, static := r.movers[i].body, r.statics[j].body
	at := mp.result.At
	if isNonFinite(eta) {
		r.poseDiag(mp, withAt(pairDiagNone(mover, static, DiagUndecidedClearance,
			"the pair is proven disjoint at this pose, but the pose's departure from the ideal motion is unbounded for this payload, so its gap is unmeasured"), at))
		return
	}
	lo, hi, exact := clearanceDeltaWiden(res.lo, res.hi, res.exact, eta, 0)
	mp.pairs[i][j] = motionPairPose{hasGap: true, lo: lo, hi: hi, diam: res.diam}
	gap := pairGapMeasurement(pairResult{lo: lo, hi: hi, exact: exact})
	mp.result.Clearances = append(mp.result.Clearances, Clearance{A: mover, B: static, Gap: gap})
	if r.cfg.minimumMM != nil && proofarith.FloatRat(hi).Cmp(r.cfg.minimumMM) < 0 {
		// The proven upper end of the ideal pose's gap lies below the spec:
		// the margin is disproven here, whatever the reading's precision.
		mp.violated = true
		violation := gap
		mp.findings = append(mp.findings, withAt(Diagnostic{
			Code:     DiagMotionClearanceViolated,
			Status:   Violating,
			Pair:     &DiagnosticPair{A: mover, B: static},
			Reading:  ReadingGap,
			Observed: &violation,
			Required: r.cfg.minimum,
			Message:  fmt.Sprintf("the gap at %s is proven below the required minimum %s", at, *r.cfg.minimum),
		}, at))
	}
	pass, ref, haveRef := scalarToleranceRef(gap, r.cfg.rel, pairToleranceInputs{diameter: res.diam}.lengthReference)
	if pass {
		return
	}
	obs := gap
	beyond := Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Pair:     &DiagnosticPair{A: mover, B: static},
		Reading:  ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		beyond.Required = requiredThreshold(r.cfg.rel*ref, gap.Value)
	}
	mp.findings = append(mp.findings, withAt(beyond, at))
}

// intervalVerdict decides one interval between adjacent poses a and b
// (docs/motion-check-design.md §5.2). A proven collision at either end makes
// it IntervalColliding. Otherwise it is IntervalClear only when EVERY (mover,
// static) pair certifies: a swept-box-excluded pair by its whole-path lower
// bound, an evaluated pair by lo_a + lo_b > τ — taken over exact rationals, so
// no rounding sits between the proven terms and the strict comparison. The
// lower envelope's minimum over the interval, (lo_a + lo_b − τ)/2, is rounded
// down to the float the Clearance publishes.
func (r *motionRun) intervalVerdict(a, b *motionPose) motionSpan {
	outcome, clearance := r.intervalOutcome(a, b)
	return motionSpan{outcome: outcome, clearance: clearance}
}

func (r *motionRun) intervalOutcome(a, b *motionPose) (IntervalOutcome, *Measurement) {
	for i := range r.movers {
		for j := range r.statics {
			if a.pairs[i][j].collision || b.pairs[i][j].collision {
				return IntervalColliding, nil
			}
		}
	}
	var lowest *big.Rat
	for i, mv := range r.movers {
		for j := range r.statics {
			pair := r.pairs[i][j]
			if pair.excluded {
				lowest = minRat(lowest, proofarith.FloatRat(pair.lower))
				continue
			}
			pa, pb := a.pairs[i][j], b.pairs[i][j]
			if !pair.evaluated() || !pa.hasGap || !pb.hasGap {
				return IntervalUndecided, nil
			}
			tau := moverTravel(r.spec.frame, mv.rho, a.param, b.param)
			if tau == nil {
				return IntervalUndecided, nil
			}
			sum := new(big.Rat).Add(proofarith.FloatRat(pa.lo), proofarith.FloatRat(pb.lo))
			if sum.Cmp(tau) <= 0 {
				return IntervalUndecided, nil
			}
			envelope := sum.Sub(sum, tau)
			lowest = minRat(lowest, envelope.Quo(envelope, big.NewRat(2, 1)))
		}
	}
	if lowest == nil {
		return IntervalClear, nil
	}
	return IntervalClear, &Measurement{
		Value:     units.Millimeters(ratFloatDown(lowest)),
		Exactness: Approximate,
		Bound:     units.Millimeters(0),
	}
}

// minRat is the smaller of an optional running minimum and a candidate.
func minRat(running, candidate *big.Rat) *big.Rat {
	if running == nil || candidate.Cmp(running) < 0 {
		return candidate
	}
	return running
}

// publish assembles the report (docs/motion-check-design.md §4).
func (r *motionRun) publish(poses []*motionPose, spans []motionSpan) *MotionReport {
	report := &MotionReport{
		Request: MotionRequest{
			RelativeTolerance: units.Scalar(r.cfg.rel),
			Resolution:        r.cfg.resolution,
			MinClearance:      r.cfg.minimum,
		},
		Motion:      r.spec.motion,
		Diagnostics: []Diagnostic{},
		Collisions:  []Collision{},
	}
	for _, mv := range r.movers {
		report.Moving = append(report.Moving, mv.body)
	}
	report.Against = []*Body{}
	for _, st := range r.statics {
		report.Against = append(report.Against, st.body)
	}

	violated := anyViolated(poses)
	allClear, met := true, true
	var lowest *Measurement
	for k, span := range spans {
		a, b := poses[k], poses[k+1]
		interval := MotionInterval{From: a.result.At, To: b.result.At, Outcome: span.outcome, Clearance: span.clearance}
		report.Intervals = append(report.Intervals, interval)
		if span.outcome != IntervalClear {
			allClear, met = false, false
		}
		if span.clearance != nil && (lowest == nil || span.clearance.Value.Base() < lowest.Value.Base()) {
			lowest = span.clearance
		}
		switch {
		case span.outcome == IntervalUndecided:
			report.Diagnostics = append(report.Diagnostics, withAt(Diagnostic{
				Code:    DiagMotionUndecidedInterval,
				Status:  Suspect,
				Reading: ReadingNone,
				Message: fmt.Sprintf("the motion from %s to %s is neither certified clear nor bounded by a proven collision", a.result.At, b.result.At),
			}, a.result.At))
		case span.outcome == IntervalClear && r.cfg.minimumMM != nil && !r.meetsMinimum(span.clearance):
			met = false
			if violated {
				break
			}
			obs := *span.clearance
			report.Diagnostics = append(report.Diagnostics, withAt(Diagnostic{
				Code:     DiagMotionUndecidedClearance,
				Status:   Suspect,
				Reading:  ReadingGap,
				Observed: &obs,
				Required: r.cfg.minimum,
				Message:  fmt.Sprintf("the motion from %s to %s is certified clear, but its proven lower bound does not reach the required minimum", a.result.At, b.result.At),
			}, a.result.At))
		}
	}
	switch {
	case r.cfg.minimumMM == nil:
		report.Assessment = AssessmentNotEvaluated
	case violated:
		report.Assessment = AssessmentViolated
	case met:
		report.Assessment = AssessmentMet
	default:
		report.Assessment = AssessmentUndecided
	}
	for _, pose := range poses {
		report.Poses = append(report.Poses, pose.result)
		report.Collisions = append(report.Collisions, pose.collisions...)
		report.Diagnostics = append(report.Diagnostics, pose.findings...)
	}
	if allClear && lowest != nil {
		if reading, diag := r.pathClearance(poses, lowest); reading != nil {
			report.Clearance = reading
			if diag != nil {
				report.Diagnostics = append(report.Diagnostics, *diag)
			}
		}
	}
	report.Status = Sound
	for _, diag := range report.Diagnostics {
		report.Status = max(report.Status, diag.Status)
	}
	return report
}

// pathClearance is docs/motion-check-design.md §5.3's whole-path reading: the
// minimum gap over the path lies between the smallest interval lower bound
// and the smallest evaluated upper bound, since any pose's upper bound bounds
// the path's minimum from above. It reports that interval's midpoint and
// half-width, never Exact, judged against the diameter of the pair that
// attained the upper end. A path whose pairs were all settled by swept-box
// exclusion has no upper bound and carries no reading.
func (r *motionRun) pathClearance(poses []*motionPose, lowest *Measurement) (*ScalarReading, *Diagnostic) {
	upper := math.Inf(1)
	diam := 0.0
	for _, pose := range poses {
		for _, row := range pose.pairs {
			for _, pp := range row {
				if pp.hasGap && pp.hi < upper {
					upper, diam = pp.hi, pp.diam
				}
			}
		}
	}
	if math.IsInf(upper, 1) {
		return nil, nil
	}
	// Lowering a proven lower bound keeps it one, so a lower end above the
	// upper end — which sound bounds never produce — is clamped rather than
	// published inverted.
	lower := math.Min(lowest.Value.Base(), upper)
	gap := pairGapMeasurement(pairResult{lo: lower, hi: upper})
	gap.Exactness = Approximate
	pass, ref, haveRef := scalarToleranceRef(gap, r.cfg.rel, pairToleranceInputs{diameter: diam}.lengthReference)
	reading := &ScalarReading{Measurement: gap, Tolerance: judgeTolerance(pass, haveRef, r.cfg.rel, ref, gap.Value)}
	if pass {
		return reading, nil
	}
	obs := gap
	diag := &Diagnostic{
		Code:     toleranceDiagnostic(reading.Tolerance),
		Status:   Suspect,
		Reading:  ReadingGap,
		Observed: &obs,
		Required: reading.Tolerance.Limit,
		Message:  fmt.Sprintf("the whole-path gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	return reading, diag
}
