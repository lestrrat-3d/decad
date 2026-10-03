package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Document.VerifyMotion (docs/motion-check-design.md §3, §5-§8):
// validation, the swept-box exclusion, the per-pose pair proof over transient
// placements, the two-sided interval certificate, and the report assembly.
// motion.go owns the vocabulary and motion_bound.go the bounds.
//
// The check evaluates the two endpoints of the path. An interval those two
// poses cannot certify reads IntervalUndecided and makes the report Suspect;
// nothing between evaluated poses is ever assumed clear.

// transientProducer is the reserved producer identity every transient pose
// placement is built under. It never enters the document: a transient body is
// never committed, never published, and its provenance is never read back.
const transientProducer producerID = -1

// motionSpec is a validated Motion read into the fields every pose and bound
// is built from.
type motionSpec struct {
	motion     Motion
	revolute   bool
	center     r3.Vec
	axis       r3.Vec
	dir        r3.Vec
	from, to   units.Value
	fromP, toP motionParam
	frame      motionFrame
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
		spec = motionSpec{revolute: true, center: mv.Center, axis: mv.Axis, from: mv.From, to: mv.To}
	case *Revolute:
		if mv == nil {
			return motionSpec{}, fmt.Errorf(`%w: a nil motion names no path`, ErrDegenerate)
		}
		return resolveMotionAs(m, *mv)
	case Prismatic:
		if err := mv.validate(); err != nil {
			return motionSpec{}, err
		}
		spec = motionSpec{dir: mv.Dir, from: mv.From, to: mv.To}
	case *Prismatic:
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
// cannot certify is IntervalUndecided and the report reads Suspect, never
// Sound. This increment evaluates the two endpoints of the path only.
//
// moving MUST be non-empty, hold live bodies of d, and list no body twice.
// Every validation error is returned before ctx is read; after validation a
// canceled context returns ctx.Err() and no report.
func (d *Document) VerifyMotion(ctx context.Context, moving []*Body, m Motion, opts ...MotionOption) (*MotionReport, error) {
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
	cfg, err := resolveMotionOptions(opts)
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
	ctx     context.Context //nolint:containedctx // motionRun is per-call state and never outlives VerifyMotion.
	d       *Document
	spec    motionSpec
	cfg     motionConfig
	cache   *bodyGeomCache
	movers  []motionMover
	statics []motionStatic
	pairs   [][]motionPair // [mover][static]
}

type motionMover struct {
	body     *Body
	validity ValidityResult
	rho      float64 // ρ_max; revolute only
	r0       float64 // the record radius poseDeviation charges at
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
	param      motionParam
	result     PoseResult
	pairs      [][]motionPairPose
	findings   []Diagnostic // this pose's findings, in report order
	collisions []Collision
}

func (r *motionRun) setup(moving []*Body) {
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
	zero := motionParam{turn: new(big.Rat), base: new(big.Rat)}
	r.pairs = make([][]motionPair, len(r.movers))
	for i := range r.movers {
		mv := &r.movers[i]
		mv.r0 = moverRecordRadius(r.ctx, mv.body)
		if r.spec.revolute {
			mv.rho = moverAxisRadius(mv.body, r.spec.frame)
		}
		// The swept box covers every pose from where the mover sits NOW — the
		// parameter 0 — so its travel runs from 0 to the farther endpoint,
		// never merely across [From, To].
		travel := maxRat(
			moverTravel(r.spec.revolute, mv.rho, zero, r.spec.fromP),
			moverTravel(r.spec.revolute, mv.rho, zero, r.spec.toP),
		)
		r.pairs[i] = make([]motionPair, len(r.statics))
		for j, st := range r.statics {
			pair := &r.pairs[i][j]
			if mv.validity.Outcome != ValidityValid || st.validity.Outcome != ValidityValid {
				pair.invalid = true
				continue
			}
			if lower, ok := sweptBoxLower(mv.body.bounds, st.body.bounds, travel); ok {
				pair.excluded, pair.lower = true, lower
				continue
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
	if err := r.ctx.Err(); err != nil {
		return nil, err
	}
	return r.publish(poses), nil
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
	ideal := r.spec.frame.at(param)
	mp := &motionPose{
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
		if err := r.evaluateMover(mp, i, pose, ideal); err != nil {
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
// evaluated pairs.
func (r *motionRun) evaluateMover(mp *motionPose, i int, pose r3.Transform, ideal idealPose) error {
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
	transient, err := mv.body.payload.placed(r.ctx, r.d, transientProducer, composed)
	if err != nil {
		return err
	}
	defer delete(r.cache.entries, transient)
	eta := poseDeviation(composed, placement, ideal, mv.r0)
	for j := range r.statics {
		if !r.pairs[i][j].evaluated() {
			continue
		}
		if err := r.evaluatePair(mp, i, j, transient, eta); err != nil {
			return err
		}
	}
	return nil
}

// evaluatePair runs Verify's own pair procedure (verify.go) on one (placed
// mover, static) pair at one pose, with the gap always asked: box separation,
// the closed-form axis-box gap or the clearance kernel, then the read-only
// overlap proof.
func (r *motionRun) evaluatePair(mp *motionPose, i, j int, transient *Body, eta float64) error {
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
	mp.pairs[i][j].collision = true
	collision := Collision{At: at, Pose: mp.result.Pose, Moving: mover, Static: static}
	diag := Diagnostic{
		Code:    DiagMotionCollision,
		Status:  Interfering,
		Pair:    &DiagnosticPair{A: mover, B: static},
		Reading: ReadingNone,
		Message: fmt.Sprintf("the moving body overlaps a static body at %s", at),
	}
	if outcome != interferenceMeasured {
		mp.collisions = append(mp.collisions, collision)
		mp.findings = append(mp.findings, withAt(diag, at))
		return nil
	}
	obs := volume
	collision.Volume = &obs
	mp.collisions = append(mp.collisions, collision)
	mp.result.Interferences = append(mp.result.Interferences, Interference{A: mover, B: static, Volume: volume})
	diag.Reading, diag.Observed = ReadingOverlapVolume, &obs
	mp.findings = append(mp.findings, withAt(diag, at))
	pairD, err := interferencePairDiameter(r.ctx, transient, static)
	if err != nil {
		return err
	}
	pass, ref, haveRef := interferenceToleranceRef(volume, transient, static, pairD, r.cfg.rel)
	if !pass {
		beyond := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Pair:     &DiagnosticPair{A: mover, B: static},
			Reading:  ReadingOverlapVolume,
			Observed: &obs,
			Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", volume.Bound),
		}
		if haveRef {
			beyond.Required = requiredThreshold(r.cfg.rel*ref, volume.Value)
		}
		mp.findings = append(mp.findings, withAt(beyond, at))
	}
	return nil
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
func (r *motionRun) intervalVerdict(a, b *motionPose) (IntervalOutcome, *Measurement) {
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
				lowest = minRat(lowest, floatRat(pair.lower))
				continue
			}
			pa, pb := a.pairs[i][j], b.pairs[i][j]
			if !pair.evaluated() || !pa.hasGap || !pb.hasGap {
				return IntervalUndecided, nil
			}
			tau := moverTravel(r.spec.revolute, mv.rho, a.param, b.param)
			if tau == nil {
				return IntervalUndecided, nil
			}
			sum := new(big.Rat).Add(floatRat(pa.lo), floatRat(pb.lo))
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
func (r *motionRun) publish(poses []*motionPose) *MotionReport {
	report := &MotionReport{
		Request:     MotionRequest{RelativeTolerance: units.Scalar(r.cfg.rel)},
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

	allClear := true
	var lowest *Measurement
	for k := 0; k+1 < len(poses); k++ {
		a, b := poses[k], poses[k+1]
		outcome, clearance := r.intervalVerdict(a, b)
		interval := MotionInterval{From: a.result.At, To: b.result.At, Outcome: outcome, Clearance: clearance}
		report.Intervals = append(report.Intervals, interval)
		if outcome != IntervalClear {
			allClear = false
		}
		if clearance != nil && (lowest == nil || clearance.Value.Base() < lowest.Value.Base()) {
			lowest = clearance
		}
		if outcome == IntervalUndecided {
			report.Diagnostics = append(report.Diagnostics, withAt(Diagnostic{
				Code:    DiagMotionUndecidedInterval,
				Status:  Suspect,
				Reading: ReadingNone,
				Message: fmt.Sprintf("the motion from %s to %s is neither certified clear nor bounded by a proven collision", a.result.At, b.result.At),
			}, a.result.At))
		}
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
