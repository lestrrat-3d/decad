package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/verifyrun"

	"github.com/lestrrat-3d/decad/internal/boundaryaudit"
	"github.com/lestrrat-3d/decad/internal/motionconclude"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
)

// evaluateMover runs mover i's row of pairs at the pose: a sheet pair's
// finding first, then every evaluated pair in pair order — a constant pair
// replaying its one evaluation, every other pair over the transient
// placements, each built when its first pair needs it, so a mover whose
// evaluated pairs are all constant is never placed.
func (r *motionRun) evaluateMover(mp *motionPose, i int, placed []*motionPlaced) error {
	mv := r.movers[i]
	for k, pair := range r.pairs[i] {
		if pair.sheet && !pair.declared && !pair.unformed {
			diag := mp.stamp(verifyrun.PairDiagNone(mv.body, r.partner(i, k), DiagUnsupportedPairSheet,
				"a sheet operand has no clearance the motion check can certify, so this pair is undecided at every pose"))
			mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
			mp.findings = append(mp.findings, diag)
		}
	}
	var a *motionPlaced
	for k := range r.pairs[i] {
		if !r.pairs[i][k].evaluated() {
			continue
		}
		if r.pairs[i][k].constant {
			if err := r.settleConstant(i, k); err != nil {
				return err
			}
		}
		if once := r.pairs[i][k].once; once != nil {
			r.replayConstant(mp, i, k, once)
			continue
		}
		var err error
		if a == nil {
			if a, err = r.place(mp, i, placed); err != nil {
				return err
			}
		}
		var b *motionPlaced
		if other := r.pairs[i][k].other; other >= 0 {
			if b, err = r.place(mp, other, placed); err != nil {
				return err
			}
		}
		if err := r.evaluatePair(mp, i, k, a, b); err != nil {
			return err
		}
	}
	return nil
}

// settleConstant runs constant pair k of mover i's one evaluation
// (docs/linkage-check-design.md §5.9) on the two bodies as they stand, the
// first time a pose reaches it: a measured gap or a transferred overlap is
// kept as the pair's outcome at every pose; any other outcome clears the
// pair's constant mark and leaves it to the per-pose procedure.
func (r *motionRun) settleConstant(i, k int) error {
	pair := &r.pairs[i][k]
	pair.constant = false
	mover, partner := r.movers[i].body, r.partner(i, k)
	boxProven := measurement.BoxesDisjoint(mover.n.Bounds, partner.n.Bounds)
	res, fast := pairkernel.AxisBoxes(&mover.n, &partner.n)
	if !fast {
		var err error
		if res, err = pairkernel.Pair(r.ctx, &mover.n, &partner.n, boxProven, r.cache); err != nil {
			return err
		}
	}
	if err := r.ctx.Err(); err != nil {
		return err
	}
	switch {
	case res.Verdict == boundaryaudit.PairDisjoint:
		pair.once = &constantOutcome{res: res}
		return nil
	case res.Verdict == boundaryaudit.PairTouching || boxProven:
		return nil
	}
	volume, outcome, err := verifyrun.MeasuredInterference(r.ctx, handleAlloc{}, bodyNode(mover), bodyNode(partner), res, pairMeshes{})
	if err != nil {
		return err
	}
	if outcome != boundaryaudit.InterferenceMeasured {
		return nil
	}
	publishedInternal, ok := motionbound.TransferredOverlap(volume, true, 0)
	if !ok {
		return nil
	}
	published := publishedInternal
	beyond, fails, err := r.collisionBeyond(mover, partner, mover, partner, published)
	if err != nil {
		return err
	}
	pair.once = &constantOutcome{collision: true, volume: published}
	if fails {
		pair.once.beyond = &beyond
	}
	return nil
}

// replayConstant publishes constant pair k of mover i's one outcome at the
// pose: its gap with η zero, or its collision, exactly as evaluatePair
// publishes them.
func (r *motionRun) replayConstant(mp *motionPose, i, k int, once *constantOutcome) {
	if !once.collision {
		r.recordGap(mp, i, k, once.res, 0, 0)
		return
	}
	r.publishCollision(mp, i, k, once.volume, once.beyond)
}

// place builds mover i's transient placement at the pose once. η is the
// largest motionbound.PoseDeviation against every ideal pose the claim at
// this parameter speaks for, and the swept-volume allowance is charged at the
// largest linear factor among them and the group's stretch base.
//
// place refuses to RE-PLACE a faceted mover that descends from an exact-rotation
// copy (facetedPayload.ExactPlaced): PoseDeviation models a mover as P*p for
// record points, which does not describe a re-placement of Q*S. The refusal
// applies where a mover is placed; a fixed group, a settled constant pair
// and a mover with no evaluated pair never reach it.
func (r *motionRun) place(mp *motionPose, i int, placed []*motionPlaced) (*motionPlaced, error) {
	if placed[i] != nil {
		return placed[i], nil
	}
	mv := r.movers[i]
	group := mp.groups[mv.group]
	switch {
	case group.fixed:
		placed[i] = &motionPlaced{body: mv.body, kept: true}
		return placed[i], nil
	case group.constant && r.constPlaced != nil && r.constPlaced[i] != nil:
		placed[i] = r.constPlaced[i]
		return placed[i], nil
	}
	if fp, ok := mv.body.n.Payload.(facetedPayload); ok && fp.ExactPlaced {
		return nil, fmt.Errorf("%w: motion verification cannot re-place a body that descends from an "+
			"exact-rotation copy (WithCopyRotation), because its pose has no deviation proof", ErrUnsupported)
	}
	placement := mv.body.n.Payload.Transform()
	composed, err := placement.Then(group.pose)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the pose onto a moving body's placement failed: %w`, ErrNotFinite, err)
	}
	cacheable := r.poseBodies != nil && !group.constant
	var transient *Body
	var key [12]uint64
	if cacheable {
		if err := r.ctx.Err(); err != nil {
			return nil, err
		}
		key = motionbound.TransformBits(composed)
		transient = r.poseBodies[i].get(key)
	}
	if transient == nil {
		transient, err = placePayload(r.ctx, mv.body.n.Payload, r.transient, transientProducer, composed)
		if err != nil {
			return nil, err
		}
		if cacheable {
			if old := r.poseBodies[i].put(key, transient); old != nil {
				r.cache.Evict(&old.n)
			}
		}
	}
	var delta r3.Transform
	_, faceted := mv.body.n.Payload.(facetedPayload)
	if faceted {
		delta, err = facetedrecord.PlacementDelta(placement, composed)
		if err != nil {
			return nil, err
		}
	}
	var eta, linear float64
	for _, ideal := range group.ideals {
		e, l := motionbound.PoseDeviation(composed, placement, ideal, mv.r0)
		if faceted {
			// The record term stays unbounded for rotation poses; the delta
			// term also charges translations applied to the current held mesh.
			eDelta, lDelta := motionbound.PoseDeviation(delta, r3.Identity(), ideal, mv.heldRadius)
			e, l = math.Max(e, eDelta), math.Max(l, lDelta)
		}
		eta, linear = math.Max(eta, e), math.Max(linear, l)
	}
	// The volume an overlap can lose between the float pose and the ideal one
	// (§5.1): every boundary point moves at most η along the straight path
	// between the two images, and the area that path carries is the mover's
	// own, scaled by the path's largest linear stretch.
	allowance := proofbound.SweptVolumeAllow(eta, motionbound.PathAreaUpper(mv.area, linear, mv.sigma, group.stretch))
	placed[i] = &motionPlaced{body: transient, eta: eta, allowance: allowance, kept: cacheable}
	if group.constant {
		if r.constPlaced == nil {
			r.constPlaced = make([]*motionPlaced, len(r.movers))
		}
		placed[i].kept = true
		r.constPlaced[i] = placed[i]
	}
	return placed[i], nil
}

// evaluatePair runs Verify's own pair procedure (verify.go) on pair k of
// mover i at one pose, with the gap always asked: box separation, the
// closed-form axis-box gap or the clearance kernel, then the read-only
// overlap proof. a is the mover's transient placement; b is its partner's,
// or nil when the partner is a static body measured as it stands. A moving
// partner adds its own η to the gap's widening and its own allowance to the
// collision transfer (docs/linkage-check-design.md §5.1).
func (r *motionRun) evaluatePair(mp *motionPose, i, k int, a, b *motionPlaced) error {
	mover, partner := r.movers[i].body, r.partner(i, k)
	target, etaB, allowance := partner, 0.0, a.allowance
	if b != nil {
		target, etaB, allowance = b.body, b.eta, proofbound.AbsSumUpper(a.allowance, b.allowance)
	}
	boxProven := measurement.BoxesDisjoint(a.body.n.Bounds, target.n.Bounds)
	res, fast := pairkernel.AxisBoxes(&a.body.n, &target.n)
	if !fast {
		var err error
		res, err = pairkernel.Pair(r.ctx, &a.body.n, &target.n, boxProven, r.cache)
		if err != nil {
			return err
		}
	}
	if err := r.ctx.Err(); err != nil {
		return err
	}
	if res.Verdict == boundaryaudit.PairDisjoint || res.Verdict == boundaryaudit.PairTouching {
		r.recordGap(mp, i, k, res, a.eta, etaB)
		return nil
	}
	declared := r.pairs[i][k].declared
	if boxProven {
		if !declared {
			r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, DiagUndecidedClearance,
				"the pair is proven disjoint at this pose but its gap is unmeasured")))
		}
		return nil
	}
	volume, outcome, err := verifyrun.MeasuredInterference(r.ctx, handleAlloc{}, bodyNode(a.body), bodyNode(target), res, pairMeshes{})
	if err != nil {
		return err
	}
	if outcome != boundaryaudit.InterferenceMeasured && res.Verdict != boundaryaudit.PairOverlapping {
		if declared {
			return nil
		}
		diag := verifyrun.UndecidedPairDiag(a.body, target, bodyNode(a.body), bodyNode(target), res.Verdict, outcome)
		diag.Pair = &DiagnosticPair{A: mover, B: partner}
		r.poseDiag(mp, mp.stamp(diag))
		return nil
	}
	publishedInternal, ok := motionbound.TransferredOverlap(volume, outcome == boundaryaudit.InterferenceMeasured, allowance)
	if !ok && declared {
		return nil
	}
	if !ok {
		msg := "the pair is proven to overlap at the evaluated float pose, but the overlap volume is unmeasured, so it cannot be carried to the ideal pose and no collision is proven at this parameter"
		if outcome == boundaryaudit.InterferenceMeasured {
			msg = fmt.Sprintf("the pair is proven to overlap at the evaluated float pose, but the measured volume %s does not clear the %v mm^3 the pose's deviation from the ideal motion can sweep, so no collision is proven at this parameter", volume.Value, allowance)
		}
		r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, DiagUndecidedInterference, msg)))
		return nil
	}
	published := publishedInternal
	beyond, fails, err := r.collisionBeyond(a.body, target, mover, partner, published)
	if err != nil {
		return err
	}
	if !fails {
		r.publishCollision(mp, i, k, published, nil)
		return nil
	}
	r.publishCollision(mp, i, k, published, &beyond)
	return nil
}

// collisionBeyond is a transferred collision's tolerance finding, read on
// the measured bodies a and b and naming the caller's mover and partner;
// fails is false, and the finding empty, when the volume reading passes the
// relative tolerance.
func (r *motionRun) collisionBeyond(a, b, mover, partner *Body, published Measurement) (Diagnostic, bool, error) {
	pairD, err := pairkernel.PairGateDiameter(r.ctx, &a.n, &b.n, nil)
	if err != nil {
		return Diagnostic{}, false, err
	}
	pass, ref, haveRef := verifyrun.InterferenceToleranceRef(published, bodyNode(a), bodyNode(b), pairD, r.cfg.Rel)
	if pass {
		return Diagnostic{}, false, nil
	}
	obs := published
	beyond := Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  ReadingOverlapVolume,
		Observed: &obs,
		Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", published.Bound),
	}
	if haveRef {
		beyond.Required = tolerance.RequiredThreshold(r.cfg.Rel*ref, published.Value)
	}
	return beyond, true, nil
}

// publishCollision records pair k of mover i's transferred collision at the
// pose: the collision, its Interference row, its DiagMotionCollision and, when
// given, its tolerance finding.
func (r *motionRun) publishCollision(mp *motionPose, i, k int, published Measurement, beyond *Diagnostic) {
	mover, partner := r.movers[i].body, r.partner(i, k)
	mp.pairs[i][k].collision = true
	obs := published
	mp.collisions = append(mp.collisions, Collision{At: mp.result.At, Pose: mp.result.Pose, Moving: mover, Static: partner, Volume: published})
	mp.result.Interferences = append(mp.result.Interferences, Interference{A: mover, B: partner, Volume: published})
	msg := fmt.Sprintf("the moving body overlaps a static body at %s", mp.where)
	if r.pairs[i][k].other >= 0 {
		msg = fmt.Sprintf("the moving body overlaps another moving body at %s", mp.where)
	}
	mp.findings = append(mp.findings, mp.stamp(Diagnostic{
		Code:     DiagMotionCollision,
		Status:   Interfering,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  ReadingOverlapVolume,
		Observed: &obs,
		Message:  msg,
	}))
	if beyond != nil {
		finding := *beyond
		observed := *finding.Observed
		finding.Observed = &observed
		mp.findings = append(mp.findings, mp.stamp(finding))
	}
}

// poseDiag records an undecided or unsupported pair finding both on the pose
// and in the report's flat list.
func (r *motionRun) poseDiag(mp *motionPose, diag Diagnostic) {
	mp.result.Diagnostics = append(mp.result.Diagnostics, diag)
	mp.findings = append(mp.findings, diag)
}

// recordGap charges η into the kernel's proven gap interval and publishes the
// pose's Clearance row. The kernel's interval is about the float placements it
// measured; widening it by both bodies' η on both sides — distance is
// 1-Lipschitz in a displacement of either set — makes it a proven interval on
// the IDEAL poses, which is what the interval certificate consumes. A static
// partner's η is zero. An η that cannot be bounded leaves the gap unmeasured.
func (r *motionRun) recordGap(mp *motionPose, i, k int, res pairkernel.Result, etaA, etaB float64) {
	mover, partner := r.movers[i].body, r.partner(i, k)
	if r.pairs[i][k].declared {
		r.recordDeclaredGap(mp, mover, partner, res, etaA, etaB)
		return
	}
	if proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, DiagUndecidedClearance,
			"the pair is proven disjoint at this pose, but the pose's departure from the ideal motion is unbounded for this payload, so its gap is unmeasured")))
		return
	}
	lo, hi, exact := pairkernel.DeltaWiden(res.Lo, res.Hi, res.Exact, etaA, etaB)
	mp.pairs[i][k] = motionPairPose{hasGap: true, lo: lo, hi: hi, diam: res.Diam}
	gap := pairkernel.GapMeasurement(pairkernel.Result{Lo: lo, Hi: hi, Exact: exact})
	mp.result.Clearances = append(mp.result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
	if r.cfg.MinimumMM != nil && proofarith.FloatRat(hi).Cmp(r.cfg.MinimumMM) < 0 {
		// The proven upper end of the ideal pose's gap lies below the spec:
		// the margin is disproven here, whatever the reading's precision.
		mp.violated = true
		violation := gap
		mp.findings = append(mp.findings, mp.stamp(Diagnostic{
			Code:     DiagMotionClearanceViolated,
			Status:   Violating,
			Pair:     &DiagnosticPair{A: mover, B: partner},
			Reading:  ReadingGap,
			Observed: &violation,
			Required: r.cfg.Minimum,
			Message:  fmt.Sprintf("the gap at %s is proven below the required minimum %s", mp.where, *r.cfg.Minimum),
		}))
	}
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, r.cfg.Rel,
		verifyrun.PairToleranceInputs{Diameter: res.Diam}.LengthReference)
	if pass {
		return
	}
	obs := gap
	beyond := Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		beyond.Required = tolerance.RequiredThreshold(r.cfg.Rel*ref, gap.Value)
	}
	mp.findings = append(mp.findings, mp.stamp(beyond))
}

// recordDeclaredGap publishes a declared joint contact's measured gap as a
// Clearance row and nothing else (docs/linkage-check-design.md §5.4): a
// touching pair or an unbounded η publishes nothing, no finding is raised,
// and the gap feeds neither an interval certificate nor the whole-path
// reading.
func (r *motionRun) recordDeclaredGap(mp *motionPose, mover, partner *Body, res pairkernel.Result, etaA, etaB float64) {
	if res.Verdict != boundaryaudit.PairDisjoint || proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		return
	}
	lo, hi, exact := pairkernel.DeltaWiden(res.Lo, res.Hi, res.Exact, etaA, etaB)
	gap := pairkernel.GapMeasurement(pairkernel.Result{Lo: lo, Hi: hi, Exact: exact})
	mp.result.Clearances = append(mp.result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
}

// intervalVerdict decides one interval between adjacent poses a and b
// (docs/motion-check-design.md §5.2). A proven collision at either end makes
// it IntervalColliding. Otherwise it is IntervalClear only when EVERY pair
// certifies: a swept-box-excluded pair by its whole-path lower bound, an
// evaluated pair by the larger of two proven lower bounds on its gap over the
// interval being positive — the travel bound (lo_a + lo_b − τ)/2, the lower
// envelope's minimum, and the driver's projection bound when it supplies one
// (docs/linkage-check-design.md §5.8) — taken over exact rationals, so no
// rounding sits between the proven terms and the strict comparison. That
// larger bound is rounded down to the float the Clearance publishes.
func (r *motionRun) intervalVerdict(a, b *motionPose) motionSpan {
	for _, p := range []*motionPose{a, b} {
		if p.unbuildable != nil {
			return motionSpan{outcome: IntervalUndecided, note: p.unbuildable.Error()}
		}
	}
	if !r.collides(a) && !r.collides(b) {
		if gate, ok := r.drive.(motionIntervalGate); ok {
			if note := gate.intervalGate(a, b); note != "" {
				return motionSpan{outcome: IntervalUndecided, note: note}
			}
		}
	}
	outcome, clearance := r.intervalOutcome(a, b)
	return motionSpan{outcome: outcome, clearance: clearance}
}

func (r *motionRun) intervalOutcome(a, b *motionPose) (IntervalOutcome, *Measurement) {
	if r.collides(a) || r.collides(b) {
		return IntervalColliding, nil
	}
	lowest, ok := r.certifyPairs(func(i, k int) *big.Rat {
		pa, pb := a.pairs[i][k], b.pairs[i][k]
		if !pa.hasGap || !pb.hasGap {
			return nil
		}
		tau := r.drive.travel(i, k, a.param, b.param)
		if r.pairs[i][k].once != nil {
			// A constant pair travels nothing (docs/linkage-check-design.md
			// §5.9).
			tau = new(big.Rat)
		}
		return motionbound.IntervalPairLower(pa.lo, pb.lo, tau, r.drive.projection(i, k, a, b))
	}, nil)
	if !ok {
		return IntervalUndecided, nil
	}
	return IntervalClear, motionbound.LowerBoundMeasurement(lowest)
}

// collides reports whether some pair at the pose carries a proven collision.
func (r *motionRun) collides(mp *motionPose) bool {
	for i := range r.movers {
		for k := range r.pairs[i] {
			if mp.pairs[i][k].collision {
				return true
			}
		}
	}
	return false
}

// certifyPairs adapts the motion run's pair states to motionbound's shared
// interval and joint-box certificate walk. A declared or unformed pair enters
// no certificate; an invalid or sheet pair cannot certify one.
func (r *motionRun) certifyPairs(certify func(i, k int) *big.Rat, held func(i, k int)) (*big.Rat, bool) {
	return motionbound.CertifyPairs(len(r.movers), func(i int) int { return len(r.pairs[i]) },
		func(i, k int) motionbound.CertificatePair {
			p := r.pairs[i][k]
			return motionbound.CertificatePair{
				Skip: p.declared || p.unformed, Excluded: p.excluded,
				Lower: p.lower, Evaluated: p.evaluated(),
			}
		}, certify, held)
}

// conclude assembles the intervals, the whole-path reading, the assessment,
// the diagnostics and the status from the evaluated poses and intervals.
func (r *motionRun) conclude(poses []*motionPose, spans []motionSpan) motionconclude.MotionConclusion[*Body, JointCell] {
	against := make([]*Body, 0, len(r.statics))
	for _, st := range r.statics {
		against = append(against, st.body)
	}
	poseFacts := make([]motionconclude.MotionPoseFinding[*Body, JointCell], len(poses))
	for i, pose := range poses {
		poseFacts[i] = motionconclude.MotionPoseFinding[*Body, JointCell]{
			At: pose.result.At, Unbuildable: pose.unbuildable != nil,
			Violated: pose.violated, Diagnostics: pose.findings,
		}
	}
	spanFacts := make([]motionconclude.MotionSpanFinding, len(spans))
	for i, span := range spans {
		spanFacts[i] = motionconclude.MotionSpanFinding{
			Outcome: reportSpanOutcome(span.outcome), Clearance: span.clearance, Note: span.note,
		}
	}
	return motionconclude.ConcludeMotion(r.cfg.Minimum, against, poseFacts, spanFacts, r.cfg.MinimumMM,
		func(lowest *measurement.Measurement) (*ScalarReading, *Diagnostic) {
			reading, diag := r.pathClearance(poses, lowest, "whole-path")
			return reading, diag
		})
}

// publish assembles VerifyMotion's report (docs/motion-check-design.md §4).
func (r *motionRun) publish(poses []*motionPose, spans []motionSpan) *MotionReport {
	c := r.conclude(poses, spans)
	report := &MotionReport{
		Request:     motionRequest(r.cfg),
		Against:     c.Against,
		Intervals:   rootMotionIntervals(c.Intervals),
		Collisions:  []Collision{},
		Clearance:   c.Clearance,
		Assessment:  c.Assessment,
		Diagnostics: c.Diagnostics,
		Status:      c.Status,
	}
	for _, mv := range r.movers {
		report.Moving = append(report.Moving, mv.body)
	}
	for _, pose := range poses {
		report.Poses = append(report.Poses, pose.result)
		report.Collisions = append(report.Collisions, pose.collisions...)
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
func (r *motionRun) pathClearance(poses []*motionPose, lowest *Measurement, scope string) (*ScalarReading, *Diagnostic) {
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
	gap := pairkernel.GapMeasurement(pairkernel.Result{Lo: lower, Hi: upper})
	gap.Exactness = Approximate
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, r.cfg.Rel,
		verifyrun.PairToleranceInputs{Diameter: diam}.LengthReference)
	reading := &ScalarReading{Measurement: gap, Tolerance: tolerance.Judge(pass, haveRef, r.cfg.Rel, ref, gap.Value)}
	if pass {
		return reading, nil
	}
	obs := gap
	diag := &Diagnostic{
		Code:     tolerance.DiagnosticCode(reading.Tolerance),
		Status:   Suspect,
		Reading:  ReadingGap,
		Observed: &obs,
		Required: reading.Tolerance.Limit,
		Message:  fmt.Sprintf("the %s gap reading's bound %s is beyond the relative tolerance", scope, gap.Bound),
	}
	return reading, diag
}
