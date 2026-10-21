package motionengine

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"

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

// evaluateMover runs mover i's row of pairs at the Pose: a sheet pair's
// finding first, then every evaluated pair in pair order — a constant pair
// replaying its one evaluation, every other pair over the transient
// placements, each built when its first pair needs it, so a mover whose
// evaluated pairs are all constant is never placed.
func (r *Run) evaluateMover(mp *Pose, i int, placed []*Placed) error {
	mv := r.Movers[i]
	for k, pair := range r.Pairs[i] {
		if pair.Sheet && !pair.Declared && !pair.Unformed {
			diag := mp.stamp(verifyrun.PairDiagNone(mv.Body, r.Partner(i, k), vocab.DiagUnsupportedPairSheet,
				"a sheet operand has no clearance the motion check can certify, so this pair is undecided at every pose"))
			mp.Result.Diagnostics = append(mp.Result.Diagnostics, diag)
			mp.Findings = append(mp.Findings, diag)
		}
	}
	var a *Placed
	for k := range r.Pairs[i] {
		if !r.Pairs[i][k].Evaluated() {
			continue
		}
		if r.Pairs[i][k].Constant {
			if err := r.settleConstant(i, k); err != nil {
				return err
			}
		}
		if once := r.Pairs[i][k].Once; once != nil {
			r.replayConstant(mp, i, k, once)
			continue
		}
		var err error
		if a == nil {
			if a, err = r.place(mp, i, placed); err != nil {
				return err
			}
		}
		var b *Placed
		if other := r.Pairs[i][k].Other; other >= 0 {
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
func (r *Run) settleConstant(i, k int) error {
	pair := &r.Pairs[i][k]
	pair.Constant = false
	mover, partner := r.Movers[i].Body, r.Partner(i, k)
	boxProven := measurement.BoxesDisjoint(mover.Bounds, partner.Bounds)
	res, fast := pairkernel.AxisBoxes(mover, partner)
	if !fast {
		var err error
		if res, err = pairkernel.Pair(r.Ctx, mover, partner, boxProven, r.Cache); err != nil {
			return err
		}
	}
	if err := r.Ctx.Err(); err != nil {
		return err
	}
	switch {
	case res.Verdict == boundaryaudit.PairDisjoint:
		pair.Once = &constantOutcome{res: res}
		return nil
	case res.Verdict == boundaryaudit.PairTouching || boxProven:
		return nil
	}
	volume, outcome, err := verifyrun.MeasuredInterference(r.Ctx, r.hooks.Alloc, mover, partner, res, r.hooks.Operand)
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
	pair.Once = &constantOutcome{Collision: true, volume: published}
	if fails {
		pair.Once.beyond = &beyond
	}
	return nil
}

// replayConstant publishes constant pair k of mover i's one outcome at the
// Pose: its gap with η zero, or its collision, exactly as evaluatePair
// publishes them.
func (r *Run) replayConstant(mp *Pose, i, k int, once *constantOutcome) {
	if !once.Collision {
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
// copy (facetedrecord.Payload.ExactPlaced): PoseDeviation models a mover as P*p for
// record points, which does not describe a re-placement of Q*S. The refusal
// applies where a mover is placed; a fixed group, a settled constant pair
// and a mover with no evaluated pair never reach it.
func (r *Run) place(mp *Pose, i int, placed []*Placed) (*Placed, error) {
	if placed[i] != nil {
		return placed[i], nil
	}
	mv := r.Movers[i]
	group := mp.Groups[mv.Group]
	switch {
	case group.Fixed:
		placed[i] = &Placed{Body: mv.Body, kept: true}
		return placed[i], nil
	case group.Constant && r.constPlaced != nil && r.constPlaced[i] != nil:
		placed[i] = r.constPlaced[i]
		return placed[i], nil
	}
	if fp, ok := mv.Body.Payload.(facetedrecord.Payload); ok && fp.ExactPlaced {
		return nil, fmt.Errorf("%w: motion verification cannot re-place a body that descends from an "+
			"exact-rotation copy (WithCopyRotation), because its pose has no deviation proof", decaderr.ErrUnsupported)
	}
	placement := mv.Body.Payload.Transform()
	composed, err := placement.Then(group.Pose)
	if err != nil {
		return nil, fmt.Errorf(`%w: composing the pose onto a moving body's placement failed: %w`, decaderr.ErrNotFinite, err)
	}
	cacheable := r.PoseBodies != nil && !group.Constant
	var transient *topo.Body
	var key [12]uint64
	if cacheable {
		if err := r.Ctx.Err(); err != nil {
			return nil, err
		}
		key = motionbound.TransformBits(composed)
		transient = r.PoseBodies[i].Get(key)
	}
	if transient == nil {
		transient, err = r.hooks.Place(r.Ctx, mv.Body.Payload, r.transient, TransientProducer, composed)
		if err != nil {
			return nil, err
		}
		if cacheable {
			if old := r.PoseBodies[i].Put(key, transient); old != nil {
				r.Cache.Evict(old)
			}
		}
	}
	var delta r3.Transform
	_, faceted := mv.Body.Payload.(facetedrecord.Payload)
	if faceted {
		delta, err = facetedrecord.PlacementDelta(placement, composed)
		if err != nil {
			return nil, err
		}
	}
	var eta, linear float64
	for _, ideal := range group.Ideals {
		e, l := motionbound.PoseDeviation(composed, placement, ideal, mv.R0)
		if faceted {
			// The record term stays unbounded for rotation poses; the delta
			// term also charges translations applied to the current held mesh.
			eDelta, lDelta := motionbound.PoseDeviation(delta, r3.Identity(), ideal, mv.HeldRadius)
			e, l = math.Max(e, eDelta), math.Max(l, lDelta)
		}
		eta, linear = math.Max(eta, e), math.Max(linear, l)
	}
	// The volume an overlap can lose between the float pose and the ideal one
	// (§5.1): every boundary point moves at most η along the straight path
	// between the two images, and the area that path carries is the mover's
	// own, scaled by the path's largest linear stretch.
	allowance := proofbound.SweptVolumeAllow(eta, motionbound.PathAreaUpper(mv.Area, linear, mv.Sigma, group.Stretch))
	placed[i] = &Placed{Body: transient, eta: eta, allowance: allowance, kept: cacheable}
	if group.Constant {
		if r.constPlaced == nil {
			r.constPlaced = make([]*Placed, len(r.Movers))
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
func (r *Run) evaluatePair(mp *Pose, i, k int, a, b *Placed) error {
	mover, partner := r.Movers[i].Body, r.Partner(i, k)
	target, etaB, allowance := partner, 0.0, a.allowance
	if b != nil {
		target, etaB, allowance = b.Body, b.eta, proofbound.AbsSumUpper(a.allowance, b.allowance)
	}
	boxProven := measurement.BoxesDisjoint(a.Body.Bounds, target.Bounds)
	res, fast := pairkernel.AxisBoxes(a.Body, target)
	if !fast {
		var err error
		res, err = pairkernel.Pair(r.Ctx, a.Body, target, boxProven, r.Cache)
		if err != nil {
			return err
		}
	}
	if err := r.Ctx.Err(); err != nil {
		return err
	}
	if res.Verdict == boundaryaudit.PairDisjoint || res.Verdict == boundaryaudit.PairTouching {
		r.recordGap(mp, i, k, res, a.eta, etaB)
		return nil
	}
	declared := r.Pairs[i][k].Declared
	if boxProven {
		if !declared {
			r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, vocab.DiagUndecidedClearance,
				"the pair is proven disjoint at this pose but its gap is unmeasured")))
		}
		return nil
	}
	volume, outcome, err := verifyrun.MeasuredInterference(r.Ctx, r.hooks.Alloc, a.Body, target, res, r.hooks.Operand)
	if err != nil {
		return err
	}
	if outcome != boundaryaudit.InterferenceMeasured && res.Verdict != boundaryaudit.PairOverlapping {
		if declared {
			return nil
		}
		diag := verifyrun.UndecidedPairDiag(a.Body, target, a.Body, target, res.Verdict, outcome)
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
		r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, vocab.DiagUndecidedInterference, msg)))
		return nil
	}
	published := publishedInternal
	beyond, fails, err := r.collisionBeyond(a.Body, target, mover, partner, published)
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
func (r *Run) collisionBeyond(a, b, mover, partner *topo.Body, published vocab.Measurement) (Diagnostic, bool, error) {
	pairD, err := pairkernel.PairGateDiameter(r.Ctx, a, b, nil)
	if err != nil {
		return Diagnostic{}, false, err
	}
	pass, ref, haveRef := verifyrun.InterferenceToleranceRef(published, a, b, pairD, r.Cfg.Rel)
	if pass {
		return Diagnostic{}, false, nil
	}
	obs := published
	beyond := Diagnostic{
		Code:     vocab.DiagMeasurementBeyondTolerance,
		Status:   vocab.Suspect,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  vocab.ReadingOverlapVolume,
		Observed: &obs,
		Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", published.Bound),
	}
	if haveRef {
		beyond.Required = tolerance.RequiredThreshold(r.Cfg.Rel*ref, published.Value)
	}
	return beyond, true, nil
}

// publishCollision records pair k of mover i's transferred collision at the
// Pose: the collision, its Interference row, its DiagMotionCollision and, when
// given, its tolerance finding.
func (r *Run) publishCollision(mp *Pose, i, k int, published vocab.Measurement, beyond *Diagnostic) {
	mover, partner := r.Movers[i].Body, r.Partner(i, k)
	mp.Pairs[i][k].Collision = true
	obs := published
	mp.Collisions = append(mp.Collisions, Collision{At: mp.Result.At, Pose: mp.Result.Pose, Moving: mover, Static: partner, Volume: published})
	mp.Result.Interferences = append(mp.Result.Interferences, Interference{A: mover, B: partner, Volume: published})
	msg := fmt.Sprintf("the moving body overlaps a static body at %s", mp.Where)
	if r.Pairs[i][k].Other >= 0 {
		msg = fmt.Sprintf("the moving body overlaps another moving body at %s", mp.Where)
	}
	mp.Findings = append(mp.Findings, mp.stamp(Diagnostic{
		Code:     vocab.DiagMotionCollision,
		Status:   vocab.Interfering,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  vocab.ReadingOverlapVolume,
		Observed: &obs,
		Message:  msg,
	}))
	if beyond != nil {
		finding := *beyond
		observed := *finding.Observed
		finding.Observed = &observed
		mp.Findings = append(mp.Findings, mp.stamp(finding))
	}
}

// poseDiag records an undecided or unsupported pair finding both on the pose
// and in the report's flat list.
func (r *Run) poseDiag(mp *Pose, diag Diagnostic) {
	mp.Result.Diagnostics = append(mp.Result.Diagnostics, diag)
	mp.Findings = append(mp.Findings, diag)
}

// recordGap charges η into the kernel's proven gap interval and publishes the
// pose's Clearance row. The kernel's interval is about the float placements it
// measured; widening it by both bodies' η on both sides — distance is
// 1-Lipschitz in a displacement of either set — makes it a proven interval on
// the IDEAL poses, which is what the interval certificate consumes. A static
// partner's η is zero. An η that cannot be bounded leaves the gap unmeasured.
func (r *Run) recordGap(mp *Pose, i, k int, res pairkernel.Result, etaA, etaB float64) {
	mover, partner := r.Movers[i].Body, r.Partner(i, k)
	if r.Pairs[i][k].Declared {
		r.recordDeclaredGap(mp, mover, partner, res, etaA, etaB)
		return
	}
	if proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		r.poseDiag(mp, mp.stamp(verifyrun.PairDiagNone(mover, partner, vocab.DiagUndecidedClearance,
			"the pair is proven disjoint at this pose, but the pose's departure from the ideal motion is unbounded for this payload, so its gap is unmeasured")))
		return
	}
	lo, hi, exact := pairkernel.DeltaWiden(res.Lo, res.Hi, res.Exact, etaA, etaB)
	mp.Pairs[i][k] = PairPose{HasGap: true, Lo: lo, Hi: hi, Diam: res.Diam}
	gap := pairkernel.GapMeasurement(pairkernel.Result{Lo: lo, Hi: hi, Exact: exact})
	mp.Result.Clearances = append(mp.Result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
	if r.Cfg.MinimumMM != nil && proofarith.FloatRat(hi).Cmp(r.Cfg.MinimumMM) < 0 {
		// The proven upper end of the ideal pose's gap lies below the spec:
		// the margin is disproven here, whatever the reading's precision.
		mp.Violated = true
		violation := gap
		mp.Findings = append(mp.Findings, mp.stamp(Diagnostic{
			Code:     vocab.DiagMotionClearanceViolated,
			Status:   vocab.Violating,
			Pair:     &DiagnosticPair{A: mover, B: partner},
			Reading:  vocab.ReadingGap,
			Observed: &violation,
			Required: r.Cfg.Minimum,
			Message:  fmt.Sprintf("the gap at %s is proven below the required minimum %s", mp.Where, *r.Cfg.Minimum),
		}))
	}
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, r.Cfg.Rel,
		verifyrun.PairToleranceInputs{Diameter: res.Diam}.LengthReference)
	if pass {
		return
	}
	obs := gap
	beyond := Diagnostic{
		Code:     vocab.DiagMeasurementBeyondTolerance,
		Status:   vocab.Suspect,
		Pair:     &DiagnosticPair{A: mover, B: partner},
		Reading:  vocab.ReadingGap,
		Observed: &obs,
		Message:  fmt.Sprintf("the gap reading's bound %s is beyond the relative tolerance", gap.Bound),
	}
	if haveRef {
		beyond.Required = tolerance.RequiredThreshold(r.Cfg.Rel*ref, gap.Value)
	}
	mp.Findings = append(mp.Findings, mp.stamp(beyond))
}

// recordDeclaredGap publishes a declared joint contact's measured gap as a
// Clearance row and nothing else (docs/linkage-check-design.md §5.4): a
// touching pair or an unbounded η publishes nothing, no finding is raised,
// and the gap feeds neither an interval certificate nor the whole-path
// reading.
func (r *Run) recordDeclaredGap(mp *Pose, mover, partner *topo.Body, res pairkernel.Result, etaA, etaB float64) {
	if res.Verdict != boundaryaudit.PairDisjoint || proofbound.IsNonFinite(etaA) || proofbound.IsNonFinite(etaB) {
		return
	}
	lo, hi, exact := pairkernel.DeltaWiden(res.Lo, res.Hi, res.Exact, etaA, etaB)
	gap := pairkernel.GapMeasurement(pairkernel.Result{Lo: lo, Hi: hi, Exact: exact})
	mp.Result.Clearances = append(mp.Result.Clearances, Clearance{A: mover, B: partner, Gap: gap})
}

// intervalVerdict decides one interval between adjacent poses a and b
// (docs/motion-check-design.md §5.2). A proven collision at either end makes
// it vocab.IntervalColliding. Otherwise it is vocab.IntervalClear only when EVERY pair
// certifies: a swept-box-excluded pair by its whole-path lower bound, an
// evaluated pair by the larger of two proven lower bounds on its gap over the
// interval being positive — the travel bound (lo_a + lo_b − τ)/2, the lower
// envelope's minimum, and the driver's projection bound when it supplies one
// (docs/linkage-check-design.md §5.8) — taken over exact rationals, so no
// rounding sits between the proven terms and the strict comparison. That
// larger bound is rounded down to the float the Clearance publishes.
func (r *Run) intervalVerdict(a, b *Pose) Span {
	for _, p := range []*Pose{a, b} {
		if p.Unbuildable != nil {
			return Span{outcome: vocab.IntervalUndecided, note: p.Unbuildable.Error()}
		}
	}
	if !r.collides(a) && !r.collides(b) {
		if gate, ok := r.Drive.(IntervalGate); ok {
			if note := gate.IntervalGate(a, b); note != "" {
				return Span{outcome: vocab.IntervalUndecided, note: note}
			}
		}
	}
	outcome, clearance := r.intervalOutcome(a, b)
	return Span{outcome: outcome, clearance: clearance}
}

func (r *Run) intervalOutcome(a, b *Pose) (vocab.IntervalOutcome, *vocab.Measurement) {
	if r.collides(a) || r.collides(b) {
		return vocab.IntervalColliding, nil
	}
	lowest, ok := r.CertifyPairs(func(i, k int) *big.Rat {
		pa, pb := a.Pairs[i][k], b.Pairs[i][k]
		if !pa.HasGap || !pb.HasGap {
			return nil
		}
		tau := r.Drive.Travel(i, k, a.Param, b.Param)
		if r.Pairs[i][k].Once != nil {
			// A constant pair travels nothing (docs/linkage-check-design.md
			// §5.9).
			tau = new(big.Rat)
		}
		return motionbound.IntervalPairLower(pa.Lo, pb.Lo, tau, r.Drive.Projection(i, k, a, b))
	}, nil)
	if !ok {
		return vocab.IntervalUndecided, nil
	}
	return vocab.IntervalClear, motionbound.LowerBoundMeasurement(lowest)
}

// collides reports whether some pair at the pose carries a proven collision.
func (r *Run) collides(mp *Pose) bool {
	for i := range r.Movers {
		for k := range r.Pairs[i] {
			if mp.Pairs[i][k].Collision {
				return true
			}
		}
	}
	return false
}

// certifyPairs adapts the motion run's pair states to motionbound's shared
// interval and joint-box certificate walk. A declared or unformed pair enters
// no certificate; an invalid or sheet pair cannot certify one.
func (r *Run) CertifyPairs(certify func(i, k int) *big.Rat, held func(i, k int)) (*big.Rat, bool) {
	return motionbound.CertifyPairs(len(r.Movers), func(i int) int { return len(r.Pairs[i]) },
		func(i, k int) motionbound.CertificatePair {
			p := r.Pairs[i][k]
			return motionbound.CertificatePair{
				Skip: p.Declared || p.Unformed, Excluded: p.Excluded,
				Lower: p.Lower, Evaluated: p.Evaluated(),
			}
		}, certify, held)
}

// Conclude assembles the intervals, the whole-path reading, the assessment,
// the diagnostics and the status from the evaluated poses and intervals.
func (r *Run) Conclude(poses []*Pose, spans []Span) motionconclude.MotionConclusion[*topo.Body, vocab.JointCell] {
	against := make([]*topo.Body, 0, len(r.Statics))
	for _, st := range r.Statics {
		against = append(against, st.Body)
	}
	poseFacts := make([]motionconclude.MotionPoseFinding[*topo.Body, vocab.JointCell], len(poses))
	for i, pose := range poses {
		poseFacts[i] = motionconclude.MotionPoseFinding[*topo.Body, vocab.JointCell]{
			At: pose.Result.At, Unbuildable: pose.Unbuildable != nil,
			Violated: pose.Violated, Diagnostics: pose.Findings,
		}
	}
	spanFacts := make([]motionconclude.MotionSpanFinding, len(spans))
	for i, span := range spans {
		spanFacts[i] = motionconclude.MotionSpanFinding{
			Outcome: reportSpanOutcome(span.outcome), Clearance: span.clearance, Note: span.note,
		}
	}
	return motionconclude.ConcludeMotion(r.Cfg.Minimum, against, poseFacts, spanFacts, r.Cfg.MinimumMM,
		func(lowest *measurement.Measurement) (*vocab.ScalarReading, *Diagnostic) {
			reading, diag := r.PathClearance(poses, lowest, "whole-path")
			return reading, diag
		})
}

// Publish assembles VerifyMotion's report (docs/motion-check-design.md §4).
func (r *Run) Publish(poses []*Pose, spans []Span) *Report {
	c := r.Conclude(poses, spans)
	report := &Report{
		Request:     MotionRequest(r.Cfg),
		Against:     c.Against,
		Intervals:   Intervals(c.Intervals),
		Collisions:  []Collision{},
		Clearance:   c.Clearance,
		Assessment:  c.Assessment,
		Diagnostics: c.Diagnostics,
		Status:      c.Status,
	}
	for _, mv := range r.Movers {
		report.Moving = append(report.Moving, mv.Body)
	}
	for _, pose := range poses {
		report.Poses = append(report.Poses, pose.Result)
		report.Collisions = append(report.Collisions, pose.Collisions...)
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
func (r *Run) PathClearance(poses []*Pose, lowest *vocab.Measurement, scope string) (*vocab.ScalarReading, *Diagnostic) {
	upper := math.Inf(1)
	diam := 0.0
	for _, pose := range poses {
		for _, row := range pose.Pairs {
			for _, pp := range row {
				if pp.HasGap && pp.Hi < upper {
					upper, diam = pp.Hi, pp.Diam
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
	gap.Exactness = vocab.Approximate
	pass, ref, haveRef := tolerance.Scalar(gap.Value, gap.Bound, r.Cfg.Rel,
		verifyrun.PairToleranceInputs{Diameter: diam}.LengthReference)
	reading := &vocab.ScalarReading{Measurement: gap, Tolerance: tolerance.Judge(pass, haveRef, r.Cfg.Rel, ref, gap.Value)}
	if pass {
		return reading, nil
	}
	obs := gap
	diag := &Diagnostic{
		Code:     tolerance.DiagnosticCode(reading.Tolerance),
		Status:   vocab.Suspect,
		Reading:  vocab.ReadingGap,
		Observed: &obs,
		Required: reading.Tolerance.Limit,
		Message:  fmt.Sprintf("the %s gap reading's bound %s is beyond the relative tolerance", scope, gap.Bound),
	}
	return reading, diag
}

func MotionRequest(cfg Config) vocab.MotionRequest {
	return vocab.MotionRequest{
		RelativeTolerance: units.Scalar(cfg.Rel),
		Resolution:        cfg.Resolution,
		MinClearance:      cfg.Minimum,
	}
}

func Intervals(intervals []motionconclude.ConcludedMotionSpan) []vocab.MotionInterval {
	if intervals == nil {
		return nil
	}
	result := make([]vocab.MotionInterval, len(intervals))
	for i, interval := range intervals {
		result[i] = vocab.MotionInterval{
			From:      interval.From,
			To:        interval.To,
			Outcome:   rootIntervalOutcome(interval.Outcome),
			Clearance: interval.Clearance,
		}
	}
	return result
}

func rootIntervalOutcome(outcome motionconclude.MotionSpanOutcome) vocab.IntervalOutcome {
	switch outcome {
	case motionconclude.SpanNotEvaluated:
		return vocab.IntervalNotEvaluated
	case motionconclude.SpanClear:
		return vocab.IntervalClear
	case motionconclude.SpanColliding:
		return vocab.IntervalColliding
	case motionconclude.SpanUndecided:
		return vocab.IntervalUndecided
	default:
		return vocab.IntervalOutcome(outcome)
	}
}

func reportSpanOutcome(outcome vocab.IntervalOutcome) motionconclude.MotionSpanOutcome {
	switch outcome {
	case vocab.IntervalNotEvaluated:
		return motionconclude.SpanNotEvaluated
	case vocab.IntervalClear:
		return motionconclude.SpanClear
	case vocab.IntervalColliding:
		return motionconclude.SpanColliding
	case vocab.IntervalUndecided:
		return motionconclude.SpanUndecided
	default:
		return motionconclude.MotionSpanOutcome(outcome)
	}
}

const poseBodyCacheSize = 32

// PoseBodyCache bounds the transient bodies retained for one joint-box mover.
// Each entry is one completed payload placement at one bit-identical float
// transform. A hit reuses its body and any completed geometry or mesh proofs;
// the caller still computes the cell's own ideal-pose allowance.
type PoseBodyCache struct {
	bodies map[[12]uint64]*topo.Body
	order  [poseBodyCacheSize][12]uint64
	next   int
}

func (c *PoseBodyCache) Get(key [12]uint64) *topo.Body {
	return c.bodies[key]
}

// put returns the body evicted from the cache, if any, so its clearance
// carriers can be removed from the call's pairkernel.Cache too.
func (c *PoseBodyCache) Put(key [12]uint64, body *topo.Body) *topo.Body {
	if c.bodies == nil {
		c.bodies = make(map[[12]uint64]*topo.Body, poseBodyCacheSize)
	}
	var evicted *topo.Body
	if len(c.bodies) == poseBodyCacheSize {
		old := c.order[c.next]
		evicted = c.bodies[old]
		delete(c.bodies, old)
	}
	c.order[c.next] = key
	c.bodies[key] = body
	c.next = (c.next + 1) % poseBodyCacheSize
	return evicted
}

// BodyPoints is a body's static point reading: its prism hull points when hull
// is set and the body has them, and otherwise the eight corners
// of its Bounds box inflated by its own Bound.
func BodyPoints(b *topo.Body, hull bool) (cornerReading, bool) {
	if hull {
		if pp, isPrism := b.Payload.(prismrecord.Payload); isPrism {
			points, pad, k, ok := linkagebound.PrismHullPoints(linkagebound.PrismHull{
				Outer: pp.Profile.Outer, Frame: pp.Frame, Transform: pp.Xform,
				Z0: pp.Z0, Z1: pp.Z1, SectionDelta: pp.SectionDelta,
				Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
			})
			if ok {
				reading := linkagebound.StaticPoints(points)
				reading.Pad, reading.PrismK = pad, k
				return reading, true
			}
		}
	}
	lo, hi, ok := motionbound.BoxCornersExact(b.Bounds, new(big.Rat))
	if !ok {
		return cornerReading{}, false
	}
	return linkagebound.StaticBox(lo, hi), true
}
