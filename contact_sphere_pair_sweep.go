package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type sourceSpherePairSweepRun struct {
	doc     *Document
	a, b    *Body
	pa, pb  affinePairPath
	req     SweepRequest
	report  *SweepReport
	sphereA sourceSphereContactProof
	sphereB sourceSphereContactProof
}

func (r *sourceSpherePairSweepRun) idealAt(f *big.Rat, at SweepInstant) SweepEvent {
	a, okA := translatedSphere(r.sphereA, r.pa.delta, f)
	b, okB := translatedSphere(r.sphereB, r.pb.delta, f)
	if !okA || !okB {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	contact := &ContactReport{Request: r.req.ContactRequest}
	classifySourceSpherePair(contact, a, b)
	return SweepEvent{At: at, Relation: contact.Relation, Gap: contact.Gap,
		Manifold: contact.Manifold, Reason: contact.Reason}
}

func (r *sourceSpherePairSweepRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.pa.poseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.pb.poseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.duration)
	ideal := r.idealAt(f, at)
	r.transferManifold(f, poseA, poseB, contact, &ideal)
	sample := SweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact, Ideal: ideal}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

func (r *sourceSpherePairSweepRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, ideal *SweepEvent) {
	if ideal.Manifold == nil {
		return
	}
	if contact.Manifold == nil || contact.Relation == ContactSeparated ||
		len(contact.Manifold.Points) != 1 || len(ideal.Manifold.Points) != 1 {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	want, actual := ideal.Manifold.Points[0], contact.Manifold.Points[0]
	if want.FeatureA != actual.FeatureA || want.FeatureB != actual.FeatureB ||
		want.Normal.Value != actual.Normal.Value {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	observedA, okA := sourceSphereAtPose(r.a, poseA)
	observedB, okB := sourceSphereAtPose(r.b, poseB)
	if !okA || !okB {
		ideal.Manifold, ideal.Reason = nil, ContactPayloadUnsupported
		return
	}
	deviation := new(big.Rat)
	for _, moving := range []struct {
		start, observed sourceSphereContactProof
		delta           [3]dyadic
	}{{r.sphereA, observedA, r.pa.delta}, {r.sphereB, observedB, r.pb.delta}} {
		for i := range 3 {
			center := new(big.Rat).Add(moving.start.center[i].rat(),
				new(big.Rat).Mul(moving.delta[i].rat(), f))
			diff := new(big.Rat).Sub(moving.observed.center[i].rat(), center)
			deviation.Add(deviation, diff.Abs(diff))
		}
	}
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
		return
	}
	for _, witness := range []*VecMeasurement{&actual.OnA, &actual.OnB} {
		bound := new(big.Rat).Add(floatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
			return
		}
		witness.Bound = units.Millimeters(ratFloatUp(bound))
		witness.Exactness = exactnessFromBound(witness.Bound.Base())
	}
	sepBound := new(big.Rat).Add(floatRat(actual.Separation.Bound.Base()), deviation)
	actual.Separation.Bound = units.Millimeters(ratFloatUp(sepBound))
	actual.Separation.Exactness = exactnessFromBound(actual.Separation.Bound.Base())
	ideal.Manifold = &ContactManifold{Points: []ContactPoint{actual}}
	ideal.Reason = ContactNoReason
}

// At a final-instant impact the right sample is the exact path endpoint.
// The left dyadic endpoint must still lie strictly before the support root.
func spherePairImpactBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	left, right, ok := sphereImpactBracket(root, duration, resolution)
	if ok {
		return left, right, true
	}
	one := big.NewRat(1, 1)
	if root.Sign() <= 0 || root.Cmp(one) > 0 {
		return nil, nil, false
	}
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if width.Cmp(resolution) <= 0 {
			left = new(big.Rat).Sub(one, new(big.Rat).SetFrac(big.NewInt(1), grid))
			if left.Sign() > 0 && left.Cmp(root) < 0 &&
				floatRat(ratFloatNearest(left)).Cmp(left) == 0 {
				return left, one, true
			}
			return nil, nil, false
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

func (r *sourceSpherePairSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.duration),
		To: sweepInstant(to, r.pa.duration)}
	r.sortSamples()
	return r.report
}

func (r *sourceSpherePairSweepRun) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

// axialGap proves that both center paths share one unchanged transverse
// coordinate pair. The signed support gap is then affine until first touch.
func (r *sourceSpherePairSweepRun) axialGap() (dyadic, dyadic, bool) {
	axis, sign, nonzero := 0, 0, 0
	for i := range 3 {
		start := dySubScalar(r.sphereB.center[i], r.sphereA.center[i])
		travel := dySubScalar(r.pb.delta[i], r.pa.delta[i])
		if start.sign() != 0 || travel.sign() != 0 {
			axis, sign, nonzero = i, start.sign(), nonzero+1
		}
	}
	if nonzero != 1 || sign == 0 {
		return dyadic{}, dyadic{}, false
	}
	initial := dyAbs(dySubScalar(r.sphereB.center[axis], r.sphereA.center[axis]))
	gap := dySubScalar(initial, dyAdd(r.sphereA.radius, r.sphereB.radius))
	slope := dySubScalar(r.pb.delta[axis], r.pa.delta[axis])
	if sign < 0 {
		slope = dyNeg(slope)
	}
	return gap, slope, true
}

func (r *sourceSpherePairSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, zero, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if first.Ideal.Relation == ContactOverlapping {
		r.report.InitialEvent, r.report.Event = &first.Ideal, &first.Ideal
		r.report.Outcome = SweepInitiallyOverlapping
		return r.report, nil
	}
	if first.Ideal.Relation != ContactSeparated && first.Ideal.Relation != ContactTouching {
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
	if first.Ideal.Relation == ContactTouching {
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
	}
	gap, slope, ok := r.axialGap()
	if !ok {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	if first.Ideal.Relation == ContactTouching {
		if slope.sign() > 0 {
			last, err := r.sample(ctx, one)
			if errors.Is(err, errSweepPoseBudget) {
				return r.undecided(zero, one, SweepPoseBudget), nil
			}
			if err != nil {
				return nil, err
			}
			if last.Ideal.Relation != ContactSeparated || last.Ideal.Gap == nil {
				return r.undecided(zero, one, SweepPoseRelation), nil
			}
			r.report.Outcome = SweepDepartedClear
			r.report.Departure = &SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
			return r.report, nil
		}
		return r.undecided(zero, one, SweepContactTrackUnproved), nil
	}
	if gap.sign() <= 0 {
		return r.undecided(zero, one, SweepPoseRelation), nil
	}
	if slope.sign() >= 0 || dyAdd(gap, slope).sign() > 0 {
		last, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != ContactSeparated {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepClear
		return r.report, nil
	}
	root := new(big.Rat).Quo(dyNeg(gap).rat(), slope.rat())
	leftF, rightF, ok := spherePairImpactBracket(root, r.pa.duration, resolution)
	if !ok || leftF.Sign() <= 0 || rightF.Cmp(one) > 0 {
		return r.undecided(zero, one, SweepTimeFloor), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, leftF, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(leftF, rightF, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != ContactSeparated ||
		(right.Ideal.Relation != ContactTouching && right.Ideal.Relation != ContactOverlapping) ||
		right.Ideal.Manifold == nil {
		return r.undecided(leftF, rightF, SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = SweepImpactBracket, &right.Ideal
	r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
	r.sortSamples()
	return r.report, nil
}
