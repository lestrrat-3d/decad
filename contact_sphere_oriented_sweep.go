package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/spherepath"
	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type orientedSphereSweepRun struct {
	doc          *Document
	a, b         *Body
	pa, pb       affinePairPath
	req          SweepRequest
	report       *SweepReport
	sphere       sourceSphereContactProof
	box          orientedSourceBox
	sphereFirst  bool
	axis, side   int
	outward      proofarith.DyV3
	start, slope proofarith.Dyadic // unnormalized signed center-to-face support
}

func (r *orientedSphereSweepRun) deltas() ([3]proofarith.Dyadic, [3]proofarith.Dyadic) {
	if r.sphereFirst {
		return r.pa.Delta, r.pb.Delta
	}
	return r.pb.Delta, r.pa.Delta
}

func (r *orientedSphereSweepRun) idealAt(f *big.Rat, at SweepInstant) SweepEvent {
	sphereDelta, boxDelta := r.deltas()
	sphere, okSphere := translatedSphere(r.sphere, sphereDelta, f)
	box, okBox := translatedOrientedBox(r.box, boxDelta, f)
	if !okSphere || !okBox {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	contact := &ContactReport{Request: r.req.ContactRequest}
	classifySourceSphereOrientedBox(contact, sphere, box, r.sphereFirst)
	return SweepEvent{At: at, Relation: contact.Relation, Gap: contact.Gap,
		Manifold: contact.Manifold, Reason: contact.Reason}
}

func (r *orientedSphereSweepRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.pa.PoseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.pb.PoseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.Duration)
	ideal := r.idealAt(f, at)
	if ideal.Manifold != nil {
		if contact.Manifold == nil || ideal.Relation != contact.Relation ||
			len(contact.Manifold.Points) != 1 ||
			ideal.Manifold.Points[0].FeatureA != contact.Manifold.Points[0].FeatureA ||
			ideal.Manifold.Points[0].FeatureB != contact.Manifold.Points[0].FeatureB ||
			ideal.Manifold.Points[0].Normal.Value != contact.Manifold.Points[0].Normal.Value {
			ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		} else {
			deviation := r.poseDeviation(f, poseA, poseB)
			point := contact.Manifold.Points[0]
			resolution, _ := sweeppath.ExactBaseValue(r.req.PointResolution)
			for _, witness := range []*VecMeasurement{&point.OnA, &point.OnB} {
				bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
				if bound.Cmp(resolution) > 0 {
					ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
					break
				}
				witness.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
				witness.Exactness = exactnessFromBound(witness.Bound.Base())
			}
			if ideal.Manifold != nil {
				bound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
				point.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
				point.Separation.Exactness = exactnessFromBound(point.Separation.Bound.Base())
				ideal.Manifold = &ContactManifold{Points: []ContactPoint{point}}
			}
		}
	}
	sample := &SweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact,
		Ideal: ideal, exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

func (r *orientedSphereSweepRun) poseDeviation(f *big.Rat, poseA, poseB r3.Transform) *big.Rat {
	spherePose, boxPose := poseB, poseA
	sphereFrom, boxFrom := r.pb.From, r.pa.From
	sphereDelta, boxDelta := r.pb.Delta, r.pa.Delta
	if r.sphereFirst {
		spherePose, boxPose = poseA, poseB
		sphereFrom, boxFrom = r.pa.From, r.pb.From
		sphereDelta, boxDelta = r.pa.Delta, r.pb.Delta
	}
	observedSphere, okSphere := translatedReplaySphere(r.sphere, sphereFrom, spherePose)
	observedBox, okBox := spherepath.TranslateObservedBox(r.box.OrientedBox, boxFrom, boxPose)
	if !okSphere || !okBox {
		return new(big.Rat).SetInt64(1 << 30)
	}
	return spherepath.OrientedSpherePoseDeviation(r.box.OrientedBox, observedBox,
		boxDelta, r.sphere.center, observedSphere.center, sphereDelta, f)
}

func (r *orientedSphereSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.Duration),
		To: sweepInstant(to, r.pa.Duration)}
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
	return r.report
}

func (r *orientedSphereSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if !pairbox.OrthogonalSourceBox(r.box.OrientedBox) {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	axis, side, outward, _, _, ok := pairbox.OrientedSphereFace(
		r.sphere.center, r.sphere.radius, r.box.OrientedBox)
	if !ok {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	r.axis, r.side, r.outward = axis, side, outward
	first, err := r.sample(ctx, zero)
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
	sphereDelta, boxDelta := r.deltas()
	r.start, r.slope, ok = spherepath.OrientedFaceCorridor(
		r.sphere.center, r.sphere.radius, r.box.OrientedBox, sphereDelta, boxDelta,
		r.axis, r.side, r.outward)
	if !ok {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	r.report.replay = &sweepReplayProof{pa: r.pa, pb: r.pb, request: r.req.ContactRequest,
		orientedSphere: &r.sphere, orientedSphereBox: &r.box, sphereFirst: r.sphereFirst,
		sphereAxis: r.axis, sphereSide: r.side, sphereGap: r.start, sphereSlope: r.slope}
	if first.Ideal.Relation == ContactTouching {
		if r.slope.Sign() <= 0 {
			return r.undecided(zero, one, SweepDepartureUnproved), nil
		}
		last, sampleErr := r.sample(ctx, one)
		if errors.Is(sampleErr, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if sampleErr != nil {
			return nil, sampleErr
		}
		if last.Ideal.Relation != ContactSeparated || last.Ideal.Gap == nil {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepDepartedClear
		r.report.Departure = &SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
		r.report.replay.snapshot(r.report)
		return r.report, nil
	}
	if spherepath.OrientedFaceSign(r.start, r.slope, r.sphere.radius, r.outward, one) > 0 {
		last, sampleErr := r.sample(ctx, one)
		if errors.Is(sampleErr, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if sampleErr != nil {
			return nil, sampleErr
		}
		if last.Ideal.Relation != ContactSeparated {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepClear
		r.report.replay.snapshot(r.report)
		return r.report, nil
	}
	leftF, rightF, ok := spherepath.OrientedFaceBracket(
		r.start, r.slope, r.sphere.radius, r.outward, r.pa.Duration, resolution)
	if !ok {
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
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.report.replay.snapshot(r.report)
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
	return r.report, nil
}
