package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactround"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spherepath"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type sourceSphereSweepRun struct {
	doc         *Document
	a, b        *Body
	pa, pb      affinePairPath
	req         SweepRequest
	report      *SweepReport
	sphere      contactround.SphereProof
	box         contactbox.SourceBox
	sphereFirst bool
}

// A rotating ball has the same occupied set as its translating center when
// its drift pivot is the proved sphere center. Keep the real rotating path for
// sampled poses and replay, while the support gap uses its exact center drift.
func (d *Document) sweepRotatingSphereBox(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	if (pa.Drift == nil) == (pb.Drift == nil) {
		return nil, false, nil
	}
	sphereBody, spherePath, boxBody, boxPath, sphereFirst := b, &pb, a, pa, false
	if pa.Drift != nil {
		sphereBody, spherePath, boxBody, boxPath, sphereFirst = a, &pa, b, pb, true
	}
	if boxPath.Drift != nil || boxPath.Screw != nil {
		return nil, false, nil
	}
	sphere, sphereOK := contactround.SphereAtPose(bodyNode(sphereBody), spherePath.From)
	box, boxOK := contactbox.SourceBoxAtPose(bodyNode(boxBody), boxPath.From)
	if !sphereOK || !boxOK {
		return nil, false, nil
	}
	// sourceSpherePathPoseAt sets the query translation to the center drift.
	// That represents the requested rigid pose only when the source center is
	// exactly at the query origin before rotation.
	from := spherePath.From.Translation()
	for axis, value := range [3]float64{from.X, from.Y, from.Z} {
		if proofarith.DyCmp(sphere.Center[axis], proofarith.MustDyOf(value)) != 0 {
			report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
			report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
				To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
			return report, true, nil
		}
	}
	center := spherePath.Drift.Center
	for axis, value := range [3]float64{center.X, center.Y, center.Z} {
		if proofarith.DyCmp(sphere.Center[axis], proofarith.MustDyOf(value)) != 0 {
			return nil, false, nil
		}
	}
	velocity := [3]units.Value{spherePath.Drift.LinearVelocity.X,
		spherePath.Drift.LinearVelocity.Y, spherePath.Drift.LinearVelocity.Z}
	for axis, value := range velocity {
		speed, ok := sweeppath.ExactBaseValue(value)
		if !ok {
			return nil, false, nil
		}
		displacement := new(big.Rat).Mul(speed, spherePath.Duration)
		component, ok := proofarith.DyOfRat(displacement)
		if !ok {
			return nil, false, nil
		}
		spherePath.Delta[axis] = component
	}
	run := &sourceSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
		req: req, report: report, sphere: sphere, box: box, sphereFirst: sphereFirst}
	result, err := run.execute(ctx, resolution)
	return result, true, err
}

func translatedContactBox(box contactbox.SourceBox, delta [3]proofarith.Dyadic,
	f *big.Rat) (contactbox.SourceBox, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return contactbox.SourceBox{}, false
	}
	for i := range 3 {
		move := proofarith.DyMul(delta[i], fraction)
		box.Lo[i], box.Hi[i] = proofarith.DyAdd(box.Lo[i], move), proofarith.DyAdd(box.Hi[i], move)
	}
	return box, true
}

func (r *sourceSphereSweepRun) deltas() ([3]proofarith.Dyadic, [3]proofarith.Dyadic) {
	if r.sphereFirst {
		return r.pa.Delta, r.pb.Delta
	}
	return r.pb.Delta, r.pa.Delta
}

func (r *sourceSphereSweepRun) idealAt(f *big.Rat, at SweepInstant) SweepEvent {
	sphereDelta, boxDelta := r.deltas()
	sphere, okSphere := contactround.TranslatedSphere(r.sphere, sphereDelta, f)
	box, okBox := translatedContactBox(r.box, boxDelta, f)
	if !okSphere || !okBox {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	contact := &nodeContactReport{Request: r.req.ContactRequest}
	contactround.ClassifySphereBox(contact, sphere, box, r.sphereFirst)
	return sweepEventOf(at, contact)
}

func (r *sourceSphereSweepRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.pa.SpherePoseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.pb.SpherePoseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.Duration)
	ideal := r.idealAt(f, at)
	r.transferManifold(f, poseA, poseB, contact, &ideal)
	sample := SweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact, Ideal: ideal,
		exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

// transferManifold uses the real posed contact witness. The source ball and
// box proofs bound its difference from the exact affine path; a relation or
// source-feature change withholds the manifold instead of guessing a match.
func (r *sourceSphereSweepRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, ideal *SweepEvent) {
	if ideal.Manifold == nil {
		return
	}
	if contact.Manifold == nil || ideal.Relation != contact.Relation ||
		len(ideal.Manifold.Points) != 1 ||
		len(contact.Manifold.Points) != 1 {
		ideal.Manifold = nil
		ideal.Reason = ContactNoNormalProof
		return
	}
	want, actual := ideal.Manifold.Points[0], contact.Manifold.Points[0]
	if want.FeatureA != actual.FeatureA || want.FeatureB != actual.FeatureB ||
		want.Normal.Value != actual.Normal.Value {
		ideal.Manifold = nil
		ideal.Reason = ContactNoNormalProof
		return
	}
	spherePose, boxPose := poseB, poseA
	if r.sphereFirst {
		spherePose, boxPose = poseA, poseB
	}
	observedSphere, okSphere := contactround.SphereAtPose(bodyNode(r.sphereBody()), spherePose)
	observedBox, okBox := contactbox.SourceBoxAtPose(bodyNode(r.boxBody()), boxPose)
	if !okSphere || !okBox {
		ideal.Manifold = nil
		ideal.Reason = ContactPayloadUnsupported
		return
	}
	sphereDelta, boxDelta := r.deltas()
	deviation := spherepath.AxisSpherePoseDeviation(
		pairbox.AxisSphere{Center: r.sphere.Center, Radius: r.sphere.Radius}, observedSphere.Center,
		r.box.AxisBox(), observedBox.AxisBox(), sphereDelta, boxDelta, f)
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold = nil
		ideal.Reason = ContactPointTooCoarse
		return
	}
	for _, witness := range []*VecMeasurement{&actual.OnA, &actual.OnB} {
		bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold = nil
			ideal.Reason = ContactPointTooCoarse
			return
		}
		witness.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
		witness.Exactness = exactnessFromBound(witness.Bound.Base())
	}
	sepBound := new(big.Rat).Add(proofarith.FloatRat(actual.Separation.Bound.Base()), deviation)
	actual.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(sepBound))
	actual.Separation.Exactness = exactnessFromBound(actual.Separation.Bound.Base())
	ideal.Manifold = &ContactManifold{Points: []ContactPoint{actual}}
	ideal.Reason = ContactNoReason
}

func (r *sourceSphereSweepRun) sphereBody() *Body {
	if r.sphereFirst {
		return r.a
	}
	return r.b
}

func (r *sourceSphereSweepRun) boxBody() *Body {
	if r.sphereFirst {
		return r.b
	}
	return r.a
}

func (r *sourceSphereSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.Duration),
		To: sweepInstant(to, r.pa.Duration)}
	r.sortSamples()
	return r.report
}

func (r *sourceSphereSweepRun) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

func (r *sourceSphereSweepRun) track(first *SweepSample) *SweepContactTrack {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 1 {
		return nil
	}
	sphereDelta, boxDelta := r.deltas()
	if !spherepath.AxisTrackPointsWithin(
		pairbox.AxisSphere{Center: r.sphere.Center, Radius: r.sphere.Radius}, r.box.AxisBox(),
		sphereDelta, boxDelta, r.req.PointResolution.Base()) {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	track := &SweepContactTrack{
		start: new(big.Rat), end: big.NewRat(1, 1), duration: new(big.Rat).Set(r.pa.Duration),
		request: r.req.ContactRequest, sphere: &r.sphere, sphereFirst: r.sphereFirst,
		features: [2]nodeContactFeature{contactFeatureToInternal(point.FeatureA), contactFeatureToInternal(point.FeatureB)},
		normal:   point.Normal,
		deltaA:   r.pa.Delta, deltaB: r.pb.Delta,
	}
	if r.sphereFirst {
		track.b = r.box
	} else {
		track.a = r.box
	}
	return track
}

func (r *sourceSphereSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
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
	sphereDelta, boxDelta := r.deltas()
	axis, side, gap, slope, ok := spherepath.AxisFaceCorridor(
		pairbox.AxisSphere{Center: r.sphere.Center, Radius: r.sphere.Radius}, r.box.AxisBox(),
		sphereDelta, boxDelta)
	if !ok {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	r.report.replay = &sweepReplayProof{pa: r.pa, pb: r.pb, request: r.req.ContactRequest,
		sphere: &r.sphere, sphereFirst: r.sphereFirst, sphereAxis: axis, sphereSide: side,
		sphereGap: gap, sphereSlope: slope}
	if r.sphereFirst {
		r.report.replay.boxB = r.box
	} else {
		r.report.replay.boxA = r.box
	}
	if first.Ideal.Relation == ContactTouching {
		if slope.Sign() > 0 {
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
			r.report.replay.snapshot(r.report)
			return r.report, nil
		}
		if slope.IsZero() && r.req.StartPolicy == ContinueCertifiedTouch {
			track := r.track(first)
			if track != nil {
				last, err := r.sample(ctx, one)
				if errors.Is(err, errSweepPoseBudget) {
					return r.undecided(zero, one, SweepPoseBudget), nil
				}
				if err != nil {
					return nil, err
				}
				if last.Ideal.Relation == ContactTouching && last.Ideal.Manifold != nil {
					r.report.Outcome, r.report.ContactTrack = SweepPersistentTouch, track
					r.report.replay.snapshot(r.report)
					return r.report, nil
				}
			}
		}
		return r.undecided(zero, one, SweepContactTrackUnproved), nil
	}
	if gap.Sign() <= 0 {
		return r.undecided(zero, one, SweepPoseRelation), nil
	}
	if slope.Sign() >= 0 || proofarith.DyAdd(gap, slope).Sign() > 0 {
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
		r.report.replay.snapshot(r.report)
		return r.report, nil
	}
	root := new(big.Rat).Quo(proofarith.DyNeg(gap).Rat(), slope.Rat())
	leftF, rightF, ok := spherepath.ImpactBracket(root, r.pa.Duration, resolution)
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
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.report.replay.snapshot(r.report)
	r.sortSamples()
	return r.report, nil
}

type orientedSphereSweepRun struct {
	doc          *Document
	a, b         *Body
	pa, pb       affinePairPath
	req          SweepRequest
	report       *SweepReport
	sphere       contactround.SphereProof
	box          contactbox.OrientedSourceBox
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
	sphere, okSphere := contactround.TranslatedSphere(r.sphere, sphereDelta, f)
	box := r.box // the translated box keeps its source face identities
	geometry, okBox := pairbox.TranslateOriented(box.OrientedBox, boxDelta, f)
	box.OrientedBox = geometry
	if !okSphere || !okBox {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	contact := &nodeContactReport{Request: r.req.ContactRequest}
	contactround.ClassifySphereOrientedBox(contact, sphere, box, r.sphereFirst)
	return sweepEventOf(at, contact)
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
		boxDelta, r.sphere.Center, observedSphere.Center, sphereDelta, f)
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
		r.sphere.Center, r.sphere.Radius, r.box.OrientedBox)
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
		r.sphere.Center, r.sphere.Radius, r.box.OrientedBox, sphereDelta, boxDelta,
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
	if spherepath.OrientedFaceSign(r.start, r.slope, r.sphere.Radius, r.outward, one) > 0 {
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
		r.start, r.slope, r.sphere.Radius, r.outward, r.pa.Duration, resolution)
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
