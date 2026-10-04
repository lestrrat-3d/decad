package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type sourceSphereSweepRun struct {
	doc         *Document
	a, b        *Body
	pa, pb      affinePairPath
	req         SweepRequest
	report      *SweepReport
	sphere      sourceSphereContactProof
	box         sourceBoxContactProof
	sphereFirst bool
}

func translatedContactBox(box sourceBoxContactProof, delta [3]dyadic,
	f *big.Rat) (sourceBoxContactProof, bool) {
	fraction, ok := dyOfRat(f)
	if !ok {
		return sourceBoxContactProof{}, false
	}
	for i := range 3 {
		move := dyMul(delta[i], fraction)
		box.lo[i], box.hi[i] = dyAdd(box.lo[i], move), dyAdd(box.hi[i], move)
	}
	return box, true
}

func (r *sourceSphereSweepRun) deltas() ([3]dyadic, [3]dyadic) {
	if r.sphereFirst {
		return r.pa.delta, r.pb.delta
	}
	return r.pb.delta, r.pa.delta
}

func (r *sourceSphereSweepRun) idealAt(f *big.Rat, at SweepInstant) SweepEvent {
	sphereDelta, boxDelta := r.deltas()
	sphere, okSphere := translatedSphere(r.sphere, sphereDelta, f)
	box, okBox := translatedContactBox(r.box, boxDelta, f)
	if !okSphere || !okBox {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	contact := &ContactReport{Request: r.req.ContactRequest}
	classifySourceSphereBox(contact, sphere, box, r.sphereFirst)
	return SweepEvent{At: at, Relation: contact.Relation, Gap: contact.Gap,
		Manifold: contact.Manifold, Reason: contact.Reason}
}

func (r *sourceSphereSweepRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
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
	observedSphere, okSphere := sourceSphereAtPose(r.sphereBody(), spherePose)
	observedBox, okBox := sourceBoxAtPose(r.boxBody(), boxPose)
	if !okSphere || !okBox {
		ideal.Manifold = nil
		ideal.Reason = ContactPayloadUnsupported
		return
	}
	sphereDelta, boxDelta := r.deltas()
	deviation := boxPoseDeviation(r.box, observedBox, boxDelta, f)
	for i := range 3 {
		move := new(big.Rat).Mul(sphereDelta[i].rat(), f)
		center := new(big.Rat).Add(r.sphere.center[i].rat(), move)
		diff := new(big.Rat).Sub(observedSphere.center[i].rat(), center)
		deviation.Add(deviation, diff.Abs(diff))
	}
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold = nil
		ideal.Reason = ContactPointTooCoarse
		return
	}
	for _, witness := range []*VecMeasurement{&actual.OnA, &actual.OnB} {
		bound := new(big.Rat).Add(floatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold = nil
			ideal.Reason = ContactPointTooCoarse
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
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.duration),
		To: sweepInstant(to, r.pa.duration)}
	r.sortSamples()
	return r.report
}

func (r *sourceSphereSweepRun) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

// faceCorridor proves that the complete sphere projection remains strictly
// inside the same two box face intervals throughout the affine path.
func (r *sourceSphereSweepRun) faceCorridor(axis int) bool {
	sphereDelta, boxDelta := r.deltas()
	for i := range 3 {
		if i == axis {
			continue
		}
		for _, f := range []*big.Rat{new(big.Rat), big.NewRat(1, 1)} {
			sphere, okSphere := translatedSphere(r.sphere, sphereDelta, f)
			box, okBox := translatedContactBox(r.box, boxDelta, f)
			if !okSphere || !okBox ||
				dyCmp(dySubScalar(sphere.center[i], sphere.radius), box.lo[i]) <= 0 ||
				dyCmp(dyAdd(sphere.center[i], sphere.radius), box.hi[i]) >= 0 {
				return false
			}
		}
	}
	return true
}

func (r *sourceSphereSweepRun) contactAxis() (int, int, dyadic, dyadic, bool) {
	sphereDelta, boxDelta := r.deltas()
	for i := range 3 {
		if dyCmp(r.sphere.center[i], r.box.hi[i]) > 0 {
			gap := dySubScalar(dySubScalar(r.sphere.center[i], r.box.hi[i]), r.sphere.radius)
			if gap.sign() < 0 {
				return 0, 0, dyadic{}, dyadic{}, false
			}
			slope := dySubScalar(sphereDelta[i], boxDelta[i])
			return i, 1, gap, slope, r.faceCorridor(i)
		}
		if dyCmp(r.sphere.center[i], r.box.lo[i]) < 0 {
			gap := dySubScalar(dySubScalar(r.box.lo[i], r.sphere.center[i]), r.sphere.radius)
			if gap.sign() < 0 {
				return 0, 0, dyadic{}, dyadic{}, false
			}
			slope := dySubScalar(boxDelta[i], sphereDelta[i])
			return i, -1, gap, slope, r.faceCorridor(i)
		}
	}
	return 0, 0, dyadic{}, dyadic{}, false
}

func (r *sourceSphereSweepRun) track(first *SweepSample) *SweepContactTrack {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 1 {
		return nil
	}
	// The exact point's coordinates move affinely. One outward ULP at the
	// largest endpoint coordinate encloses every intermediate conversion.
	maximum := new(big.Rat)
	sphereDelta, boxDelta := r.deltas()
	for i := range 3 {
		for _, v := range []dyadic{r.sphere.center[i],
			dyAdd(r.sphere.center[i], sphereDelta[i]),
			dyAdd(r.sphere.center[i], r.sphere.radius),
			dySubScalar(r.sphere.center[i], r.sphere.radius),
			dyAdd(dyAdd(r.sphere.center[i], sphereDelta[i]), r.sphere.radius),
			dySubScalar(dyAdd(r.sphere.center[i], sphereDelta[i]), r.sphere.radius),
			r.box.lo[i], r.box.hi[i],
			dyAdd(r.box.lo[i], boxDelta[i]), dyAdd(r.box.hi[i], boxDelta[i])} {
			abs := new(big.Rat).Abs(v.rat())
			if abs.Cmp(maximum) > 0 {
				maximum = abs
			}
		}
	}
	maxFloat := ratFloatUp(maximum)
	if !finiteMeasurementValues(maxFloat) ||
		radius3D(math.Nextafter(maxFloat, math.Inf(1))-maxFloat) > r.req.PointResolution.Base() {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	track := &SweepContactTrack{
		start: new(big.Rat), end: big.NewRat(1, 1), duration: new(big.Rat).Set(r.pa.duration),
		request: r.req.ContactRequest, sphere: &r.sphere, sphereFirst: r.sphereFirst,
		features: [2]ContactFeature{point.FeatureA, point.FeatureB}, normal: point.Normal,
		deltaA: r.pa.delta, deltaB: r.pb.delta,
	}
	if r.sphereFirst {
		track.b = r.box
	} else {
		track.a = r.box
	}
	return track
}

// sphereImpactBracket leaves a representable positive-overlap margin at the
// right endpoint. A root arbitrarily close to a dyadic slice can otherwise
// put the ideal contact just inside the sphere while the float pose is just
// outside it, leaving no correctable point for the response solver.
func sphereImpactBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	grid := big.NewInt(1)
	four := big.NewRat(4, 1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if new(big.Rat).Mul(width, four).Cmp(resolution) <= 0 {
			scaled := new(big.Rat).Mul(root, new(big.Rat).SetInt(grid))
			floor := new(big.Int).Quo(scaled.Num(), scaled.Denom())
			leftIdx := new(big.Int).Sub(new(big.Int).Set(floor), big.NewInt(1))
			rightIdx := new(big.Int).Add(new(big.Int).Set(floor), big.NewInt(2))
			left := new(big.Rat).SetFrac(leftIdx, grid)
			right := new(big.Rat).SetFrac(rightIdx, grid)
			if left.Sign() <= 0 || right.Cmp(big.NewRat(1, 1)) > 0 ||
				floatRat(ratFloatNearest(left)).Cmp(left) != 0 ||
				floatRat(ratFloatNearest(right)).Cmp(right) != 0 {
				return nil, nil, false
			}
			return left, right, true
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
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
	_, _, gap, slope, ok := r.contactAxis()
	if !ok {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	if first.Ideal.Relation == ContactTouching {
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
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
		if slope.isZero() && r.req.StartPolicy == ContinueCertifiedTouch {
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
					return r.report, nil
				}
			}
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
	leftF, rightF, ok := sphereImpactBracket(root, r.pa.duration, resolution)
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
