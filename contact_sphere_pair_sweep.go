package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/spherepath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	// rotating marks a centered spinning drift. Its real start pose must
	// report the ideal relation and a clear path's real end pose separation;
	// ContinueSeparatingTouch needs a touching start.
	rotating bool
}

// A source ball centered at its query origin is unchanged by its own spin.
// A center-pivot drift therefore has the same exact occupied-set path as an
// affine translation, while sampled poses still carry the requested rotation.
func (d *Document) sweepRotatingSpherePair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	if pa.Drift == nil && pb.Drift == nil ||
		(req.StartPolicy != ContinueSeparatingTouch && req.StartPolicy != StopAtInitialContact) {
		return nil, false, nil
	}
	sphereA, okA := sourceSphereAtPose(a, pa.From)
	sphereB, okB := sourceSphereAtPose(b, pb.From)
	if !okA || !okB {
		return nil, false, nil
	}
	for _, moving := range []struct {
		path   *affinePairPath
		sphere sourceSphereContactProof
	}{{&pa, sphereA}, {&pb, sphereB}} {
		if moving.path.Drift == nil {
			continue
		}
		start := moving.path.From.Translation()
		pivot := moving.path.Drift.Center
		for axis, pair := range [3][2]float64{{start.X, pivot.X}, {start.Y, pivot.Y}, {start.Z, pivot.Z}} {
			if proofarith.DyCmp(moving.sphere.center[axis], proofarith.MustDyOf(pair[0])) != 0 ||
				proofarith.DyCmp(moving.sphere.center[axis], proofarith.MustDyOf(pair[1])) != 0 {
				return report.undecidedRotatingSpherePair(pa.Duration), true, nil
			}
		}
		for axis, velocity := range [3]units.Value{moving.path.Drift.LinearVelocity.X,
			moving.path.Drift.LinearVelocity.Y, moving.path.Drift.LinearVelocity.Z} {
			speed, ok := sweeppath.ExactBaseValue(velocity)
			if !ok {
				return report.undecidedRotatingSpherePair(pa.Duration), true, nil
			}
			full := new(big.Rat).Mul(speed, moving.path.Duration)
			component, ok := proofarith.DyOfRat(full)
			if !ok {
				return report.undecidedRotatingSpherePair(pa.Duration), true, nil
			}
			moving.path.Delta[axis] = component
		}
	}
	pair := [2]sourceSphereContactProof{sphereA, sphereB}
	report.replay = &sweepReplayProof{pa: pa, pb: pb, spherePair: &pair,
		request: req.ContactRequest}
	run := &sourceSpherePairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
		req: req, report: report, sphereA: sphereA, sphereB: sphereB, rotating: true}
	result, err := run.execute(ctx, resolution)
	return result, true, err
}

func (r *SweepReport) undecidedRotatingSpherePair(duration *big.Rat) *SweepReport {
	r.Outcome, r.Cause = SweepUndecided, SweepContactUnsupported
	r.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return r
}

func (r *sourceSpherePairSweepRun) idealAt(f *big.Rat, at SweepInstant) SweepEvent {
	a, okA := translatedSphere(r.sphereA, r.pa.Delta, f)
	b, okB := translatedSphere(r.sphereB, r.pb.Delta, f)
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
	poseA, err := sourceSpherePathPoseAt(r.pa, f)
	if err != nil {
		return nil, err
	}
	poseB, err := sourceSpherePathPoseAt(r.pb, f)
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
	if want.FeatureA != actual.FeatureA || want.FeatureB != actual.FeatureB {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	observedA, okA := sourceSphereAtPose(r.a, poseA)
	observedB, okB := sourceSphereAtPose(r.b, poseB)
	if !okA || !okB {
		ideal.Manifold, ideal.Reason = nil, ContactPayloadUnsupported
		return
	}
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
		return
	}
	bounds, status := spherepath.TransferPairBounds(spherepath.PairTransferInput{
		Motion: r.pairMotion(), ObservedA: observedA.center, ObservedB: observedB.center, Fraction: f,
		IdealNormal: want.Normal.Value, ObservedNormal: actual.Normal.Value,
		IdealNormalBound: want.Normal.Bound.Base(), ObservedNormalBound: actual.Normal.Bound.Base(),
		ObservedNormalAngle: actual.NormalAngle.Base(),
		WitnessBounds:       [2]float64{actual.OnA.Bound.Base(), actual.OnB.Bound.Base()},
		SeparationBound:     actual.Separation.Bound.Base(),
		NormalResolution:    r.req.NormalResolution.Base(), PointResolution: resolution,
	})
	if status != spherepath.PairTransferOK {
		ideal.Manifold = nil
		switch status {
		case spherepath.PairTransferPayloadUnsupported:
			ideal.Reason = ContactPayloadUnsupported
		case spherepath.PairTransferPointTooCoarse:
			ideal.Reason = ContactPointTooCoarse
		default:
			ideal.Reason = ContactNoNormalProof
		}
		return
	}
	if bounds.NormalChanged {
		actual.Normal.Bound = units.Scalar(bounds.NormalBound)
		actual.Normal.Exactness = exactnessFromBound(bounds.NormalBound)
		actual.NormalAngle = units.Radians(bounds.NormalAngle)
	}
	for i, witness := range []*VecMeasurement{&actual.OnA, &actual.OnB} {
		witness.Bound = units.Millimeters(bounds.WitnessBounds[i])
		witness.Exactness = exactnessFromBound(bounds.WitnessBounds[i])
	}
	actual.Separation.Bound = units.Millimeters(bounds.SeparationBound)
	actual.Separation.Exactness = exactnessFromBound(actual.Separation.Bound.Base())
	ideal.Manifold = &ContactManifold{Points: []ContactPoint{actual}}
	ideal.Reason = ContactNoReason
}

func (r *sourceSpherePairSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.Duration),
		To: sweepInstant(to, r.pa.Duration)}
	r.sortSamples()
	return r.report
}

func (r *sourceSpherePairSweepRun) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

func (r *sourceSpherePairSweepRun) pairMotion() spherepath.PairMotion {
	return spherepath.PairMotion{
		CenterA: r.sphereA.center, CenterB: r.sphereB.center,
		RadiusA: r.sphereA.radius, RadiusB: r.sphereB.radius,
		DeltaA: r.pa.Delta, DeltaB: r.pb.Delta,
	}
}

// grazingTouch accepts only an exactly representable interior double root.
// The positive leading coefficient proves strict separation on both sides.
func (r *sourceSpherePairSweepRun) grazingTouch(ctx context.Context, first *SweepSample,
	vertex *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if vertex.Sign() <= 0 || vertex.Cmp(one) >= 0 {
		return r.undecided(zero, one, SweepTimeFloor), nil
	}
	if proofarith.FloatRat(sweeppath.RatFloatNearest(vertex)).Cmp(vertex) != 0 {
		return r.undecided(zero, one, SweepEventUnrepresentable), nil
	}
	eventTime := new(big.Rat).Mul(vertex, r.pa.Duration)
	if proofarith.FloatRat(sweeppath.RatFloatNearest(eventTime)).Cmp(eventTime) != 0 {
		return r.undecided(zero, one, SweepEventUnrepresentable), nil
	}
	touch, err := r.sample(ctx, vertex)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, vertex, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if first.Ideal.Relation != ContactSeparated ||
		touch.Ideal.Relation != ContactTouching || touch.FloatContact.Relation != ContactTouching {
		return r.undecided(zero, vertex, SweepPoseRelation), nil
	}
	if touch.Ideal.Manifold == nil || touch.FloatContact.Manifold == nil ||
		len(touch.Ideal.Manifold.Points) != 1 || len(touch.FloatContact.Manifold.Points) != 1 {
		return r.undecided(zero, vertex, SweepContactUnsupported), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(vertex, one, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if last.Ideal.Relation != ContactSeparated || last.FloatContact.Relation != ContactSeparated {
		return r.undecided(vertex, one, SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = SweepGrazingTouch, &touch.Ideal
	r.report.replay.grazingAt = new(big.Rat).Set(vertex)
	r.sortSamples()
	return r.report, nil
}

func (r *sourceSpherePairSweepRun) transverse(ctx context.Context, first *SweepSample,
	resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	a, b, c := r.pairMotion().SquaredGap()
	if first.Ideal.Relation == ContactTouching {
		if b.Sign() < 0 || a.Sign() == 0 && b.Sign() == 0 {
			return r.undecided(zero, one, SweepContactTrackUnproved), nil
		}
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
	if c.Sign() <= 0 {
		return r.undecided(zero, one, SweepPoseRelation), nil
	}
	isClear := a.Sign() == 0 || b.Sign() >= 0
	vertex := new(big.Rat)
	if !isClear {
		vertex.Quo(proofarith.DyNeg(b).Rat(), proofarith.DyAdd(a, a).Rat())
		minimum := one
		if vertex.Cmp(one) < 0 {
			minimum = vertex
		}
		isClear = spherepath.QuadraticAt(a, b, c, minimum).Sign() > 0
	}
	if isClear {
		last, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != ContactSeparated ||
			r.rotating && last.FloatContact.Relation != ContactSeparated {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepClear
		return r.report, nil
	}
	if a.Sign() > 0 && vertex.Cmp(one) == 0 &&
		spherepath.QuadraticAt(a, b, c, one).Sign() == 0 {
		return r.undecided(zero, one, SweepTimeFloor), nil
	}
	if a.Sign() > 0 && vertex.Sign() > 0 && vertex.Cmp(one) < 0 &&
		spherepath.QuadraticAt(a, b, c, vertex).Sign() == 0 {
		return r.grazingTouch(ctx, first, vertex)
	}
	leftF, rightF, ok := spherepath.QuadraticBracket(a, b, c, vertex, r.pa.Duration, resolution)
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
		(right.Ideal.Relation != ContactTouching && right.Ideal.Relation != ContactOverlapping) {
		return r.undecided(leftF, rightF, SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = SweepImpactBracket, &right.Ideal
	r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.sortSamples()
	return r.report, nil
}

// Equal source-sphere displacements keep the exact center-distance polynomial
// constant. The initial point and source faces then describe every fraction.
func (r *sourceSpherePairSweepRun) persistentTrack(first *SweepSample) *SweepContactTrack {
	if first.Ideal.Relation != ContactTouching || first.Ideal.Manifold == nil ||
		len(first.Ideal.Manifold.Points) != 1 {
		return nil
	}
	for i := range 3 {
		if proofarith.DyCmp(r.pa.Delta[i], r.pb.Delta[i]) != 0 {
			return nil
		}
	}
	a, b, c := r.pairMotion().SquaredGap()
	if !a.IsZero() || !b.IsZero() || !c.IsZero() {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	pair := [2]sourceSphereContactProof{r.sphereA, r.sphereB}
	return &SweepContactTrack{
		start: new(big.Rat), end: big.NewRat(1, 1), duration: new(big.Rat).Set(r.pa.Duration),
		request: r.req.ContactRequest, spherePair: &pair,
		deltaA: r.pa.Delta, deltaB: r.pb.Delta,
		features: [2]ContactFeature{point.FeatureA, point.FeatureB}, normal: point.Normal,
		pointCount: 1,
	}
}

func (r *sourceSpherePairSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
	defer r.report.replay.snapshot(r.report)
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, zero, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if r.rotating && r.req.StartPolicy == ContinueSeparatingTouch && first.Ideal.Relation != ContactTouching {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	if r.rotating && first.Ideal.Relation != first.FloatContact.Relation {
		return r.undecided(zero, one, SweepPoseRelation), nil
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
		if r.req.StartPolicy == ContinueCertifiedTouch {
			if track := r.persistentTrack(first); track != nil {
				last, err := r.sample(ctx, one)
				if errors.Is(err, errSweepPoseBudget) {
					return r.undecided(zero, one, SweepPoseBudget), nil
				}
				if err != nil {
					return nil, err
				}
				if last.Ideal.Relation == ContactTouching && last.Ideal.Manifold != nil &&
					len(last.Ideal.Manifold.Points) == 1 &&
					last.Ideal.Manifold.Points[0].FeatureA == track.features[0] &&
					last.Ideal.Manifold.Points[0].FeatureB == track.features[1] {
					r.report.Outcome, r.report.ContactTrack = SweepPersistentTouch, track
					r.report.replay.track = track
					return r.report, nil
				}
			}
		}
	}
	gap, slope, ok := r.pairMotion().AxialGap()
	if !ok {
		return r.transverse(ctx, first, resolution)
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
			return r.report, nil
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
		if last.Ideal.Relation != ContactSeparated ||
			r.rotating && last.FloatContact.Relation != ContactSeparated {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepClear
		return r.report, nil
	}
	root := new(big.Rat).Quo(proofarith.DyNeg(gap).Rat(), slope.Rat())
	leftF, rightF, ok := spherepath.PairImpactBracket(root, r.pa.Duration, resolution)
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
	r.sortSamples()
	return r.report, nil
}
