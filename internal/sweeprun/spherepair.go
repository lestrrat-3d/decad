package sweeprun

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/spherepath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type sourceSpherePairSweepRun struct {
	hooks   Hooks
	a, b    *topo.Body
	pa, pb  affinePairPath
	req     vocab.SweepRequest
	report  *sweepReport
	sphereA contactround.SphereProof
	sphereB contactround.SphereProof
	// rotating marks a centered spinning drift. Its real start pose must
	// report the ideal relation and a clear path's real end pose separation;
	// ContinueSeparatingTouch needs a touching start.
	rotating bool
}

// A source ball centered at its query origin is unchanged by its own spin.
// A center-pivot drift therefore has the same exact occupied-set path as an
// affine translation, while sampled poses still carry the requested rotation.
func sweepRotatingSpherePair(ctx context.Context, h Hooks, a, b *topo.Body,
	pa, pb affinePairPath, req vocab.SweepRequest, resolution *big.Rat,
	report *sweepReport) (*sweepReport, bool, error) {
	if pa.Drift == nil && pb.Drift == nil ||
		(req.StartPolicy != vocab.ContinueSeparatingTouch && req.StartPolicy != vocab.StopAtInitialContact) {
		return nil, false, nil
	}
	sphereA, okA := contactround.SphereAtPose(a, pa.From)
	sphereB, okB := contactround.SphereAtPose(b, pb.From)
	if !okA || !okB {
		return nil, false, nil
	}
	for _, moving := range []struct {
		path   *affinePairPath
		sphere contactround.SphereProof
	}{{&pa, sphereA}, {&pb, sphereB}} {
		if moving.path.Drift == nil {
			continue
		}
		start := moving.path.From.Translation()
		pivot := moving.path.Drift.Center
		for axis, pair := range [3][2]float64{{start.X, pivot.X}, {start.Y, pivot.Y}, {start.Z, pivot.Z}} {
			if proofarith.DyCmp(moving.sphere.Center[axis], proofarith.MustDyOf(pair[0])) != 0 ||
				proofarith.DyCmp(moving.sphere.Center[axis], proofarith.MustDyOf(pair[1])) != 0 {
				return undecidedRotatingSpherePair(report, pa.Duration), true, nil
			}
		}
		for axis, velocity := range [3]units.Value{moving.path.Drift.LinearVelocity.X,
			moving.path.Drift.LinearVelocity.Y, moving.path.Drift.LinearVelocity.Z} {
			speed, ok := sweeppath.ExactBaseValue(velocity)
			if !ok {
				return undecidedRotatingSpherePair(report, pa.Duration), true, nil
			}
			full := new(big.Rat).Mul(speed, moving.path.Duration)
			component, ok := proofarith.DyOfRat(full)
			if !ok {
				return undecidedRotatingSpherePair(report, pa.Duration), true, nil
			}
			moving.path.Delta[axis] = component
		}
	}
	pair := [2]contactround.SphereProof{sphereA, sphereB}
	report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, SpherePair: &pair,
		Request: req.ContactRequest})
	run := &sourceSpherePairSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
		req: req, report: report, sphereA: sphereA, sphereB: sphereB, rotating: true}
	result, err := run.execute(ctx, resolution)
	return result, true, err
}

func undecidedRotatingSpherePair(r *sweepReport, duration *big.Rat) *sweepReport {
	r.Outcome, r.Cause = vocab.SweepUndecided, vocab.SweepContactUnsupported
	r.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), duration),
		To: sweeppath.Instant(big.NewRat(1, 1), duration)}
	return r
}

func (r *sourceSpherePairSweepRun) idealAt(f *big.Rat, at vocab.SweepInstant) sweepEvent {
	a, okA := contactround.TranslatedSphere(r.sphereA, r.pa.Delta, f)
	b, okB := contactround.TranslatedSphere(r.sphereB, r.pb.Delta, f)
	if !okA || !okB {
		return sweepEvent{At: at, Relation: vocab.ContactUndecided, Reason: vocab.ContactPayloadUnsupported}
	}
	contact := &contactReport{Request: r.req.ContactRequest}
	contactround.ClassifySpherePair(contact, a, b)
	return sweepEventOf(at, contact)
}

func (r *sourceSpherePairSweepRun) sample(ctx context.Context, f *big.Rat) (*sweepSample, error) {
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
	contact, err := r.hooks.Contact(ctx, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweeppath.Instant(f, r.pa.Duration)
	ideal := r.idealAt(f, at)
	r.transferManifold(f, poseA, poseB, contact, &ideal)
	sample := sweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact, Ideal: ideal}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

func (r *sourceSpherePairSweepRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	contact *contactReport, ideal *sweepEvent) {
	if ideal.Manifold == nil {
		return
	}
	if contact.Manifold == nil || contact.Relation == vocab.ContactSeparated ||
		len(contact.Manifold.Points) != 1 || len(ideal.Manifold.Points) != 1 {
		ideal.Manifold, ideal.Reason = nil, vocab.ContactNoNormalProof
		return
	}
	want, actual := ideal.Manifold.Points[0], contact.Manifold.Points[0]
	if want.FeatureA != actual.FeatureA || want.FeatureB != actual.FeatureB {
		ideal.Manifold, ideal.Reason = nil, vocab.ContactNoNormalProof
		return
	}
	observedA, okA := contactround.SphereAtPose(r.a, poseA)
	observedB, okB := contactround.SphereAtPose(r.b, poseB)
	if !okA || !okB {
		ideal.Manifold, ideal.Reason = nil, vocab.ContactPayloadUnsupported
		return
	}
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold, ideal.Reason = nil, vocab.ContactPointTooCoarse
		return
	}
	bounds, status := spherepath.TransferPairBounds(spherepath.PairTransferInput{
		Motion: r.pairMotion(), ObservedA: observedA.Center, ObservedB: observedB.Center, Fraction: f,
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
			ideal.Reason = vocab.ContactPayloadUnsupported
		case spherepath.PairTransferPointTooCoarse:
			ideal.Reason = vocab.ContactPointTooCoarse
		default:
			ideal.Reason = vocab.ContactNoNormalProof
		}
		return
	}
	if bounds.NormalChanged {
		actual.Normal.Bound = units.Scalar(bounds.NormalBound)
		actual.Normal.Exactness = measurement.ExactnessOf(bounds.NormalBound)
		actual.NormalAngle = units.Radians(bounds.NormalAngle)
	}
	for i, witness := range []*vocab.VecMeasurement{&actual.OnA, &actual.OnB} {
		witness.Bound = units.Millimeters(bounds.WitnessBounds[i])
		witness.Exactness = measurement.ExactnessOf(bounds.WitnessBounds[i])
	}
	actual.Separation.Bound = units.Millimeters(bounds.SeparationBound)
	actual.Separation.Exactness = measurement.ExactnessOf(actual.Separation.Bound.Base())
	ideal.Manifold = &contactManifold{Points: []contactPoint{actual}}
	ideal.Reason = vocab.ContactNoReason
}

func (r *sourceSpherePairSweepRun) undecided(from, to *big.Rat, cause vocab.SweepCause) *sweepReport {
	r.report.Outcome, r.report.Cause = vocab.SweepUndecided, cause
	r.report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(from, r.pa.Duration),
		To: sweeppath.Instant(to, r.pa.Duration)}
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
		CenterA: r.sphereA.Center, CenterB: r.sphereB.Center,
		RadiusA: r.sphereA.Radius, RadiusB: r.sphereB.Radius,
		DeltaA: r.pa.Delta, DeltaB: r.pb.Delta,
	}
}

// grazingTouch accepts only an exactly representable interior double root.
// The positive leading coefficient proves strict separation on both sides.
func (r *sourceSpherePairSweepRun) grazingTouch(ctx context.Context, first *sweepSample,
	vertex *big.Rat) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if vertex.Sign() <= 0 || vertex.Cmp(one) >= 0 {
		return r.undecided(zero, one, vocab.SweepTimeFloor), nil
	}
	if proofarith.FloatRat(sweeppath.RatFloatNearest(vertex)).Cmp(vertex) != 0 {
		return r.undecided(zero, one, vocab.SweepEventUnrepresentable), nil
	}
	eventTime := new(big.Rat).Mul(vertex, r.pa.Duration)
	if proofarith.FloatRat(sweeppath.RatFloatNearest(eventTime)).Cmp(eventTime) != 0 {
		return r.undecided(zero, one, vocab.SweepEventUnrepresentable), nil
	}
	touch, err := r.sample(ctx, vertex)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, vertex, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if first.Ideal.Relation != vocab.ContactSeparated ||
		touch.Ideal.Relation != vocab.ContactTouching || touch.FloatContact.Relation != vocab.ContactTouching {
		return r.undecided(zero, vertex, vocab.SweepPoseRelation), nil
	}
	if touch.Ideal.Manifold == nil || touch.FloatContact.Manifold == nil ||
		len(touch.Ideal.Manifold.Points) != 1 || len(touch.FloatContact.Manifold.Points) != 1 {
		return r.undecided(zero, vertex, vocab.SweepContactUnsupported), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(vertex, one, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if last.Ideal.Relation != vocab.ContactSeparated || last.FloatContact.Relation != vocab.ContactSeparated {
		return r.undecided(vertex, one, vocab.SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = vocab.SweepGrazingTouch, &touch.Ideal
	r.report.Replay(seal.Key{}).GrazingAt = new(big.Rat).Set(vertex)
	r.sortSamples()
	return r.report, nil
}

func (r *sourceSpherePairSweepRun) transverse(ctx context.Context, first *sweepSample,
	resolution *big.Rat) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	a, b, c := r.pairMotion().SquaredGap()
	if first.Ideal.Relation == vocab.ContactTouching {
		if b.Sign() < 0 || a.Sign() == 0 && b.Sign() == 0 {
			return r.undecided(zero, one, vocab.SweepContactTrackUnproved), nil
		}
		last, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, vocab.SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != vocab.ContactSeparated || last.Ideal.Gap == nil {
			return r.undecided(zero, one, vocab.SweepPoseRelation), nil
		}
		r.report.Outcome = vocab.SweepDepartedClear
		r.report.Departure = &vocab.SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
		return r.report, nil
	}
	if c.Sign() <= 0 {
		return r.undecided(zero, one, vocab.SweepPoseRelation), nil
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
			return r.undecided(zero, one, vocab.SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != vocab.ContactSeparated ||
			r.rotating && last.FloatContact.Relation != vocab.ContactSeparated {
			return r.undecided(zero, one, vocab.SweepPoseRelation), nil
		}
		r.report.Outcome = vocab.SweepClear
		return r.report, nil
	}
	if a.Sign() > 0 && vertex.Cmp(one) == 0 &&
		spherepath.QuadraticAt(a, b, c, one).Sign() == 0 {
		return r.undecided(zero, one, vocab.SweepTimeFloor), nil
	}
	if a.Sign() > 0 && vertex.Sign() > 0 && vertex.Cmp(one) < 0 &&
		spherepath.QuadraticAt(a, b, c, vertex).Sign() == 0 {
		return r.grazingTouch(ctx, first, vertex)
	}
	leftF, rightF, ok := spherepath.QuadraticBracket(a, b, c, vertex, r.pa.Duration, resolution)
	if !ok {
		return r.undecided(zero, one, vocab.SweepTimeFloor), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, leftF, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(leftF, rightF, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != vocab.ContactSeparated ||
		(right.Ideal.Relation != vocab.ContactTouching && right.Ideal.Relation != vocab.ContactOverlapping) {
		return r.undecided(leftF, rightF, vocab.SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = vocab.SweepImpactBracket, &right.Ideal
	r.report.Bracket = &vocab.SweepInterval{From: left.At, To: right.At}
	r.report.SetBracketRight(seal.Key{}, new(big.Rat).Set(rightF))
	r.report.Replay(seal.Key{}).SetBracket(leftF, rightF)
	r.sortSamples()
	return r.report, nil
}

// Equal source-sphere displacements keep the exact center-distance polynomial
// constant. The initial point and source faces then describe every fraction.
func (r *sourceSpherePairSweepRun) persistentTrack(first *sweepSample) *sweepproof.Track {
	if first.Ideal.Relation != vocab.ContactTouching || first.Ideal.Manifold == nil ||
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
	pair := [2]contactround.SphereProof{r.sphereA, r.sphereB}
	return &sweepproof.Track{
		From: new(big.Rat), To: big.NewRat(1, 1), Duration: new(big.Rat).Set(r.pa.Duration),
		Request: r.req.ContactRequest, SpherePair: &pair,
		DeltaA: r.pa.Delta, DeltaB: r.pb.Delta,
		Sources:    [2]contactFeature{point.FeatureA, point.FeatureB},
		NormalRead: point.Normal,
		PointCount: 1,
	}
}

func (r *sourceSpherePairSweepRun) execute(ctx context.Context, resolution *big.Rat) (*sweepReport, error) {
	proof := r.report.Replay(seal.Key{})
	defer func() { proof.Snapshot(r.report.Outcome) }()
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, zero, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if r.rotating && r.req.StartPolicy == vocab.ContinueSeparatingTouch && first.Ideal.Relation != vocab.ContactTouching {
		return r.undecided(zero, one, vocab.SweepContactUnsupported), nil
	}
	if r.rotating && first.Ideal.Relation != first.FloatContact.Relation {
		return r.undecided(zero, one, vocab.SweepPoseRelation), nil
	}
	if first.Ideal.Relation == vocab.ContactOverlapping {
		r.report.InitialEvent, r.report.Event = &first.Ideal, &first.Ideal
		r.report.Outcome = vocab.SweepInitiallyOverlapping
		return r.report, nil
	}
	if first.Ideal.Relation != vocab.ContactSeparated && first.Ideal.Relation != vocab.ContactTouching {
		return r.undecided(zero, zero, vocab.SweepPoseRelation), nil
	}
	if first.Ideal.Relation == vocab.ContactTouching {
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == vocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = vocab.SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if r.req.StartPolicy == vocab.ContinueCertifiedTouch {
			if track := r.persistentTrack(first); track != nil {
				last, err := r.sample(ctx, one)
				if errors.Is(err, errSweepPoseBudget) {
					return r.undecided(zero, one, vocab.SweepPoseBudget), nil
				}
				if err != nil {
					return nil, err
				}
				if last.Ideal.Relation == vocab.ContactTouching && last.Ideal.Manifold != nil &&
					len(last.Ideal.Manifold.Points) == 1 &&
					last.Ideal.Manifold.Points[0].FeatureA == track.Sources[0] &&
					last.Ideal.Manifold.Points[0].FeatureB == track.Sources[1] {
					r.report.Outcome, r.report.ContactTrack = vocab.SweepPersistentTouch, nodeTrack(track)
					r.report.Replay(seal.Key{}).Track = track
					return r.report, nil
				}
			}
		}
	}
	gap, slope, ok := r.pairMotion().AxialGap()
	if !ok {
		return r.transverse(ctx, first, resolution)
	}
	if first.Ideal.Relation == vocab.ContactTouching {
		if slope.Sign() > 0 {
			last, err := r.sample(ctx, one)
			if errors.Is(err, errSweepPoseBudget) {
				return r.undecided(zero, one, vocab.SweepPoseBudget), nil
			}
			if err != nil {
				return nil, err
			}
			if last.Ideal.Relation != vocab.ContactSeparated || last.Ideal.Gap == nil {
				return r.undecided(zero, one, vocab.SweepPoseRelation), nil
			}
			r.report.Outcome = vocab.SweepDepartedClear
			r.report.Departure = &vocab.SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
			return r.report, nil
		}
		return r.undecided(zero, one, vocab.SweepContactTrackUnproved), nil
	}
	if gap.Sign() <= 0 {
		return r.undecided(zero, one, vocab.SweepPoseRelation), nil
	}
	if slope.Sign() >= 0 || proofarith.DyAdd(gap, slope).Sign() > 0 {
		last, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, vocab.SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != vocab.ContactSeparated ||
			r.rotating && last.FloatContact.Relation != vocab.ContactSeparated {
			return r.undecided(zero, one, vocab.SweepPoseRelation), nil
		}
		r.report.Outcome = vocab.SweepClear
		return r.report, nil
	}
	root := new(big.Rat).Quo(proofarith.DyNeg(gap).Rat(), slope.Rat())
	leftF, rightF, ok := spherepath.PairImpactBracket(root, r.pa.Duration, resolution)
	if !ok || leftF.Sign() <= 0 || rightF.Cmp(one) > 0 {
		return r.undecided(zero, one, vocab.SweepTimeFloor), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, leftF, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(leftF, rightF, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != vocab.ContactSeparated ||
		(right.Ideal.Relation != vocab.ContactTouching && right.Ideal.Relation != vocab.ContactOverlapping) ||
		right.Ideal.Manifold == nil {
		return r.undecided(leftF, rightF, vocab.SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = vocab.SweepImpactBracket, &right.Ideal
	r.report.Bracket = &vocab.SweepInterval{From: left.At, To: right.At}
	r.report.SetBracketRight(seal.Key{}, new(big.Rat).Set(rightF))
	r.report.Replay(seal.Key{}).SetBracket(leftF, rightF)
	r.sortSamples()
	return r.report, nil
}
