package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
	if pa.drift == nil && pb.drift == nil ||
		(req.StartPolicy != ContinueSeparatingTouch && req.StartPolicy != StopAtInitialContact) {
		return nil, false, nil
	}
	sphereA, okA := sourceSphereAtPose(a, pa.from)
	sphereB, okB := sourceSphereAtPose(b, pb.from)
	if !okA || !okB {
		return nil, false, nil
	}
	for _, moving := range []struct {
		path   *affinePairPath
		sphere sourceSphereContactProof
	}{{&pa, sphereA}, {&pb, sphereB}} {
		if moving.path.drift == nil {
			continue
		}
		start := moving.path.from.Translation()
		pivot := moving.path.drift.Center
		for axis, pair := range [3][2]float64{{start.X, pivot.X}, {start.Y, pivot.Y}, {start.Z, pivot.Z}} {
			if proofarith.DyCmp(moving.sphere.center[axis], proofarith.MustDyOf(pair[0])) != 0 ||
				proofarith.DyCmp(moving.sphere.center[axis], proofarith.MustDyOf(pair[1])) != 0 {
				return report.undecidedRotatingSpherePair(pa.duration), true, nil
			}
		}
		for axis, velocity := range [3]units.Value{moving.path.drift.LinearVelocity.X,
			moving.path.drift.LinearVelocity.Y, moving.path.drift.LinearVelocity.Z} {
			speed, ok := exactBaseValue(velocity)
			if !ok {
				return report.undecidedRotatingSpherePair(pa.duration), true, nil
			}
			full := new(big.Rat).Mul(speed, moving.path.duration)
			component, ok := proofarith.DyOfRat(full)
			if !ok {
				return report.undecidedRotatingSpherePair(pa.duration), true, nil
			}
			moving.path.delta[axis] = component
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
	deviation := new(big.Rat)
	for _, moving := range []struct {
		start, observed sourceSphereContactProof
		delta           [3]proofarith.Dyadic
	}{{r.sphereA, observedA, r.pa.delta}, {r.sphereB, observedB, r.pb.delta}} {
		for i := range 3 {
			center := new(big.Rat).Add(moving.start.center[i].Rat(),
				new(big.Rat).Mul(moving.delta[i].Rat(), f))
			diff := new(big.Rat).Sub(moving.observed.center[i].Rat(), center)
			deviation.Add(deviation, diff.Abs(diff))
		}
	}
	idealA, okA := translatedSphere(r.sphereA, r.pa.delta, f)
	idealB, okB := translatedSphere(r.sphereB, r.pb.delta, f)
	if !okA || !okB {
		ideal.Manifold, ideal.Reason = nil, ContactPayloadUnsupported
		return
	}
	minimumDistance := math.Inf(1)
	for _, pair := range [][2]sourceSphereContactProof{{idealA, idealB}, {observedA, observedB}} {
		squared := proofarith.DyZero()
		for i := range 3 {
			delta := proofarith.DySubScalar(pair[1].center[i], pair[0].center[i])
			squared = proofarith.DyAdd(squared, proofarith.DyMul(delta, delta))
		}
		minimumDistance = math.Min(minimumDistance, proofarith.DySqrtDown(squared))
	}
	if minimumDistance <= 0 || !finiteMeasurementValues(minimumDistance) {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	// Unit-vector normalization changes by at most twice the center-line
	// displacement divided by the shorter center-line length.
	normalMotion := 0.0
	if deviation.Sign() > 0 {
		normalMotion = proofbound.ProvenUpRound(2 * proofbound.RatFloatUp(deviation) / minimumDistance)
	}
	observedNormal := actual.Normal.Value
	idealNormal := want.Normal.Value
	if actual.Normal.Bound.Base() == 0 && want.Normal.Bound.Base() == 0 &&
		observedNormal == idealNormal && spherePairCardinalNormal(observedNormal) {
		normalMotion = 0
	}
	normalDifference := math.Hypot(observedNormal.X-idealNormal.X,
		math.Hypot(observedNormal.Y-idealNormal.Y, observedNormal.Z-idealNormal.Z))
	if !finiteMeasurementValues(normalMotion, normalDifference) ||
		normalDifference > proofbound.ProvenUpRound(actual.Normal.Bound.Base()+want.Normal.Bound.Base()+normalMotion) {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	if normalMotion > 0 {
		actual.Normal.Bound = units.Scalar(proofbound.ProvenUpRound(actual.Normal.Bound.Base() + normalMotion))
		actual.Normal.Exactness = exactnessFromBound(actual.Normal.Bound.Base())
		actual.NormalAngle = units.Radians(proofbound.ProvenUpRound(actual.NormalAngle.Base() + 4*normalMotion))
	}
	if actual.Normal.Bound.Base() > r.req.NormalResolution.Base() ||
		actual.NormalAngle.Base() > r.req.NormalResolution.Base() {
		ideal.Manifold, ideal.Reason = nil, ContactNoNormalProof
		return
	}
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok {
		ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
		return
	}
	for _, witness := range []struct {
		point  *VecMeasurement
		radius proofarith.Dyadic
	}{{&actual.OnA, r.sphereA.radius}, {&actual.OnB, r.sphereB.radius}} {
		bound := new(big.Rat).Add(proofarith.FloatRat(witness.point.Bound.Base()), deviation)
		if normalMotion > 0 {
			bound.Add(bound, proofarith.FloatRat(proofbound.ProvenUpRound(proofbound.RatFloatUp(witness.radius.Rat())*normalMotion)))
		}
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
			return
		}
		witness.point.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
		witness.point.Exactness = exactnessFromBound(witness.point.Bound.Base())
	}
	sepBound := new(big.Rat).Add(proofarith.FloatRat(actual.Separation.Bound.Base()), deviation)
	if normalMotion > 0 {
		sepBound.Add(sepBound, proofarith.FloatRat(proofbound.ProvenUpRound(
			proofbound.RatFloatUp(proofarith.DyAdd(r.sphereA.radius, r.sphereB.radius).Rat())*normalMotion)))
	}
	actual.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(sepBound))
	actual.Separation.Exactness = exactnessFromBound(actual.Separation.Bound.Base())
	ideal.Manifold = &ContactManifold{Points: []ContactPoint{actual}}
	ideal.Reason = ContactNoReason
}

func spherePairCardinalNormal(n r3.Vec) bool {
	return n == (r3.Vec{X: 1}) || n == (r3.Vec{X: -1}) ||
		n == (r3.Vec{Y: 1}) || n == (r3.Vec{Y: -1}) ||
		n == (r3.Vec{Z: 1}) || n == (r3.Vec{Z: -1})
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
				proofarith.FloatRat(ratFloatNearest(left)).Cmp(left) == 0 {
				return left, one, true
			}
			if left.Cmp(root) >= 0 {
				return nil, nil, false
			}
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
func (r *sourceSpherePairSweepRun) axialGap() (proofarith.Dyadic, proofarith.Dyadic, bool) {
	axis, sign, nonzero := 0, 0, 0
	for i := range 3 {
		start := proofarith.DySubScalar(r.sphereB.center[i], r.sphereA.center[i])
		travel := proofarith.DySubScalar(r.pb.delta[i], r.pa.delta[i])
		if start.Sign() != 0 || travel.Sign() != 0 {
			axis, sign, nonzero = i, start.Sign(), nonzero+1
		}
	}
	if nonzero != 1 || sign == 0 {
		return proofarith.Dyadic{}, proofarith.Dyadic{}, false
	}
	initial := proofarith.DyAbs(proofarith.DySubScalar(r.sphereB.center[axis], r.sphereA.center[axis]))
	gap := proofarith.DySubScalar(initial, proofarith.DyAdd(r.sphereA.radius, r.sphereB.radius))
	slope := proofarith.DySubScalar(r.pb.delta[axis], r.pa.delta[axis])
	if sign < 0 {
		slope = proofarith.DyNeg(slope)
	}
	return gap, slope, true
}

// squaredGap is |centerB-centerA+f*(deltaB-deltaA)|²-(radiusA+radiusB)².
// Its coefficients are exact over the held source coordinates and affine path.
func (r *sourceSpherePairSweepRun) squaredGap() (proofarith.Dyadic, proofarith.Dyadic, proofarith.Dyadic) {
	a, b, c := proofarith.DyZero(), proofarith.DyZero(), proofarith.DyZero()
	for i := range 3 {
		p := proofarith.DySubScalar(r.sphereB.center[i], r.sphereA.center[i])
		v := proofarith.DySubScalar(r.pb.delta[i], r.pa.delta[i])
		a = proofarith.DyAdd(a, proofarith.DyMul(v, v))
		b = proofarith.DyAdd(b, proofarith.DyMul(p, v))
		c = proofarith.DyAdd(c, proofarith.DyMul(p, p))
	}
	radius := proofarith.DyAdd(r.sphereA.radius, r.sphereB.radius)
	return a, proofarith.DyAdd(b, b), proofarith.DySubScalar(c, proofarith.DyMul(radius, radius))
}

func spherePairQuadraticAt(a, b, c proofarith.Dyadic, f *big.Rat) *big.Rat {
	out := new(big.Rat).Mul(a.Rat(), f)
	out.Add(out, b.Rat())
	out.Mul(out, f)
	return out.Add(out, c.Rat())
}

// grazingTouch accepts only an exactly representable interior double root.
// The positive leading coefficient proves strict separation on both sides.
func (r *sourceSpherePairSweepRun) grazingTouch(ctx context.Context, first *SweepSample,
	vertex *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if vertex.Sign() <= 0 || vertex.Cmp(one) >= 0 {
		return r.undecided(zero, one, SweepTimeFloor), nil
	}
	if proofarith.FloatRat(ratFloatNearest(vertex)).Cmp(vertex) != 0 {
		return r.undecided(zero, one, SweepEventUnrepresentable), nil
	}
	eventTime := new(big.Rat).Mul(vertex, r.pa.duration)
	if proofarith.FloatRat(ratFloatNearest(eventTime)).Cmp(eventTime) != 0 {
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

// quadraticBracket searches only the decreasing side of the exact squared
// distance. The right seed is already at or inside first contact, so a later
// exit cannot be mistaken for the first encounter.
func spherePairQuadraticBracket(a, b, c proofarith.Dyadic, vertex, duration, resolution *big.Rat) (
	*big.Rat, *big.Rat, bool) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	right := new(big.Rat).Set(one)
	if spherePairQuadraticAt(a, b, c, one).Sign() > 0 {
		found := false
		grid := big.NewInt(1)
		for range 61 {
			index := new(big.Int).Quo(new(big.Int).Mul(vertex.Num(), grid), vertex.Denom())
			candidate := new(big.Rat).SetFrac(index, grid)
			if candidate.Sign() > 0 && candidate.Cmp(one) < 0 &&
				spherePairQuadraticAt(a, b, c, candidate).Sign() <= 0 {
				right, found = candidate, true
				break
			}
			grid.Lsh(grid, 1)
		}
		if !found {
			return nil, nil, false
		}
	}
	left := zero
	for range 61 {
		width := new(big.Rat).Sub(right, left)
		span := new(big.Rat).Mul(width, duration)
		if new(big.Rat).Mul(span, big.NewRat(4, 1)).Cmp(resolution) <= 0 {
			before := new(big.Rat).Sub(left, width)
			after := new(big.Rat).Add(right, width)
			if after.Cmp(one) > 0 {
				after = one
			}
			if before.Sign() > 0 && spherePairQuadraticAt(a, b, c, before).Sign() > 0 &&
				spherePairQuadraticAt(a, b, c, after).Sign() <= 0 &&
				proofarith.FloatRat(ratFloatNearest(before)).Cmp(before) == 0 &&
				proofarith.FloatRat(ratFloatNearest(after)).Cmp(after) == 0 {
				return before, after, true
			}
		}
		mid := new(big.Rat).Add(left, right)
		mid.Quo(mid, big.NewRat(2, 1))
		if spherePairQuadraticAt(a, b, c, mid).Sign() > 0 {
			left = mid
		} else {
			right = mid
		}
	}
	return nil, nil, false
}

func (r *sourceSpherePairSweepRun) transverse(ctx context.Context, first *SweepSample,
	resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	a, b, c := r.squaredGap()
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
		isClear = spherePairQuadraticAt(a, b, c, minimum).Sign() > 0
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
		spherePairQuadraticAt(a, b, c, one).Sign() == 0 {
		return r.undecided(zero, one, SweepTimeFloor), nil
	}
	if a.Sign() > 0 && vertex.Sign() > 0 && vertex.Cmp(one) < 0 &&
		spherePairQuadraticAt(a, b, c, vertex).Sign() == 0 {
		return r.grazingTouch(ctx, first, vertex)
	}
	leftF, rightF, ok := spherePairQuadraticBracket(a, b, c, vertex, r.pa.duration, resolution)
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
		if proofarith.DyCmp(r.pa.delta[i], r.pb.delta[i]) != 0 {
			return nil
		}
	}
	a, b, c := r.squaredGap()
	if !a.IsZero() || !b.IsZero() || !c.IsZero() {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	pair := [2]sourceSphereContactProof{r.sphereA, r.sphereB}
	return &SweepContactTrack{
		start: new(big.Rat), end: big.NewRat(1, 1), duration: new(big.Rat).Set(r.pa.duration),
		request: r.req.ContactRequest, spherePair: &pair,
		deltaA: r.pa.delta, deltaB: r.pb.delta,
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
	gap, slope, ok := r.axialGap()
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
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.sortSamples()
	return r.report, nil
}
