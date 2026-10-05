package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

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
		return r.pa.delta, r.pb.delta
	}
	return r.pb.delta, r.pa.delta
}

func (r *orientedSphereSweepRun) sourceCorridor() bool {
	sphereDelta, boxDelta := r.deltas()
	for _, f := range []*big.Rat{new(big.Rat), big.NewRat(1, 1)} {
		sphere, okSphere := translatedSphere(r.sphere, sphereDelta, f)
		box, okBox := translatedOrientedBox(r.box, boxDelta, f)
		if !okSphere || !okBox {
			return false
		}
		axis, side, outward, _, distance2, ok := orientedSphereFace(sphere, box)
		if !ok || axis != r.axis || side != r.side || !sameDyV3(outward, r.outward) ||
			distance2 == nil {
			return false
		}
		// The opposite support stays beyond the ball throughout the affine path.
		face := box.corner[0]
		if side == 1 {
			face = proofarith.DvAdd(face, box.edge[axis])
		}
		d := proofarith.DvDot(proofarith.DvSub(sphere.center, face), outward)
		thickness := proofarith.DvDot(box.edge[axis], outward)
		if side == 0 {
			thickness = proofarith.DyNeg(thickness)
		}
		opposite := proofarith.DyAdd(d, thickness)
		if proofarith.DyCmp(proofarith.DyMul(opposite, opposite), proofarith.DyMul(proofarith.DyMul(sphere.radius, sphere.radius),
			proofarith.DvDot(outward, outward))) <= 0 {
			return false
		}
	}
	face := r.box.corner[0]
	if r.side == 1 {
		face = proofarith.DvAdd(face, r.box.edge[r.axis])
	}
	r.start = proofarith.DvDot(proofarith.DvSub(r.sphere.center, face), r.outward)
	r.slope = proofarith.DvDot(proofarith.DvSub(sphereDelta, boxDelta), r.outward)
	return r.start.Sign() > 0
}

func sameDyV3(a, b proofarith.DyV3) bool {
	for k := range 3 {
		if proofarith.DyCmp(a[k], b[k]) != 0 {
			return false
		}
	}
	return true
}

func (r *orientedSphereSweepRun) signedAt(f *big.Rat) int {
	d := new(big.Rat).Add(r.start.Rat(), new(big.Rat).Mul(r.slope.Rat(), f))
	if d.Sign() <= 0 {
		return -1
	}
	radius2 := proofarith.DyMul(r.sphere.radius, r.sphere.radius).Rat()
	return new(big.Rat).Mul(d, d).Cmp(new(big.Rat).Mul(radius2,
		proofarith.DvDot(r.outward, r.outward).Rat()))
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
			resolution, _ := exactBaseValue(r.req.PointResolution)
			for _, witness := range []*VecMeasurement{&point.OnA, &point.OnB} {
				bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
				if bound.Cmp(resolution) > 0 {
					ideal.Manifold, ideal.Reason = nil, ContactPointTooCoarse
					break
				}
				witness.Bound = units.Millimeters(ratFloatUp(bound))
				witness.Exactness = exactnessFromBound(witness.Bound.Base())
			}
			if ideal.Manifold != nil {
				bound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
				point.Separation.Bound = units.Millimeters(ratFloatUp(bound))
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
	sphereFrom, boxFrom := r.pb.from, r.pa.from
	sphereDelta, boxDelta := r.pb.delta, r.pa.delta
	if r.sphereFirst {
		spherePose, boxPose = poseA, poseB
		sphereFrom, boxFrom = r.pa.from, r.pb.from
		sphereDelta, boxDelta = r.pa.delta, r.pb.delta
	}
	observedSphere, okSphere := translatedReplaySphere(r.sphere, sphereFrom, spherePose)
	observedBox, okBox := translatedReplayOrientedBox(r.box, boxFrom, boxPose)
	if !okSphere || !okBox {
		return new(big.Rat).SetInt64(1 << 30)
	}
	deviation := orientedBoxPoseDeviation(r.box, observedBox, boxDelta, f)
	for k := range 3 {
		expected := new(big.Rat).Add(r.sphere.center[k].Rat(),
			new(big.Rat).Mul(sphereDelta[k].Rat(), f))
		diff := new(big.Rat).Sub(observedSphere.center[k].Rat(), expected)
		deviation.Add(deviation, diff.Abs(diff))
	}
	return deviation
}

func (r *orientedSphereSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.duration),
		To: sweepInstant(to, r.pa.duration)}
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
	return r.report
}

func (r *orientedSphereSweepRun) bracket(resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(r.pa.duration, new(big.Rat).SetInt(grid))
		if new(big.Rat).Mul(width, big.NewRat(4, 1)).Cmp(resolution) <= 0 {
			leftIndex, rightIndex := big.NewInt(0), new(big.Int).Set(grid)
			for new(big.Int).Sub(rightIndex, leftIndex).Cmp(big.NewInt(1)) > 0 {
				middle := new(big.Int).Add(leftIndex, rightIndex)
				middle.Rsh(middle, 1)
				f := new(big.Rat).SetFrac(middle, grid)
				if r.signedAt(f) > 0 {
					leftIndex = middle
				} else {
					rightIndex = middle
				}
			}
			leftIndex.Sub(leftIndex, big.NewInt(1))
			rightIndex.Add(rightIndex, big.NewInt(2))
			left := new(big.Rat).SetFrac(leftIndex, grid)
			right := new(big.Rat).SetFrac(rightIndex, grid)
			if right.Cmp(big.NewRat(1, 1)) > 0 {
				right = big.NewRat(1, 1)
			}
			endSpan := new(big.Rat).Mul(new(big.Rat).Sub(big.NewRat(1, 1), left), r.pa.duration)
			if endSpan.Cmp(resolution) <= 0 && r.signedAt(big.NewRat(1, 1)) <= 0 {
				right = big.NewRat(1, 1)
			}
			span := new(big.Rat).Mul(new(big.Rat).Sub(right, left), r.pa.duration)
			return left, right, left.Sign() > 0 && span.Cmp(resolution) <= 0 &&
				r.signedAt(left) > 0 && r.signedAt(right) <= 0
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

func (r *orientedSphereSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if !orthogonalSourceBox(r.box) {
		return r.undecided(zero, one, SweepContactUnsupported), nil
	}
	axis, side, outward, _, _, ok := orientedSphereFace(r.sphere, r.box)
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
	if !r.sourceCorridor() {
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
	if r.signedAt(one) > 0 {
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
	leftF, rightF, ok := r.bracket(resolution)
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

func translatedReplayOrientedBox(box orientedSourceBox, from, at r3.Transform) (orientedSourceBox, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return orientedSourceBox{}, false
	}
	a, b := from.Translation(), at.Translation()
	before, after := [3]float64{a.X, a.Y, a.Z}, [3]float64{b.X, b.Y, b.Z}
	for k := range 3 {
		if !finiteMeasurementValues(before[k], after[k]) {
			return orientedSourceBox{}, false
		}
		move := proofarith.DySubScalar(proofarith.MustDyOf(after[k]), proofarith.MustDyOf(before[k]))
		for i := range box.corner {
			box.corner[i][k] = proofarith.DyAdd(box.corner[i][k], move)
		}
	}
	return box, true
}

func orientedBoxPoseDeviation(start, observed orientedSourceBox, delta [3]proofarith.Dyadic,
	f *big.Rat) *big.Rat {
	maximum := new(big.Rat)
	for i := range start.corner {
		sum := new(big.Rat)
		for k := range 3 {
			expected := new(big.Rat).Add(start.corner[i][k].Rat(),
				new(big.Rat).Mul(delta[k].Rat(), f))
			difference := new(big.Rat).Sub(observed.corner[i][k].Rat(), expected)
			sum.Add(sum, difference.Abs(difference))
		}
		if sum.Cmp(maximum) > 0 {
			maximum = sum
		}
	}
	return maximum
}
