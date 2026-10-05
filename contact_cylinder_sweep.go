package decad

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceCylinderAxialSweep first tries full-span separation. Otherwise it
// brackets a strictly axial face impact or proves one-sided departure.
func (d *Document) sourceCylinderAxialSweep(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	cylinder sourceCylinderContactProof, box sourceBoxContactProof, cylinderFirst bool) (*SweepReport, error) {
	firstBox, secondBox := box, cylinder.box
	if cylinderFirst {
		firstBox, secondBox = cylinder.box, box
	}
	fullA, fullB := sweptAffineBox(firstBox, pa.delta), sweptAffineBox(secondBox, pb.delta)
	resolution, _ := exactBaseValue(req.PointResolution)
	axis := cylinder.axis
	axisGap := func(a, b sourceBoxContactProof) (int, *big.Rat, bool) {
		if gap := new(big.Rat).Sub(b.lo[axis].rat(), a.hi[axis].rat()); gap.Cmp(resolution) > 0 {
			return 1, gap, true
		}
		if gap := new(big.Rat).Sub(a.lo[axis].rat(), b.hi[axis].rat()); gap.Cmp(resolution) > 0 {
			return -1, gap, true
		}
		return 0, nil, false
	}
	sign, gap, ok := axisGap(fullA, fullB)
	startCylinder, startBox := secondBox, firstBox
	cylinderDelta, boxDelta := pb.delta, pa.delta
	if cylinderFirst {
		startCylinder, startBox = firstBox, secondBox
		cylinderDelta, boxDelta = pa.delta, pb.delta
	}
	endCylinder := translatedAffineBox(startCylinder, cylinderDelta)
	endBox := translatedAffineBox(startBox, boxDelta)
	if !cylinderInsideBoxFace(startCylinder, startBox, axis) ||
		!cylinderInsideBoxFace(endCylinder, endBox, axis) {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	if !ok {
		return (&sourceCylinderImpactRun{doc: d, a: a, b: b, pa: pa, pb: pb,
			req: req, report: report, cylinder: cylinder, box: box,
			cylinderFirst: cylinderFirst}).execute(ctx)
	}
	startA, startB := pa.from, pb.from
	endA, err := pa.poseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	endB, err := pb.poseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	for index, poses := range [][2]r3.Transform{{startA, startB}, {endA, endB}} {
		contact, err := d.ContactPair(ctx, a, b, poses[0], poses[1], req.ContactRequest)
		if err != nil {
			return nil, err
		}
		if contact.Relation != ContactSeparated {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		fraction := big.NewRat(int64(index), 1)
		observedA, okA := translatedReplayBox(firstBox, pa.from, poses[0])
		observedB, okB := translatedReplayBox(secondBox, pb.from, poses[1])
		if !okA || !okB {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		deviation := boxPoseDeviation(firstBox, observedA, pa.delta, fraction)
		deviation.Add(deviation, boxPoseDeviation(secondBox, observedB, pb.delta, fraction))
		if deviation.Cmp(resolution) > 0 || !outerBoxGapExceeds(observedA, observedB, axis, sign, deviation) {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		at := sweepInstant(fraction, pa.duration)
		ideal := SweepEvent{At: at, Relation: ContactSeparated, Gap: contact.Gap}
		report.Samples = append(report.Samples, SweepSample{At: at, PoseA: poses[0], PoseB: poses[1],
			FloatContact: contact, Ideal: ideal, exactFraction: fraction})
		report.PoseEvaluations++
	}
	report.Outcome, report.BoxExcluded = SweepClear, true
	report.replay = &sweepReplayProof{pa: pa, pb: pb, request: req.ContactRequest,
		cylinder: &cylinder, cylinderFirst: cylinderFirst, boxA: firstBox, boxB: secondBox,
		clearAxis: axis, clearSign: sign, clearGap: gap}
	report.replay.snapshot(report)
	return report, nil
}

type sourceCylinderImpactRun struct {
	doc           *Document
	a, b          *Body
	pa, pb        affinePairPath
	req           SweepRequest
	report        *SweepReport
	cylinder      sourceCylinderContactProof
	box           sourceBoxContactProof
	cylinderFirst bool
	side, axis    int
	gap, slope    dyadic
}

func (r *sourceCylinderImpactRun) boxesAt(f *big.Rat) (sourceCylinderContactProof,
	sourceBoxContactProof, bool) {
	cylinderDelta, boxDelta := r.pb.delta, r.pa.delta
	if r.cylinderFirst {
		cylinderDelta, boxDelta = r.pa.delta, r.pb.delta
	}
	cylinderBox, okCylinder := translatedContactBox(r.cylinder.box, cylinderDelta, f)
	box, okBox := translatedContactBox(r.box, boxDelta, f)
	cylinder := r.cylinder
	cylinder.box = cylinderBox
	return cylinder, box, okCylinder && okBox
}

func (r *sourceCylinderImpactRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
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
	actual, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.duration)
	cylinder, box, ok := r.boxesAt(f)
	ideal := SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	if ok {
		contact := &ContactReport{Request: r.req.ContactRequest}
		classifySourceCylinderBox(contact, cylinder, box, r.cylinderFirst)
		ideal = SweepEvent{At: at, Relation: contact.Relation, Gap: contact.Gap,
			Manifold: contact.Manifold, Reason: contact.Reason}
	}
	r.transferManifold(f, poseA, poseB, actual, &ideal)
	sample := SweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: actual, Ideal: ideal, exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

func (r *sourceCylinderImpactRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	actual *ContactReport, ideal *SweepEvent) {
	if ideal.Manifold == nil {
		return
	}
	if actual.Relation != ideal.Relation || !matchingCylinderManifold(actual.Manifold, ideal.Manifold) {
		ideal.Manifold = nil
		ideal.Reason = ContactNoNormalProof
		return
	}
	cylinderPose, boxPose := poseB, poseA
	if r.cylinderFirst {
		cylinderPose, boxPose = poseA, poseB
	}
	observedCylinder, okCylinder := sourceCylinderAtPose(r.cylinderBody(), cylinderPose)
	observedBox, okBox := sourceBoxAtPose(r.boxBody(), boxPose)
	if !okCylinder || !okBox {
		ideal.Manifold = nil
		ideal.Reason = ContactPayloadUnsupported
		return
	}
	firstBox, secondBox := r.box, r.cylinder.box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.box, r.box
	}
	observedA, observedB := observedBox, observedCylinder.box
	if r.cylinderFirst {
		observedA, observedB = observedCylinder.box, observedBox
	}
	deviation := boxPoseDeviation(firstBox, observedA, r.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(secondBox, observedB, r.pb.delta, f))
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		ideal.Manifold = nil
		ideal.Reason = ContactPointTooCoarse
		return
	}
	point := actual.Manifold.Points[0]
	for _, witness := range []*VecMeasurement{&point.OnA, &point.OnB} {
		bound := new(big.Rat).Add(floatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold = nil
			ideal.Reason = ContactPointTooCoarse
			return
		}
		witness.Bound = units.Millimeters(ratFloatUp(bound))
		witness.Exactness = exactnessFromBound(witness.Bound.Base())
	}
	separationBound := new(big.Rat).Add(floatRat(point.Separation.Bound.Base()), deviation)
	point.Separation.Bound = units.Millimeters(ratFloatUp(separationBound))
	point.Separation.Exactness = exactnessFromBound(point.Separation.Bound.Base())
	ideal.Manifold = &ContactManifold{Points: []ContactPoint{point}}
	ideal.Reason = ContactNoReason
}

func (r *sourceCylinderImpactRun) cylinderBody() *Body {
	if r.cylinderFirst {
		return r.a
	}
	return r.b
}

func (r *sourceCylinderImpactRun) boxBody() *Body {
	if r.cylinderFirst {
		return r.b
	}
	return r.a
}

func matchingCylinderManifold(actual, ideal *ContactManifold) bool {
	if actual == nil || ideal == nil || len(actual.Points) != 1 || len(ideal.Points) != 1 {
		return false
	}
	a, b := actual.Points[0], ideal.Points[0]
	return a.FeatureA == b.FeatureA && a.FeatureB == b.FeatureB &&
		a.Normal.Value == b.Normal.Value
}

func (r *sourceCylinderImpactRun) execute(ctx context.Context) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	r.axis = r.cylinder.axis
	cylinderDelta, boxDelta := r.pb.delta, r.pa.delta
	if r.cylinderFirst {
		cylinderDelta, boxDelta = r.pa.delta, r.pb.delta
	}
	for i := range 3 {
		if i != r.axis && dyCmp(cylinderDelta[i], boxDelta[i]) != 0 {
			return cylinderSweepUndecided(r.report, r.pa.duration), nil
		}
	}
	switch {
	case dyCmp(r.cylinder.box.lo[r.axis], r.box.hi[r.axis]) >= 0:
		r.side = 1
		r.gap = dySubScalar(r.cylinder.box.lo[r.axis], r.box.hi[r.axis])
		r.slope = dySubScalar(cylinderDelta[r.axis], boxDelta[r.axis])
	case dyCmp(r.cylinder.box.hi[r.axis], r.box.lo[r.axis]) <= 0:
		r.side = 0
		r.gap = dySubScalar(r.box.lo[r.axis], r.cylinder.box.hi[r.axis])
		r.slope = dySubScalar(boxDelta[r.axis], cylinderDelta[r.axis])
	default:
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	if err != nil {
		return nil, err
	}
	if first.Ideal.Relation != ContactSeparated && first.Ideal.Relation != ContactTouching {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	if first.Ideal.Relation == ContactTouching {
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
	}
	firstBox, secondBox := r.box, r.cylinder.box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.box, r.box
	}
	r.report.replay = &sweepReplayProof{pa: r.pa, pb: r.pb, request: r.req.ContactRequest,
		cylinder: &r.cylinder, cylinderFirst: r.cylinderFirst,
		boxA: firstBox, boxB: secondBox, clearAxis: r.axis,
		cylinderSide: r.side, cylinderGap: r.gap, cylinderSlope: r.slope}
	if first.Ideal.Relation == ContactTouching {
		if r.slope.sign() <= 0 || r.req.StartPolicy != ContinueSeparatingTouch ||
			first.Ideal.Manifold == nil {
			return cylinderSweepUndecided(r.report, r.pa.duration), nil
		}
		last, err := r.sample(ctx, one)
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != ContactSeparated || last.Ideal.Gap == nil {
			return cylinderSweepUndecided(r.report, r.pa.duration), nil
		}
		r.report.Outcome = SweepDepartedClear
		r.report.Departure = &SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
		r.report.replay.snapshot(r.report)
		return r.report, nil
	}
	if r.gap.sign() <= 0 || r.slope.sign() >= 0 ||
		dyAdd(r.gap, r.slope).sign() >= 0 {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	root := new(big.Rat).Quo(dyNeg(r.gap).rat(), r.slope.rat())
	resolution, _ := exactBaseValue(r.req.TimeResolution)
	leftF, rightF, ok := sphereImpactBracket(root, r.pa.duration, resolution)
	if !ok || leftF.Sign() <= 0 || rightF.Cmp(one) > 0 {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	left, err := r.sample(ctx, leftF)
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != ContactSeparated ||
		(right.Ideal.Relation != ContactTouching && right.Ideal.Relation != ContactOverlapping) ||
		right.Ideal.Manifold == nil {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	pointResolution, _ := exactBaseValue(r.req.PointResolution)
	rightGap := new(big.Rat).Add(r.gap.rat(), new(big.Rat).Mul(r.slope.rat(), rightF))
	if new(big.Rat).Abs(rightGap).Cmp(pointResolution) > 0 {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	r.report.Outcome, r.report.Event = SweepImpactBracket, &right.Ideal
	r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.report.replay.snapshot(r.report)
	return r.report, nil
}

func cylinderSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return report
}

func sweptAffineBox(start sourceBoxContactProof, delta [3]dyadic) sourceBoxContactProof {
	for axis := range 3 {
		endLo := dyAdd(start.lo[axis], delta[axis])
		endHi := dyAdd(start.hi[axis], delta[axis])
		start.lo[axis] = dyMin(start.lo[axis], endLo)
		start.hi[axis] = dyMax(start.hi[axis], endHi)
	}
	return start
}

func translatedAffineBox(start sourceBoxContactProof, delta [3]dyadic) sourceBoxContactProof {
	for axis := range 3 {
		start.lo[axis] = dyAdd(start.lo[axis], delta[axis])
		start.hi[axis] = dyAdd(start.hi[axis], delta[axis])
	}
	return start
}

func outerBoxGapExceeds(a, b sourceBoxContactProof, axis, sign int, minimum *big.Rat) bool {
	if sign > 0 {
		return new(big.Rat).Sub(b.lo[axis].rat(), a.hi[axis].rat()).Cmp(minimum) > 0
	}
	return new(big.Rat).Sub(a.lo[axis].rat(), b.hi[axis].rat()).Cmp(minimum) > 0
}
