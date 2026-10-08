package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spherepath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceCylinderFaceSweep uses one complete axial-disk or circular-sidewall
// box-face corridor for clear motion, first impact, and one-sided departure.
func (d *Document) sourceCylinderFaceSweep(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	cylinder sourceCylinderContactProof, box sourceBoxContactProof, cylinderFirst bool) (*SweepReport, error) {
	firstBox, secondBox := box, cylinder.box
	if cylinderFirst {
		firstBox, secondBox = cylinder.box, box
	}
	fullA, okA := sweptBoxOf(a, pa)
	fullB, okB := sweptBoxOf(b, pb)
	if !okA || !okB {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	resolution, _ := exactBaseValue(req.PointResolution)
	axis, _, signedGap, selected := sourceCylinderBoxFace(cylinder, box)
	if !selected || signedGap.Sign() < 0 {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	// The swept boxes enclose both bodies over the whole path, so a gap
	// between them on the face axis proves the pair clear throughout.
	axisGap := func(a, b SweptBox) (int, *big.Rat, bool) {
		if gap := new(big.Rat).Sub(b.lo[axis].Rat(), a.hi[axis].Rat()); gap.Cmp(resolution) > 0 {
			return 1, gap, true
		}
		if gap := new(big.Rat).Sub(a.lo[axis].Rat(), b.hi[axis].Rat()); gap.Cmp(resolution) > 0 {
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
	if axis != cylinder.axis && proofarith.DyCmp(cylinderDelta[2], boxDelta[2]) != 0 {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	endCylinder := translatedAffineBox(startCylinder, cylinderDelta)
	endBox := translatedAffineBox(startBox, boxDelta)
	// Every transverse edge difference is affine. Strict containment at both
	// endpoints therefore keeps the disk inside the same face for the full path.
	if !cylinderInsideBoxFace(startCylinder, startBox, axis) ||
		!cylinderInsideBoxFace(endCylinder, endBox, axis) {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	if !ok {
		return (&sourceCylinderImpactRun{doc: d, a: a, b: b, pa: pa, pb: pb,
			req: req, report: report, cylinder: cylinder, box: box,
			cylinderFirst: cylinderFirst, axis: axis}).execute(ctx)
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
	gap, slope    proofarith.Dyadic
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
		bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold = nil
			ideal.Reason = ContactPointTooCoarse
			return
		}
		witness.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
		witness.Exactness = exactnessFromBound(witness.Bound.Base())
	}
	separationBound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
	point.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(separationBound))
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
	cylinderDelta, boxDelta := r.pb.delta, r.pa.delta
	if r.cylinderFirst {
		cylinderDelta, boxDelta = r.pa.delta, r.pb.delta
	}
	switch {
	case proofarith.DyCmp(r.cylinder.box.lo[r.axis], r.box.hi[r.axis]) >= 0:
		r.side = 1
		r.gap = proofarith.DySubScalar(r.cylinder.box.lo[r.axis], r.box.hi[r.axis])
		r.slope = proofarith.DySubScalar(cylinderDelta[r.axis], boxDelta[r.axis])
	case proofarith.DyCmp(r.cylinder.box.hi[r.axis], r.box.lo[r.axis]) <= 0:
		r.side = 0
		r.gap = proofarith.DySubScalar(r.box.lo[r.axis], r.cylinder.box.hi[r.axis])
		r.slope = proofarith.DySubScalar(boxDelta[r.axis], cylinderDelta[r.axis])
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
		if first.Ideal.Manifold == nil {
			return cylinderSweepUndecided(r.report, r.pa.duration), nil
		}
		if r.slope.Sign() <= 0 {
			return r.persistentTrack(ctx, first)
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
	if r.gap.Sign() <= 0 || r.slope.Sign() >= 0 ||
		proofarith.DyAdd(r.gap, r.slope).Sign() > 0 {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	root := new(big.Rat).Quo(proofarith.DyNeg(r.gap).Rat(), r.slope.Rat())
	resolution, _ := exactBaseValue(r.req.TimeResolution)
	leftF, rightF, ok := spherepath.PairImpactBracket(root, r.pa.duration, resolution)
	if !ok || leftF.Sign() <= 0 || rightF.Cmp(one) > 0 {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.duration, zero, leftF), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.duration, leftF, rightF), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != ContactSeparated ||
		(right.Ideal.Relation != ContactTouching && right.Ideal.Relation != ContactOverlapping) ||
		right.Ideal.Manifold == nil {
		return cylinderSweepUndecided(r.report, r.pa.duration), nil
	}
	pointResolution, _ := exactBaseValue(r.req.PointResolution)
	rightGap := new(big.Rat).Add(r.gap.Rat(), new(big.Rat).Mul(r.slope.Rat(), rightF))
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

// persistentTrack certifies an end disk resting on the selected box face for
// the whole path. The axial support gap is zero at the start and its affine
// slope is zero, so it is zero at every fraction; the caller's strict
// endpoint containment keeps the complete disk inside the same face
// throughout. The two source faces and the exact normal therefore hold over
// the closed span, and ManifoldAt reduces the exact pair at any fraction.
func (r *sourceCylinderImpactRun) persistentTrack(ctx context.Context, first *SweepSample) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if r.req.StartPolicy != ContinueCertifiedTouch {
		return cylinderSweepCause(r.report, r.pa.duration, SweepDepartureUnproved), nil
	}
	firstBox, secondBox := r.box, r.cylinder.box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.box, r.box
	}
	if !r.slope.IsZero() || r.axis != r.cylinder.axis || len(first.Ideal.Manifold.Points) != 1 ||
		!pairbox.TrackPointsWithin(firstBox.axisBox(), secondBox.axisBox(), r.pa.delta, r.pb.delta,
			r.req.PointResolution.Base()) {
		return cylinderSweepCause(r.report, r.pa.duration, SweepContactTrackUnproved), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.duration, zero, one), nil
	}
	if err != nil {
		return nil, err
	}
	point := first.Ideal.Manifold.Points[0]
	if last.Ideal.Relation != ContactTouching || last.Ideal.Manifold == nil ||
		len(last.Ideal.Manifold.Points) != 1 || last.Ideal.Manifold.Points[0].FeatureA != point.FeatureA ||
		last.Ideal.Manifold.Points[0].FeatureB != point.FeatureB {
		return cylinderSweepCause(r.report, r.pa.duration, SweepContactTrackUnproved), nil
	}
	cylinder := r.cylinder
	track := &SweepContactTrack{a: firstBox, b: secondBox, deltaA: r.pa.delta, deltaB: r.pb.delta,
		cylinder: &cylinder, cylinderFirst: r.cylinderFirst,
		start: new(big.Rat), end: big.NewRat(1, 1), duration: new(big.Rat).Set(r.pa.duration),
		request: r.req.ContactRequest, features: [2]ContactFeature{point.FeatureA, point.FeatureB},
		normal: point.Normal, pointCount: 1}
	r.report.Outcome, r.report.ContactTrack = SweepPersistentTouch, track
	r.report.replay.track = track
	r.report.replay.snapshot(r.report)
	return r.report, nil
}

// cylinderTrackManifold reduces the track's exact cylinder and box at one
// fraction and requires the track's single disk-face point.
func (t *SweepContactTrack) cylinderTrackManifold(f *big.Rat) (*ContactManifold, error) {
	cylinderBox, box := t.b, t.a
	cylinderDelta, boxDelta := t.deltaB, t.deltaA
	if t.cylinderFirst {
		cylinderBox, box = t.a, t.b
		cylinderDelta, boxDelta = t.deltaA, t.deltaB
	}
	movedCylinder, okCylinder := translatedContactBox(cylinderBox, cylinderDelta, f)
	movedBox, okBox := translatedContactBox(box, boxDelta, f)
	if !okCylinder || !okBox {
		return nil, fmt.Errorf("%w: cylinder contact-track fraction cannot be represented", ErrUnsupported)
	}
	cylinder := *t.cylinder
	cylinder.box = movedCylinder
	report := &ContactReport{Request: t.request}
	classifySourceCylinderBox(report, cylinder, movedBox, t.cylinderFirst)
	if report.Relation != ContactTouching || report.Manifold == nil || len(report.Manifold.Points) != 1 ||
		report.Manifold.Points[0].FeatureA != t.features[0] ||
		report.Manifold.Points[0].FeatureB != t.features[1] {
		return nil, fmt.Errorf("%w: cylinder contact track lost its disk point", ErrUnsupported)
	}
	return report.Manifold, nil
}

func cylinderSweepCause(report *SweepReport, duration *big.Rat, cause SweepCause) *SweepReport {
	cylinderSweepUndecided(report, duration)
	report.Cause = cause
	return report
}

func cylinderSweepBudget(report *SweepReport, duration, from, to *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepPoseBudget
	report.Unresolved = &SweepInterval{From: sweepInstant(from, duration), To: sweepInstant(to, duration)}
	return report
}

func cylinderSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return report
}

func translatedAffineBox(start sourceBoxContactProof, delta [3]proofarith.Dyadic) sourceBoxContactProof {
	for axis := range 3 {
		start.lo[axis] = proofarith.DyAdd(start.lo[axis], delta[axis])
		start.hi[axis] = proofarith.DyAdd(start.hi[axis], delta[axis])
	}
	return start
}

func outerBoxGapExceeds(a, b sourceBoxContactProof, axis, sign int, minimum *big.Rat) bool {
	if sign > 0 {
		return new(big.Rat).Sub(b.lo[axis].Rat(), a.hi[axis].Rat()).Cmp(minimum) > 0
	}
	return new(big.Rat).Sub(a.lo[axis].Rat(), b.hi[axis].Rat()).Cmp(minimum) > 0
}
