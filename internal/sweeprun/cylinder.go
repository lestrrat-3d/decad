package sweeprun

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/sweeppath"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spherepath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceCylinderFaceSweep uses one complete axial-disk or circular-sidewall
// box-face corridor for clear motion, first impact, and one-sided departure.
func sourceCylinderFaceSweep(ctx context.Context, h Hooks, a, b *topo.Body,
	pa, pb affinePairPath, req vocab.SweepRequest, report *sweepReport,
	cylinder contactround.CylinderProof, box contactbox.SourceBox, cylinderFirst bool) (*sweepReport, error) {
	firstBox, secondBox := box, cylinder.Box
	if cylinderFirst {
		firstBox, secondBox = cylinder.Box, box
	}
	fullA, okA := sweptBoundsOf(a, pa)
	fullB, okB := sweptBoundsOf(b, pb)
	if !okA || !okB {
		return cylinderSweepUndecided(report, pa.Duration), nil
	}
	resolution, _ := sweeppath.ExactBaseValue(req.PointResolution)
	axis, _, signedGap, selected := pairbox.CylinderFaceCorridor(
		cylinder.Box.AxisBox(), box.AxisBox(), cylinder.Axis, cylinder.Wall != nil)
	if !selected || signedGap.Sign() < 0 {
		return cylinderSweepUndecided(report, pa.Duration), nil
	}
	// The swept boxes enclose both bodies over the whole path, so a gap
	// between them on the face axis proves the pair clear throughout.
	axisGap := func(a, b sweeppath.SweptBounds) (int, *big.Rat, bool) {
		if gap := new(big.Rat).Sub(b.Lo[axis].Rat(), a.Hi[axis].Rat()); gap.Cmp(resolution) > 0 {
			return 1, gap, true
		}
		if gap := new(big.Rat).Sub(a.Lo[axis].Rat(), b.Hi[axis].Rat()); gap.Cmp(resolution) > 0 {
			return -1, gap, true
		}
		return 0, nil, false
	}
	sign, gap, ok := axisGap(fullA, fullB)
	startCylinder, startBox := secondBox, firstBox
	cylinderDelta, boxDelta := pb.Delta, pa.Delta
	if cylinderFirst {
		startCylinder, startBox = firstBox, secondBox
		cylinderDelta, boxDelta = pa.Delta, pb.Delta
	}
	if axis != cylinder.Axis && proofarith.DyCmp(cylinderDelta[2], boxDelta[2]) != 0 {
		return cylinderSweepUndecided(report, pa.Duration), nil
	}
	endCylinder := translatedAffineBox(startCylinder, cylinderDelta)
	endBox := translatedAffineBox(startBox, boxDelta)
	// Every transverse edge difference is affine. Strict containment at both
	// endpoints therefore keeps the disk inside the same face for the full path.
	if !pairbox.CylinderInsideFace(startCylinder.AxisBox(), startBox.AxisBox(), axis) ||
		!pairbox.CylinderInsideFace(endCylinder.AxisBox(), endBox.AxisBox(), axis) {
		return cylinderSweepUndecided(report, pa.Duration), nil
	}
	if !ok {
		return (&sourceCylinderImpactRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
			req: req, report: report, cylinder: cylinder, box: box,
			cylinderFirst: cylinderFirst, axis: axis}).execute(ctx)
	}
	startA, startB := pa.From, pb.From
	endA, err := pa.PoseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	endB, err := pb.PoseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	for index, poses := range [][2]r3.Transform{{startA, startB}, {endA, endB}} {
		contact, err := h.Contact(ctx, poses[0], poses[1], req.ContactRequest)
		if err != nil {
			return nil, err
		}
		if contact.Relation != vocab.ContactSeparated {
			return cylinderSweepUndecided(report, pa.Duration), nil
		}
		fraction := big.NewRat(int64(index), 1)
		observedA, okA := sweepproof.TranslatedReplayBox(firstBox, pa.From, poses[0])
		observedB, okB := sweepproof.TranslatedReplayBox(secondBox, pb.From, poses[1])
		if !okA || !okB {
			return cylinderSweepUndecided(report, pa.Duration), nil
		}
		deviation := sweepproof.BoxPoseDeviation(firstBox, observedA, pa.Delta, fraction)
		deviation.Add(deviation, sweepproof.BoxPoseDeviation(secondBox, observedB, pb.Delta, fraction))
		if deviation.Cmp(resolution) > 0 || !sweepproof.OuterBoxGapExceeds(observedA, observedB, axis, sign, deviation) {
			return cylinderSweepUndecided(report, pa.Duration), nil
		}
		at := sweeppath.Instant(fraction, pa.Duration)
		ideal := sweepEvent{At: at, Relation: vocab.ContactSeparated, Gap: contact.Gap}
		report.Samples = append(report.Samples, sampleWith(sweepSample{At: at, PoseA: poses[0], PoseB: poses[1],
			FloatContact: contact, Ideal: ideal}, fraction))
		report.PoseEvaluations++
	}
	report.Outcome, report.BoxExcluded = vocab.SweepClear, true
	report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, Request: req.ContactRequest,
		Cylinder: &cylinder, CylinderFirst: cylinderFirst, BoxA: firstBox, BoxB: secondBox,
		ClearAxis: axis, ClearSign: sign, ClearGap: gap})
	report.Replay(seal.Key{}).Snapshot(report.Outcome)
	return report, nil
}

type sourceCylinderImpactRun struct {
	hooks         Hooks
	a, b          *topo.Body
	pa, pb        affinePairPath
	req           vocab.SweepRequest
	report        *sweepReport
	cylinder      contactround.CylinderProof
	box           contactbox.SourceBox
	cylinderFirst bool
	side, axis    int
	gap, slope    proofarith.Dyadic
}

func (r *sourceCylinderImpactRun) boxesAt(f *big.Rat) (contactround.CylinderProof,
	contactbox.SourceBox, bool) {
	cylinderDelta, boxDelta := r.pb.Delta, r.pa.Delta
	if r.cylinderFirst {
		cylinderDelta, boxDelta = r.pa.Delta, r.pb.Delta
	}
	cylinderBox, okCylinder := sweepproof.TranslatedContactBox(r.cylinder.Box, cylinderDelta, f)
	box, okBox := sweepproof.TranslatedContactBox(r.box, boxDelta, f)
	cylinder := r.cylinder
	cylinder.Box = cylinderBox
	return cylinder, box, okCylinder && okBox
}

func (r *sourceCylinderImpactRun) sample(ctx context.Context, f *big.Rat) (*sweepSample, error) {
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
	actual, err := r.hooks.Contact(ctx, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweeppath.Instant(f, r.pa.Duration)
	cylinder, box, ok := r.boxesAt(f)
	ideal := sweepEvent{At: at, Relation: vocab.ContactUndecided, Reason: vocab.ContactPayloadUnsupported}
	if ok {
		contact := &contactReport{Request: r.req.ContactRequest}
		contactround.ClassifyCylinderBox(contact, cylinder, box, r.cylinderFirst)
		ideal = sweepEventOf(at, contact)
	}
	r.transferManifold(f, poseA, poseB, actual, &ideal)
	sample := sampleWith(sweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: actual, Ideal: ideal}, new(big.Rat).Set(f))
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

func (r *sourceCylinderImpactRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	actual *contactReport, ideal *sweepEvent) {
	if ideal.Manifold == nil {
		return
	}
	if actual.Relation != ideal.Relation || !matchingCylinderManifold(actual.Manifold, ideal.Manifold) {
		ideal.Manifold = nil
		ideal.Reason = vocab.ContactNoNormalProof
		return
	}
	cylinderPose, boxPose := poseB, poseA
	if r.cylinderFirst {
		cylinderPose, boxPose = poseA, poseB
	}
	observedCylinder, okCylinder := contactround.CylinderAtPose(r.cylinderBody(), cylinderPose)
	observedBox, okBox := contactbox.SourceBoxAtPose(r.boxBody(), boxPose)
	if !okCylinder || !okBox {
		ideal.Manifold = nil
		ideal.Reason = vocab.ContactPayloadUnsupported
		return
	}
	firstBox, secondBox := r.box, r.cylinder.Box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.Box, r.box
	}
	observedA, observedB := observedBox, observedCylinder.Box
	if r.cylinderFirst {
		observedA, observedB = observedCylinder.Box, observedBox
	}
	deviation := sweepproof.BoxPoseDeviation(firstBox, observedA, r.pa.Delta, f)
	deviation.Add(deviation, sweepproof.BoxPoseDeviation(secondBox, observedB, r.pb.Delta, f))
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		ideal.Manifold = nil
		ideal.Reason = vocab.ContactPointTooCoarse
		return
	}
	point := actual.Manifold.Points[0]
	for _, witness := range []*vocab.VecMeasurement{&point.OnA, &point.OnB} {
		bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
		if bound.Cmp(resolution) > 0 {
			ideal.Manifold = nil
			ideal.Reason = vocab.ContactPointTooCoarse
			return
		}
		witness.Bound = units.Millimeters(proofbound.RatFloatUp(bound))
		witness.Exactness = measurement.ExactnessOf(witness.Bound.Base())
	}
	separationBound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
	point.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(separationBound))
	point.Separation.Exactness = measurement.ExactnessOf(point.Separation.Bound.Base())
	ideal.Manifold = &contactManifold{Points: []contactPoint{point}}
	ideal.Reason = vocab.ContactNoReason
}

func (r *sourceCylinderImpactRun) cylinderBody() *topo.Body {
	if r.cylinderFirst {
		return r.a
	}
	return r.b
}

func (r *sourceCylinderImpactRun) boxBody() *topo.Body {
	if r.cylinderFirst {
		return r.b
	}
	return r.a
}

func matchingCylinderManifold(actual, ideal *contactManifold) bool {
	if actual == nil || ideal == nil || len(actual.Points) != 1 || len(ideal.Points) != 1 {
		return false
	}
	a, b := actual.Points[0], ideal.Points[0]
	return a.FeatureA == b.FeatureA && a.FeatureB == b.FeatureB &&
		a.Normal.Value == b.Normal.Value
}

func (r *sourceCylinderImpactRun) execute(ctx context.Context) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	cylinderDelta, boxDelta := r.pb.Delta, r.pa.Delta
	if r.cylinderFirst {
		cylinderDelta, boxDelta = r.pa.Delta, r.pb.Delta
	}
	switch {
	case proofarith.DyCmp(r.cylinder.Box.Lo[r.axis], r.box.Hi[r.axis]) >= 0:
		r.side = 1
		r.gap = proofarith.DySubScalar(r.cylinder.Box.Lo[r.axis], r.box.Hi[r.axis])
		r.slope = proofarith.DySubScalar(cylinderDelta[r.axis], boxDelta[r.axis])
	case proofarith.DyCmp(r.cylinder.Box.Hi[r.axis], r.box.Lo[r.axis]) <= 0:
		r.side = 0
		r.gap = proofarith.DySubScalar(r.box.Lo[r.axis], r.cylinder.Box.Hi[r.axis])
		r.slope = proofarith.DySubScalar(boxDelta[r.axis], cylinderDelta[r.axis])
	default:
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	if err != nil {
		return nil, err
	}
	if first.Ideal.Relation != vocab.ContactSeparated && first.Ideal.Relation != vocab.ContactTouching {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	if first.Ideal.Relation == vocab.ContactTouching {
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == vocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = vocab.SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
	}
	firstBox, secondBox := r.box, r.cylinder.Box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.Box, r.box
	}
	r.report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: r.pa, PB: r.pb, Request: r.req.ContactRequest,
		Cylinder: &r.cylinder, CylinderFirst: r.cylinderFirst,
		BoxA: firstBox, BoxB: secondBox, ClearAxis: r.axis,
		CylinderSide: r.side, CylinderGap: r.gap, CylinderSlope: r.slope})
	if first.Ideal.Relation == vocab.ContactTouching {
		if first.Ideal.Manifold == nil {
			return cylinderSweepUndecided(r.report, r.pa.Duration), nil
		}
		if r.slope.Sign() <= 0 {
			return r.persistentTrack(ctx, first)
		}
		last, err := r.sample(ctx, one)
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != vocab.ContactSeparated || last.Ideal.Gap == nil {
			return cylinderSweepUndecided(r.report, r.pa.Duration), nil
		}
		r.report.Outcome = vocab.SweepDepartedClear
		r.report.Departure = &vocab.SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
		r.report.Replay(seal.Key{}).Snapshot(r.report.Outcome)
		return r.report, nil
	}
	if r.gap.Sign() <= 0 || r.slope.Sign() >= 0 ||
		proofarith.DyAdd(r.gap, r.slope).Sign() > 0 {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	root := new(big.Rat).Quo(proofarith.DyNeg(r.gap).Rat(), r.slope.Rat())
	resolution, _ := sweeppath.ExactBaseValue(r.req.TimeResolution)
	leftF, rightF, ok := spherepath.PairImpactBracket(root, r.pa.Duration, resolution)
	if !ok || leftF.Sign() <= 0 || rightF.Cmp(one) > 0 {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.Duration, zero, leftF), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.Duration, leftF, rightF), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != vocab.ContactSeparated ||
		(right.Ideal.Relation != vocab.ContactTouching && right.Ideal.Relation != vocab.ContactOverlapping) ||
		right.Ideal.Manifold == nil {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	pointResolution, _ := sweeppath.ExactBaseValue(r.req.PointResolution)
	rightGap := new(big.Rat).Add(r.gap.Rat(), new(big.Rat).Mul(r.slope.Rat(), rightF))
	if new(big.Rat).Abs(rightGap).Cmp(pointResolution) > 0 {
		return cylinderSweepUndecided(r.report, r.pa.Duration), nil
	}
	r.report.Outcome, r.report.Event = vocab.SweepImpactBracket, &right.Ideal
	r.report.Bracket = &vocab.SweepInterval{From: left.At, To: right.At}
	r.report.SetBracketRight(seal.Key{}, new(big.Rat).Set(rightF))
	r.report.Replay(seal.Key{}).SetBracket(leftF, rightF)
	r.report.Replay(seal.Key{}).Snapshot(r.report.Outcome)
	return r.report, nil
}

// persistentTrack certifies an end disk resting on the selected box face for
// the whole path. The axial support gap is zero at the start and its affine
// slope is zero, so it is zero at every fraction; the caller's strict
// endpoint containment keeps the complete disk inside the same face
// throughout. The two source faces and the exact normal therefore hold over
// the closed span, and ManifoldAt reduces the exact pair at any fraction.
func (r *sourceCylinderImpactRun) persistentTrack(ctx context.Context, first *sweepSample) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	if r.req.StartPolicy != vocab.ContinueCertifiedTouch {
		return cylinderSweepCause(r.report, r.pa.Duration, vocab.SweepDepartureUnproved), nil
	}
	firstBox, secondBox := r.box, r.cylinder.Box
	if r.cylinderFirst {
		firstBox, secondBox = r.cylinder.Box, r.box
	}
	if !r.slope.IsZero() || r.axis != r.cylinder.Axis || len(first.Ideal.Manifold.Points) != 1 ||
		!pairbox.TrackPointsWithin(firstBox.AxisBox(), secondBox.AxisBox(), r.pa.Delta, r.pb.Delta,
			r.req.PointResolution.Base()) {
		return cylinderSweepCause(r.report, r.pa.Duration, vocab.SweepContactTrackUnproved), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return cylinderSweepBudget(r.report, r.pa.Duration, zero, one), nil
	}
	if err != nil {
		return nil, err
	}
	point := first.Ideal.Manifold.Points[0]
	if last.Ideal.Relation != vocab.ContactTouching || last.Ideal.Manifold == nil ||
		len(last.Ideal.Manifold.Points) != 1 || last.Ideal.Manifold.Points[0].FeatureA != point.FeatureA ||
		last.Ideal.Manifold.Points[0].FeatureB != point.FeatureB {
		return cylinderSweepCause(r.report, r.pa.Duration, vocab.SweepContactTrackUnproved), nil
	}
	cylinder := r.cylinder
	track := &sweepproof.Track{A: firstBox, B: secondBox, DeltaA: r.pa.Delta, DeltaB: r.pb.Delta,
		Cylinder: &cylinder, CylinderFirst: r.cylinderFirst,
		From: new(big.Rat), To: big.NewRat(1, 1), Duration: new(big.Rat).Set(r.pa.Duration),
		Request:    r.req.ContactRequest,
		Sources:    [2]contactFeature{point.FeatureA, point.FeatureB},
		NormalRead: point.Normal, PointCount: 1}
	r.report.Outcome, r.report.ContactTrack = vocab.SweepPersistentTouch, nodeTrack(track)
	r.report.Replay(seal.Key{}).Track = track
	r.report.Replay(seal.Key{}).Snapshot(r.report.Outcome)
	return r.report, nil
}

func cylinderSweepCause(report *sweepReport, duration *big.Rat, cause vocab.SweepCause) *sweepReport {
	cylinderSweepUndecided(report, duration)
	report.Cause = cause
	return report
}

func cylinderSweepBudget(report *sweepReport, duration, from, to *big.Rat) *sweepReport {
	report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepPoseBudget
	report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(from, duration), To: sweeppath.Instant(to, duration)}
	return report
}

func cylinderSweepUndecided(report *sweepReport, duration *big.Rat) *sweepReport {
	report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepContactUnsupported
	report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), duration),
		To: sweeppath.Instant(big.NewRat(1, 1), duration)}
	return report
}

func translatedAffineBox(start contactbox.SourceBox, delta [3]proofarith.Dyadic) contactbox.SourceBox {
	for axis := range 3 {
		start.Lo[axis] = proofarith.DyAdd(start.Lo[axis], delta[axis])
		start.Hi[axis] = proofarith.DyAdd(start.Hi[axis], delta[axis])
	}
	return start
}
