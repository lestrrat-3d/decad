package decad

import (
	"context"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the general rotating sweep of docs/multibody-dynamics-design.md
// §10.1: rotationalPairSweep's search, travel bound and pose deviation run
// over the exact vertex set of two §9 planar bodies instead of eight source-box
// corners. Each sample's relation and gap come from ContactPair's exact planar
// relation at the rounded pose, and the float-to-ideal deviation is the largest
// outward vertex distance (planarsweep.BodyPath.PointDeviation). Affine paths
// take the same run with a zero angular term.

// sweepPlanarPair runs the general sweep when both bodies are admitted §9
// planar solids. When either is not, a source cylinder rolling on a planar
// body takes sweepRollingPair (contact_sweep_rolling.go); any other pair
// reports false, leaving report untouched. A path whose ideal motion or
// travel bound cannot be formed is SweepUndecided with SweepMissingBound.
func (d *Document) sweepPlanarPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req reportvocab.SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	chord := req.HeldChord.Base()
	solidA, deltaA, okA, err := planarpose.SolidAtPose(ctx, budget, verifyAllMesher{}, bodyNode(a), r3.Identity(), chord)
	if err != nil {
		return nil, true, err
	}
	solidB, deltaB, okB, err := planarpose.SolidAtPose(ctx, budget, verifyAllMesher{}, bodyNode(b), r3.Identity(), chord)
	if err != nil {
		return nil, true, err
	}
	if !okA || !okB {
		return d.sweepRollingPair(ctx, a, b, pa, pb, req, resolution, report)
	}
	planarA, okA := planarsweep.PreparePlanarPath(&a.n, pa, &solidA, deltaA)
	planarB, okB := planarsweep.PreparePlanarPath(&b.n, pb, &solidB, deltaB)
	if !okA || !okB {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
			To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
		return report, true, nil
	}
	aPath, bPath := rotationalSweepPath{BodyPath: planarA, body: a}, rotationalSweepPath{BodyPath: planarB, body: b}
	planarsweep.AttachMemos(&aPath.BodyPath, &bPath.BodyPath)
	defer planarsweep.CloseMemos(&aPath.BodyPath, &bPath.BodyPath)
	run := rotationalPairSweep{doc: d, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report, planar: true}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, true, err
	}
	result.replay = run.planarReplayProof(result)
	return result, true, nil
}

// planarContinuation reruns a source-box pair whose box continuation proofs
// could not settle an initial touch, or a §10.5 ContactBand start, as two
// exact planar bodies, whose §10.2 and §10.3 proofs read every vertex. It
// reports false, and the box report stands, when the planar run is not
// admitted or settles nothing either.
func (d *Document) planarContinuation(ctx context.Context, boxes *SweepReport,
	pa, pb affinePairPath, resolution *big.Rat) (*SweepReport, bool, error) {
	if boxes.Outcome != SweepUndecided || boxes.InitialEvent == nil ||
		boxes.InitialEvent.Relation != ContactTouching && boxes.InitialEvent.Relation != ContactBand ||
		boxes.Cause != SweepDepartureUnproved && boxes.Cause != SweepContactTrackUnproved {
		return nil, false, nil
	}
	fresh := &SweepReport{A: boxes.A, B: boxes.B, PathA: boxes.PathA, PathB: boxes.PathB, Request: boxes.Request}
	result, admitted, err := d.sweepPlanarPair(ctx, boxes.A, boxes.B, pa, pb, sweepRequestToInternal(boxes.Request),
		resolution, fresh)
	if err != nil {
		return nil, true, err
	}
	if !admitted || result == nil || result.Outcome == SweepUndecided {
		return nil, false, nil
	}
	return result, true, nil
}

// planarReplayProof records what replay needs from a finished planar run:
// the track, or the departure and every certified clear interval up to the
// end of the clear prefix. It returns nil when the outcome has no replay.
func (r *rotationalPairSweep) planarReplayProof(result *SweepReport) *sweepReplayProof {
	proof := &sweepReplayProof{rotation: &[2]rotationalSweepPath{r.a, r.b},
		request: r.req.ContactRequest, outcome: result.Outcome}
	planar := &planarsweep.Replay{Departure: r.departure,
		Travel: new(big.Rat).Add(r.a.FullTravel, r.b.FullTravel)}
	proof.planar = planar
	start, end := new(big.Rat), big.NewRat(1, 1)
	switch result.Outcome {
	case SweepPersistentTouch, SweepPersistentBand:
		if result.ContactTrack == nil || result.ContactTrack.planar == nil {
			return nil
		}
		proof.track = result.ContactTrack
		return proof
	case SweepImpactBracket:
		if result.Bracket == nil {
			return nil
		}
		left, leftOK := sweeppath.ExactBaseValue(result.Bracket.From.Fraction)
		right, rightOK := sweeppath.ExactBaseValue(result.Bracket.To.Fraction)
		if !leftOK || !rightOK || left.Cmp(right) >= 0 {
			return nil
		}
		proof.setBracket(left, right)
		proof.bracketTravel = planar.Travel
		end = left
	case SweepClear, SweepDepartedClear:
	default:
		return nil
	}
	if r.departure != nil {
		start = r.departure.Until
	}
	samples := append([]SweepSample(nil), result.Samples...)
	sort.Slice(samples, func(i, j int) bool { return samples[i].exactFraction.Cmp(samples[j].exactFraction) < 0 })
	covered := new(big.Rat).Set(start)
	for i := 0; i+1 < len(samples); i++ {
		from, to := samples[i].exactFraction, samples[i+1].exactFraction
		if from.Cmp(start) < 0 || to.Cmp(end) > 0 {
			continue
		}
		if from.Cmp(covered) != 0 {
			return nil
		}
		span := planarsweep.ClearSpan{From: from, To: to, Left: sampleLowerGap(&samples[i]),
			Right: sampleLowerGap(&samples[i+1]), Axis: r.intervalAxisGap(from, to)}
		if !planar.AddSpan(span) {
			return nil
		}
		covered = to
	}
	if covered.Cmp(end) != 0 {
		return nil
	}
	if proof.bracketLo != nil {
		proof.bracketGap = planar.LowerGap(proof.bracketLo)
	}
	return proof
}

// sampleLowerGap is a separated sample's proven positive lower gap, or nil.
func sampleLowerGap(sample *SweepSample) *big.Rat {
	if sample.Ideal.Relation != ContactSeparated || sample.Ideal.Gap == nil {
		return nil
	}
	value, okValue := sweeppath.ExactBaseValue(sample.Ideal.Gap.Value)
	bound, okBound := sweeppath.ExactBaseValue(sample.Ideal.Gap.Bound)
	if !okValue || !okBound {
		return nil
	}
	lower := new(big.Rat).Sub(value, bound)
	if lower.Sign() <= 0 {
		return nil
	}
	return lower
}

// planarIdealEvent transfers the float-pose planar relation to the ideal path
// (contact-sweep §3). With zero deviation on both bodies the rounded pose is
// the ideal pose and the report transfers whole. Otherwise a gap transfers
// with both deviations charged; an overlap transfers only through a vertex of
// one body that lies inside the other farther than both deviations and both
// held displacements at the rounded poses from its boundary
// (planar.PlanarDeepVertex); a touch cannot survive a nonzero deviation and
// stays undecided. A §10.4 band transfers with both deviations added to its
// width and its manifold dropped, as a touch's would be. The ideal pose is a
// rigid motion, so it moves each true body within its δ of its held one; the
// rounded pose's ContactPair report already charged δ, stretched by that
// pose's own scale, which is at least one.
func (r *rotationalPairSweep) planarIdealEvent(ctx context.Context, f *big.Rat, at SweepInstant,
	poseA, poseB r3.Transform, contact *ContactReport) (SweepEvent, error) {
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	budget := proofbound.NewWorkBudget(ctx)
	vertsA, etaA, okA, err := r.a.PointDeviation(poseA, f, budget.Step)
	if err != nil {
		return SweepEvent{}, err
	}
	vertsB, etaB, okB, err := r.b.PointDeviation(poseB, f, budget.Step)
	if err != nil {
		return SweepEvent{}, err
	}
	if !okA || !okB {
		return event, nil
	}
	if etaA == 0 && etaB == 0 && (contact.Relation == ContactTouching ||
		contact.Relation == ContactOverlapping || contact.Relation == ContactBand) {
		event.Relation, event.Gap, event.Overlap = contact.Relation, contact.Gap, contact.Overlap
		event.Manifold = contact.Manifold
		return event, nil
	}
	switch contact.Relation {
	case ContactSeparated:
		if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
			event.Relation, event.Gap, event.Reason = ContactSeparated, gap, ContactNoReason
		}
	case ContactOverlapping:
		if !planarsnapshot.PositiveAffine(poseA) || !planarsnapshot.PositiveAffine(poseB) {
			return event, nil
		}
		margin := proofarith.DyAdd(proofarith.MustDyOf(etaA), proofarith.MustDyOf(etaB))
		margin = proofarith.DyAdd(margin, proofarith.DyMul(r.a.HeldDelta, planarsnapshot.PoseScale(poseA)))
		margin = proofarith.DyAdd(margin, proofarith.DyMul(r.b.HeldDelta, planarsnapshot.PoseScale(poseB)))
		a := planar.PlanarSolid{Verts: vertsA, Tris: r.a.Solid.Tris}
		b := planar.PlanarSolid{Verts: vertsB, Tris: r.b.Solid.Tris}
		deep, err := planar.PlanarDeepVertex(&a, &b, margin, budget.Step)
		if err != nil {
			return SweepEvent{}, err
		}
		if deep {
			event.Relation = ContactOverlapping
		}
	case ContactBand:
		if gap, ok := widenedBand(contact, etaA, etaB); ok {
			event.Relation, event.Gap, event.Reason = ContactBand, gap, ContactNoNormalProof
		}
	}
	return event, nil
}

// widenedBand transfers a ContactBand gap to the ideal path: its width grows
// by both pose deviations.
func widenedBand(contact *ContactReport, etaA, etaB float64) (*Measurement, bool) {
	if contact.Gap == nil {
		return nil, false
	}
	width := new(big.Rat).Add(proofarith.FloatRat(contact.Gap.Bound.Base()),
		new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)))
	published := proofbound.RatFloatUp(width)
	if !measurement.FiniteValues(published) {
		return nil, false
	}
	return &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(published),
		Exactness: exactnessFromBound(published)}, true
}

// The §10.2 departure and §10.3 band certificates live in
// internal/planarsweep (certificate.go). The run below reads them: it finds
// the support planes, records a departure, and publishes a proven band as a
// SweepContactTrack.

// planarSupports lists every support plane of the initial touch
// (planarsweep.FindPathSupports) under the request's support band and rest
// speed.
func (r *rotationalPairSweep) planarSupports(poll func() error) ([]planarsweep.Support, error) {
	rest := new(big.Rat)
	if r.req.RestSpeed != (units.Value{}) {
		speed, ok := sweeppath.ExactBaseValue(r.req.RestSpeed)
		if !ok {
			return nil, nil
		}
		rest = speed
	}
	return planarsweep.FindPathSupports([2]*planarsweep.BodyPath{&r.a.BodyPath, &r.b.BodyPath},
		planarband.SupportBand(r.req.ContactRequest), rest, poll)
}

// planarDepartureFraction proves §10.2's departure (planarsweep.ProveDeparture),
// records it on the run, and returns the largest grid fraction it covers.
func (r *rotationalPairSweep) planarDepartureFraction(ctx context.Context) (*big.Rat, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	supports, err := r.planarSupports(budget.Step)
	if err != nil {
		return nil, false, err
	}
	departure, ok, err := planarsweep.ProveDeparture(supports, r.resolution, r.a.Path.Duration, budget.Step)
	if err != nil || !ok {
		return nil, false, err
	}
	r.departure = departure
	return departure.Until, true, nil
}

// planarBand publishes §10.3's band track on the first support plane, in
// order, that admits one (planarsweep.ProveBand) and whose track publishes.
func (r *rotationalPairSweep) planarBand(ctx context.Context) (*SweepContactTrack, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	supports, err := r.planarSupports(budget.Step)
	if err != nil {
		return nil, false, err
	}
	for i := range supports {
		band, ok, err := planarsweep.ProveBand(&supports[i], r.resolution, r.a.Path.Duration, budget.Step)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		track, ok, err := r.planarTrack(&band)
		if err != nil {
			return nil, false, err
		}
		if ok {
			return track, true, nil
		}
	}
	return nil, false, nil
}

// planarTrack builds the public track of a proven band and checks that its
// manifold publishes at the start. It stays in the root because the feature
// map reads both public bodies and the track is a public type.
func (r *rotationalPairSweep) planarTrack(band *planarsweep.BandHorizon) (*SweepContactTrack, bool, error) {
	features, err := reportvocab.NewPlanarFeatureMap(bodyNode(r.a.body), bodyNode(r.b.body), r.a.Solid, r.b.Solid)
	if err != nil {
		return nil, false, err
	}
	proof, ok := planarsweep.NewTrackProof([2]planarsweep.BodyPath{r.a.BodyPath, r.b.BodyPath}, band, features,
		r.req.NormalResolution)
	if !ok {
		return nil, false, nil
	}
	track := &SweepContactTrack{start: new(big.Rat), end: new(big.Rat).Set(band.End), duration: r.a.Path.Duration,
		request: r.req.ContactRequest, normal: vecMeasurementFromInternal(proof.Direction), planar: proof}
	// A track whose start manifold is refused (a point ball over
	// PointResolution) is not published; the refusal is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	track.features[proof.M], track.features[proof.S] = proof.FeatureM[0], proof.FeatureS
	track.pointCount = len(proof.Contact)
	return track, true, nil
}

// planarManifoldAt publishes a planar track's manifold at an exact fraction
// (planarsweep.TrackProof.Points). The separation is the band's two-sided
// bound.
func planarManifoldAt(p *planarsweep.TrackProof, f *big.Rat, req reportvocab.ContactRequest) (*ContactManifold, error) {
	verts, eta, ok := p.RoundedVertices(f)
	if !ok {
		return nil, fmt.Errorf("%w: planar contact track pose has no finite bound", ErrUnsupported)
	}
	resolution, okResolution := sweeppath.ExactBaseValue(req.PointResolution)
	if !okResolution {
		return nil, fmt.Errorf("%w: planar contact track resolution is invalid", ErrUnsupported)
	}
	readings, ok := p.Points(verts, eta, resolution)
	if !ok {
		return nil, fmt.Errorf("%w: planar contact track point exceeds point resolution", ErrUnsupported)
	}
	separation := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(p.DepthUp),
		Exactness: exactnessFromBound(p.DepthUp)}
	points := make([]ContactPoint, 0, len(readings))
	for slot, reading := range readings {
		onM, onS := vecMeasurementFromInternal(reading.OnM), vecMeasurementFromInternal(reading.OnS)
		point := ContactPoint{Normal: vecMeasurementFromInternal(p.Direction), NormalAngle: p.Angle,
			Separation: separation}
		featureM, featureS := contactFeatureFromInternal(p.FeatureM[slot]), contactFeatureFromInternal(p.FeatureS)
		if p.M == 0 {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = featureM, featureS
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = featureS, featureM
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	return &ContactManifold{Points: points}, nil
}
