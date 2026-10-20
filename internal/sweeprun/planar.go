package sweeprun

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/planarfeaturemap"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// body takes sweepRollingPair (internal/sweeprun/rolling.go); any other pair
// reports false, leaving report untouched. A path whose ideal motion or
// travel bound cannot be formed is SweepUndecided with SweepMissingBound.
func sweepPlanarPair(ctx context.Context, h Hooks, a, b *topo.Body,
	pa, pb affinePairPath, req vocab.SweepRequest, resolution *big.Rat,
	report *sweepReport) (*sweepReport, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	chord := req.HeldChord.Base()
	solidA, deltaA, okA, err := planarpose.SolidAtPose(ctx, budget, h.Mesher, a, r3.Identity(), chord)
	if err != nil {
		return nil, true, err
	}
	solidB, deltaB, okB, err := planarpose.SolidAtPose(ctx, budget, h.Mesher, b, r3.Identity(), chord)
	if err != nil {
		return nil, true, err
	}
	if !okA || !okB {
		return sweepRollingPair(ctx, h, a, b, pa, pb, req, resolution, report)
	}
	planarA, okA := planarsweep.PreparePlanarPath(a, pa, &solidA, deltaA)
	planarB, okB := planarsweep.PreparePlanarPath(b, pb, &solidB, deltaB)
	if !okA || !okB {
		report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepMissingBound
		report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), pa.Duration),
			To: sweeppath.Instant(big.NewRat(1, 1), pa.Duration)}
		return report, true, nil
	}
	aPath, bPath := rotationalSweepPath{RotationalPath: sweepproof.RotationalPath{BodyPath: planarA}, body: a}, rotationalSweepPath{RotationalPath: sweepproof.RotationalPath{BodyPath: planarB}, body: b}
	planarsweep.AttachMemos(&aPath.BodyPath, &bPath.BodyPath)
	defer planarsweep.CloseMemos(&aPath.BodyPath, &bPath.BodyPath)
	run := rotationalPairSweep{hooks: h, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report, planar: true}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, true, err
	}
	result.SetReplay(seal.Key{}, run.planarReplayProof(result))
	return result, true, nil
}

// planarContinuation reruns a source-box pair whose box continuation proofs
// could not settle an initial touch, or a §10.5 ContactBand start, as two
// exact planar bodies, whose §10.2 and §10.3 proofs read every vertex. It
// reports false, and the box report stands, when the planar run is not
// admitted or settles nothing either.
func planarContinuation(ctx context.Context, h Hooks, boxes *sweepReport,
	pa, pb affinePairPath, resolution *big.Rat) (*sweepReport, bool, error) {
	if boxes.Outcome != vocab.SweepUndecided || boxes.InitialEvent == nil ||
		boxes.InitialEvent.Relation != vocab.ContactTouching && boxes.InitialEvent.Relation != vocab.ContactBand ||
		boxes.Cause != vocab.SweepDepartureUnproved && boxes.Cause != vocab.SweepContactTrackUnproved {
		return nil, false, nil
	}
	fresh := &sweepReport{A: boxes.A, B: boxes.B, PathA: boxes.PathA, PathB: boxes.PathB, Request: boxes.Request}
	result, admitted, err := sweepPlanarPair(ctx, h, boxes.A, boxes.B, pa, pb, boxes.Request,
		resolution, fresh)
	if err != nil {
		return nil, true, err
	}
	if !admitted || result == nil || result.Outcome == vocab.SweepUndecided {
		return nil, false, nil
	}
	return result, true, nil
}

// planarReplayProof records what replay needs from a finished planar run:
// the track, or the departure and every certified clear interval up to the
// end of the clear prefix. It returns nil when the outcome has no replay.
func (r *rotationalPairSweep) planarReplayProof(result *sweepReport) *sweepproof.Proof {
	proof := &sweepproof.Proof{Rotation: &[2]sweepproof.RotationalPath{r.a.RotationalPath, r.b.RotationalPath},
		Request: r.req.ContactRequest, Outcome: result.Outcome}
	planar := &planarsweep.Replay{Departure: r.departure,
		Travel: new(big.Rat).Add(r.a.FullTravel, r.b.FullTravel)}
	proof.Planar = planar
	start, end := new(big.Rat), big.NewRat(1, 1)
	switch result.Outcome {
	case vocab.SweepPersistentTouch, vocab.SweepPersistentBand:
		if result.ContactTrack == nil || trackOf(result.ContactTrack).Planar == nil {
			return nil
		}
		proof.Track = trackOf(result.ContactTrack)
		return proof
	case vocab.SweepImpactBracket:
		if result.Bracket == nil {
			return nil
		}
		left, leftOK := sweeppath.ExactBaseValue(result.Bracket.From.Fraction)
		right, rightOK := sweeppath.ExactBaseValue(result.Bracket.To.Fraction)
		if !leftOK || !rightOK || left.Cmp(right) >= 0 {
			return nil
		}
		proof.SetBracket(left, right)
		proof.BracketTravel = planar.Travel
		end = left
	case vocab.SweepClear, vocab.SweepDepartedClear:
	default:
		return nil
	}
	if r.departure != nil {
		start = r.departure.Until
	}
	samples := append([]sweepSample(nil), result.Samples...)
	sort.Slice(samples, func(i, j int) bool {
		return samples[i].ExactFraction(seal.Key{}).Cmp(samples[j].ExactFraction(seal.Key{})) < 0
	})
	covered := new(big.Rat).Set(start)
	for i := 0; i+1 < len(samples); i++ {
		from, to := samples[i].ExactFraction(seal.Key{}), samples[i+1].ExactFraction(seal.Key{})
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
	if proof.BracketLo != nil {
		proof.BracketGap = planar.LowerGap(proof.BracketLo)
	}
	return proof
}

// sampleLowerGap is a separated sample's proven positive lower gap, or nil.
func sampleLowerGap(sample *sweepSample) *big.Rat {
	if sample.Ideal.Relation != vocab.ContactSeparated || sample.Ideal.Gap == nil {
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
func (r *rotationalPairSweep) planarIdealEvent(ctx context.Context, f *big.Rat, at vocab.SweepInstant,
	poseA, poseB r3.Transform, contact *contactReport) (sweepEvent, error) {
	event := sweepEvent{At: at, Relation: vocab.ContactUndecided, Reason: contact.Reason}
	budget := proofbound.NewWorkBudget(ctx)
	vertsA, etaA, okA, err := r.a.PointDeviation(poseA, f, budget.Step)
	if err != nil {
		return sweepEvent{}, err
	}
	vertsB, etaB, okB, err := r.b.PointDeviation(poseB, f, budget.Step)
	if err != nil {
		return sweepEvent{}, err
	}
	if !okA || !okB {
		return event, nil
	}
	if etaA == 0 && etaB == 0 && (contact.Relation == vocab.ContactTouching ||
		contact.Relation == vocab.ContactOverlapping || contact.Relation == vocab.ContactBand) {
		event.Relation, event.Gap, event.Overlap = contact.Relation, contact.Gap, contact.Overlap
		event.Manifold = contact.Manifold
		return event, nil
	}
	switch contact.Relation {
	case vocab.ContactSeparated:
		if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
			event.Relation, event.Gap, event.Reason = vocab.ContactSeparated, gap, vocab.ContactNoReason
		}
	case vocab.ContactOverlapping:
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
			return sweepEvent{}, err
		}
		if deep {
			event.Relation = vocab.ContactOverlapping
		}
	case vocab.ContactBand:
		if gap, ok := widenedBand(contact, etaA, etaB); ok {
			event.Relation, event.Gap, event.Reason = vocab.ContactBand, gap, vocab.ContactNoNormalProof
		}
	}
	return event, nil
}

// widenedBand transfers a ContactBand gap to the ideal path: its width grows
// by both pose deviations.
func widenedBand(contact *contactReport, etaA, etaB float64) (*vocab.Measurement, bool) {
	if contact.Gap == nil {
		return nil, false
	}
	width := new(big.Rat).Add(proofarith.FloatRat(contact.Gap.Bound.Base()),
		new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)))
	published := proofbound.RatFloatUp(width)
	if !measurement.FiniteValues(published) {
		return nil, false
	}
	return &vocab.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(published),
		Exactness: measurement.ExactnessOf(published)}, true
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
func (r *rotationalPairSweep) planarBand(ctx context.Context) (*sweepproof.Track, bool, error) {
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
func (r *rotationalPairSweep) planarTrack(band *planarsweep.BandHorizon) (*sweepproof.Track, bool, error) {
	features, err := planarfeaturemap.NewPlanarFeatureMap(r.a.body, r.b.body, r.a.Solid, r.b.Solid)
	if err != nil {
		return nil, false, err
	}
	proof, ok := planarsweep.NewTrackProof([2]planarsweep.BodyPath{r.a.BodyPath, r.b.BodyPath}, band, features,
		r.req.NormalResolution)
	if !ok {
		return nil, false, nil
	}
	track := &sweepproof.Track{From: new(big.Rat), To: new(big.Rat).Set(band.End), Duration: r.a.Path.Duration,
		Request: r.req.ContactRequest, NormalRead: proof.Direction, Planar: proof}
	// A track whose start manifold is refused (a point ball over
	// PointResolution) is not published; the refusal is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	track.Sources[proof.M], track.Sources[proof.S] = proof.FeatureM[0], proof.FeatureS
	track.PointCount = len(proof.Contact)
	return track, true, nil
}
