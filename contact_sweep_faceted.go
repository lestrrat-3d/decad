package decad

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the general rotating sweep of docs/multibody-dynamics-design.md
// §10.1: rotationalPairSweep's search, travel bound and pose deviation run
// over the exact vertex set of two §9 planar bodies instead of eight source-box
// corners. Each sample's relation and gap come from ContactPair's exact planar
// relation at the rounded pose, and the float-to-ideal deviation is the largest
// outward vertex distance (pointDeviation). Affine paths take the same run with
// a zero angular term.

// sweepPlanarPair runs the general sweep when both bodies are admitted §9
// planar solids. It reports false, leaving report untouched, when either is
// not. A path whose ideal motion or travel bound cannot be formed is
// SweepUndecided with SweepMissingBound.
func (d *Document) sweepPlanarPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	budget := newWorkBudget(ctx)
	solidA, deltaA, okA, err := planarSolidAtPose(ctx, budget, a, r3.Identity())
	if err != nil {
		return nil, true, err
	}
	solidB, deltaB, okB, err := planarSolidAtPose(ctx, budget, b, r3.Identity())
	if err != nil {
		return nil, true, err
	}
	if !okA || !okB {
		return nil, false, nil
	}
	aPath, okA := preparePlanarSweepPath(a, pa, &solidA, deltaA)
	bPath, okB := preparePlanarSweepPath(b, pb, &solidB, deltaB)
	if !okA || !okB {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, true, nil
	}
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
// could not settle an initial touch as two exact planar bodies, whose §10.2
// and §10.3 proofs read every vertex. It reports false, and the box report
// stands, when the planar run is not admitted or settles nothing either.
func (d *Document) planarContinuation(ctx context.Context, boxes *SweepReport,
	pa, pb affinePairPath, resolution *big.Rat) (*SweepReport, bool, error) {
	if boxes.Outcome != SweepUndecided || boxes.InitialEvent == nil ||
		boxes.InitialEvent.Relation != ContactTouching ||
		boxes.Cause != SweepDepartureUnproved && boxes.Cause != SweepContactTrackUnproved {
		return nil, false, nil
	}
	fresh := &SweepReport{A: boxes.A, B: boxes.B, PathA: boxes.PathA, PathB: boxes.PathB, Request: boxes.Request}
	result, admitted, err := d.sweepPlanarPair(ctx, boxes.A, boxes.B, pa, pb, boxes.Request, resolution, fresh)
	if err != nil {
		return nil, true, err
	}
	if !admitted || result == nil || result.Outcome == SweepUndecided {
		return nil, false, nil
	}
	return result, true, nil
}

// planarReplay is the replay certificate of a planar sweep. It never reruns
// the pair relation: a rounded pose is accepted when its vertex deviation from
// the ideal path stays below a lower bound on the ideal separation that the
// sweep already proved, so each replay costs one pass over both vertex sets.
type planarReplay struct {
	departure *planarDepartureProof // (0, until], when the sweep departed
	spans     []planarClearSpan     // the certified clear intervals, in order
	travel    *big.Rat              // both bodies' travel bound per unit fraction
}

// planarClearSpan is one interval the search certified clear: by its two
// ends' lower gaps and the §4.3 travel bound, or by separated vertex hulls.
type planarClearSpan struct {
	from, to    *big.Rat
	left, right *big.Rat // lower gaps at the two ends, nil when unproved
	axis        *big.Rat // hull gap over the span, nil when the hulls meet
}

// lowerGap bounds the ideal separation at f from below, or returns nil when
// no recorded certificate covers f.
func (p *planarReplay) lowerGap(f *big.Rat) *big.Rat {
	if p.departure != nil && f.Sign() > 0 && f.Cmp(p.departure.until) <= 0 {
		return p.departure.lowerGap(f)
	}
	for _, span := range p.spans {
		if f.Cmp(span.from) < 0 || f.Cmp(span.to) > 0 {
			continue
		}
		var best *big.Rat
		candidates := []*big.Rat{span.axis}
		if span.left != nil {
			candidates = append(candidates, new(big.Rat).Sub(span.left,
				new(big.Rat).Mul(new(big.Rat).Sub(f, span.from), p.travel)))
		}
		if span.right != nil {
			candidates = append(candidates, new(big.Rat).Sub(span.right,
				new(big.Rat).Mul(new(big.Rat).Sub(span.to, f), p.travel)))
		}
		for _, candidate := range candidates {
			if candidate != nil && (best == nil || candidate.Cmp(best) > 0) {
				best = candidate
			}
		}
		return best
	}
	return nil
}

// planarReplayProof records what replay needs from a finished planar run:
// the track, or the departure and every certified clear interval up to the
// end of the clear prefix. It returns nil when the outcome has no replay.
func (r *rotationalPairSweep) planarReplayProof(result *SweepReport) *sweepReplayProof {
	proof := &sweepReplayProof{rotation: &[2]rotationalSweepPath{r.a, r.b},
		request: r.req.ContactRequest, outcome: result.Outcome}
	planar := &planarReplay{departure: r.departure,
		travel: new(big.Rat).Add(r.a.fullTravel, r.b.fullTravel)}
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
		left, leftOK := exactBaseValue(result.Bracket.From.Fraction)
		right, rightOK := exactBaseValue(result.Bracket.To.Fraction)
		if !leftOK || !rightOK || left.Cmp(right) >= 0 {
			return nil
		}
		proof.setBracket(left, right)
		end = left
	case SweepClear, SweepDepartedClear:
	default:
		return nil
	}
	if r.departure != nil {
		start = r.departure.until
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
		span := planarClearSpan{from: from, to: to, left: sampleLowerGap(&samples[i]),
			right: sampleLowerGap(&samples[i+1]), axis: r.intervalAxisGap(from, to)}
		travel := new(big.Rat).Mul(new(big.Rat).Sub(to, from), planar.travel)
		if span.axis == nil && (span.left == nil || span.right == nil ||
			new(big.Rat).Add(span.left, span.right).Cmp(travel) <= 0) {
			return nil
		}
		planar.spans = append(planar.spans, span)
		covered = to
	}
	if covered.Cmp(end) != 0 {
		return nil
	}
	return proof
}

// sampleLowerGap is a separated sample's proven positive lower gap, or nil.
func sampleLowerGap(sample *SweepSample) *big.Rat {
	if sample.Ideal.Relation != ContactSeparated || sample.Ideal.Gap == nil {
		return nil
	}
	value, okValue := exactBaseValue(sample.Ideal.Gap.Value)
	bound, okBound := exactBaseValue(sample.Ideal.Gap.Bound)
	if !okValue || !okBound {
		return nil
	}
	lower := new(big.Rat).Sub(value, bound)
	if lower.Sign() <= 0 {
		return nil
	}
	return lower
}

// preparePlanarSweepPath pairs a body's ideal motion with its identity-pose
// planar snapshot and its held displacement δ (§10.4, zero for an exact
// body). The snapshot's vertices are the source points: ContactPair stages
// the same vertices through the query pose, and the held body is their hull's
// subset, so they bound its deviation (pointDeviation) and its coordinate span
// (cornerSpan); the true body lies within δ of that hull.
func preparePlanarSweepPath(body *Body, path affinePairPath, solid *pair.PlanarSolid,
	delta proofarith.Dyadic) (rotationalSweepPath, bool) {
	prepared, ok := prepareSweepMotion(body, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.solid, prepared.delta = solid, delta
	prepared.sourcePoints = solid.Verts
	prepared.startPoints = make([]proofarith.DyV3, len(solid.Verts))
	for i, v := range solid.Verts {
		prepared.startPoints[i] = exactContactTransform(path.from, v)
	}
	return prepared, true
}

// planarIdealEvent transfers the float-pose planar relation to the ideal path
// (contact-sweep §3). With zero deviation on both bodies the rounded pose is
// the ideal pose and the report transfers whole. Otherwise a gap transfers
// with both deviations charged; an overlap transfers only through a vertex of
// one body that lies inside the other farther than both deviations and both
// held displacements at the rounded poses from its boundary
// (pair.PlanarDeepVertex); a touch cannot survive a nonzero deviation and
// stays undecided. A §10.4 band transfers with both deviations added to its
// width and its manifold dropped, as a touch's would be. The ideal pose is a
// rigid motion, so it moves each true body within its δ of its held one; the
// rounded pose's ContactPair report already charged δ, stretched by that
// pose's own scale, which is at least one.
func (r *rotationalPairSweep) planarIdealEvent(ctx context.Context, f *big.Rat, at SweepInstant,
	poseA, poseB r3.Transform, contact *ContactReport) (SweepEvent, error) {
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	budget := newWorkBudget(ctx)
	vertsA, etaA, okA, err := r.a.pointDeviation(poseA, f, budget.step)
	if err != nil {
		return SweepEvent{}, err
	}
	vertsB, etaB, okB, err := r.b.pointDeviation(poseB, f, budget.step)
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
		if !positiveAffine(poseA) || !positiveAffine(poseB) {
			return event, nil
		}
		margin := proofarith.DyAdd(proofarith.MustDyOf(etaA), proofarith.MustDyOf(etaB))
		margin = proofarith.DyAdd(margin, proofarith.DyMul(r.a.delta, planarPoseScale(poseA)))
		margin = proofarith.DyAdd(margin, proofarith.DyMul(r.b.delta, planarPoseScale(poseB)))
		a := pair.PlanarSolid{Verts: vertsA, Tris: r.a.solid.Tris}
		b := pair.PlanarSolid{Verts: vertsB, Tris: r.b.solid.Tris}
		deep, err := pair.PlanarDeepVertex(&a, &b, margin, budget.step)
		if err != nil {
			return SweepEvent{}, err
		}
		if deep {
			event.Relation = ContactOverlapping
		}
	case ContactBand:
		if contact.Gap == nil {
			return event, nil
		}
		width := new(big.Rat).Add(proofarith.FloatRat(contact.Gap.Bound.Base()),
			new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)))
		published := ratFloatUp(width)
		if !finiteMeasurementValues(published) {
			return event, nil
		}
		event.Relation, event.Reason = ContactBand, ContactNoNormalProof
		event.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(published),
			Exactness: exactnessFromBound(published)}
	}
	return event, nil
}
