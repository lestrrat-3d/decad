package decad

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rotationalSweepPath is one body's prepared path (planarsweep.BodyPath)
// with the public body ContactPair reads and, for a source box, its start box
// with the source face identities. It stays in the root because both are
// public handles.
type rotationalSweepPath struct {
	planarsweep.BodyPath
	body     *Body
	startBox orientedSourceBox
}

func prepareRotationalSweepPath(body *Body, path affinePairPath) (rotationalSweepPath, bool) {
	startBox, ok := sourceOrientedBoxAtPose(body, path.From)
	if !ok {
		return rotationalSweepPath{}, false
	}
	sourceBox, ok := sourceOrientedBoxAtPose(body, r3.Identity())
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared, ok := planarsweep.PrepareBodyPath(&body.n, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.StartPoints = append([]proofarith.DyV3(nil), startBox.Corner[:]...)
	prepared.SourcePoints = append([]proofarith.DyV3(nil), sourceBox.Corner[:]...)
	return rotationalSweepPath{BodyPath: prepared, body: body, startBox: startBox}, true
}

type rotationalPairSweep struct {
	doc        *Document
	a, b       rotationalSweepPath
	req        reportvocab.SweepRequest
	resolution *big.Rat
	report     *SweepReport
	planar     bool                        // both paths carry exact planar vertex sets (contact_sweep_planar.go)
	departure  *planarsweep.DepartureProof // a planar pair's §10.2 departure, when proved
}

func (d *Document) sweepRotatingPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req reportvocab.SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, error) {
	aPath, okA := prepareRotationalSweepPath(a, pa)
	bPath, okB := prepareRotationalSweepPath(b, pb)
	if !okA || !okB {
		if result, admitted, err := d.sweepPlanarPair(ctx, a, b, pa, pb, req, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
			To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	planarsweep.AttachMemos(&aPath.BodyPath, &bPath.BodyPath)
	defer planarsweep.CloseMemos(&aPath.BodyPath, &bPath.BodyPath)
	run := rotationalPairSweep{doc: d, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, err
	}
	if planar, ok, planarErr := d.planarContinuation(ctx, result, pa, pb, resolution); planarErr != nil || ok {
		return planar, planarErr
	}
	if result.Outcome == SweepClear || result.Outcome == SweepDepartedClear ||
		result.Outcome == SweepImpactBracket && result.Bracket != nil ||
		result.Outcome == SweepPersistentTouch &&
			result.ContactTrack != nil && result.ContactTrack.orientedA != nil &&
			result.ContactTrack.orientedB != nil && aPath.Path.Drift == nil && bPath.Path.Drift == nil {
		result.replay = &sweepReplayProof{rotation: &[2]rotationalSweepPath{aPath, bPath},
			track: result.ContactTrack, request: req.ContactRequest}
		result.replay.snapshot(result)
		if result.Outcome == SweepImpactBracket {
			left, leftOK := sweeppath.ExactBaseValue(result.Bracket.From.Fraction)
			right, rightOK := sweeppath.ExactBaseValue(result.Bracket.To.Fraction)
			if !leftOK || !rightOK || left.Cmp(right) >= 0 {
				result.replay = nil
			} else {
				result.replay.setBracket(left, right)
				result.replay.setRotatingBracketGap(result, new(big.Rat).Add(aPath.FullTravel, bPath.FullTravel))
			}
		}
	}
	return result, nil
}

func (r *rotationalPairSweep) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.a.Path.RoundedPoseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.b.Path.RoundedPoseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a.body, r.b.body, poseA, poseB, contactRequestFromInternal(r.req.ContactRequest))
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.a.Path.Duration)
	var event SweepEvent
	if r.planar {
		event, err = r.planarIdealEvent(ctx, f, at, poseA, poseB, contact)
		if err != nil {
			return nil, err
		}
	} else {
		event = r.orientedIdealEvent(f, at, poseA, poseB, contact)
	}
	sample := &SweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: contact, Ideal: event, exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

// separatedIdealGap transfers a float-pose gap to the ideal path: its bound
// grows by both pose deviations, and the result must stay strictly positive.
func separatedIdealGap(contact *ContactReport, etaA, etaB float64) (*Measurement, bool) {
	if contact.Gap == nil {
		return nil, false
	}
	value, okValue := sweeppath.ExactBaseValue(contact.Gap.Value)
	bound, okBound := sweeppath.ExactBaseValue(contact.Gap.Bound)
	if !okValue || !okBound {
		return nil, false
	}
	charge := new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB))
	newBound := new(big.Rat).Add(bound, charge)
	if new(big.Rat).Sub(value, newBound).Sign() <= 0 {
		return nil, false
	}
	published := proofbound.RatFloatUp(newBound)
	if !measurement.FiniteValues(published) || value.Cmp(proofarith.FloatRat(published)) <= 0 {
		return nil, false
	}
	gap := *contact.Gap
	gap.Bound = units.Millimeters(published)
	gap.Exactness = exactnessFromBound(published)
	return &gap, true
}

// orientedIdealEvent transfers a source-box pair's float-pose relation to the
// ideal path through the rounded source boxes.
func (r *rotationalPairSweep) orientedIdealEvent(f *big.Rat, at SweepInstant,
	poseA, poseB r3.Transform, contact *ContactReport) SweepEvent {
	boxA, etaA, okA := r.a.RoundedBox(poseA, f)
	boxB, etaB, okB := r.b.RoundedBox(poseB, f)
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	if okA && okB {
		switch contact.Relation {
		case ContactSeparated:
			if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = ContactSeparated, gap, ContactNoReason
			}
		case ContactOverlapping:
			if pairbox.OrientedInteriorWitness(boxA, boxB,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)) {
				event.Relation, event.Reason = ContactOverlapping, contact.Reason
			}
			if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case ContactTouching:
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = ContactTouching, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case ContactBand:
			// §10.5: the exact planar path's band of two boxes apart within the
			// SupportBand transfers whole at zero deviation, and otherwise, as
			// planarIdealEvent's does, with both deviations added to its width
			// and its manifold dropped.
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = ContactBand, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if gap, ok := widenedBand(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = ContactBand, gap, ContactNoNormalProof
			}
		}
	}
	return event
}

// horizontalSpinContact uses the invariant support height of a Z-axis spin.
// The read face patch must remain strictly inside the stationary face after
// charging both pose deviations. Exact ideal support heights decide the event.
func (r *rotationalPairSweep) horizontalSpinContact(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, etaA, etaB *big.Rat) (SweepEvent, bool) {
	if contact.Manifold == nil || len(contact.Manifold.Points) != 4 {
		return SweepEvent{}, false
	}
	stationary, spinning := -1, -1
	paths := [2]rotationalSweepPath{r.a, r.b}
	for i, path := range paths {
		if path.Path.Drift == nil && path.Path.Delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.Path.Drift != nil && path.Frame.Axis[0].Sign() == 0 &&
			path.Frame.Axis[1].Sign() == 0 && path.Frame.Axis[2].Sign() != 0 &&
			path.Velocity[0].Sign() == 0 && path.Velocity[1].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return SweepEvent{}, false
	}
	poses := [2]r3.Transform{poseA, poseB}
	base, okBase := sourceBoxAtPose(paths[stationary].body, poses[stationary])
	startA, okA := sourceBoxAtPose(r.a.body, r.a.Path.From)
	startB, okB := sourceBoxAtPose(r.b.body, r.b.Path.From)
	if !okBase || !okA || !okB {
		return SweepEvent{}, false
	}
	normal := contact.Manifold.Points[0].Normal.Value
	if normal != (r3.Vec{Z: 1}) && normal != (r3.Vec{Z: -1}) {
		return SweepEvent{}, false
	}
	elapsed := new(big.Rat).Mul(r.a.Path.Duration, f)
	travel := new(big.Rat).Mul(paths[spinning].Velocity[2], elapsed)
	loA, hiA := startA.lo[2].Rat(), startA.hi[2].Rat()
	loB, hiB := startB.lo[2].Rat(), startB.hi[2].Rat()
	if spinning == 0 {
		loA.Add(loA, travel)
		hiA.Add(hiA, travel)
	} else {
		loB.Add(loB, travel)
		hiB.Add(hiB, travel)
	}
	gap := new(big.Rat)
	if normal.Z > 0 {
		gap.Sub(loB, hiA)
		if loA.Cmp(loB) >= 0 || hiA.Cmp(hiB) >= 0 {
			return SweepEvent{}, false
		}
	} else {
		gap.Sub(loA, hiB)
		if loB.Cmp(loA) >= 0 || hiB.Cmp(hiA) >= 0 {
			return SweepEvent{}, false
		}
	}
	if gap.Sign() > 0 {
		return SweepEvent{}, false
	}
	value := sweeppath.RatFloatNearest(gap)
	bound := proofarith.RationalFloatError(gap, value)
	if !measurement.FiniteValues(value, bound) {
		return SweepEvent{}, false
	}
	deviation := new(big.Rat).Add(etaA, etaB)
	points := append([]ContactPoint(nil), contact.Manifold.Points...)
	for i := range points {
		point := &points[i]
		if point.Normal.Value != normal || point.Normal.Bound.Base() != 0 ||
			point.NormalAngle.Base() != 0 {
			return SweepEvent{}, false
		}
		witness := point.OnA
		if spinning == 1 {
			witness = point.OnB
		}
		margin := new(big.Rat).Add(deviation, proofarith.FloatRat(witness.Bound.Base()))
		margin.Sub(margin, gap)
		coordinates := [2]float64{witness.Value.X, witness.Value.Y}
		for axis := range 2 {
			value := proofarith.FloatRat(coordinates[axis])
			if value.Cmp(new(big.Rat).Add(base.lo[axis].Rat(), margin)) <= 0 ||
				value.Cmp(new(big.Rat).Sub(base.hi[axis].Rat(), margin)) >= 0 {
				return SweepEvent{}, false
			}
		}
		for _, position := range []*VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), deviation)
			if bound.Cmp(proofarith.FloatRat(r.req.PointResolution.Base())) > 0 {
				return SweepEvent{}, false
			}
			published := proofbound.RatFloatUp(bound)
			if !measurement.FiniteValues(published) || published > r.req.PointResolution.Base() {
				return SweepEvent{}, false
			}
			position.Bound = units.Millimeters(published)
			position.Exactness = exactnessFromBound(position.Bound.Base())
		}
		point.Separation = Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessFromBound(bound)}
	}
	zero := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
	event := SweepEvent{At: sweepInstant(f, r.a.Path.Duration), Manifold: &ContactManifold{Points: points}}
	if gap.Sign() == 0 {
		event.Relation, event.Gap = ContactTouching, &zero
	} else {
		event.Relation = ContactOverlapping
	}
	return event, true
}

func (r *rotationalPairSweep) execute(ctx context.Context) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case ContactOverlapping:
		r.report.Outcome, r.report.InitialEvent, r.report.Event =
			SweepInitiallyOverlapping, &first.Ideal, &first.Ideal
		return r.report, nil
	case ContactTouching:
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == reportvocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if !r.planar && r.req.StartPolicy == reportvocab.ContinueCertifiedTouch && r.coMovingOrientedTouch(first) {
			last, sampleErr := r.sample(ctx, one)
			if errors.Is(sampleErr, errSweepPoseBudget) {
				return r.undecided(zero, one, SweepPoseBudget), nil
			}
			if sampleErr != nil {
				return nil, sampleErr
			}
			if last.Ideal.Relation != ContactTouching || last.Ideal.Manifold == nil {
				return r.undecided(zero, one, SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack = &SweepContactTrack{orientedA: &r.a.startBox,
				orientedB: &r.b.startBox, orientedDelta: r.a.Path.Delta,
				start: zero, end: one,
				duration: r.a.Path.Duration, request: r.req.ContactRequest,
				pointCount: len(first.Ideal.Manifold.Points),
				features: [2]reportvocab.ContactFeature{contactFeatureToInternal(first.Ideal.Manifold.Points[0].FeatureA),
					contactFeatureToInternal(first.Ideal.Manifold.Points[0].FeatureB)},
				normal: first.Ideal.Manifold.Points[0].Normal}
			r.report.Outcome = SweepPersistentTouch
			r.sortSamples()
			return r.report, nil
		}
		var departure *big.Rat
		var departs bool
		if r.planar {
			departure, departs, err = r.planarDepartureFraction(ctx)
			if err != nil {
				return nil, err
			}
		} else {
			departure, departs = r.rotationalDepartureFraction(first)
		}
		if departs {
			return r.depart(ctx, departure)
		}
		if r.planar && r.req.StartPolicy == reportvocab.ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
				if track.planar.Band == nil {
					r.report.Outcome = SweepPersistentTouch
				}
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
	case ContactBand:
		// §10.4: a positive-displacement pair that starts inside its band is
		// an initial contact. No departure is published, since the true pair
		// may stay inside the band however fast the held pair separates; a
		// certified continuation is §10.3's band track over the held touch.
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == reportvocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		// §10.5: an exact pair apart within its SupportBand has every vertex
		// above the support plane, so it departs when §10.2's proof holds.
		if r.planar && r.req.StartPolicy == reportvocab.ContinueSeparatingTouch &&
			r.a.HeldDelta.Sign() == 0 && r.b.HeldDelta.Sign() == 0 {
			departure, departs, err := r.planarDepartureFraction(ctx)
			if err != nil {
				return nil, err
			}
			if departs {
				return r.depart(ctx, departure)
			}
		}
		if r.planar && r.req.StartPolicy == reportvocab.ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
	case ContactSeparated:
	default:
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, one, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	done, err := r.refine(ctx, first, last, 0)
	if err != nil {
		return nil, err
	}
	if !done {
		r.report.Outcome = SweepClear
	}
	r.sortSamples()
	return r.report, nil
}
