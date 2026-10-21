package sweeprun

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rotationalSweepPath is one body's prepared path (planarsweep.BodyPath)
// with the public body ContactPair reads and, for a source box, its start box
// with the source face identities. It stays in the root because both are
// public handles.
type rotationalSweepPath struct {
	sweepproof.RotationalPath
	body *topo.Body
}

func prepareRotationalSweepPath(body *topo.Body, path affinePairPath) (rotationalSweepPath, bool) {
	startBox, ok := contactbox.SourceOrientedBoxAtPose(body, path.From)
	if !ok {
		return rotationalSweepPath{}, false
	}
	sourceBox, ok := contactbox.SourceOrientedBoxAtPose(body, r3.Identity())
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared, ok := planarsweep.PrepareBodyPath(body, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.StartPoints = append([]proofarith.DyV3(nil), startBox.Corner[:]...)
	prepared.SourcePoints = append([]proofarith.DyV3(nil), sourceBox.Corner[:]...)
	return rotationalSweepPath{RotationalPath: sweepproof.RotationalPath{BodyPath: prepared, StartBox: startBox}, body: body}, true
}

type rotationalPairSweep struct {
	hooks      Hooks
	a, b       rotationalSweepPath
	req        vocab.SweepRequest
	resolution *big.Rat
	report     *sweepReport
	planar     bool                        // both paths carry exact planar vertex sets (internal/sweeprun/planar.go)
	departure  *planarsweep.DepartureProof // a planar pair's §10.2 departure, when proved
}

func sweepRotatingPair(ctx context.Context, h Hooks, a, b *topo.Body,
	pa, pb affinePairPath, req vocab.SweepRequest, resolution *big.Rat,
	report *sweepReport) (*sweepReport, error) {
	aPath, okA := prepareRotationalSweepPath(a, pa)
	bPath, okB := prepareRotationalSweepPath(b, pb)
	if !okA || !okB {
		if result, admitted, err := sweepPlanarPair(ctx, h, a, b, pa, pb, req, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepMissingBound
		report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), pa.Duration),
			To: sweeppath.Instant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	planarsweep.AttachMemos(&aPath.BodyPath, &bPath.BodyPath)
	defer planarsweep.CloseMemos(&aPath.BodyPath, &bPath.BodyPath)
	run := rotationalPairSweep{hooks: h, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, err
	}
	if planar, ok, planarErr := planarContinuation(ctx, h, result, pa, pb, resolution); planarErr != nil || ok {
		return planar, planarErr
	}
	if result.Outcome == vocab.SweepClear || result.Outcome == vocab.SweepDepartedClear ||
		result.Outcome == vocab.SweepImpactBracket && result.Bracket != nil ||
		result.Outcome == vocab.SweepPersistentTouch &&
			result.ContactTrack != nil && trackOf(result.ContactTrack).OrientedA != nil &&
			trackOf(result.ContactTrack).OrientedB != nil && aPath.Path.Drift == nil && bPath.Path.Drift == nil {
		result.SetReplay(seal.Key{}, &sweepproof.Proof{Rotation: &[2]sweepproof.RotationalPath{aPath.RotationalPath, bPath.RotationalPath},
			Track: trackOf(result.ContactTrack), Request: req.ContactRequest})
		result.Replay(seal.Key{}).Snapshot(result.Outcome)
		if result.Outcome == vocab.SweepImpactBracket {
			left, leftOK := sweeppath.ExactBaseValue(result.Bracket.From.Fraction)
			right, rightOK := sweeppath.ExactBaseValue(result.Bracket.To.Fraction)
			if !leftOK || !rightOK || left.Cmp(right) >= 0 {
				result.SetReplay(seal.Key{}, nil)
			} else {
				result.Replay(seal.Key{}).SetBracket(left, right)
				sweepproof.SetRotatingBracketGap(result.Replay(seal.Key{}), result.Samples, new(big.Rat).Add(aPath.FullTravel, bPath.FullTravel))
			}
		}
	}
	return result, nil
}

func (r *rotationalPairSweep) sample(ctx context.Context, f *big.Rat) (*sweepSample, error) {
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
	contact, err := r.hooks.Contact(ctx, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweeppath.Instant(f, r.a.Path.Duration)
	var event sweepEvent
	if r.planar {
		event, err = r.planarIdealEvent(ctx, f, at, poseA, poseB, contact)
		if err != nil {
			return nil, err
		}
	} else {
		event = r.orientedIdealEvent(f, at, poseA, poseB, contact)
	}
	sample := sampleWithPtr(sweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: contact, Ideal: event}, new(big.Rat).Set(f))
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

// separatedIdealGap transfers a float-pose gap to the ideal path: its bound
// grows by both pose deviations, and the result must stay strictly positive.
func separatedIdealGap(contact *contactReport, etaA, etaB float64) (*vocab.Measurement, bool) {
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
	gap.Exactness = measurement.ExactnessOf(published)
	return &gap, true
}

// orientedIdealEvent transfers a source-box pair's float-pose relation to the
// ideal path through the rounded source boxes.
func (r *rotationalPairSweep) orientedIdealEvent(f *big.Rat, at vocab.SweepInstant,
	poseA, poseB r3.Transform, contact *contactReport) sweepEvent {
	boxA, etaA, okA := r.a.RoundedBox(poseA, f)
	boxB, etaB, okB := r.b.RoundedBox(poseB, f)
	event := sweepEvent{At: at, Relation: vocab.ContactUndecided, Reason: contact.Reason}
	if okA && okB {
		switch contact.Relation {
		case vocab.ContactSeparated:
			if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = vocab.ContactSeparated, gap, vocab.ContactNoReason
			}
		case vocab.ContactOverlapping:
			if pairbox.OrientedInteriorWitness(boxA, boxB,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)) {
				event.Relation, event.Reason = vocab.ContactOverlapping, contact.Reason
			}
			if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case vocab.ContactTouching:
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = vocab.ContactTouching, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case vocab.ContactBand:
			// §10.5: the exact planar path's band of two boxes apart within the
			// SupportBand transfers whole at zero deviation, and otherwise, as
			// planarIdealEvent's does, with both deviations added to its width
			// and its manifold dropped.
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = vocab.ContactBand, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if gap, ok := widenedBand(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = vocab.ContactBand, gap, vocab.ContactNoNormalProof
			}
		}
	}
	return event
}

// horizontalSpinContact uses the invariant support height of a Z-axis spin.
// The read face patch must remain strictly inside the stationary face after
// charging both pose deviations. Exact ideal support heights decide the event.
func (r *rotationalPairSweep) horizontalSpinContact(f *big.Rat, poseA, poseB r3.Transform,
	contact *contactReport, etaA, etaB *big.Rat) (sweepEvent, bool) {
	if contact.Manifold == nil || len(contact.Manifold.Points) != 4 {
		return sweepEvent{}, false
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
		return sweepEvent{}, false
	}
	poses := [2]r3.Transform{poseA, poseB}
	base, okBase := contactbox.SourceBoxAtPose(paths[stationary].body, poses[stationary])
	startA, okA := contactbox.SourceBoxAtPose(r.a.body, r.a.Path.From)
	startB, okB := contactbox.SourceBoxAtPose(r.b.body, r.b.Path.From)
	if !okBase || !okA || !okB {
		return sweepEvent{}, false
	}
	normal := contact.Manifold.Points[0].Normal.Value
	if normal != (r3.Vec{Z: 1}) && normal != (r3.Vec{Z: -1}) {
		return sweepEvent{}, false
	}
	elapsed := new(big.Rat).Mul(r.a.Path.Duration, f)
	travel := new(big.Rat).Mul(paths[spinning].Velocity[2], elapsed)
	loA, hiA := startA.Lo[2].Rat(), startA.Hi[2].Rat()
	loB, hiB := startB.Lo[2].Rat(), startB.Hi[2].Rat()
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
			return sweepEvent{}, false
		}
	} else {
		gap.Sub(loA, hiB)
		if loB.Cmp(loA) >= 0 || hiB.Cmp(hiA) >= 0 {
			return sweepEvent{}, false
		}
	}
	if gap.Sign() > 0 {
		return sweepEvent{}, false
	}
	value := sweeppath.RatFloatNearest(gap)
	bound := proofarith.RationalFloatError(gap, value)
	if !measurement.FiniteValues(value, bound) {
		return sweepEvent{}, false
	}
	deviation := new(big.Rat).Add(etaA, etaB)
	points := append([]contactPoint(nil), contact.Manifold.Points...)
	for i := range points {
		point := &points[i]
		if point.Normal.Value != normal || point.Normal.Bound.Base() != 0 ||
			point.NormalAngle.Base() != 0 {
			return sweepEvent{}, false
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
			if value.Cmp(new(big.Rat).Add(base.Lo[axis].Rat(), margin)) <= 0 ||
				value.Cmp(new(big.Rat).Sub(base.Hi[axis].Rat(), margin)) >= 0 {
				return sweepEvent{}, false
			}
		}
		for _, position := range []*vocab.VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), deviation)
			if bound.Cmp(proofarith.FloatRat(r.req.PointResolution.Base())) > 0 {
				return sweepEvent{}, false
			}
			published := proofbound.RatFloatUp(bound)
			if !measurement.FiniteValues(published) || published > r.req.PointResolution.Base() {
				return sweepEvent{}, false
			}
			position.Bound = units.Millimeters(published)
			position.Exactness = measurement.ExactnessOf(position.Bound.Base())
		}
		point.Separation = vocab.Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: measurement.ExactnessOf(bound)}
	}
	zero := vocab.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: vocab.Exact}
	event := sweepEvent{At: sweeppath.Instant(f, r.a.Path.Duration), Manifold: &contactManifold{Points: points}}
	if gap.Sign() == 0 {
		event.Relation, event.Gap = vocab.ContactTouching, &zero
	} else {
		event.Relation = vocab.ContactOverlapping
	}
	return event, true
}

func (r *rotationalPairSweep) execute(ctx context.Context) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case vocab.ContactOverlapping:
		r.report.Outcome, r.report.InitialEvent, r.report.Event =
			vocab.SweepInitiallyOverlapping, &first.Ideal, &first.Ideal
		return r.report, nil
	case vocab.ContactTouching:
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == vocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = vocab.SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if !r.planar && r.req.StartPolicy == vocab.ContinueCertifiedTouch && r.coMovingOrientedTouch(first) {
			last, sampleErr := r.sample(ctx, one)
			if errors.Is(sampleErr, errSweepPoseBudget) {
				return r.undecided(zero, one, vocab.SweepPoseBudget), nil
			}
			if sampleErr != nil {
				return nil, sampleErr
			}
			if last.Ideal.Relation != vocab.ContactTouching || last.Ideal.Manifold == nil {
				return r.undecided(zero, one, vocab.SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack = nodeTrack(&sweepproof.Track{OrientedA: &r.a.StartBox,
				OrientedB: &r.b.StartBox, OrientedDelta: r.a.Path.Delta,
				From: zero, To: one,
				Duration: r.a.Path.Duration, Request: r.req.ContactRequest,
				PointCount: len(first.Ideal.Manifold.Points),
				Sources: [2]contactFeature{first.Ideal.Manifold.Points[0].FeatureA,
					first.Ideal.Manifold.Points[0].FeatureB},
				NormalRead: first.Ideal.Manifold.Points[0].Normal})
			r.report.Outcome = vocab.SweepPersistentTouch
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
		if r.planar && r.req.StartPolicy == vocab.ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = nodeTrack(track), vocab.SweepPersistentBand
				if track.Planar.Band == nil {
					r.report.Outcome = vocab.SweepPersistentTouch
				}
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
	case vocab.ContactBand:
		// §10.4: a positive-displacement pair that starts inside its band is
		// an initial contact. No departure is published, since the true pair
		// may stay inside the band however fast the held pair separates; a
		// certified continuation is §10.3's band track over the held touch.
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == vocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = vocab.SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		// §10.5: an exact pair apart within its SupportBand has every vertex
		// above the support plane, so it departs when §10.2's proof holds.
		if r.planar && r.req.StartPolicy == vocab.ContinueSeparatingTouch &&
			r.a.HeldDelta.Sign() == 0 && r.b.HeldDelta.Sign() == 0 {
			departure, departs, err := r.planarDepartureFraction(ctx)
			if err != nil {
				return nil, err
			}
			if departs {
				return r.depart(ctx, departure)
			}
		}
		if r.planar && r.req.StartPolicy == vocab.ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = nodeTrack(track), vocab.SweepPersistentBand
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
	case vocab.ContactSeparated:
	default:
		return r.undecided(zero, zero, vocab.SweepPoseRelation), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, one, vocab.SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	done, err := r.refine(ctx, first, last, 0)
	if err != nil {
		return nil, err
	}
	if !done {
		r.report.Outcome = vocab.SweepClear
	}
	r.sortSamples()
	return r.report, nil
}
