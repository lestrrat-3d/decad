package dynamics

import (
	"context"
	"fmt"
	"maps"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// This file is one event time of the scheduled step
// (docs/multibody-dynamics-design.md §5 steps 7 and 8, §6.1): the contact
// set's upkeep, the pairs gathered at the event, the zero-impulse transition
// and graze events, and the hand-off to the island solve.

// solveEvent gathers the slice's events at plan.cut, solves their islands
// at the pre-event state and updates the contact set. It returns the event
// record, or nil when no event was published at this time.
func (r *scheduleRun) solveEvent(ctx context.Context, sweeps sliceSweeps, plan slicePlan, pre State,
	label units.Value) (*traceEvent, []StepDiagnostic, error) {
	w := r.w
	r.keepContactSet(sweeps, plan)
	fraction, _ := plan.cut.Float64()
	var gathered []islandPair
	transitions := 0
	for _, key := range plan.at {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		item := islandPair{key: key, a: pair.a, b: pair.b, at: plan.instant}
		if _, band := plan.bands[key]; band {
			// §10.3: the band track ends the slice here. Its manifold at the
			// cut enters the island, the rounded poses show the penetration
			// the correction removes, and the band's depth widens that
			// correction's allowance (§6.6).
			manifold, err := sweep.ContactTrack.ManifoldAt(units.Scalar(fraction))
			depth, okDepth := bandDepth(sweep.ContactTrack, fraction)
			if err != nil || !okDepth || !w.manifoldWithin(manifold) {
				//nolint:nilerr // a refused manifold is the step's refusal, not a failure
				return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
					"band track has no manifold within the contact request at its end")}, nil
			}
			item.manifold, item.band = cloneManifold(*manifold), depth
			rounded, diagnostics, err := r.roundedDepth(ctx, key, pre)
			if err != nil || len(diagnostics) != 0 {
				return nil, diagnostics, err
			}
			item.depth = rounded
			gathered = append(gathered, item)
			continue
		}
		switch sweep.Outcome {
		case decad.SweepContactTransitionBracket:
			// A transition ends the pair's track and enters no solve; the next
			// slice sweeps the pair afresh from the event state (§5.1).
			delete(r.policies, key)
			r.published = append(r.published, w.zeroImpulseEvent(ContactTransition, key, sweep, pre, *sweep.Bracket,
				label, r.at, sweeps.paths[pair.a]))
			transitions++
			continue
		case decad.SweepImpactBracket:
			item.bracket = sweep.Bracket
			// §6.6 removes the penetration the rounded event poses show; the
			// manifold the producer certified at the bracket's right sample
			// carries the solve.
			rounded, diagnostics, err := r.roundedDepth(ctx, key, pre)
			if err != nil || len(diagnostics) != 0 {
				return nil, diagnostics, err
			}
			item.depth = rounded
			switch {
			case w.manifoldWithin(sweep.Event.Manifold):
				item.manifold = cloneManifold(*sweep.Event.Manifold)
			case rounded != nil:
				// A rotating pair's right sample deviates from its ideal pose,
				// so the producer publishes no manifold there; the solve takes
				// the one the rounded event poses show, which are the poses the
				// step publishes.
				item.manifold = cloneManifold(*rounded)
			default:
				return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
					"impact has no manifold within the contact request")}, nil
			}
		default:
			event := sweep.InitialEvent
			if event == nil {
				event = sweep.Event
			}
			if event != nil && event.Relation == decad.ContactBand && !w.contactBandWithin(event.Gap) {
				d := scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
					"initial contact band exceeds the penetration residual")
				d.Limit = w.step.PenetrationResidual
				return nil, []StepDiagnostic{d}, nil
			}
			if event == nil || !w.manifoldWithin(event.Manifold) {
				return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
					"initial contact has no manifold within the contact request")}, nil
			}
			item.manifold, item.at = cloneManifold(*event.Manifold), event.At
			item.depth = &item.manifold
		}
		gathered = append(gathered, item)
	}
	if len(gathered) != 0 {
		tracks, diagnostics := r.trackPairs(sweeps, plan, fraction)
		if len(diagnostics) != 0 {
			return nil, diagnostics, nil
		}
		gathered = append(gathered, tracks...)
	}
	drive := r.driverVelocities(sweeps, gathered)
	solved, diagnostics, err := w.solveIslands(ctx, eventIslands{pre: pre, at: label, sliceStart: r.at,
		sliceSpan: r.remaining(), gathered: gathered, drive: drive, eventBase: len(r.published),
		islandBase: len(r.islands), work: r.work}, r.scheduled)
	if err != nil || len(diagnostics) != 0 {
		return nil, diagnostics, err
	}
	maps.Copy(r.policies, solved.policies)
	if transitions == 0 && len(solved.events) == 0 {
		return nil, nil, nil
	}
	event := &traceEvent{at: exactBase(label), pre: pre, post: solved.post}
	for i := range solved.islands {
		event.islands = append(event.islands, len(r.islands)+i)
	}
	r.published = append(r.published, solved.events...)
	r.islands = append(r.islands, solved.islands...)
	return event, nil, nil
}

// roundedDepth reads a gathered pair at the rounded event poses, the poses
// the step publishes: §6.6 removes the penetration they show. A separated
// pair returns no manifold; a touching or overlapping one, or a ContactBand
// within PenetrationResidual (§10.4), returns its bounded manifold. Any other
// relation stops the step.
func (r *scheduleRun) roundedDepth(ctx context.Context, key int, pre State) (*decad.ContactManifold,
	[]StepDiagnostic, error) {
	w := r.w
	pair := w.pairs[key]
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	contact, err := w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body,
		w.bodies[pair.b].definition.Body, pre.entries[pair.a].Pose, pre.entries[pair.b].Pose, w.step.Contact)
	if err != nil {
		return nil, nil, err
	}
	switch {
	case contact.Relation == decad.ContactSeparated:
		return nil, nil, nil
	case contact.Relation == decad.ContactBand && !w.contactBandWithin(contact.Gap):
		d := scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
			"rounded event poses show a contact band beyond the penetration residual")
		d.Limit = w.step.PenetrationResidual
		return nil, []StepDiagnostic{d}, nil
	case (contact.Relation == decad.ContactTouching || contact.Relation == decad.ContactOverlapping ||
		contact.Relation == decad.ContactBand) && w.manifoldWithin(contact.Manifold):
		depth := cloneManifold(*contact.Manifold)
		return &depth, nil, nil
	default:
		return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
			fmt.Sprintf("rounded event poses have relation %v without a bounded manifold", contact.Relation))}, nil
	}
}

// eventBudget is §12's StepEventBudget: the published events reach
// MaxEvents with time remaining.
func (r *scheduleRun) eventBudget(at units.Value) StepDiagnostic {
	d := scheduleDiagnostic(StepEventBudget, BodyPair{},
		fmt.Sprintf("%d events reach MaxEvents %d with time remaining", len(r.published), r.w.step.MaxEvents))
	d.From, d.To, d.Limit = at, r.dt, units.Scalar(float64(r.w.step.MaxEvents))
	return d
}

// keepContactSet updates the contact set for the pairs whose sweeps hold no
// event at the cut: a pair continued on a persistent track stays in; a pair
// whose departure completed by the cut, a clear pair and a pair the broad
// phase no longer selects leave it. Pairs with an event at the cut take the
// policy their solve assigns.
func (r *scheduleRun) keepContactSet(sweeps sliceSweeps, plan slicePlan) {
	for key := range r.policies {
		sweep, ok := sweeps.swept[key]
		if !ok {
			delete(r.policies, key)
			continue
		}
		switch sweep.Outcome {
		case decad.SweepPersistentTouch, decad.SweepPersistentBand, decad.SweepContactTransitionBracket:
		case decad.SweepDepartedClear:
			if sweep.Departure == nil {
				delete(r.policies, key)
				continue
			}
			if until := exactBase(sweep.Departure.Until.Fraction); until == nil || until.Cmp(plan.cut) <= 0 {
				delete(r.policies, key)
			}
		default:
			delete(r.policies, key)
		}
	}
}

// trackPairs gathers §6.1's persistent-track constraints at the cut: every
// pair the contact set continues on a track that covers the cut joins the
// gathered pairs with the track's manifold there. The island solve keeps only
// the islands an event pair reaches, so a resting stack elsewhere is left to
// drift.
func (r *scheduleRun) trackPairs(sweeps sliceSweeps, plan slicePlan, fraction float64) ([]islandPair, []StepDiagnostic) {
	w := r.w
	keys := make([]int, 0, len(r.policies))
	for key, policy := range r.policies {
		if policy == decad.ContinueCertifiedTouch && !slices.Contains(plan.at, key) {
			keys = append(keys, key)
		}
	}
	slices.Sort(keys)
	var out []islandPair
	for _, key := range keys {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		track := sweep.ContactTrack
		if track == nil {
			continue
		}
		start, end := exactBase(track.Start().Fraction), exactBase(track.End().Fraction)
		if start == nil || end == nil || start.Cmp(plan.cut) > 0 || end.Cmp(plan.cut) < 0 {
			continue
		}
		manifold, err := track.ManifoldAt(units.Scalar(fraction))
		if err != nil || !w.manifoldWithin(manifold) {
			return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
				"persistent track has no manifold within the contact request at the event")}
		}
		held := cloneManifold(*manifold)
		out = append(out, islandPair{key: key, a: pair.a, b: pair.b, manifold: held, at: plan.instant,
			depth: &held, track: true})
	}
	return out, nil
}

// driverVelocities reads the exact translation velocity of every kinematic
// participant of the gathered pairs from its slice driver: the displacement
// over the duration. A driver that rotates has no single contact-point
// velocity and gets no entry; the island solve refuses it only when it
// joins an island that holds an event.
func (r *scheduleRun) driverVelocities(sweeps sliceSweeps, gathered []islandPair) map[int][3]*big.Rat {
	drive := map[int][3]*big.Rat{}
	for _, pair := range gathered {
		for _, index := range [2]int{pair.a, pair.b} {
			if r.w.bodies[index].definition.Role != Kinematic {
				continue
			}
			if v, ok := translationVelocity(sweeps.paths[index]); ok {
				drive[index] = v
			}
		}
	}
	return drive
}

// translationVelocity is the exact velocity of a PoseSegment that only
// translates.
func translationVelocity(path decad.PairPath) ([3]*big.Rat, bool) {
	segment, ok := path.(decad.PoseSegment)
	duration := exactBase(segment.Duration)
	if !ok || segment.From.Basis() != segment.To.Basis() || duration == nil || duration.Sign() <= 0 {
		return [3]*big.Rat{}, false
	}
	from, to := segment.From.Translation(), segment.To.Translation()
	var out [3]*big.Rat
	for axis, pair := range [3][2]float64{{from.X, to.X}, {from.Y, to.Y}, {from.Z, to.Z}} {
		start, end := ratFloat(pair[0]), ratFloat(pair[1])
		if start == nil || end == nil {
			return [3]*big.Rat{}, false
		}
		out[axis] = new(big.Rat).Quo(new(big.Rat).Sub(end, start), duration)
	}
	return out, true
}

// publishGrazes publishes each graze before the cut as a zero-impulse
// ContactGraze. A graze ends no slice: its pair's sweep replays the whole
// slice, through the touch and the separation after it. The pair must be
// frictionless, its poses at the graze are the ones its sweep replays, and
// its enclosed relative normal speed there must lie within
// VelocityResidual of zero, as the two-body graze requires.
func (r *scheduleRun) publishGrazes(sweeps sliceSweeps, plan slicePlan) []StepDiagnostic {
	w := r.w
	for _, key := range plan.grazes {
		sweep, pair := sweeps.swept[key], w.pairs[key]
		f := plan.fractions[key]
		fraction, _ := f.Float64()
		diagnostic := func(reason string) []StepDiagnostic {
			d := r.diagnostic(StepUnsupported, w.bodyPair(pair), reason)
			d.From = r.sliceTime(units.Scalar(fraction))
			return []StepDiagnostic{d}
		}
		if upper := pair.friction.upper; upper != nil && upper.Sign() > 0 {
			return diagnostic("a positive-friction graze has no solver yet")
		}
		if !w.manifoldWithin(sweep.Event.Manifold) {
			return diagnostic("graze has no manifold within the contact request")
		}
		poseA, poseB, err := sweep.CertifiedPosesAtInterval(units.Seconds(fraction), units.Seconds(0), units.Seconds(1))
		if err != nil {
			return diagnostic(fmt.Sprintf("rounded graze poses lack the pair certificate: %v", err))
		}
		at := r.state.clone()
		at.entries[pair.a].Pose, at.entries[pair.b].Pose = poseA, poseB
		drive := map[int][3]*big.Rat{}
		for _, index := range [2]int{pair.a, pair.b} {
			if w.bodies[index].definition.Role != Kinematic {
				continue
			}
			v, ok := translationVelocity(sweeps.paths[index])
			if !ok {
				return diagnostic("a graze with a rotating kinematic body has no solver yet")
			}
			drive[index] = v
		}
		item := islandPair{key: key, a: pair.a, b: pair.b, manifold: *sweep.Event.Manifold}
		if !w.grazeSpeedWithin(item, at, drive) {
			return diagnostic("graze normal speed exceeds the velocity residual")
		}
		r.published = append(r.published, w.zeroImpulseEvent(ContactGraze, key, sweep, at,
			decad.SweepInterval{From: sweep.Event.At, To: sweep.Event.At}, r.sliceTime(units.Scalar(fraction)),
			r.at, sweeps.paths[pair.a]))
		if len(r.published) >= w.step.MaxEvents {
			return []StepDiagnostic{r.eventBudget(r.sliceTime(units.Scalar(fraction)))}
		}
	}
	return nil
}

// grazeSpeedWithin encloses the pair's relative normal speed at every
// manifold point and requires each enclosure within VelocityResidual of
// zero.
func (w *World) grazeSpeedWithin(pair islandPair, state State, drive map[int][3]*big.Rat) bool {
	a, okA := w.newCertBody(pair.a, state.entries[pair.a], state.entries[pair.a], drive)
	b, okB := w.newCertBody(pair.b, state.entries[pair.b], state.entries[pair.b], drive)
	if !okA || !okB {
		return false
	}
	bodies := []certBody{a, b}
	limit := exactBase(w.step.VelocityResidual)
	for _, point := range pair.manifold.Points {
		p, ok := newCertPoint(0, 0, 1, point, bodies, new(big.Rat))
		if !ok || magnitude(preNormalSpeed(p, bodies)).Cmp(limit) > 0 {
			return false
		}
	}
	return true
}

// zeroImpulseEvent publishes a graze or transition: no impulse, unchanged
// velocities, and no island (Island is -1).
func (w *World) zeroImpulseEvent(kind ContactEventKind, key int, sweep *decad.SweepReport, state State,
	bracket decad.SweepInterval, at, sliceStart units.Value, path decad.PairPath) ContactEvent {
	pair := w.pairs[key]
	event := ContactEvent{Kind: kind, Pair: w.bodyPair(pair), Bracket: bracket, SliceStart: sliceStart,
		SliceDuration: pathDuration(path), Time: at, NormalImpulse: units.KilogramMillimetersPerSecond(0),
		TangentImpulse: zeroImpulseVec(), Island: -1,
		PoseA: state.entries[pair.a].Pose, PoseB: state.entries[pair.b].Pose}
	if sweep.Event != nil && sweep.Event.Manifold != nil {
		event.Manifold = cloneManifold(*sweep.Event.Manifold)
		for range event.Manifold.Points {
			event.PointImpulses = append(event.PointImpulses, ContactPointImpulse{
				Normal: units.KilogramMillimetersPerSecond(0), Tangent: zeroImpulseVec()})
		}
	}
	a, b := state.entries[pair.a], state.entries[pair.b]
	event.PreVelocityA, event.PostVelocityA = a.LinearVelocity, a.LinearVelocity
	event.PreVelocityB, event.PostVelocityB = b.LinearVelocity, b.LinearVelocity
	event.PreAngularVelocityA, event.PostAngularVelocityA = a.AngularVelocity, a.AngularVelocity
	event.PreAngularVelocityB, event.PostAngularVelocityB = b.AngularVelocity, b.AngularVelocity
	event.PreVelocity, event.PostVelocity = b.LinearVelocity, b.LinearVelocity
	if w.bodies[pair.b].definition.Role != Dynamic {
		event.PreVelocity, event.PostVelocity = a.LinearVelocity, a.LinearVelocity
	}
	return event
}

// pathDuration is a slice path's duration.
func pathDuration(path decad.PairPath) units.Value {
	switch p := path.(type) {
	case decad.PoseSegment:
		return p.Duration
	case decad.RigidDriftSegment:
		return p.Duration
	default:
		return units.Value{}
	}
}
