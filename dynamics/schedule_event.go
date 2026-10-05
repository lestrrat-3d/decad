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
			contact, err := r.roundedContact(ctx, key, pre)
			if err != nil {
				return nil, nil, err
			}
			item, diagnostics := r.bandEndPair(item, sweep.ContactTrack, fraction, contact)
			if len(diagnostics) != 0 {
				return nil, diagnostics, nil
			}
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
			rounded, relation, diagnostics, err := r.roundedDepth(ctx, key, pre)
			if err != nil || len(diagnostics) != 0 {
				return nil, diagnostics, err
			}
			item.depth, item.rounded = rounded, relation
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
			if event != nil && event.Relation == decad.ContactOverlapping && !w.manifoldWithin(event.Manifold) {
				// §10.8: a rested vertex may stand below the plane when the
				// slice starts, and the rotating source-box path transfers no
				// manifold for an overlapping start. The slice-start poses are
				// the poses at fraction zero, which deviate from the ideal path
				// by nothing, so the manifold ContactPair publishes there
				// carries the solve, and the correction removes the penetration
				// it shows (§6.6).
				rounded, relation, diagnostics, err := r.roundedDepth(ctx, key, pre)
				if err != nil || len(diagnostics) != 0 {
					return nil, diagnostics, err
				}
				if rounded == nil {
					return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
						"overlapping initial contact has no manifold within the contact request")}, nil
				}
				item.manifold, item.at = cloneManifold(*rounded), event.At
				item.depth, item.rounded = rounded, relation
				item.band = r.carriedPenetration(key, pre)
				gathered = append(gathered, item)
				continue
			}
			if event == nil || !w.manifoldWithin(event.Manifold) {
				return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
					"initial contact has no manifold within the contact request")}, nil
			}
			item.manifold, item.at = cloneManifold(*event.Manifold), event.At
			item.depth = &item.manifold
			item.band = r.carriedPenetration(key, pre)
		}
		gathered = append(gathered, item)
	}
	if len(gathered) != 0 {
		tracks, diagnostics, err := r.trackPairs(ctx, sweeps, plan, fraction, pre)
		if err != nil || len(diagnostics) != 0 {
			return nil, diagnostics, err
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
	for key := range solved.leave {
		// §10.7: a pair whose event poses read Separated leaves the contact
		// set; its next slice starts clear, under StopAtInitialContact.
		delete(r.policies, key)
	}
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

// carriedPenetration widens an initial contact's correction allowance
// (§6.6) by PenetrationResidual when the pair ended the previous step in its
// contact set and both bodies still stand at the poses that step published:
// §5 step 5 admitted the pair there with a penetration within the residual,
// which a rested vertex held below the plane by its band track leaves
// (§10.8). Any other initial contact carries nothing.
func (r *scheduleRun) carriedPenetration(key int, pre State) float64 {
	pair := r.w.pairs[key]
	if !slices.Contains(r.from.contacts, key) || pre.entries[pair.a].Pose != r.from.entries[pair.a].Pose ||
		pre.entries[pair.b].Pose != r.from.entries[pair.b].Pose {
		return 0
	}
	return r.w.step.PenetrationResidual.Base()
}

// bandEndPair completes a pair gathered at a band end (§10.3): the band
// track ends the slice at the cut, or another pair's event cuts a band track
// whose rounded poses read Overlapping there (§10.8). Its manifold at the cut
// enters the island, the rounded poses, read as contact, show the penetration
// the correction removes, and the band's depth widens that correction's
// allowance (§6.6).
func (r *scheduleRun) bandEndPair(item islandPair, track *decad.SweepContactTrack, fraction float64,
	contact *decad.ContactReport) (islandPair, []StepDiagnostic) {
	w := r.w
	pair := w.pairs[item.key]
	manifold, err := track.ManifoldAt(units.Scalar(fraction))
	depth, okDepth := bandDepth(track, fraction)
	if err != nil || !okDepth || !w.manifoldWithin(manifold) {
		return islandPair{}, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
			"band track has no manifold within the contact request at its end")}
	}
	item.manifold, item.band, item.bandEnd, item.track = cloneManifold(*manifold), depth, true, false
	rounded, relation, diagnostics := r.roundedDepthOf(item.key, contact)
	if len(diagnostics) != 0 {
		return islandPair{}, diagnostics
	}
	item.depth, item.rounded = rounded, relation
	if rounded != nil && w.step.Contact.SupportBand.Base() > 0 {
		// §10.5: a track ends before a vertex reaches the plane, so at its
		// end that vertex lies within a grid step of the plane. The support
		// set ContactPair publishes at the rounded event poses holds it
		// beside the track's own set; the solve takes that manifold,
		// certified at the poses the step publishes.
		item.manifold = cloneManifold(*rounded)
	}
	return item, nil
}

// roundedContact runs ContactPair on a gathered pair at the rounded event
// poses, the poses the step publishes.
func (r *scheduleRun) roundedContact(ctx context.Context, key int, pre State) (*decad.ContactReport, error) {
	w := r.w
	pair := w.pairs[key]
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return w.doc.ContactPair(ctx, w.bodies[pair.a].definition.Body,
		w.bodies[pair.b].definition.Body, pre.entries[pair.a].Pose, pre.entries[pair.b].Pose, w.step.Contact)
}

// roundedDepth reads a gathered pair at the rounded event poses, the poses
// the step publishes: §6.6 removes the penetration they show, and §5.2 reads
// the relation for the pair's continuation policy. A separated pair returns
// no manifold; a touching or overlapping one, or a ContactBand within
// PenetrationResidual (§10.4), returns its bounded manifold. Any other
// relation stops the step.
func (r *scheduleRun) roundedDepth(ctx context.Context, key int, pre State) (*decad.ContactManifold,
	decad.ContactRelation, []StepDiagnostic, error) {
	contact, err := r.roundedContact(ctx, key, pre)
	if err != nil {
		return nil, decad.ContactUndecided, nil, err
	}
	rounded, relation, diagnostics := r.roundedDepthOf(key, contact)
	return rounded, relation, diagnostics, nil
}

// roundedDepthOf is roundedDepth's reading of the contact report at the
// rounded event poses.
func (r *scheduleRun) roundedDepthOf(key int, contact *decad.ContactReport) (*decad.ContactManifold,
	decad.ContactRelation, []StepDiagnostic) {
	w := r.w
	pair := w.pairs[key]
	switch {
	case contact.Relation == decad.ContactSeparated:
		return nil, contact.Relation, nil
	case contact.Relation == decad.ContactBand && !w.contactBandWithin(contact.Gap):
		d := scheduleDiagnostic(StepPairUndecided, w.bodyPair(pair),
			"rounded event poses show a contact band beyond the penetration residual")
		d.Limit = w.step.PenetrationResidual
		return nil, decad.ContactUndecided, []StepDiagnostic{d}
	case (contact.Relation == decad.ContactTouching || contact.Relation == decad.ContactOverlapping ||
		contact.Relation == decad.ContactBand) && w.manifoldWithin(contact.Manifold):
		depth := cloneManifold(*contact.Manifold)
		return &depth, contact.Relation, nil
	default:
		return nil, decad.ContactUndecided, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
			fmt.Sprintf("rounded event poses have relation %v without a bounded manifold", contact.Relation))}
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
//
// A band track with a positive Band() may hold a rested vertex (§10.8) below
// the plane at the cut, and a sweep from poses that read Overlapping
// continues under neither policy. Such a pair is read at the rounded event
// poses, and when they read Overlapping it is gathered as a band end at the
// cut instead (bandEndPair): it enters the solve whether or not a point
// closes, and its island's correction removes the penetration.
func (r *scheduleRun) trackPairs(ctx context.Context, sweeps sliceSweeps, plan slicePlan, fraction float64,
	pre State) ([]islandPair, []StepDiagnostic, error) {
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
		item := islandPair{key: key, a: pair.a, b: pair.b, at: plan.instant}
		if band := track.Band(); band != nil && band.Value.Base() > 0 {
			contact, err := r.roundedContact(ctx, key, pre)
			if err != nil {
				return nil, nil, err
			}
			if contact.Relation == decad.ContactOverlapping {
				cut, diagnostics := r.bandEndPair(item, track, fraction, contact)
				if len(diagnostics) != 0 {
					return nil, diagnostics, nil
				}
				out = append(out, cut)
				continue
			}
		}
		manifold, err := track.ManifoldAt(units.Scalar(fraction))
		if err != nil || !w.manifoldWithin(manifold) {
			//nolint:nilerr // a refused manifold is the step's refusal, not a failure
			return nil, []StepDiagnostic{scheduleDiagnostic(StepManifoldMissing, w.bodyPair(pair),
				"persistent track has no manifold within the contact request at the event")}, nil
		}
		held := cloneManifold(*manifold)
		item.manifold, item.depth, item.track = held, &held, true
		out = append(out, item)
	}
	return out, nil, nil
}

// driverMotion is a kinematic participant's exact velocity field over its
// slice driver: a world point x moves at linear + angular × x. A translating
// driver has a zero angular part.
type driverMotion struct {
	linear, angular [3]*big.Rat
}

// at is the field's velocity at the world point x.
func (m driverMotion) at(x [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for axis := range out {
		a, b := (axis+1)%3, (axis+2)%3
		out[axis] = new(big.Rat).Add(m.linear[axis], new(big.Rat).Sub(
			new(big.Rat).Mul(m.angular[a], x[b]), new(big.Rat).Mul(m.angular[b], x[a])))
	}
	return out
}

// rotates reports whether the field has an angular part.
func (m driverMotion) rotates() bool {
	return m.angular[0].Sign() != 0 || m.angular[1].Sign() != 0 || m.angular[2].Sign() != 0
}

// driverVelocities reads the exact velocity field of every kinematic
// participant of the gathered pairs from its slice driver (driverMotionOf).
// A driver whose field cannot be read gets no entry; the island solve
// refuses it only when it joins an island that holds an event.
func (r *scheduleRun) driverVelocities(sweeps sliceSweeps, gathered []islandPair) map[int]driverMotion {
	drive := map[int]driverMotion{}
	for _, pair := range gathered {
		for _, index := range [2]int{pair.a, pair.b} {
			if r.w.bodies[index].definition.Role != Kinematic {
				continue
			}
			if motion, ok := driverMotionOf(sweeps.paths[index]); ok {
				drive[index] = motion
			}
		}
	}
	return drive
}

// driverMotionOf reads a slice driver's exact velocity field. A translating
// PoseSegment moves every point by its displacement over its duration. A
// rotating one follows the screw its endpoints define (pathPoseAt): with
// axis a through the point p, angle θ and slide s over the duration T, read
// as the exact rationals of their float values, a point x moves at
// ω × (x − p) + a·s/T with ω = a·θ/T.
func driverMotionOf(path decad.PairPath) (driverMotion, bool) {
	zero := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	if v, ok := translationVelocity(path); ok {
		return driverMotion{linear: v, angular: zero}, true
	}
	segment, ok := path.(decad.PoseSegment)
	duration := exactBase(segment.Duration)
	if !ok || duration == nil || duration.Sign() <= 0 {
		return driverMotion{}, false
	}
	inverse, err := segment.From.Inverse()
	if err != nil {
		return driverMotion{}, false
	}
	relative, err := inverse.Then(segment.To)
	if err != nil {
		return driverMotion{}, false
	}
	screw, err := relative.Screw()
	if err != nil {
		return driverMotion{}, false
	}
	axis, okAxis := ratVec(screw.Axis)
	point, okPoint := ratVec(screw.Point)
	angle, slide := exactBase(screw.Angle), ratFloat(screw.Slide)
	if !okAxis || !okPoint || angle == nil || slide == nil {
		return driverMotion{}, false
	}
	rate := new(big.Rat).Quo(angle, duration)
	speed := new(big.Rat).Quo(slide, duration)
	motion := driverMotion{}
	for i := range 3 {
		motion.angular[i] = new(big.Rat).Mul(axis[i], rate)
		motion.linear[i] = new(big.Rat).Mul(axis[i], speed)
	}
	// linear = a·s/T − ω × p, so that at(x) = ω × (x − p) + a·s/T.
	spin := driverMotion{linear: zero, angular: motion.angular}.at(point)
	for i := range 3 {
		motion.linear[i].Sub(motion.linear[i], spin[i])
	}
	return motion, true
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
// VelocityResidual of zero.
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
		drive := map[int]driverMotion{}
		zero := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
		for _, index := range [2]int{pair.a, pair.b} {
			if w.bodies[index].definition.Role != Kinematic {
				continue
			}
			v, ok := translationVelocity(sweeps.paths[index])
			if !ok {
				return diagnostic("a graze with a rotating kinematic body has no solver yet")
			}
			drive[index] = driverMotion{linear: v, angular: zero}
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
func (w *World) grazeSpeedWithin(pair islandPair, state State, drive map[int]driverMotion) bool {
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
