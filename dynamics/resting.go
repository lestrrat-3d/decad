package dynamics

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// stepStill accepts a stationary pair only after its full path is certified
// clear or continuously touching. It applies no contact impulse.
func (w *World) stepStill(ctx context.Context, from, kicked State, dt units.Value) (*StepReport, error) {
	ideal, err := w.sweep(ctx, kicked, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	return w.stepNoImpulse(ctx, from, kicked, dt, ideal)
}

// stepNoImpulse accepts a full clear or persistent path without changing
// velocity. Its caller can require persistent touch for a moving initial pair.
func (w *World) stepNoImpulse(ctx context.Context, from, kicked State, dt units.Value,
	ideal *decad.SweepReport) (*StepReport, error) {
	end, err := driftState(kicked, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite no-impulse drift", err)
	}
	actual, err := w.sweepPoses(ctx, kicked, end, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	switch ideal.Outcome {
	case decad.SweepClear:
		if actual.Outcome != decad.SweepClear || finalContact.Relation != decad.ContactSeparated {
			return undecided(w, "clear path lacks a rounded clearance proof"), nil
		}
	case decad.SweepPersistentTouch:
		if ideal.InitialEvent == nil || ideal.InitialEvent.Manifold == nil {
			return undecided(w, "persistent contact lacks an initial manifold"), nil
		}
		normal, separation, bound, ok := reducedContact(ideal.InitialEvent.Manifold, w.step.Contact)
		if !ok || !finite(separation, bound) ||
			math.Abs(separation)+bound > w.step.PenetrationResidual.Base() ||
			!w.persistentTrackWithin(ideal, normal) || !w.persistentTrackWithin(actual, normal) ||
			finalContact.Relation != decad.ContactTouching || finalContact.Manifold == nil {
			return undecided(w, "persistent touch lacks a full bounded track"), nil
		}
		finalNormal, finalSeparation, finalBound, valid := reducedContact(finalContact.Manifold, w.step.Contact)
		if !valid || finalNormal != normal ||
			math.Abs(finalSeparation)+finalBound > w.step.PenetrationResidual.Base() {
			return undecided(w, "persistent endpoint exceeds penetration residual"), nil
		}
	default:
		return undecided(w, fmt.Sprintf("no-impulse sweep returned %v", ideal.Outcome)), nil
	}
	return &StepReport{Status: Advanced, Next: &end,
		Trace: Trace{start: from, end: end, duration: dt}}, nil
}

// stepInitialTouch solves an incoming frictionless pair at its certified
// initial face contact, then admits only a complete persistent-touch track.
func (w *World) stepInitialTouch(ctx context.Context, from, kicked State, dt units.Value,
	first *decad.SweepReport) (*StepReport, error) {
	if first.Event == nil || first.Event.Relation != decad.ContactTouching ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) == 0 {
		return undecided(w, "initial touch has no certified face manifold"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok || !finite(separation, bound) ||
		math.Abs(separation)+bound > w.step.PenetrationResidual.Base() {
		return undecided(w, "initial contact exceeds the penetration residual"), nil
	}
	axis, sign, valid := axisNormal(normal)
	if !valid {
		return undecided(w, "initial contact normal is not a supported axis"), nil
	}
	preSpeed := [2]units.Value{
		velocityComponent(kicked.entries[0].LinearVelocity, axis),
		velocityComponent(kicked.entries[1].LinearVelocity, axis),
	}
	if exactBase(preSpeed[0]).Cmp(exactBase(preSpeed[1])) == 0 {
		continuation, err := w.sweep(ctx, kicked, dt, decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !w.persistentTrackWithin(continuation, normal) {
			return undecided(w, fmt.Sprintf("initial slide returned %v", continuation.Outcome)), nil
		}
		return w.stepNoImpulse(ctx, from, kicked, dt, continuation)
	}
	closing := (preSpeed[1].Base() - preSpeed[0].Base()) * sign
	if !finite(closing) || closing >= -w.step.VelocityResidual.Base() {
		return undecided(w, "initial contact has no certified closing speed"), nil
	}
	var inverseMass [2]float64
	for i, part := range w.parts {
		if part.definition.Role == Dynamic {
			inverseMass[i] = 1 / part.mass.Mass.Value.Base()
		}
	}
	denominator := inverseMass[0] + inverseMass[1]
	if !finite(denominator) || denominator <= 0 {
		return undecided(w, "initial contact effective mass is invalid"), nil
	}
	impulse := -closing / denominator
	postSpeed := [2]float64{
		preSpeed[0].Base() - impulse*sign*inverseMass[0],
		preSpeed[1].Base() + impulse*sign*inverseMass[1],
	}
	if !finite(impulse, postSpeed[0], postSpeed[1]) || impulse <= 0 ||
		!responsePairResidualsWithin(preSpeed, sign, units.Scalar(0), w.parts,
			0, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) {
		return undecided(w, "initial contact response exceeds velocity or impulse residual"), nil
	}
	for i, part := range w.parts {
		if part.definition.Role != Dynamic {
			continue
		}
		if !omittedSpinWithin(first.Event.Manifold, kicked.entries[i].Pose, part.mass,
			i, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
			return undecided(w, "initial contact requires angular response"), nil
		}
	}
	post := kicked
	for i := range post.entries {
		setVelocityComponent(&post.entries[i].LinearVelocity, axis, units.MillimetersPerSecond(postSpeed[i]))
	}
	if math.Abs((postSpeed[1]-postSpeed[0])*sign) > w.step.VelocityResidual.Base() {
		return undecided(w, "initial contact normal speed exceeds residual"), nil
	}
	continuation, err := w.sweep(ctx, post, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(continuation, normal) {
		return undecided(w, fmt.Sprintf("resting continuation returned %v", continuation.Outcome)), nil
	}
	end, err := driftState(post, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite resting drift", err)
	}
	actual, err := w.sweepPoses(ctx, post, end, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(actual, normal) {
		return undecided(w, fmt.Sprintf("numerical resting path returned %v", actual.Outcome)), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if finalContact.Relation != decad.ContactTouching || finalContact.Manifold == nil {
		return undecided(w, "resting endpoint lacks certified contact"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(finalContact.Manifold, w.step.Contact)
	if !valid || finalNormal != normal ||
		math.Abs(finalSeparation)+finalBound > w.step.PenetrationResidual.Base() {
		return undecided(w, "resting endpoint exceeds penetration residual"), nil
	}
	reportBody := 0
	if w.parts[0].definition.Role == Fixed {
		reportBody = 1
	}
	instant := first.Event.At
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Pair:          BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:       decad.SweepInterval{From: instant, To: instant},
		Time:          instant.Elapsed.Value,
		Manifold:      cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulse),
		PreVelocity:   kicked.entries[reportBody].LinearVelocity,
		PostVelocity:  post.entries[reportBody].LinearVelocity,
		PreVelocityA:  kicked.entries[0].LinearVelocity,
		PreVelocityB:  kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity,
		PostVelocityB: post.entries[1].LinearVelocity,
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true}
	return report, nil
}

// persistentTrackWithin checks the reported whole-span proof and each exposed
// manifold's numerical bounds before the step publishes a resting endpoint.
func (w *World) persistentTrackWithin(sweep *decad.SweepReport, normal r3.Vec) bool {
	if sweep == nil || sweep.Outcome != decad.SweepPersistentTouch || sweep.ContactTrack == nil ||
		sweep.ContactTrack.Start().Fraction.Base() != 0 ||
		sweep.ContactTrack.End().Fraction.Base() != 1 ||
		sweep.ContactTrack.Normal().Value != normal {
		return false
	}
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(0.5), units.Scalar(1)} {
		manifold, err := sweep.ContactTrack.ManifoldAt(fraction)
		if err != nil || manifold == nil || len(manifold.Points) == 0 {
			return false
		}
		checkNormal, separation, bound, ok := reducedContact(manifold, w.step.Contact)
		if !ok || checkNormal != normal || !finite(separation, bound) ||
			math.Abs(separation)+bound > w.step.PenetrationResidual.Base() {
			return false
		}
	}
	return true
}
