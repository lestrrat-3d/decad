package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// stepInitialTwoDynamicFriction consumes one real initial face manifold and
// certifies the post-impulse translation path for both dynamic bodies.
func (w *World) stepInitialTwoDynamicFriction(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	if first.Event == nil || first.Event.Relation != decad.ContactTouching ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 4 ||
		kicked.entries[0].Pose != r3.Identity() || kicked.entries[1].Pose != r3.Identity() ||
		!zeroAngularVelocity(kicked.entries[0].AngularVelocity) ||
		!zeroAngularVelocity(kicked.entries[1].AngularVelocity) ||
		w.step.MaxEvents <= 1 {
		return undecided(w, "two-dynamic friction requires an initial translating face patch"), nil
	}
	manifold := first.Event.Manifold
	normal, separation, bound, ok := reducedContact(manifold, w.step.Contact)
	penetration := outwardSum(math.Abs(separation), bound)
	if !ok || normal != (r3.Vec{Z: 1}) || !finite(penetration) ||
		penetration > w.step.PenetrationResidual.Base() ||
		!w.fixedFloorPatchWitnesses(manifold) {
		return undecided(w, "two-dynamic friction manifold exceeds its point or normal bounds"), nil
	}
	var mass [2]decad.MassProperties
	var pose [2]r3.Transform
	var pre [2]QuantityVec
	var whole [2]*big.Rat
	for i := range w.parts {
		mass[i] = w.parts[i].mass
		pose[i] = kicked.entries[i].Pose
		pre[i] = kicked.entries[i].LinearVelocity
		witnesses := cloneManifold(*manifold)
		if i == 0 {
			for j := range witnesses.Points {
				witnesses.Points[j].OnB = witnesses.Points[j].OnA
			}
		}
		var leverOK bool
		whole[i], leverOK = w.frictionWholeBodyLeverWithin(&witnesses, i)
		if !leverOK {
			return undecided(w, "two-dynamic friction path exceeds an audited corner lever"), nil
		}
	}
	response, ok := solveTwoDynamicFrictionPatch(manifold, mass, pose, pre, w.friction,
		w.restitution, w.step)
	if !ok {
		return undecided(w, "two-dynamic friction response exceeds solver residuals"), nil
	}
	for _, spin := range response.PostAngular {
		if !zeroAngularVelocity(spin) {
			return undecided(w, "two-dynamic frictional spin needs a certified rotational remainder"), nil
		}
	}
	angularUpper := response.AngularUpper[0]
	for i := range w.parts {
		spinTravel := new(big.Rat).Mul(exactBase(response.AngularUpper[i]), exactBase(dt))
		spinTravel.Mul(spinTravel, whole[i])
		if spinTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 {
			return undecided(w, "two-dynamic omitted spin exceeds path penetration residual"), nil
		}
		if response.AngularUpper[i].Base() > angularUpper.Base() {
			angularUpper = response.AngularUpper[i]
		}
	}
	normalImpulse, tangentImpulse, points, ok := aggregatePatchImpulses(response.Points, w.step.ImpulseResidual)
	if !ok {
		return undecided(w, "two-dynamic aggregate impulse cannot be published"), nil
	}
	post := kicked
	for i := range post.entries {
		post.entries[i].LinearVelocity = response.Post[i]
	}
	ideal, err := w.sweep(ctx, post, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(ideal, normal) {
		return undecided(w, fmt.Sprintf("two-dynamic ideal continuation returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite two-dynamic frictional drift", err)
	}
	rounded, err := w.sweepPoses(ctx, post, end, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(rounded, normal) {
		return undecided(w, fmt.Sprintf("two-dynamic rounded continuation returned %v", rounded.Outcome)), nil
	}
	endpoint, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil {
		return undecided(w, "two-dynamic endpoint lacks certified contact"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(endpoint.Manifold, w.step.Contact)
	if !valid || finalNormal != normal || !finite(finalSeparation, finalBound) ||
		outwardSum(math.Abs(finalSeparation), finalBound) > w.step.PenetrationResidual.Base() {
		return undecided(w, "two-dynamic endpoint exceeds penetration residual"), nil
	}
	penetration = math.Max(penetration, outwardSum(math.Abs(finalSeparation), finalBound))
	instant := first.Event.At
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:           ContactImpact,
		Pair:           BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:        decad.SweepInterval{From: instant, To: instant},
		Time:           instant.Elapsed.Value,
		Manifold:       cloneManifold(*manifold),
		NormalImpulse:  normalImpulse,
		TangentImpulse: tangentImpulse,
		PointImpulses:  points,
		Solver: &ContactSolverReport{NormalResidual: response.NormalResidual,
			TangentResidual: response.TangentResidual, ConeResidual: response.ConeResidual,
			PenetrationResidual: units.Millimeters(penetration),
			AngularUpper:        angularUpper, Iterations: response.Iterations},
		PreVelocity:   pre[1],
		PostVelocity:  response.Post[1],
		PreVelocityA:  pre[0],
		PreVelocityB:  pre[1],
		PostVelocityA: response.Post[0],
		PostVelocityB: response.Post[1],
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true}
	return report, nil
}
