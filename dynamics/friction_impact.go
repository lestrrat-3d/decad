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

// stepInteriorFriction consumes a first-impact bracket for a translating box
// above a fixed horizontal floor. The corrected touching manifold supplies
// the four point response; both remaining paths must certify that same touch.
func (w *World) stepInteriorFriction(ctx context.Context, from, kicked, pre State,
	dt, eventAt units.Value, impactTime float64, first, prefix *decad.SweepReport) (*StepReport, error) {
	dynamic := 1
	if w.bodies[0].definition.Role == Dynamic {
		dynamic = 0
	}
	if w.bodies[1-dynamic].definition.Role != Fixed || w.bodies[dynamic].definition.Role != Dynamic ||
		w.pairs[0].restitution.Base() != 0 || first.Bracket == nil || first.Event == nil ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 4 ||
		!first.HasAffineReplayProof() ||
		!roundedImpactPrefixAtEnd(prefix, first, w.step.PenetrationResidual) ||
		kicked.entries[1-dynamic].Pose != r3.Identity() ||
		kicked.entries[dynamic].Pose.Basis() != r3.Identity().Basis() ||
		!zeroAngularVelocity(kicked.entries[dynamic].AngularVelocity) ||
		kicked.entries[dynamic].LinearVelocity.X.Base() <= 0 ||
		kicked.entries[dynamic].LinearVelocity.Y.Mag() != 0 ||
		kicked.entries[dynamic].LinearVelocity.Z.Base() >= -w.step.VelocityResidual.Base() ||
		w.step.MaxEvents <= 1 && impactTime < dt.Base() {
		return undecided(w, "frictional interior impact is outside the fixed-floor patch"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	expectedNormal := r3.Vec{Z: 1}
	if dynamic == 0 {
		expectedNormal.Z = -1
	}
	if !ok || normal != expectedNormal || !finite(separation, bound) || separation > bound {
		return undecided(w, "frictional impact manifold exceeds its normal or point bounds"), nil
	}
	travel, valid := boundBracketTravel(*first.Bracket, kicked.entries[dynamic].LinearVelocity.Z.Base())
	if !valid {
		return undecided(w, "frictional impact bracket travel is not certified"), nil
	}
	allowance := outwardSum(travel, w.step.ContactSlop.Base(), bound)
	if !finite(allowance) || -separation > allowance {
		return undecided(w, "frictional impact penetration exceeds its bracket"), nil
	}
	var inverseMass [2]float64
	inverseMass[dynamic] = 1 / w.bodies[dynamic].mass.Mass.Value.Base()
	post, err := correctPair(pre, normal, -separation, inverseMass)
	if err != nil {
		return undecidedArithmetic(w, "frictional impact correction is not finite", err)
	}
	if !pairCorrectionWithin(pre, post, 2, allowance) {
		return undecided(w, "frictional impact correction exceeds its allowance"), nil
	}
	contact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	for iteration := 1; iteration < w.step.MaxIterations && contact.Relation == decad.ContactOverlapping; iteration++ {
		if contact.Manifold == nil {
			break
		}
		nextNormal, nextSeparation, nextBound, valid := reducedContact(contact.Manifold, w.step.Contact)
		if !valid || nextNormal != normal || nextSeparation >= 0 || !finite(nextSeparation, nextBound) {
			break
		}
		candidate, correctionErr := correctPair(post, normal, -nextSeparation, inverseMass)
		if correctionErr != nil {
			return undecidedArithmetic(w, "frictional impact correction is not finite", correctionErr)
		}
		allowance = outwardSum(allowance, nextBound)
		if !finite(allowance) || !pairCorrectionWithin(pre, candidate, 2, allowance) {
			break
		}
		post = candidate
		contact, err = w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
			post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	if contact.Relation != decad.ContactTouching || contact.Manifold == nil ||
		len(contact.Manifold.Points) != 4 {
		return undecided(w, fmt.Sprintf("frictional corrected impact returned %v", contact.Relation)), nil
	}
	for _, point := range contact.Manifold.Points {
		if point.FaceA != first.Event.Manifold.Points[0].FaceA ||
			point.FaceB != first.Event.Manifold.Points[0].FaceB {
			return undecided(w, "frictional correction changed the impact faces"), nil
		}
	}
	correctedNormal, correctedSeparation, correctedBound, valid := reducedContact(contact.Manifold, w.step.Contact)
	penetration := outwardSum(math.Abs(correctedSeparation), correctedBound)
	patch := floorToBoxManifold(contact.Manifold, dynamic)
	if !valid || correctedNormal != normal || !finite(penetration) ||
		penetration > w.step.PenetrationResidual.Base() || !w.fixedFloorPatchWitnesses(&patch) {
		return undecided(w, "frictional corrected impact exceeds its contact bounds"), nil
	}
	maximumLever, valid := w.frictionWholeBodyLeverWithin(&patch, dynamic, post.entries[dynamic].Pose)
	if !valid {
		return undecided(w, "frictional impact patch exceeds its audited corner lever"), nil
	}
	response, valid := solveCenteredInteriorFrictionPatch(&patch, w.bodies[dynamic].mass,
		post.entries[dynamic].Pose, kicked.entries[dynamic].LinearVelocity, w.pairs[0].friction, w.step)
	if !valid {
		return undecided(w, "frictional interior response exceeds solver residuals"), nil
	}
	remaining := dt.Base() - impactTime
	spinTravel := new(big.Rat).Mul(exactBase(response.AngularUpper), exactBase(units.Seconds(remaining)))
	spinTravel.Mul(spinTravel, maximumLever)
	if spinTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 {
		return undecided(w, "frictional impact omitted spin exceeds penetration residual"), nil
	}
	normalImpulse, tangentImpulse, points, valid := aggregatePatchImpulses(response.Points, w.step.ImpulseResidual)
	if !valid {
		return undecided(w, "frictional impact aggregate impulse cannot be published"), nil
	}
	if dynamic == 0 {
		tangentImpulse, points = reversePatchTangent(tangentImpulse, points)
	}
	post.entries[dynamic].LinearVelocity = response.Post
	var ideal, rounded *decad.SweepReport
	if remaining > 0 {
		ideal, err = w.sweep(ctx, post, units.Seconds(remaining), decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !w.persistentTrackWithin(ideal, normal) {
			return undecided(w, fmt.Sprintf("frictional ideal remainder returned %v", ideal.Outcome)), nil
		}
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "non-finite frictional impact remainder", err)
	}
	if remaining > 0 {
		rounded, err = w.sweepPoses(ctx, post, end, units.Seconds(remaining), decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !w.persistentTrackWithin(rounded, normal) {
			return undecided(w, fmt.Sprintf("frictional rounded remainder returned %v", rounded.Outcome)), nil
		}
	}
	endpoint, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil {
		return undecided(w, "frictional impact endpoint lacks certified touch"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(endpoint.Manifold, w.step.Contact)
	if !valid || finalNormal != normal || !finite(finalSeparation, finalBound) ||
		outwardSum(math.Abs(finalSeparation), finalBound) > w.step.PenetrationResidual.Base() {
		return undecided(w, "frictional impact endpoint exceeds penetration residual"), nil
	}
	penetration = math.Max(penetration, outwardSum(math.Abs(finalSeparation), finalBound))
	change := post.entries[dynamic].Pose.Translation().Sub(pre.entries[dynamic].Pose.Translation())
	changeA, changeB := r3.Vec{}, r3.Vec{}
	if dynamic == 0 {
		changeA = change
	} else {
		changeB = change
	}
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
		Bracket: *first.Bracket, Time: eventAt, Manifold: cloneManifold(*contact.Manifold),
		NormalImpulse: normalImpulse, TangentImpulse: tangentImpulse, PointImpulses: points,
		Solver: &ContactSolverReport{NormalResidual: response.NormalResidual,
			TangentResidual: response.TangentResidual, ConeResidual: response.ConeResidual,
			PenetrationResidual: units.Millimeters(penetration),
			AngularUpper:        response.AngularUpper, Iterations: response.Iterations},
		PreVelocity:  kicked.entries[dynamic].LinearVelocity,
		PostVelocity: response.Post, PositionChange: change,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PositionChangeA: changeA, PositionChangeB: changeB,
		PreAngularVelocityA:  pre.entries[0].AngularVelocity,
		PreAngularVelocityB:  pre.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                post.entries[0].Pose, PoseB: post.entries[1].Pose}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: pre, post: post, end: end, duration: dt,
			eventAt: eventAt, hasEvent: true, preSweep: prefix, postSweep: rounded}}, nil
}
