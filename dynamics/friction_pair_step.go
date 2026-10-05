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

// offcenterPairPatch selects a normal-impact patch whose four-point center
// misses at least one mass center. The response still validates every witness.
func offcenterPairPatch(event *decad.SweepEvent, a, b decad.MassProperties) bool {
	if event == nil || event.Manifold == nil || len(event.Manifold.Points) != 4 {
		return false
	}
	center := r3.Vec{}
	for _, point := range event.Manifold.Points {
		center = center.Add(point.OnB.Value)
	}
	center = center.Scale(.25)
	return center.X != a.Center.Value.X || center.Y != a.Center.Value.Y ||
		center.X != b.Center.Value.X || center.Y != b.Center.Value.Y
}

// stepInitialTwoDynamicFriction consumes one real initial face manifold and
// certifies the post-impulse path for both dynamic bodies. Zero friction also
// admits an off-center normal impact whose response develops common spin.
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
	for i := range w.bodies {
		mass[i] = w.bodies[i].mass
		pose[i] = kicked.entries[i].Pose
		pre[i] = kicked.entries[i].LinearVelocity
	}
	response, ok := solveTwoDynamicFrictionPatch(manifold, mass, pose, pre, w.pairs[0].friction,
		w.pairs[0].restitution, w.step)
	if !ok {
		return undecided(w, "two-dynamic friction response exceeds solver residuals"), nil
	}
	spinning := false
	for _, spin := range response.PostAngular {
		spinning = spinning || !zeroAngularVelocity(spin)
	}
	angularUpper := response.AngularUpper[0]
	for i := range w.bodies {
		if response.AngularUpper[i].Base() > angularUpper.Base() {
			angularUpper = response.AngularUpper[i]
		}
	}
	normalImpulse, tangentImpulse, points, ok := aggregatePatchImpulses(response.Points, w.step.ImpulseResidual)
	if !ok {
		return undecided(w, "two-dynamic aggregate impulse cannot be published"), nil
	}
	post := kicked.clone()
	for i := range post.entries {
		post.entries[i].LinearVelocity = response.Post[i]
		post.entries[i].AngularVelocity = response.PostAngular[i]
	}
	var end State
	var rotationalRemainder *decad.SweepReport
	if spinning {
		ideal, err := w.sweep(ctx, post, dt, decad.ContinueSeparatingTouch)
		if err != nil {
			return nil, err
		}
		poses, valid := w.twoDynamicRotationalEndpoint(post, dt, ideal)
		if !valid {
			return undecided(w, "two-dynamic frictional spin needs a certified rotational remainder"), nil
		}
		end = post.clone()
		end.entries[0].Pose, end.entries[1].Pose = poses[0], poses[1]
		rotationalRemainder = ideal
	} else {
		var whole [2]*big.Rat
		for i := range w.bodies {
			witnesses := cloneManifold(*manifold)
			if i == 0 {
				for j := range witnesses.Points {
					witnesses.Points[j].OnB = witnesses.Points[j].OnA
				}
			}
			var leverOK bool
			whole[i], leverOK = w.frictionWholeBodyLeverWithin(&witnesses, i, post.entries[i].Pose)
			if !leverOK {
				return undecided(w, "two-dynamic friction path exceeds an audited corner lever"), nil
			}
			spinTravel := new(big.Rat).Mul(exactBase(response.AngularUpper[i]), exactBase(dt))
			spinTravel.Mul(spinTravel, whole[i])
			if spinTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 {
				return undecided(w, "two-dynamic omitted spin exceeds path penetration residual"), nil
			}
		}
		ideal, err := w.sweep(ctx, post, dt, decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !w.persistentTrackWithin(ideal, normal) {
			return undecided(w, fmt.Sprintf("two-dynamic ideal continuation returned %v", ideal.Outcome)), nil
		}
		end, err = driftState(post, dt.Base())
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
		endpoint, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
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
	}
	instant := first.Event.At
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:           ContactImpact,
		Pair:           BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
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
		PreVelocity:          pre[1],
		PostVelocity:         response.Post[1],
		PreVelocityA:         pre[0],
		PreVelocityB:         pre[1],
		PostVelocityA:        response.Post[0],
		PostVelocityB:        response.Post[1],
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                kicked.entries[0].Pose,
		PoseB:                kicked.entries[1].Pose,
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true,
		rotationalRemainder: rotationalRemainder}
	return report, nil
}

// The sweep's fraction-one sample is the only published spinning endpoint.
// Its separated relation includes float-pose deviation from the ideal drift.
func (w *World) twoDynamicRotationalEndpoint(post State, dt units.Value,
	sweep *decad.SweepReport) ([2]r3.Transform, bool) {
	var poses [2]r3.Transform
	if sweep == nil || sweep.Outcome != decad.SweepDepartedClear ||
		sweep.Departure == nil || sweep.InitialEvent == nil ||
		sweep.InitialEvent.Relation != decad.ContactTouching {
		return poses, false
	}
	paths := [2]decad.PairPath{sweep.PathA, sweep.PathB}
	for i, path := range paths {
		drift, ok := path.(decad.RigidDriftSegment)
		if !ok || drift.From != post.entries[i].Pose ||
			drift.Center != post.entries[i].Pose.Apply(w.bodies[i].mass.Center.Value) ||
			drift.LinearVelocity != post.entries[i].LinearVelocity ||
			drift.AngularVelocity != post.entries[i].AngularVelocity || drift.Duration != dt {
			return poses, false
		}
	}
	found := false
	for _, sample := range sweep.Samples {
		if sample.At.Fraction.Base() != 1 {
			continue
		}
		if found || sample.Ideal.Relation != decad.ContactSeparated ||
			sample.Ideal.Gap == nil || sample.FloatContact == nil ||
			sample.FloatContact.Relation != decad.ContactSeparated {
			return poses, false
		}
		value, bound := exactBase(sample.Ideal.Gap.Value), exactBase(sample.Ideal.Gap.Bound)
		if value == nil || bound == nil || value.Cmp(bound) <= 0 {
			return poses, false
		}
		poses, found = [2]r3.Transform{sample.PoseA, sample.PoseB}, true
	}
	return poses, found
}
