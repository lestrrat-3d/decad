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

// sphereCornerContact holds one real source-sphere/source-box point. The
// outward normal and impulse act on the shared dynamic sphere.
type sphereCornerContact struct {
	key     int
	pair    *World
	sweep   *decad.SweepReport
	point   decad.ContactPoint
	normal  r3.Vec
	outward r3.Vec
	dynamic int
	mu      *big.Rat
}

func (w *World) stepThreeFrictionIsland(ctx context.Context, from, kicked State,
	gravity QuantityVec, loads [2]*BodyLoad, dt units.Value, sweeps [3]*decad.SweepReport,
	reference *World) (*StepReport, error) {
	if w.three.dynamicCount != 1 || w.step.MaxEvents <= 2 {
		return w.threeUndecided(-1, "friction island needs one sphere and room for two events"), nil
	}
	pre, _ := kicked.Body(w.bodies[w.three.dynamic].definition.Body)
	if pre.Pose.Basis() != r3.Identity().Basis() || !zeroAngularVelocity(pre.AngularVelocity) ||
		pre.LinearVelocity.X.Base() >= -w.step.VelocityResidual.Base() ||
		pre.LinearVelocity.Z.Base() >= -w.step.VelocityResidual.Base() ||
		pre.LinearVelocity.Y.Base() == 0 {
		return w.threeUndecided(-1, "friction island needs closing X and Z and Y slip without spin"), nil
	}
	contacts := make([]sphereCornerContact, 0, 2)
	var sphere decad.Sphere
	for key, sweep := range sweeps {
		if sweep == nil {
			continue
		}
		pair := w.three.pairs[key]
		if sweep.Outcome != decad.SweepInitiallyTouching || sweep.Event == nil ||
			sweep.Event.Relation != decad.ContactTouching || sweep.Event.Manifold == nil ||
			len(sweep.Event.Manifold.Points) != 1 || sweep.Event.At.Fraction.Base() != 0 ||
			pair.pairs[0].restitution.Base() != 0 || pair.pairs[0].friction.lower == nil ||
			pair.pairs[0].friction.upper == nil || pair.pairs[0].friction.lower.Sign() <= 0 ||
			pair.pairs[0].friction.lower.Cmp(pair.pairs[0].friction.upper) != 0 {
			return w.threeUndecided(key, "friction island needs two exact positive-friction initial points"), nil
		}
		dynamic := 0
		if pair.bodies[1].definition.Role == Dynamic {
			dynamic = 1
		}
		fixedState := pairState(kicked, pair).entries[1-dynamic]
		point := sweep.Event.Manifold.Points[0]
		sphereFace := []*decad.Face{point.FaceA, point.FaceB}[dynamic]
		fixedFace := []*decad.Face{point.FaceA, point.FaceB}[1-dynamic]
		if sphereFace == nil || fixedFace == nil {
			return w.threeUndecided(key, "friction island lacks source faces"), nil
		}
		candidate, sphereOK := sphereFace.Surface().(decad.Sphere)
		normal, separation, bound, contactOK := reducedContact(sweep.Event.Manifold, w.step.Contact)
		outward := normal
		if dynamic == 0 {
			outward = outward.Scale(-1)
		}
		if !sphereOK || !sphereFaceOnBody(sphereFace, pre.Body) ||
			!boxFaceOnBody(fixedFace, fixedState.Body) || fixedState.Pose != r3.Identity() ||
			!contactOK || !finite(separation, bound) ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() ||
			point.OnA.Bound.Base() != 0 || point.OnB.Bound.Base() != 0 ||
			point.OnA.Value != point.OnB.Value || point.Normal.Bound.Base() != 0 ||
			point.NormalAngle.Base() != 0 ||
			(outward != (r3.Vec{X: 1}) && outward != (r3.Vec{Z: 1})) {
			return w.threeUndecided(key, "friction island point or source witness is unproved"), nil
		}
		if len(contacts) == 0 {
			sphere = candidate
		} else if sphere != candidate || contacts[0].outward == outward {
			return w.threeUndecided(key, "friction island needs distinct X and Z faces"), nil
		}
		contacts = append(contacts, sphereCornerContact{key: key, pair: pair, sweep: sweep,
			point: point, normal: normal, outward: outward, dynamic: dynamic,
			mu: pair.pairs[0].friction.lower})
	}
	if len(contacts) != 2 {
		return w.threeUndecided(-1, "friction island needs two source-box contacts"), nil
	}
	mass, moment, radius, ok := exactSphereFloorMass(w.bodies[w.three.dynamic].mass, sphere)
	if !ok || !sphereCornerWitnesses(pre.Pose.Apply(sphere.Center), radius, contacts) {
		return w.threeUndecided(-1, "friction island mass center or contact arms are unproved"), nil
	}
	response, ok := solveSphereCornerStick(pre.LinearVelocity, mass, moment, radius, contacts, w.step)
	if !ok {
		return w.threeUndecided(-1, "friction island coupled stick response exceeds its residuals"), nil
	}
	post := kicked
	pre.LinearVelocity = response.velocity
	pre.AngularVelocity = response.angular
	post = withBodyState(post, pre)
	var events []ContactEvent
	stage := kicked
	for i, contact := range contacts {
		beforePair := pairState(stage, contact.pair)
		if i == len(contacts)-1 {
			stage = post
		} else {
			stage = sphereCornerStage(stage, pre.Body, contact, response)
		}
		afterPair := pairState(stage, contact.pair)
		tangentB := response.tangent
		if contact.dynamic == 0 {
			tangentB = -tangentB
		}
		zero := units.KilogramMillimetersPerSecond(0)
		tangent := QuantityVec{X: zero, Y: units.KilogramMillimetersPerSecond(tangentB), Z: zero}
		normal := response.normalX
		if contact.outward.Z == 1 {
			normal = response.normalZ
		}
		impulse := units.KilogramMillimetersPerSecond(normal)
		instant := contact.sweep.Event.At
		events = append(events, ContactEvent{Kind: ContactImpact,
			Pair: BodyPair{A: contact.pair.bodies[0].definition.Body,
				B: contact.pair.bodies[1].definition.Body},
			Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
			Manifold:      cloneManifold(*contact.sweep.Event.Manifold),
			NormalImpulse: impulse, TangentImpulse: tangent,
			PointImpulses:        []ContactPointImpulse{{Normal: impulse, Tangent: tangent}},
			PreVelocity:          beforePair.entries[contact.dynamic].LinearVelocity,
			PostVelocity:         afterPair.entries[contact.dynamic].LinearVelocity,
			PreVelocityA:         beforePair.entries[0].LinearVelocity,
			PreVelocityB:         beforePair.entries[1].LinearVelocity,
			PostVelocityA:        afterPair.entries[0].LinearVelocity,
			PostVelocityB:        afterPair.entries[1].LinearVelocity,
			PreAngularVelocityA:  beforePair.entries[0].AngularVelocity,
			PreAngularVelocityB:  beforePair.entries[1].AngularVelocity,
			PostAngularVelocityA: afterPair.entries[0].AngularVelocity,
			PostAngularVelocityB: afterPair.entries[1].AngularVelocity,
			PoseA:                beforePair.entries[0].Pose, PoseB: beforePair.entries[1].Pose,
			Solver: &ContactSolverReport{NormalResidual: response.normalResidual,
				TangentResidual: response.tangentResidual, ConeResidual: response.coneResidual,
				PenetrationResidual: w.step.PenetrationResidual,
				AngularUpper:        response.angularUpper, Iterations: 1}})
		event := events[len(events)-1]
		if !sphereCornerEventMomentumWithin(event, contact.pair, contact.dynamic) {
			return w.threeUndecided(contact.key, "friction island event momentum exceeds its residual"), nil
		}
		if reason := contact.pair.eventAngularImpulseFailure(event, contact.dynamic,
			eventAngularVelocity(event, contact.dynamic, true),
			eventAngularVelocity(event, contact.dynamic, false),
			[]r3.Transform{event.PoseA, event.PoseB}[contact.dynamic],
			exactBase(w.step.ImpulseResidual)); reason != "" {
			return w.threeUndecided(contact.key, reason), nil
		}
	}
	roundedDrift, err := w.threeDriftState(post, dt.Base())
	if err != nil {
		return w.threeUndecidedArithmetic(-1, "friction island final pose is not finite", err)
	}
	end := post
	haveEndpoint := false
	var rounded [3]*decad.SweepReport
	for i, contact := range contacts {
		startPair := pairState(post, contact.pair)
		ideal, sweepErr := contact.pair.sweep(ctx, startPair, dt, decad.ContinueCertifiedTouch)
		if sweepErr != nil {
			return nil, sweepErr
		}
		witness := sphereIslandContact{normal: contact.normal}
		if !sphereIslandTrackWithin(ideal, witness, decad.SweepPersistentTouch, w.step) {
			return w.threeUndecided(contact.key,
				fmt.Sprintf("friction island continuation returned %v", ideal.Outcome)), nil
		}
		// The certified rotating path supplies the published rounded pose.
		// PoseSegment uses another rotation law and cannot certify this drift.
		poseA, poseB, replayErr := ideal.CertifiedPosesAt(dt)
		if replayErr != nil {
			//nolint:nilerr // Missing replay proof is an undecided step.
			return w.threeUndecided(contact.key, "friction island rotating endpoint lacks replay proof"), nil
		}
		poses := [2]r3.Transform{poseA, poseB}
		if poses[1-contact.dynamic] != startPair.entries[1-contact.dynamic].Pose {
			return w.threeUndecided(contact.key, "friction island fixed endpoint moved"), nil
		}
		ballPose := poses[contact.dynamic]
		driftBall, _ := roundedDrift.Body(pre.Body)
		if ballPose.Translation() != driftBall.Pose.Translation() {
			return w.threeUndecided(contact.key, "friction island drift center disagrees with replay"), nil
		}
		if haveEndpoint {
			heldBall, _ := end.Body(pre.Body)
			if heldBall.Pose != ballPose {
				return w.threeUndecided(contact.key, "friction island pair replays disagree"), nil
			}
		} else {
			ball, _ := end.Body(pre.Body)
			ball.Pose = ballPose
			end = withBodyState(end, ball)
			haveEndpoint = true
		}
		rounded[contact.key] = ideal
		endPair := pairState(end, contact.pair)
		endpoint, contactErr := w.doc.ContactPair(ctx, contact.pair.bodies[0].definition.Body,
			contact.pair.bodies[1].definition.Body, endPair.entries[0].Pose,
			endPair.entries[1].Pose, w.step.Contact)
		if contactErr != nil {
			return nil, contactErr
		}
		if endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil ||
			len(endpoint.Manifold.Points) != 1 {
			return w.threeUndecided(contact.key, "friction island endpoint lacks its point"), nil
		}
		finalNormal, separation, bound, valid := reducedContact(endpoint.Manifold, w.step.Contact)
		if !valid || finalNormal != contact.normal ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
			return w.threeUndecided(contact.key, "friction island endpoint exceeds contact bounds"), nil
		}
		initial := outwardSum(math.Abs(contact.point.Separation.Value.Base()),
			contact.point.Separation.Bound.Base())
		events[i].Solver.PenetrationResidual = units.Millimeters(math.Max(initial,
			outwardSum(math.Abs(separation), bound)))
	}
	trace := Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: units.Seconds(0), hasEvent: true, threeSweeps: rounded}
	childTrace := Trace{start: pairState(from, reference), pre: pairState(kicked, reference),
		post: pairState(post, reference), end: pairState(end, reference),
		duration: dt, eventAt: units.Seconds(0), hasEvent: true}
	conservation, valid := reference.conservationReadings(childTrace.start, childTrace.pre,
		childTrace.end, childTrace, nil, gravity, loads, dt)
	if !valid {
		return w.threeUndecided(-1, "friction island conservation is not finite"), nil
	}
	impulse := [3]*big.Rat{ratFloat(response.normalX),
		new(big.Rat).Mul(big.NewRat(2, 1), ratFloat(response.tangent)), ratFloat(response.normalZ)}
	contactImpulse, valid := boundedMomentum(impulse, impulse, impulse)
	if !valid {
		return w.threeUndecided(-1, "friction island contact impulse is not finite"), nil
	}
	conservation.ContactImpulse = contactImpulse
	beforeEnergy := new(big.Rat).Sub(exactBase(conservation.AfterKick.KineticEnergy.Value),
		exactBase(conservation.AfterKick.KineticEnergy.Bound))
	afterEnergy := new(big.Rat).Add(exactBase(conservation.Completion.KineticEnergy.Value),
		exactBase(conservation.Completion.KineticEnergy.Bound))
	if afterEnergy.Cmp(beforeEnergy) > 0 {
		return w.threeUndecided(-1, "friction island may gain kinetic energy"), nil
	}
	return &StepReport{Status: Advanced, Next: &end, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, nil
}

func sphereCornerWitnesses(center r3.Vec, radius *big.Rat, contacts []sphereCornerContact) bool {
	for _, contact := range contacts {
		witness := contact.point.OnA.Value
		for axis, value := range []float64{witness.X, witness.Y, witness.Z} {
			expected := ratFloat([]float64{center.X, center.Y, center.Z}[axis])
			if []float64{contact.outward.X, contact.outward.Y, contact.outward.Z}[axis] == 1 {
				expected.Sub(expected, radius)
			}
			if ratFloat(value).Cmp(expected) != 0 {
				return false
			}
		}
	}
	return true
}

type sphereCornerResponse struct {
	velocity, angular               QuantityVec
	normalX, normalZ, tangent       float64
	normalResidual, tangentResidual units.Value
	coneResidual, angularUpper      units.Value
}

// The two sticking tangent equations share vY and the same spherical inertia.
// Solving them together gives one Y impulse at each contact. The zero X/Z
// tangent components are a valid member of the redundant point-impulse family.
func solveSphereCornerStick(before QuantityVec, mass, moment, radius *big.Rat,
	contacts []sphereCornerContact, cfg StepConfig) (sphereCornerResponse, bool) {
	var result sphereCornerResponse
	vx, vy, vz := exactBase(before.X), exactBase(before.Y), exactBase(before.Z)
	if vx == nil || vy == nil || vz == nil || radius.Sign() <= 0 {
		return result, false
	}
	radius2 := new(big.Rat).Mul(radius, radius)
	denominator := new(big.Rat).Add(new(big.Rat).Mul(mass, radius2),
		new(big.Rat).Mul(big.NewRat(2, 1), moment))
	postY := new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).Mul(vy, mass), radius2), denominator)
	jt := new(big.Rat).Neg(new(big.Rat).Quo(new(big.Rat).Mul(moment, postY), radius2))
	jnX := new(big.Rat).Neg(new(big.Rat).Mul(mass, vx))
	jnZ := new(big.Rat).Neg(new(big.Rat).Mul(mass, vz))
	omegaX := new(big.Rat).Quo(new(big.Rat).Mul(radius, jt), moment)
	omegaZ := new(big.Rat).Neg(omegaX)
	if jnX.Sign() <= 0 || jnZ.Sign() <= 0 {
		return result, false
	}
	for _, contact := range contacts {
		jn := jnX
		if contact.outward.Z == 1 {
			jn = jnZ
		}
		if absRat(new(big.Rat).Set(jt)).Cmp(new(big.Rat).Mul(contact.mu, jn)) > 0 {
			return result, false
		}
	}
	result.normalX, result.normalZ = sphereRatFloat(jnX), sphereRatFloat(jnZ)
	result.tangent = sphereRatFloat(jt)
	postYFloat, omegaXFloat, omegaZFloat := sphereRatFloat(postY), sphereRatFloat(omegaX), sphereRatFloat(omegaZ)
	if !finite(result.normalX, result.normalZ, result.tangent, postYFloat, omegaXFloat, omegaZFloat) ||
		!rationalRoundedWithin(jnX, result.normalX, cfg.ImpulseResidual) ||
		!rationalRoundedWithin(jnZ, result.normalZ, cfg.ImpulseResidual) ||
		!rationalRoundedWithin(jt, result.tangent, cfg.ImpulseResidual) ||
		!rationalRoundedWithin(postY, postYFloat, cfg.VelocityResidual) ||
		!rationalRoundedWithin(omegaX, omegaXFloat, cfg.AngularVelocityResidual) ||
		!rationalRoundedWithin(omegaZ, omegaZFloat, cfg.AngularVelocityResidual) {
		return sphereCornerResponse{}, false
	}
	result.velocity = QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(postYFloat), Z: units.MillimetersPerSecond(0)}
	result.angular = QuantityVec{X: units.RadiansPerSecond(omegaXFloat),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(omegaZFloat)}
	if !sphereCornerResponseWithin(before, &result, mass, moment, radius, contacts, cfg) {
		return sphereCornerResponse{}, false
	}
	return result, true
}

func sphereCornerResponseWithin(before QuantityVec, response *sphereCornerResponse,
	mass, moment, radius *big.Rat, contacts []sphereCornerContact, cfg StepConfig) bool {
	jnX, jnZ, jt := ratFloat(response.normalX), ratFloat(response.normalZ), ratFloat(response.tangent)
	postY := exactBase(response.velocity.Y)
	omegaX, omegaZ := exactBase(response.angular.X), exactBase(response.angular.Z)
	linear := [3]*big.Rat{jnX, new(big.Rat).Mul(big.NewRat(2, 1), jt), jnZ}
	for axis, value := range []units.Value{response.velocity.X, response.velocity.Y, response.velocity.Z} {
		change := new(big.Rat).Sub(exactBase(value), exactBase(
			[]units.Value{before.X, before.Y, before.Z}[axis]))
		residual := absRat(new(big.Rat).Sub(new(big.Rat).Mul(mass, change), linear[axis]))
		if residual.Cmp(exactBase(cfg.ImpulseResidual)) > 0 {
			return false
		}
	}
	spinX := new(big.Rat).Sub(new(big.Rat).Mul(moment, omegaX), new(big.Rat).Mul(radius, jt))
	spinZ := new(big.Rat).Add(new(big.Rat).Mul(moment, omegaZ), new(big.Rat).Mul(radius, jt))
	spinLimit := new(big.Rat).Mul(moment, exactBase(cfg.AngularVelocityResidual))
	if absRat(spinX).Cmp(spinLimit) > 0 || absRat(spinZ).Cmp(spinLimit) > 0 {
		return false
	}
	wallSlip := new(big.Rat).Sub(postY, new(big.Rat).Mul(radius, omegaZ))
	floorSlip := new(big.Rat).Add(postY, new(big.Rat).Mul(radius, omegaX))
	slip := absRat(wallSlip)
	if absRat(floorSlip).Cmp(slip) > 0 {
		slip = absRat(floorSlip)
	}
	if slip.Cmp(exactBase(cfg.VelocityResidual)) > 0 {
		return false
	}
	var maxCone big.Rat
	for _, contact := range contacts {
		jn := jnX
		if contact.outward.Z == 1 {
			jn = jnZ
		}
		cone := new(big.Rat).Sub(absRat(new(big.Rat).Set(jt)), new(big.Rat).Mul(contact.mu, jn))
		if cone.Cmp(&maxCone) > 0 {
			maxCone.Set(cone)
		}
	}
	if maxCone.Cmp(exactBase(cfg.ImpulseResidual)) > 0 {
		return false
	}
	response.normalResidual = units.MillimetersPerSecond(0)
	response.tangentResidual = units.MillimetersPerSecond(outwardRatFloat(slip))
	response.coneResidual = units.KilogramMillimetersPerSecond(outwardRatFloat(&maxCone))
	response.angularUpper = units.RadiansPerSecond(math.Nextafter(
		math.Hypot(response.angular.X.Base(), response.angular.Z.Base()), math.Inf(1)))
	return finite(response.angularUpper.Base())
}

func sphereCornerStage(state State, body *decad.Body, contact sphereCornerContact,
	response sphereCornerResponse) State {
	entry, _ := state.Body(body)
	mass := contact.pair.bodies[contact.dynamic].mass.Mass.Value.Base()
	moment := contact.pair.bodies[contact.dynamic].mass.Inertia.XX.Value.Base()
	radius := contact.point.OnA.Value.Sub(entry.Pose.Apply(contact.pair.bodies[contact.dynamic].mass.Center.Value))
	jn := response.normalX
	if contact.outward.Z == 1 {
		jn = response.normalZ
	}
	entry.LinearVelocity.X = units.MillimetersPerSecond(
		entry.LinearVelocity.X.Base() + contact.outward.X*jn/mass)
	entry.LinearVelocity.Y = units.MillimetersPerSecond(
		entry.LinearVelocity.Y.Base() + response.tangent/mass)
	entry.LinearVelocity.Z = units.MillimetersPerSecond(
		entry.LinearVelocity.Z.Base() + contact.outward.Z*jn/mass)
	torque := radius.Cross(r3.Vec{Y: response.tangent})
	entry.AngularVelocity.X = units.RadiansPerSecond(entry.AngularVelocity.X.Base() + torque.X/moment)
	entry.AngularVelocity.Z = units.RadiansPerSecond(entry.AngularVelocity.Z.Base() + torque.Z/moment)
	return withBodyState(state, entry)
}

func sphereCornerEventMomentumWithin(event ContactEvent, pair *World, dynamic int) bool {
	if len(event.PointImpulses) != 1 || len(event.Manifold.Points) != 1 ||
		event.PointImpulses[0].Normal != event.NormalImpulse ||
		event.PointImpulses[0].Tangent != event.TangentImpulse {
		return false
	}
	applied, ok := eventAppliedImpulse(event)
	if !ok {
		return false
	}
	mass := exactBase(pair.bodies[dynamic].mass.Mass.Value)
	if mass == nil || mass.Sign() <= 0 {
		return false
	}
	sign := big.NewRat(1, 1)
	if dynamic == 0 {
		sign.Neg(sign)
	}
	pre, post := eventBodyVelocities(event, dynamic)
	for axis := range applied {
		before, after := exactBase(velocityComponent(pre, axis)), exactBase(velocityComponent(post, axis))
		if before == nil || after == nil {
			return false
		}
		change := new(big.Rat).Mul(mass, new(big.Rat).Sub(after, before))
		impulse := new(big.Rat).Mul(sign, applied[axis])
		if absRat(new(big.Rat).Sub(change, impulse)).Cmp(exactBase(pair.step.ImpulseResidual)) > 0 {
			return false
		}
	}
	return true
}
