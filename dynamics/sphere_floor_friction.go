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

func (w *World) sphereFloorFrictionCandidate(event *decad.SweepEvent) bool {
	if event == nil || event.Manifold == nil || len(event.Manifold.Points) != 1 ||
		(w.parts[0].definition.Role != Fixed || w.parts[1].definition.Role != Dynamic) &&
			(w.parts[0].definition.Role != Dynamic || w.parts[1].definition.Role != Fixed) {
		return false
	}
	dynamic := 1
	if w.parts[0].definition.Role == Dynamic {
		dynamic = 0
	}
	point := event.Manifold.Points[0]
	face := point.FaceB
	if dynamic == 0 {
		face = point.FaceA
	}
	if face == nil {
		return false
	}
	_, ok := face.Surface().(decad.Sphere)
	return ok
}

func exactSphereFloorMass(m decad.MassProperties, sphere decad.Sphere) (*big.Rat, *big.Rat, *big.Rat, bool) {
	if m.Mass.Exactness != decad.Exact || m.Mass.Bound.Base() != 0 ||
		m.Center.Exactness != decad.Exact || m.Center.Bound.Base() != 0 ||
		m.Center.Value != sphere.Center || sphere.Radius.Kind() != units.Length {
		return nil, nil, nil, false
	}
	inertia := m.Inertia
	readings := []decad.Measurement{inertia.XX, inertia.YY, inertia.ZZ,
		inertia.XY, inertia.XZ, inertia.YZ}
	for _, reading := range readings {
		if reading.Exactness != decad.Exact || reading.Bound.Base() != 0 {
			return nil, nil, nil, false
		}
	}
	if inertia.XX.Value.Base() != inertia.YY.Value.Base() ||
		inertia.ZZ.Value.Base() != inertia.YY.Value.Base() ||
		inertia.XY.Value.Base() != 0 || inertia.XZ.Value.Base() != 0 ||
		inertia.YZ.Value.Base() != 0 {
		return nil, nil, nil, false
	}
	mass, moment, radius := exactBase(m.Mass.Value), exactBase(inertia.YY.Value), exactBase(sphere.Radius)
	if mass == nil || moment == nil || radius == nil || mass.Sign() <= 0 ||
		moment.Sign() <= 0 || radius.Sign() <= 0 {
		return nil, nil, nil, false
	}
	return mass, moment, radius, true
}

func rationalRoundedWithin(ideal *big.Rat, rounded float64, residual units.Value) bool {
	if !finite(rounded) || ideal == nil || exactBase(residual) == nil {
		return false
	}
	difference := new(big.Rat).Sub(ideal, ratFloat(rounded))
	return absRat(difference).Cmp(exactBase(residual)) <= 0
}

func sphereRatFloat(value *big.Rat) float64 {
	result, _ := value.Float64()
	return result
}

// stepInitialSphereFloorFriction solves one source-sphere point against an
// identity-placed fixed floor. Exact mass and isotropic inertia keep the
// one-point Coulomb and spin law rational; every published float is checked
// against the configured residual before the rotating contact path is used.
func (w *World) stepInitialSphereFloorFriction(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	dynamic := 1
	if w.parts[0].definition.Role == Dynamic {
		dynamic = 0
	}
	point := first.Event.Manifold.Points[0]
	face, witness := point.FaceB, point.OnB
	if dynamic == 0 {
		face, witness = point.FaceA, point.OnA
	}
	sphere, ok := face.Surface().(decad.Sphere)
	if !ok || first.Event.Relation != decad.ContactTouching ||
		first.Event.At.Fraction.Base() != 0 ||
		w.parts[1-dynamic].definition.Role != Fixed ||
		kicked.entries[1-dynamic].Pose != r3.Identity() ||
		w.step.MaxEvents < 2 || w.restitution.Base() != 0 ||
		w.friction.lower == nil || w.friction.upper == nil ||
		w.friction.lower.Sign() <= 0 || w.friction.lower.Cmp(w.friction.upper) != 0 {
		return undecided(w, "sphere-floor friction needs an exact initial point and coefficient"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	expected := r3.Vec{Z: 1}
	if dynamic == 0 {
		expected.Z = -1
	}
	if !ok || normal != expected ||
		outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() ||
		witness.Bound.Base() != 0 || point.OnA.Value != point.OnB.Value ||
		point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return undecided(w, "sphere-floor point exceeds its contact bounds"), nil
	}
	massRecord := w.parts[dynamic].mass
	mass, moment, radius, ok := exactSphereFloorMass(massRecord, sphere)
	if !ok {
		return undecided(w, "sphere-floor friction needs exact centered isotropic mass"), nil
	}
	center := kicked.entries[dynamic].Pose.Apply(massRecord.Center.Value)
	if witness.Value.X != center.X || witness.Value.Y != center.Y ||
		!rationalRoundedWithin(new(big.Rat).Sub(ratFloat(center.Z), radius),
			witness.Value.Z, w.step.Contact.PointResolution) {
		return undecided(w, "sphere-floor point is not below its mass center"), nil
	}
	pre := kicked.entries[dynamic].LinearVelocity
	preSpin := kicked.entries[dynamic].AngularVelocity
	if pre.Y.Base() != 0 || preSpin.X.Base() != 0 || preSpin.Z.Base() != 0 ||
		pre.Z.Base() >= -w.step.VelocityResidual.Base() {
		return undecided(w, "sphere-floor friction needs a closing planar velocity"), nil
	}
	velocityX, velocityZ := exactBase(pre.X), exactBase(pre.Z)
	spinY := exactBase(preSpin.Y)
	if velocityX == nil || velocityZ == nil || spinY == nil {
		return undecided(w, "sphere-floor input velocity is not representable"), nil
	}
	normalImpulse := new(big.Rat).Neg(new(big.Rat).Mul(mass, velocityZ))
	slip := new(big.Rat).Sub(velocityX, new(big.Rat).Mul(radius, spinY))
	inverseMass := new(big.Rat).Inv(mass)
	tangentMass := new(big.Rat).Add(inverseMass,
		new(big.Rat).Quo(new(big.Rat).Mul(radius, radius), moment))
	tangentImpulse := new(big.Rat).Neg(new(big.Rat).Quo(slip, tangentMass))
	cone := new(big.Rat).Mul(w.friction.lower, normalImpulse)
	if absRat(new(big.Rat).Set(tangentImpulse)).Cmp(cone) > 0 {
		tangentImpulse = new(big.Rat).Set(cone)
		if slip.Sign() > 0 {
			tangentImpulse.Neg(tangentImpulse)
		}
	}
	postX := new(big.Rat).Add(velocityX, new(big.Rat).Mul(tangentImpulse, inverseMass))
	postSpin := new(big.Rat).Sub(spinY,
		new(big.Rat).Quo(new(big.Rat).Mul(radius, tangentImpulse), moment))
	impulseN, impulseT := sphereRatFloat(normalImpulse), sphereRatFloat(tangentImpulse)
	velocity, angular := sphereRatFloat(postX), sphereRatFloat(postSpin)
	if !rationalRoundedWithin(normalImpulse, impulseN, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(tangentImpulse, impulseT, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(postX, velocity, w.step.VelocityResidual) ||
		!rationalRoundedWithin(postSpin, angular, w.step.AngularVelocityResidual) ||
		!finite(impulseN, impulseT, velocity, angular) || impulseN <= 0 {
		return undecided(w, "sphere-floor impulse exceeds its numerical residual"), nil
	}
	idealSlip := new(big.Rat).Sub(postX, new(big.Rat).Mul(radius, postSpin))
	actualSlip := new(big.Rat).Sub(ratFloat(velocity), new(big.Rat).Mul(radius, ratFloat(angular)))
	tangentResidual := absRat(new(big.Rat).Sub(actualSlip, idealSlip))
	coneResidual := new(big.Rat).Sub(absRat(ratFloat(impulseT)),
		new(big.Rat).Mul(w.friction.lower, ratFloat(impulseN)))
	if coneResidual.Sign() < 0 {
		coneResidual = new(big.Rat)
	}
	angularError := absRat(new(big.Rat).Sub(ratFloat(angular), postSpin))
	angularUpper := new(big.Rat).Add(absRat(new(big.Rat).Set(postSpin)), angularError)
	tangentResidualValue := outwardRatFloat(tangentResidual)
	coneResidualValue := outwardRatFloat(coneResidual)
	angularUpperValue := outwardRatFloat(angularUpper)
	if tangentResidual.Cmp(exactBase(w.step.VelocityResidual)) > 0 ||
		coneResidual.Cmp(exactBase(w.step.ImpulseResidual)) > 0 ||
		!finite(tangentResidualValue, coneResidualValue, angularUpperValue) {
		return undecided(w, "sphere-floor tangent or cone residual exceeds its limit"), nil
	}
	post := kicked
	post.entries[dynamic].LinearVelocity = QuantityVec{X: units.MillimetersPerSecond(velocity),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	post.entries[dynamic].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(angular), Z: units.RadiansPerSecond(0)}
	continuation, err := w.sweep(ctx, post, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(continuation, normal) {
		return undecided(w, fmt.Sprintf("sphere-floor rotating continuation returned %v", continuation.Outcome)), nil
	}
	poseA, poseB, err := continuation.CertifiedPosesAt(dt)
	if err != nil {
		return undecided(w, "sphere-floor rotating endpoint lacks replay proof"), nil
	}
	end := post
	end.entries[0].Pose, end.entries[1].Pose = poseA, poseB
	endpoint, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		poseA, poseB, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil ||
		len(endpoint.Manifold.Points) != 1 {
		return undecided(w, "sphere-floor endpoint has no point contact"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(endpoint.Manifold, w.step.Contact)
	if !valid || finalNormal != normal ||
		outwardSum(math.Abs(finalSeparation), finalBound) > w.step.PenetrationResidual.Base() {
		return undecided(w, "sphere-floor endpoint exceeds penetration residual"), nil
	}
	tangentOnB := impulseT
	if dynamic == 0 {
		tangentOnB = -tangentOnB
	}
	zero := units.KilogramMillimetersPerSecond(0)
	pointTangent := QuantityVec{X: units.KilogramMillimetersPerSecond(tangentOnB), Y: zero, Z: zero}
	instant := first.Event.At
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
		Manifold:      cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulseN), TangentImpulse: pointTangent,
		PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(impulseN),
			Tangent: pointTangent}},
		Solver: &ContactSolverReport{NormalResidual: units.MillimetersPerSecond(0),
			TangentResidual:     units.MillimetersPerSecond(tangentResidualValue),
			ConeResidual:        units.KilogramMillimetersPerSecond(coneResidualValue),
			PenetrationResidual: units.Millimeters(math.Max(bound, finalBound)),
			AngularUpper:        units.RadiansPerSecond(angularUpperValue), Iterations: 1},
		PreVelocity: pre, PostVelocity: post.entries[dynamic].LinearVelocity,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                kicked.entries[0].Pose, PoseB: kicked.entries[1].Pose,
	}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
			eventAt: instant.Elapsed.Value, hasEvent: true, rotationalRemainder: continuation}}, nil
}
