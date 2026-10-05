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
		(w.bodies[0].definition.Role != Fixed || w.bodies[1].definition.Role != Dynamic) &&
			(w.bodies[0].definition.Role != Dynamic || w.bodies[1].definition.Role != Fixed) {
		return false
	}
	dynamic := 1
	if w.bodies[0].definition.Role == Dynamic {
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

// stepInteriorSphereFloorFriction consumes the real sphere-box first-impact
// bracket. The corrected point is queried again before it enters the same
// one-point response used by initial touch.
func (w *World) stepInteriorSphereFloorFriction(ctx context.Context, from, kicked, pre State,
	dt, eventAt units.Value, impactTime float64, first, prefix *decad.SweepReport) (*StepReport, error) {
	dynamic := 1
	if w.bodies[0].definition.Role == Dynamic {
		dynamic = 0
	}
	if first == nil || first.Bracket == nil || first.Event == nil ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 1 ||
		!first.HasAffineReplayProof() || prefix == nil || !prefix.HasAffineReplayProof() ||
		!roundedImpactPrefixAtEnd(prefix, first, w.step.PenetrationResidual) ||
		w.bodies[1-dynamic].definition.Role != Fixed ||
		kicked.entries[1-dynamic].Pose != r3.Identity() ||
		kicked.entries[dynamic].Pose.Basis() != r3.Identity().Basis() ||
		!zeroAngularVelocity(kicked.entries[dynamic].AngularVelocity) ||
		kicked.entries[dynamic].LinearVelocity.Y.Base() != 0 ||
		kicked.entries[dynamic].LinearVelocity.Z.Base() >= -w.step.VelocityResidual.Base() ||
		w.step.MaxEvents < 2 || impactTime >= dt.Base() {
		return undecided(w, "sphere-floor interior impact is outside the fixed-floor point path"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	expected := r3.Vec{Z: 1}
	if dynamic == 0 {
		expected.Z = -1
	}
	if !ok || normal != expected || !finite(separation, bound) || separation > bound {
		return undecided(w, "sphere-floor impact point exceeds its contact bounds"), nil
	}
	travel, valid := boundBracketTravel(*first.Bracket, kicked.entries[dynamic].LinearVelocity.Z.Base())
	if !valid {
		return undecided(w, "sphere-floor impact bracket travel is not certified"), nil
	}
	allowance := outwardSum(travel, w.step.ContactSlop.Base(), bound)
	if !finite(allowance) || -separation > allowance {
		return undecided(w, "sphere-floor impact penetration exceeds its bracket"), nil
	}
	var inverseMass [2]float64
	inverseMass[dynamic] = 1 / w.bodies[dynamic].mass.Mass.Value.Base()
	at, err := correctPair(pre, normal, -separation, inverseMass)
	if err != nil {
		return undecidedArithmetic(w, "sphere-floor impact correction is not finite", err)
	}
	if !pairCorrectionWithin(pre, at, 2, allowance) {
		return undecided(w, "sphere-floor impact correction exceeds its allowance"), nil
	}
	contact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		at.entries[0].Pose, at.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	for iteration := 1; iteration < w.step.MaxIterations && contact.Relation == decad.ContactOverlapping; iteration++ {
		if contact.Manifold == nil {
			break
		}
		_, nextSeparation, nextBound, valid := reducedContact(contact.Manifold, w.step.Contact)
		if !valid || nextSeparation >= 0 || !finite(nextSeparation, nextBound) {
			break
		}
		candidate, correctionErr := correctPair(at, normal, -nextSeparation, inverseMass)
		if correctionErr != nil {
			return undecidedArithmetic(w, "sphere-floor impact correction is not finite", correctionErr)
		}
		allowance = outwardSum(allowance, nextBound)
		if !finite(allowance) || !pairCorrectionWithin(pre, candidate, 2, allowance) {
			break
		}
		at = candidate
		contact, err = w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
			at.entries[0].Pose, at.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	if contact.Relation != decad.ContactTouching || contact.Manifold == nil ||
		len(contact.Manifold.Points) != 1 {
		return undecided(w, fmt.Sprintf("sphere-floor corrected impact returned %v", contact.Relation)), nil
	}
	point, original := contact.Manifold.Points[0], first.Event.Manifold.Points[0]
	if point.FaceA != original.FaceA || point.FaceB != original.FaceB ||
		point.FeatureA != original.FeatureA || point.FeatureB != original.FeatureB {
		return undecided(w, "sphere-floor correction changed the impact features"), nil
	}
	correctedNormal, correctedSeparation, correctedBound, valid := reducedContact(contact.Manifold, w.step.Contact)
	if !valid || correctedNormal != normal ||
		outwardSum(math.Abs(correctedSeparation), correctedBound) > w.step.PenetrationResidual.Base() {
		return undecided(w, "sphere-floor corrected impact exceeds penetration residual"), nil
	}
	return w.stepSphereFloorFriction(ctx, from, kicked, pre, at, dt,
		eventAt, first, prefix, contact.Manifold)
}

func (w *World) stepInitialSphereFloorFriction(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	if first.Event == nil || first.Event.Manifold == nil ||
		first.Event.Relation != decad.ContactTouching ||
		first.Event.At.Fraction.Base() != 0 {
		return undecided(w, "sphere-floor friction needs an initial touching point"), nil
	}
	return w.stepSphereFloorFriction(ctx, from, kicked, kicked, kicked, dt,
		units.Seconds(0), first, nil, first.Event.Manifold)
}

// stepSphereFloorFriction solves one source-sphere point against an
// identity-placed fixed floor. Its caller supplies a certified touching
// manifold and pose, either at the start or after an interior bracket.
func (w *World) stepSphereFloorFriction(ctx context.Context, from, kicked, prePose, at State,
	dt, eventAt units.Value, first, prefix *decad.SweepReport,
	manifold *decad.ContactManifold) (*StepReport, error) {
	dynamic := 1
	if w.bodies[0].definition.Role == Dynamic {
		dynamic = 0
	}
	if manifold == nil || len(manifold.Points) != 1 {
		return undecided(w, "sphere-floor friction needs one touching point"), nil
	}
	point := manifold.Points[0]
	face, witness := point.FaceB, point.OnB
	if dynamic == 0 {
		face, witness = point.FaceA, point.OnA
	}
	sphere, ok := face.Surface().(decad.Sphere)
	if !ok ||
		w.bodies[1-dynamic].definition.Role != Fixed ||
		at.entries[1-dynamic].Pose != r3.Identity() ||
		w.step.MaxEvents < 2 || w.pairs[0].restitution.Base() != 0 ||
		w.pairs[0].friction.lower == nil || w.pairs[0].friction.upper == nil ||
		w.pairs[0].friction.lower.Sign() <= 0 || w.pairs[0].friction.lower.Cmp(w.pairs[0].friction.upper) != 0 {
		return undecided(w, "sphere-floor friction needs an exact point and coefficient"), nil
	}
	normal, separation, bound, ok := reducedContact(manifold, w.step.Contact)
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
	massRecord := w.bodies[dynamic].mass
	mass, moment, radius, ok := exactSphereFloorMass(massRecord, sphere)
	if !ok {
		return undecided(w, "sphere-floor friction needs exact centered isotropic mass"), nil
	}
	center := at.entries[dynamic].Pose.Apply(massRecord.Center.Value)
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
	cone := new(big.Rat).Mul(w.pairs[0].friction.lower, normalImpulse)
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
		new(big.Rat).Mul(w.pairs[0].friction.lower, ratFloat(impulseN)))
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
	post := at.clone()
	post.entries[dynamic].LinearVelocity = QuantityVec{X: units.MillimetersPerSecond(velocity),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	post.entries[dynamic].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(angular), Z: units.RadiansPerSecond(0)}
	remaining := units.Seconds(dt.Base() - eventAt.Base())
	if remaining.Base() <= 0 || !finite(remaining.Base()) {
		return undecided(w, "sphere-floor friction has no positive continuation"), nil
	}
	continuation, err := w.sweep(ctx, post, remaining, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(continuation, normal) {
		return undecided(w, fmt.Sprintf("sphere-floor rotating continuation returned %v", continuation.Outcome)), nil
	}
	poseA, poseB, err := continuation.CertifiedPosesAt(remaining)
	if err != nil {
		// The report carries the unsupported replay outcome.
		//nolint:nilerr
		return undecided(w, "sphere-floor rotating endpoint lacks replay proof"), nil
	}
	end := post.clone()
	end.entries[0].Pose, end.entries[1].Pose = poseA, poseB
	endpoint, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
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
	bracket := decad.SweepInterval{From: instant, To: instant}
	if first.Bracket != nil {
		bracket = *first.Bracket
	}
	change := at.entries[dynamic].Pose.Translation().Sub(prePose.entries[dynamic].Pose.Translation())
	changeA, changeB := r3.Vec{}, r3.Vec{}
	if dynamic == 0 {
		changeA = change
	} else {
		changeB = change
	}
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
		Bracket: bracket, Time: eventAt,
		Manifold:      cloneManifold(*manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulseN), TangentImpulse: pointTangent,
		PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(impulseN),
			Tangent: pointTangent}},
		Solver: &ContactSolverReport{NormalResidual: units.MillimetersPerSecond(0),
			TangentResidual: units.MillimetersPerSecond(tangentResidualValue),
			ConeResidual:    units.KilogramMillimetersPerSecond(coneResidualValue),
			PenetrationResidual: units.Millimeters(math.Max(
				outwardSum(math.Abs(separation), bound),
				outwardSum(math.Abs(finalSeparation), finalBound))),
			AngularUpper: units.RadiansPerSecond(angularUpperValue), Iterations: 1},
		PreVelocity: pre, PostVelocity: post.entries[dynamic].LinearVelocity, PositionChange: change,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                at.entries[0].Pose, PoseB: at.entries[1].Pose,
		PositionChangeA: changeA, PositionChangeB: changeB,
	}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: prePose, post: post, end: end, duration: dt,
			eventAt: eventAt, hasEvent: true, preSweep: prefix,
			rotationalRemainder: continuation}}, nil
}
