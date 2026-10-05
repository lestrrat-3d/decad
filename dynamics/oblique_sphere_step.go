package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// stepObliqueSphereImpact consumes the single source-sphere/rotated-box face
// point. Its normal impulse is centered; a small outward position correction
// must be followed by two real clear-path sweeps before the step advances.
func (w *World) stepObliqueSphereImpact(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	if w.friction.upper.Sign() != 0 || first.Bracket == nil || first.Event == nil ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 1 ||
		!first.HasAffineReplayProof() {
		return undecided(w, "tilted sphere impact lacks a frictionless point proof"), nil
	}
	dynamic := -1
	for i, part := range w.parts {
		if part.definition.Role == Dynamic {
			dynamic = i
		}
	}
	if dynamic < 0 || w.parts[1-dynamic].definition.Role != Fixed {
		return undecided(w, "tilted sphere impact needs one fixed box"), nil
	}
	point := first.Event.Manifold.Points[0]
	sphereFace, boxFace := point.FaceA, point.FaceB
	if dynamic == 1 {
		sphereFace, boxFace = point.FaceB, point.FaceA
	}
	if sphereFace == nil || boxFace == nil ||
		!sphereFaceOnBody(sphereFace, w.parts[dynamic].definition.Body) ||
		!boxFaceOnBody(boxFace, w.parts[1-dynamic].definition.Body) {
		return undecided(w, "tilted impact does not name the source sphere and box face"), nil
	}
	normal, separation, bound, ok := boundedObliqueSphereContact(point, w.step.Contact)
	if !ok || !finite(separation, bound) {
		return undecided(w, "tilted sphere point exceeds contact resolution"), nil
	}
	if _, _, axis := axisNormal(normal); axis {
		return undecided(w, "tilted sphere path needs an oblique normal"), nil
	}
	impactTime := dt.Base() * first.Bracket.To.Fraction.Base()
	if !finite(impactTime) || impactTime <= 0 || impactTime >= dt.Base() {
		return undecided(w, "tilted sphere impact time is outside the step"), nil
	}
	eventAt := units.Seconds(impactTime)
	pre, err := driftState(kicked, impactTime)
	if err != nil {
		return undecidedArithmetic(w, "tilted sphere impact pose is not finite", err)
	}
	roundedPrefix, err := w.sweepPoses(ctx, kicked, pre, eventAt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if !roundedImpactPrefixAtEnd(roundedPrefix, first, w.step.PenetrationResidual) {
		return undecided(w, "tilted sphere impact prefix lacks a matching rounded endpoint"), nil
	}
	velocity := kicked.entries[dynamic].LinearVelocity
	preVec := r3.Vec{X: velocity.X.Base(), Y: velocity.Y.Base(), Z: velocity.Z.Base()}
	sign := 1.0
	if dynamic == 0 {
		sign = -1
	}
	closing := sign * preVec.Dot(normal)
	speedUpper, speedOK := sphereNormUpper(preVec)
	if !finite(closing) || !speedOK {
		return undecided(w, "tilted sphere closing speed cannot be bounded"), nil
	}
	exactClosing := sphereDotExact(preVec, normal)
	if sign < 0 {
		exactClosing.Neg(exactClosing)
	}
	roundedError := absRat(new(big.Rat).Sub(ratFloat(closing), exactClosing))
	normalError := new(big.Rat).Add(exactBase(point.Normal.Bound), exactBase(point.NormalAngle))
	approachError := new(big.Rat).Add(roundedError,
		new(big.Rat).Mul(ratFloat(speedUpper), normalError))
	closingUpper := new(big.Rat).Add(exactClosing, approachError)
	if closingUpper.Cmp(new(big.Rat).Neg(exactBase(w.step.VelocityResidual))) >= 0 {
		return undecided(w, "tilted sphere impact is not closing"), nil
	}
	mass := w.parts[dynamic].mass.Mass
	stopImpulse := -closing * mass.Value.Base()
	if !finite(stopImpulse) || stopImpulse <= 0 {
		return undecided(w, "tilted sphere stopping impulse is not finite"), nil
	}
	approach := new(big.Rat).Neg(exactClosing)
	approachLow := new(big.Rat).Sub(approach, approachError)
	approachHigh := new(big.Rat).Add(approach, approachError)
	threshold := exactBase(w.step.ImpactSpeed)
	if approachLow.Cmp(threshold) <= 0 && approachHigh.Cmp(threshold) > 0 {
		return undecided(w, "tilted sphere approach speed straddles restitution threshold"), nil
	}
	restitution := 0.0
	if approachLow.Cmp(threshold) > 0 {
		restitution = w.restitution.Base()
	}
	if restitution <= 0 {
		return undecided(w, "tilted sphere impact needs positive restitution"), nil
	}
	impulse := stopImpulse * (1 + restitution)
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "tilted sphere impulse is not finite"), nil
	}
	if !w.omittedSphereRotationWithin(point, pre.entries[dynamic].Pose,
		dynamic, normal, impulse, dt) {
		return undecided(w, "tilted sphere omitted rotation exceeds response residuals"), nil
	}
	delta := sign * impulse / mass.Value.Base()
	postVec := preVec.Add(normal.Scale(delta))
	if !finite(delta, postVec.X, postVec.Y, postVec.Z) {
		return undecided(w, "tilted sphere response velocity is not finite"), nil
	}
	postVelocity := QuantityVec{X: units.MillimetersPerSecond(postVec.X),
		Y: units.MillimetersPerSecond(postVec.Y), Z: units.MillimetersPerSecond(postVec.Z)}
	if !obliqueSphereImpulseResidual(velocity, postVelocity, normal, impulse,
		mass, point, dynamic, w.step.ImpulseResidual) {
		return undecided(w, "tilted sphere impulse exceeds momentum residual"), nil
	}
	if !obliqueSphereNormalResponseWithin(preVec, postVec, normal, delta,
		sign, point, w.restitution, w.step.VelocityResidual) {
		return undecided(w, "tilted sphere normal or tangent response exceeds velocity residual"), nil
	}
	bracketTravel, ok := boundBracketTravel(*first.Bracket, math.Abs(closing))
	if !ok || separation > bound {
		return undecided(w, "tilted sphere bracket has no penetration bound"), nil
	}
	allowance := outwardSum(bracketTravel, w.step.ContactSlop.Base(), bound)
	if !finite(allowance) || -separation > allowance {
		return undecided(w, "tilted sphere penetration exceeds its bracket"), nil
	}
	pad := math.Max(4*bound, w.step.ContactSlop.Base()/16)
	if !finite(pad) || pad <= 0 {
		return undecided(w, "tilted sphere correction has no positive margin"), nil
	}
	correction := -separation + pad
	if correction < pad {
		correction = pad
	}
	if correction > allowance {
		return undecided(w, "tilted sphere correction exceeds its bracket"), nil
	}
	var inverseMass [2]float64
	inverseMass[dynamic] = 1 / mass.Value.Base()
	post, err := correctPair(pre, normal, correction, inverseMass)
	if err != nil {
		return undecidedArithmetic(w, "tilted sphere correction is not finite", err)
	}
	post.entries[dynamic].LinearVelocity = postVelocity
	contact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if contact.Relation != decad.ContactSeparated || contact.Gap == nil ||
		contact.Gap.Value.Base()-contact.Gap.Bound.Base() <= 0 ||
		contact.Gap.Value.Base()+contact.Gap.Bound.Base() > w.step.ContactSlop.Base() {
		return undecided(w, fmt.Sprintf("tilted sphere correction returned relation %v", contact.Relation)), nil
	}
	remaining := dt.Base() - impactTime
	if !finite(remaining) || remaining <= 0 {
		return undecided(w, "tilted sphere impact has no positive remainder"), nil
	}
	ideal, err := w.sweep(ctx, post, units.Seconds(remaining), decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if ideal.Outcome != decad.SweepClear {
		return undecided(w, fmt.Sprintf("tilted sphere rebound returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "tilted sphere rebound endpoint is not finite", err)
	}
	rounded, err := w.sweepPoses(ctx, post, end, units.Seconds(remaining), decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if rounded.Outcome != decad.SweepClear || !rounded.HasAffineReplayProof() {
		return undecided(w, fmt.Sprintf("rounded tilted sphere rebound returned %v", rounded.Outcome)), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if finalContact.Relation != decad.ContactSeparated || finalContact.Gap == nil ||
		finalContact.Gap.Value.Base()-finalContact.Gap.Bound.Base() <= 0 {
		return undecided(w, "tilted sphere rebound endpoint is not separated"), nil
	}
	change := post.entries[dynamic].Pose.Translation().Sub(pre.entries[dynamic].Pose.Translation())
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: *first.Bracket, Time: eventAt, Manifold: cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulse), TangentImpulse: zeroImpulseVec(),
		PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(impulse),
			Tangent: zeroImpulseVec()}},
		PreVelocity: velocity, PostVelocity: postVelocity, PositionChange: change,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  pre.entries[0].AngularVelocity,
		PreAngularVelocityB:  pre.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                pre.entries[0].Pose, PoseB: pre.entries[1].Pose}
	if dynamic == 0 {
		event.PositionChangeA = change
	} else {
		event.PositionChangeB = change
	}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: pre, post: post, end: end, duration: dt,
			eventAt: eventAt, hasEvent: true, preSweep: roundedPrefix, postSweep: rounded}}, nil
}

func boundedObliqueSphereContact(point decad.ContactPoint,
	req decad.ContactRequest) (r3.Vec, float64, float64, bool) {
	if !finite(point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z,
		point.Normal.Bound.Base(), point.NormalAngle.Base(), point.OnA.Bound.Base(),
		point.OnB.Bound.Base(), point.Separation.Value.Base(), point.Separation.Bound.Base()) ||
		point.Normal.Bound.Base() < 0 || point.Normal.Bound.Base() > req.NormalResolution.Base() ||
		point.NormalAngle.Base() > req.NormalResolution.Base() ||
		point.OnA.Bound.Base() > req.PointResolution.Base() ||
		point.OnB.Bound.Base() > req.PointResolution.Base() {
		return r3.Vec{}, 0, 0, false
	}
	separation := point.Separation.Value.Base()
	bound := outwardSum(point.Separation.Bound.Base(), point.OnA.Bound.Base(), point.OnB.Bound.Base(),
		math.Abs(separation)*(point.Normal.Bound.Base()+point.NormalAngle.Base()))
	return point.Normal.Value, separation, bound, finite(bound)
}

func sphereFaceOnBody(face *decad.Face, body *decad.Body) bool {
	if _, ok := face.Surface().(decad.Sphere); !ok {
		return false
	}
	return slices.Contains(body.Faces(), face)
}

func boxFaceOnBody(face *decad.Face, body *decad.Body) bool {
	if _, ok := face.Surface().(decad.Plane); !ok {
		return false
	}
	return slices.Contains(body.Faces(), face)
}

// omittedSphereRotationWithin bounds every effect omitted by the zero-spin
// response. A small angular speed alone is insufficient when an admitted
// supplied mass center is far from the spherical contact point.
func (w *World) omittedSphereRotationWithin(point decad.ContactPoint, pose r3.Transform,
	dynamic int, normal r3.Vec, impulse float64, dt units.Value) bool {
	mass := w.parts[dynamic].mass
	witness := point.OnA
	if dynamic == 1 {
		witness = point.OnB
	}
	sphereFace := point.FaceA
	if dynamic == 1 {
		sphereFace = point.FaceB
	}
	if sphereFace == nil {
		return false
	}
	sphere, sphereOK := sphereFace.Surface().(decad.Sphere)
	if !sphereOK {
		return false
	}
	center, centerError, centerOK := worldCenterReading(pose, mass.Center)
	pointBound, normalBound := exactBase(witness.Bound), exactBase(point.Normal.Bound)
	angleBound := exactBase(point.NormalAngle)
	lower := certifiedInertiaLower(mass)
	impulseLimit, angularLimit := exactBase(w.step.ImpulseResidual),
		exactBase(w.step.AngularVelocityResidual)
	velocityLimit, pointLimit := exactBase(w.step.VelocityResidual),
		exactBase(w.step.Contact.PointResolution)
	contactSlop, penetrationLimit := exactBase(w.step.ContactSlop),
		exactBase(w.step.PenetrationResidual)
	duration, radius := exactBase(dt), exactBase(sphere.Radius)
	if !centerOK || pointBound == nil || normalBound == nil || angleBound == nil ||
		lower == nil || lower.Sign() <= 0 || impulseLimit == nil || angularLimit == nil ||
		velocityLimit == nil || pointLimit == nil || contactSlop == nil ||
		penetrationLimit == nil || duration == nil || radius == nil || radius.Sign() <= 0 {
		return false
	}
	var arm, direction [3]*big.Rat
	armUpper, armError, normalUpper := new(big.Rat), new(big.Rat), new(big.Rat)
	witnessCoordinates := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
	normalCoordinates := [3]float64{normal.X, normal.Y, normal.Z}
	for axis := range 3 {
		coordinate := ratFloat(witnessCoordinates[axis])
		arm[axis] = new(big.Rat).Sub(coordinate, center[axis])
		direction[axis] = ratFloat(normalCoordinates[axis])
		coordinateError := new(big.Rat).Add(pointBound, centerError[axis])
		armUpper.Add(armUpper, absRat(new(big.Rat).Set(arm[axis])))
		armUpper.Add(armUpper, coordinateError)
		armError.Add(armError, coordinateError)
		normalUpper.Add(normalUpper, absRat(new(big.Rat).Set(direction[axis])))
	}
	nominalTorque := new(big.Rat)
	for axis := range 3 {
		a, b := (axis+1)%3, (axis+2)%3
		component := new(big.Rat).Sub(new(big.Rat).Mul(arm[a], direction[b]),
			new(big.Rat).Mul(arm[b], direction[a]))
		nominalTorque.Add(nominalTorque, absRat(component))
	}
	normalError := new(big.Rat).Add(normalBound, angleBound)
	normalError.Mul(normalError, big.NewRat(3, 1))
	torqueUpper := new(big.Rat).Add(nominalTorque,
		new(big.Rat).Mul(big.NewRat(2, 1),
			new(big.Rat).Add(new(big.Rat).Mul(armError, normalUpper),
				new(big.Rat).Mul(armUpper, normalError))))
	impulseUpper := new(big.Rat).Add(ratFloat(math.Abs(impulse)), impulseLimit)
	angularMomentum := new(big.Rat).Mul(impulseUpper, torqueUpper)
	spin := new(big.Rat).Quo(angularMomentum, lower)
	if spin.Cmp(angularLimit) > 0 {
		return false
	}
	if new(big.Rat).Mul(spin, armUpper).Cmp(velocityLimit) > 0 {
		return false
	}
	energy := new(big.Rat).Mul(angularMomentum, angularMomentum)
	energy.Quo(energy, new(big.Rat).Mul(big.NewRat(2, 1), lower))
	massUpper := new(big.Rat).Add(exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound))
	energyAllowance := new(big.Rat).Mul(massUpper,
		new(big.Rat).Mul(velocityLimit, velocityLimit))
	energyAllowance.Quo(energyAllowance, big.NewRat(2, 1))
	if energy.Cmp(energyAllowance) > 0 {
		return false
	}
	// Any point on the source sphere is within one diameter of the witness.
	reach := new(big.Rat).Add(armUpper, new(big.Rat).Mul(big.NewRat(2, 1), radius))
	travel := new(big.Rat).Mul(new(big.Rat).Mul(spin, reach), duration)
	return travel.Cmp(pointLimit) <= 0 && travel.Cmp(contactSlop) <= 0 &&
		travel.Cmp(penetrationLimit) <= 0
}

func obliqueSphereImpulseResidual(pre, post QuantityVec, normal r3.Vec, impulse float64,
	mass decad.Measurement, point decad.ContactPoint, dynamic int, limit units.Value) bool {
	m, mb, j, residual := exactBase(mass.Value), exactBase(mass.Bound),
		new(big.Rat).SetFloat64(impulse), exactBase(limit)
	if m == nil || mb == nil || j == nil || residual == nil {
		return false
	}
	sign := int64(1)
	if dynamic == 0 {
		sign = -1
	}
	for axis, component := range []float64{normal.X, normal.Y, normal.Z} {
		delta := new(big.Rat).Sub(exactBase(velocityComponent(post, axis)),
			exactBase(velocityComponent(pre, axis)))
		left := new(big.Rat).Mul(m, delta)
		right := new(big.Rat).Mul(j, new(big.Rat).Mul(big.NewRat(sign, 1),
			new(big.Rat).SetFloat64(component)))
		deviation := absRat(new(big.Rat).Sub(left, right))
		deviation.Add(deviation, new(big.Rat).Mul(mb, absRat(delta)))
		normalError := exactBase(point.Normal.Bound)
		angleError := exactBase(point.NormalAngle)
		if normalError == nil || angleError == nil {
			return false
		}
		deviation.Add(deviation, new(big.Rat).Mul(j, new(big.Rat).Add(normalError, angleError)))
		if deviation.Cmp(residual) > 0 {
			return false
		}
	}
	return true
}

// obliqueSphereNormalResponseWithin bounds restitution and the tangent change
// against every contact normal enclosed by the source point's certificate.
func obliqueSphereNormalResponseWithin(pre, post, normal r3.Vec, delta, sign float64,
	point decad.ContactPoint, restitution, velocityLimit units.Value) bool {
	angle := exactBase(point.NormalAngle)
	vector := exactBase(point.Normal.Bound)
	e := exactBase(restitution)
	limit := exactBase(velocityLimit)
	if angle == nil || vector == nil || e == nil || limit == nil {
		return false
	}
	normalError := new(big.Rat).Add(angle, vector)
	preSpeed, preOK := sphereNormUpper(pre)
	postSpeed, postOK := sphereNormUpper(post)
	if !preOK || !postOK {
		return false
	}
	preNormal := sphereDotExact(pre, normal)
	postNormal := sphereDotExact(post, normal)
	if sign < 0 {
		preNormal.Neg(preNormal)
		postNormal.Neg(postNormal)
	}
	restitutionError := absRat(new(big.Rat).Add(postNormal, new(big.Rat).Mul(e, preNormal)))
	uncertainSpeed := new(big.Rat).Add(ratFloat(postSpeed),
		new(big.Rat).Mul(e, ratFloat(preSpeed)))
	restitutionError.Add(restitutionError, new(big.Rat).Mul(uncertainSpeed, normalError))
	if restitutionError.Cmp(limit) > 0 {
		return false
	}
	tangentError := new(big.Rat)
	for _, components := range [][3]float64{
		{pre.X, post.X, normal.X}, {pre.Y, post.Y, normal.Y}, {pre.Z, post.Z, normal.Z},
	} {
		change := new(big.Rat).Sub(ratFloat(components[1]), ratFloat(components[0]))
		tangentError.Add(tangentError, absRat(new(big.Rat).Sub(change,
			new(big.Rat).Mul(ratFloat(delta), ratFloat(components[2])))))
	}
	tangentError.Add(tangentError, new(big.Rat).Mul(absRat(ratFloat(delta)), normalError))
	return tangentError.Cmp(limit) <= 0
}
