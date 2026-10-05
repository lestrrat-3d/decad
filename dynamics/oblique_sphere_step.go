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
	if !finite(closing) || closing >= -w.step.VelocityResidual.Base() {
		return undecided(w, "tilted sphere impact is not closing"), nil
	}
	mass := w.parts[dynamic].mass.Mass
	stopImpulse := -closing * mass.Value.Base()
	if !finite(stopImpulse) || stopImpulse <= 0 ||
		!obliqueStopMomentumWithin(velocity, normal, first.Event.Manifold, mass,
			stopImpulse, dynamic, w.step) {
		return undecided(w, "tilted sphere impact has unresolved tangent momentum"), nil
	}
	restitution := 0.0
	if -closing > w.step.ImpactSpeed.Base() {
		restitution = w.restitution.Base()
	}
	if restitution <= 0 {
		return undecided(w, "tilted sphere impact needs positive restitution"), nil
	}
	impulse := stopImpulse * (1 + restitution)
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "tilted sphere impulse is not finite"), nil
	}
	if !w.centeredSphereImpulseWithin(point, pre.entries[dynamic].Pose, dynamic, normal, impulse) {
		return undecided(w, "tilted sphere impact is not centered within angular residual"), nil
	}
	postVelocity := QuantityVec{X: units.MillimetersPerSecond(-restitution * velocity.X.Base()),
		Y: units.MillimetersPerSecond(-restitution * velocity.Y.Base()),
		Z: units.MillimetersPerSecond(-restitution * velocity.Z.Base())}
	for axis := range 3 {
		component := velocityComponent(postVelocity, axis).Base()
		if !finite(component) {
			return undecided(w, "tilted sphere response velocity is not finite"), nil
		}
	}
	if !obliqueSphereImpulseResidual(velocity, postVelocity, normal, impulse,
		mass, point, dynamic, w.step.ImpulseResidual) {
		return undecided(w, "tilted sphere impulse exceeds momentum residual"), nil
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

func (w *World) centeredSphereImpulseWithin(point decad.ContactPoint, pose r3.Transform,
	dynamic int, normal r3.Vec, impulse float64) bool {
	mass := w.parts[dynamic].mass
	center := pose.Apply(mass.Center.Value)
	witness := point.OnA
	if dynamic == 1 {
		witness = point.OnB
	}
	lever := witness.Value.Sub(center)
	torque := lever.Cross(normal)
	errRadius := witness.Bound.Base() + mass.Center.Bound.Base() +
		lever.Len()*(point.Normal.Bound.Base()+point.NormalAngle.Base())
	xy := math.Abs(mass.Inertia.XY.Value.Base()) + mass.Inertia.XY.Bound.Base()
	xz := math.Abs(mass.Inertia.XZ.Value.Base()) + mass.Inertia.XZ.Bound.Base()
	yz := math.Abs(mass.Inertia.YZ.Value.Base()) + mass.Inertia.YZ.Bound.Base()
	inertia := math.Min(mass.Inertia.XX.Value.Base()-mass.Inertia.XX.Bound.Base()-xy-xz,
		math.Min(mass.Inertia.YY.Value.Base()-mass.Inertia.YY.Bound.Base()-xy-yz,
			mass.Inertia.ZZ.Value.Base()-mass.Inertia.ZZ.Bound.Base()-xz-yz))
	if inertia <= 0 || !finite(torque.X, torque.Y, torque.Z, errRadius) {
		return false
	}
	limit := w.step.AngularVelocityResidual.Base() * inertia / (impulse + w.step.ImpulseResidual.Base())
	return torque.Len()+errRadius <= limit
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
