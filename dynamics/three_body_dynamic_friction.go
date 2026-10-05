package dynamics

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// stepThreeDynamicSphereFriction admits the symmetric two-point island. Both
// contacts act on body zero; solving their normal and tangent impulses together
// accounts for the velocity change that either impulse causes at the other.
func (w *World) stepThreeDynamicSphereFriction(ctx context.Context, from, kicked State,
	input StepInput, dt units.Value) (*StepReport, bool, error) {
	if w.three.dynamicCount != 3 || w.step.MaxEvents <= 2 || w.hasExcluded() {
		return nil, false, nil
	}
	var first [3]*decad.SweepReport
	for key, pair := range w.three.pairs {
		var err error
		first[key], err = pair.sweep(ctx, pairState(kicked, pair), dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, true, err
		}
	}
	if first[0].Outcome != decad.SweepInitiallyTouching ||
		first[1].Outcome != decad.SweepInitiallyTouching ||
		first[2].Outcome != decad.SweepClear {
		return nil, false, nil
	}
	for key := range 2 {
		sweep := first[key]
		if sweep.Event == nil || sweep.Event.Relation != decad.ContactTouching ||
			!isSourceSpherePairEvent(sweep.Event.Manifold) ||
			sweep.Event.At.Fraction != units.Scalar(0) ||
			len(sweep.Samples) == 0 || sweep.Samples[0].FloatContact == nil ||
			sweep.Samples[0].FloatContact.Relation != decad.ContactTouching {
			return w.threeUndecided(key, "three-dynamic friction lacks a certified initial sphere point"), true, nil
		}
		point := sweep.Event.Manifold.Points[0]
		want := r3.Vec{X: 1}
		if key == 1 {
			want = r3.Vec{Y: 1}
		}
		normal, separation, bound, ok := reducedContact(sweep.Event.Manifold, w.step.Contact)
		if !ok || normal != want || separation != 0 || bound != 0 ||
			point.OnA.Bound.Base() != 0 || point.OnB.Bound.Base() != 0 ||
			point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 ||
			point.OnA.Value != point.OnB.Value {
			return w.threeUndecided(key, "three-dynamic friction needs exact cardinal source points"), true, nil
		}
	}
	var mass, moment, radius *big.Rat
	for i, entry := range kicked.Entries() {
		if w.bodies[i].definition.Supplied == nil || entry.Pose.Basis() != r3.Identity().Basis() ||
			!zeroAngularVelocity(entry.AngularVelocity) || entry.LinearVelocity.Z.Base() != 0 {
			return w.threeUndecided(i, "three-dynamic friction needs nonspinning centered supplied spheres"), true, nil
		}
		point := first[0].Event.Manifold.Points[0]
		face := point.FaceA
		switch i {
		case 1:
			face = point.FaceB
		case 2:
			face = first[1].Event.Manifold.Points[0].FaceB
		}
		sphere, ok := face.Surface().(decad.Sphere)
		if !ok || sphere.Center != (r3.Vec{}) {
			return w.threeUndecided(i, "three-dynamic friction needs centered source spheres"), true, nil
		}
		m, inertia, r, ok := exactSphereFloorMass(w.bodies[i].mass, sphere)
		if !ok || i != 0 && (m.Cmp(mass) != 0 || inertia.Cmp(moment) != 0 || r.Cmp(radius) != 0) {
			return w.threeUndecided(i, "three-dynamic friction needs equal exact masses and radii"), true, nil
		}
		if i == 0 {
			mass, moment, radius = m, inertia, r
		}
	}
	entries := kicked.Entries()
	centerA, centerB, centerC := entries[0].Pose.Translation(),
		entries[1].Pose.Translation(), entries[2].Pose.Translation()
	radiusTwice := new(big.Rat).Mul(radius, big.NewRat(2, 1))
	if ratFloat(centerB.X).Cmp(new(big.Rat).Add(ratFloat(centerA.X), radiusTwice)) != 0 ||
		centerB.Y != centerA.Y || centerB.Z != centerA.Z ||
		ratFloat(centerC.Y).Cmp(new(big.Rat).Add(ratFloat(centerA.Y), radiusTwice)) != 0 ||
		centerC.X != centerA.X || centerC.Z != centerA.Z {
		return w.threeUndecided(0, "three-dynamic friction needs orthogonal exact center spacing"), true, nil
	}
	for key := range 2 {
		point := first[key].Event.Manifold.Points[0]
		want := centerA
		expectedX, expectedY := ratFloat(centerA.X), ratFloat(centerA.Y)
		if key == 0 {
			expectedX.Add(expectedX, radius)
			want.X = sphereRatFloat(expectedX)
		} else {
			expectedY.Add(expectedY, radius)
			want.Y = sphereRatFloat(expectedY)
		}
		if point.OnA.Value != want || point.OnB.Value != want ||
			ratFloat(point.OnA.Value.X).Cmp(expectedX) != 0 ||
			ratFloat(point.OnA.Value.Y).Cmp(expectedY) != 0 {
			return w.threeUndecided(key, "three-dynamic friction witness misses the shared radius"), true, nil
		}
	}
	velocity := exactBase(entries[0].LinearVelocity.X)
	if velocity.Sign() <= 0 ||
		exactBase(entries[0].LinearVelocity.Y).Cmp(velocity) != 0 ||
		entries[1].LinearVelocity != spherePairQuantityVelocity(r3.Vec{}) ||
		entries[2].LinearVelocity != spherePairQuantityVelocity(r3.Vec{}) ||
		velocity.Cmp(exactBase(w.step.ImpactSpeed)) <= 0 ||
		velocity.Cmp(exactBase(w.step.VelocityResidual)) <= 0 {
		return w.threeUndecided(0, "three-dynamic friction needs symmetric closing velocities"), true, nil
	}
	firstPair, secondPair := w.three.pairs[0], w.three.pairs[1]
	if firstPair.pairs[0].restitution.Base() <= 0 || firstPair.pairs[0].restitution.Base() >= 1 ||
		firstPair.pairs[0].restitution != secondPair.pairs[0].restitution ||
		firstPair.pairs[0].friction.lower == nil || firstPair.pairs[0].friction.upper == nil ||
		secondPair.pairs[0].friction.lower == nil || secondPair.pairs[0].friction.upper == nil ||
		firstPair.pairs[0].friction.lower.Cmp(firstPair.pairs[0].friction.upper) != 0 ||
		firstPair.pairs[0].friction.lower.Cmp(secondPair.pairs[0].friction.lower) != 0 ||
		secondPair.pairs[0].friction.lower.Cmp(secondPair.pairs[0].friction.upper) != 0 ||
		firstPair.pairs[0].friction.lower.Sign() <= 0 {
		return w.threeUndecided(0, "three-dynamic friction needs matching exact impact materials"), true, nil
	}
	restitution := exactBase(firstPair.pairs[0].restitution)
	// 2N+T=m(1+e)v and N+(2+mr²/I)T=mv. The second equation
	// enforces zero slip at both points, including the outer spheres' spins.
	radiusSquared := new(big.Rat).Mul(radius, radius)
	rotationalInverse := new(big.Rat).Quo(new(big.Rat).Mul(mass, radiusSquared), moment)
	denominator := new(big.Rat).Add(big.NewRat(3, 1),
		new(big.Rat).Mul(big.NewRat(2, 1), rotationalInverse))
	tangent := new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).Sub(big.NewRat(1, 1), restitution),
		new(big.Rat).Mul(mass, velocity)), denominator)
	normal := new(big.Rat).Quo(new(big.Rat).Sub(
		new(big.Rat).Mul(new(big.Rat).Add(big.NewRat(1, 1), restitution),
			new(big.Rat).Mul(mass, velocity)), tangent), big.NewRat(2, 1))
	if normal.Sign() <= 0 || tangent.Sign() <= 0 ||
		tangent.Cmp(new(big.Rat).Mul(firstPair.pairs[0].friction.lower, normal)) > 0 {
		return w.threeUndecided(0, "three-dynamic friction impulse leaves the Coulomb cone"), true, nil
	}
	n, t := sphereRatFloat(normal), sphereRatFloat(tangent)
	if !finite(n, t) || !rationalRoundedWithin(normal, n, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(tangent, t, w.step.ImpulseResidual) ||
		new(big.Rat).Sub(ratFloat(t),
			new(big.Rat).Mul(firstPair.pairs[0].friction.lower, ratFloat(n))).Cmp(
			exactBase(w.step.ImpulseResidual)) > 0 {
		return w.threeUndecided(0, "three-dynamic friction impulse exceeds rounding residual"), true, nil
	}
	normalSpeed := new(big.Rat).Quo(normal, mass)
	tangentSpeed := new(big.Rat).Quo(tangent, mass)
	commonSpeed := new(big.Rat).Sub(velocity, new(big.Rat).Add(normalSpeed, tangentSpeed))
	spin := new(big.Rat).Quo(new(big.Rat).Mul(radius, tangent), moment)
	for _, item := range []*big.Rat{commonSpeed, normalSpeed, tangentSpeed} {
		if !rationalRoundedWithin(item, sphereRatFloat(item), w.step.VelocityResidual) {
			return w.threeUndecided(0, "three-dynamic friction speed exceeds rounding residual"), true, nil
		}
	}
	if !rationalRoundedWithin(spin, sphereRatFloat(spin), w.step.AngularVelocityResidual) {
		return w.threeUndecided(0, "three-dynamic friction spin exceeds rounding residual"), true, nil
	}
	post := kicked
	after := post.Entries()
	after[0].LinearVelocity = spherePairQuantityVelocity(r3.Vec{X: sphereRatFloat(commonSpeed),
		Y: sphereRatFloat(commonSpeed)})
	after[1].LinearVelocity = spherePairQuantityVelocity(r3.Vec{X: sphereRatFloat(normalSpeed),
		Y: sphereRatFloat(tangentSpeed)})
	after[2].LinearVelocity = spherePairQuantityVelocity(r3.Vec{X: sphereRatFloat(tangentSpeed),
		Y: sphereRatFloat(normalSpeed)})
	after[1].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(-sphereRatFloat(spin))}
	after[2].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(sphereRatFloat(spin))}
	for _, entry := range after {
		post = withBodyState(post, entry)
	}
	if !w.threeDynamicFrictionResiduals(kicked, post, mass, moment, radius,
		normal, tangent, restitution) {
		return w.threeUndecided(0, "three-dynamic friction residual or energy is unproved"), true, nil
	}
	var proofs [3]*decad.SweepReport
	end := post
	poses := make(map[*decad.Body]r3.Transform, 3)
	for key, pair := range w.three.pairs {
		policy := decad.ContinueSeparatingTouch
		want := decad.SweepDepartedClear
		if key == 2 {
			policy, want = decad.StopAtInitialContact, decad.SweepClear
		}
		proof, err := pair.sweep(ctx, pairState(post, pair), dt, policy)
		if err != nil {
			return nil, true, err
		}
		if proof.Outcome != want || !proof.HasAffineReplayProof() ||
			key != 2 && !spherePairContinuationWithin(proof, want,
				first[key].Event.Manifold.Points[0], w.step) {
			return w.threeUndecided(key, fmt.Sprintf("three-dynamic friction pair %d remainder lacks proof: outcome=%v cause=%v replay=%v", key, proof.Outcome, proof.Cause, proof.HasAffineReplayProof())), true, nil
		}
		poseA, poseB, err := proof.CertifiedPosesAt(dt)
		if err != nil {
			// A missing replay certificate is a supported refusal.
			//nolint:nilerr
			return w.threeUndecided(key, "three-dynamic friction endpoint lacks replay proof"), true, nil
		}
		for side, pose := range [2]r3.Transform{poseA, poseB} {
			body := pair.bodies[side].definition.Body
			if held, seen := poses[body]; seen && held != pose {
				return w.threeUndecided(key, "three-dynamic friction shared replay poses disagree"), true, nil
			}
			poses[body] = pose
		}
		proofs[key] = proof
	}
	if len(poses) != 3 {
		return w.threeUndecided(0, "three-dynamic friction lacks a body proof"), true, nil
	}
	for body, pose := range poses {
		entry, _ := end.Body(body)
		entry.Pose = pose
		end = withBodyState(end, entry)
	}
	for key, pair := range w.three.pairs {
		pairEnd := pairState(end, pair)
		contact, err := w.doc.ContactPair(ctx, pair.bodies[0].definition.Body,
			pair.bodies[1].definition.Body, pairEnd.entries[0].Pose,
			pairEnd.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, true, err
		}
		if contact.Relation != decad.ContactSeparated || contact.Gap == nil ||
			contact.Gap.Value.Base()-contact.Gap.Bound.Base() <= 0 {
			return w.threeUndecided(key, "three-dynamic friction endpoint is not separated"), true, nil
		}
	}
	events := w.threeDynamicFrictionEvents(kicked, post, first, n, t,
		sphereRatFloat(spin), sphereRatFloat(mass))
	trace := Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: units.Seconds(0), hasEvent: true, threeSweeps: proofs}
	conservation, ok := w.threeAllDynamicConservation(from, kicked, end, trace, events, input, dt)
	if !ok {
		return w.threeUndecided(0, "three-dynamic friction conservation is not finite"), true, nil
	}
	return &StepReport{Status: Advanced, Next: &end, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, true, nil
}

func (w *World) threeDynamicFrictionResiduals(pre, post State, mass, moment, radius,
	normal, tangent, restitution *big.Rat) bool {
	before, after := pre.Entries(), post.Entries()
	limitV, limitJ := exactBase(w.step.VelocityResidual), exactBase(w.step.ImpulseResidual)
	if !zeroAngularVelocity(after[0].AngularVelocity) {
		return false
	}
	for axis := range 2 {
		total := new(big.Rat).Add(normal, tangent)
		old := exactBase(velocityComponent(before[0].LinearVelocity, axis))
		newValue := exactBase(velocityComponent(after[0].LinearVelocity, axis))
		residual := absRat(new(big.Rat).Add(new(big.Rat).Mul(mass,
			new(big.Rat).Sub(newValue, old)), total))
		if residual.Cmp(limitJ) > 0 {
			return false
		}
	}
	for i := 1; i < 3; i++ {
		for axis := range 2 {
			applied := normal
			if i == 1 && axis == 1 || i == 2 && axis == 0 {
				applied = tangent
			}
			change := new(big.Rat).Sub(exactBase(velocityComponent(after[i].LinearVelocity, axis)),
				exactBase(velocityComponent(before[i].LinearVelocity, axis)))
			if absRat(new(big.Rat).Sub(new(big.Rat).Mul(mass, change), applied)).Cmp(limitJ) > 0 {
				return false
			}
		}
		want := new(big.Rat).Quo(new(big.Rat).Mul(radius, tangent), moment)
		if i == 1 {
			want.Neg(want)
		}
		if absRat(new(big.Rat).Sub(exactBase(after[i].AngularVelocity.Z), want)).Cmp(
			exactBase(w.step.AngularVelocityResidual)) > 0 {
			return false
		}
		angularChange := new(big.Rat).Sub(exactBase(after[i].AngularVelocity.Z),
			exactBase(before[i].AngularVelocity.Z))
		angularImpulse := new(big.Rat).Mul(moment, angularChange)
		contactTorque := new(big.Rat).Mul(radius, tangent)
		if i == 2 {
			contactTorque.Neg(contactTorque)
		}
		angularImpulse.Add(angularImpulse, contactTorque)
		angularLimit := new(big.Rat).Mul(radius, limitJ)
		if absRat(angularImpulse).Cmp(angularLimit) > 0 {
			return false
		}
	}
	preNormal := exactBase(before[0].LinearVelocity.X)
	postNormal := new(big.Rat).Sub(exactBase(after[1].LinearVelocity.X),
		exactBase(after[0].LinearVelocity.X))
	if absRat(new(big.Rat).Sub(postNormal, new(big.Rat).Mul(restitution, preNormal))).Cmp(limitV) > 0 {
		return false
	}
	postNormal = new(big.Rat).Sub(exactBase(after[2].LinearVelocity.Y),
		exactBase(after[0].LinearVelocity.Y))
	if absRat(new(big.Rat).Sub(postNormal, new(big.Rat).Mul(restitution, preNormal))).Cmp(limitV) > 0 {
		return false
	}
	// Body zero's equal and opposite torques cancel. Each outer sphere's
	// spin contributes to the point slip on its own contact.
	slip := new(big.Rat).Sub(exactBase(after[1].LinearVelocity.Y),
		exactBase(after[0].LinearVelocity.Y))
	slip.Sub(slip, new(big.Rat).Mul(radius, exactBase(after[1].AngularVelocity.Z)))
	if absRat(slip).Cmp(limitV) > 0 {
		return false
	}
	slip = new(big.Rat).Sub(exactBase(after[2].LinearVelocity.X),
		exactBase(after[0].LinearVelocity.X))
	slip.Add(slip, new(big.Rat).Mul(radius, exactBase(after[2].AngularVelocity.Z)))
	if absRat(slip).Cmp(limitV) > 0 {
		return false
	}
	preEnergy := new(big.Rat).Mul(mass, new(big.Rat).Mul(preNormal, preNormal))
	postEnergy := new(big.Rat)
	for _, entry := range after {
		for _, component := range []units.Value{entry.LinearVelocity.X, entry.LinearVelocity.Y} {
			value := exactBase(component)
			postEnergy.Add(postEnergy, new(big.Rat).Mul(mass,
				new(big.Rat).Mul(value, value)))
		}
		omega := exactBase(entry.AngularVelocity.Z)
		postEnergy.Add(postEnergy, new(big.Rat).Mul(moment,
			new(big.Rat).Mul(omega, omega)))
	}
	postEnergy.Quo(postEnergy, big.NewRat(2, 1))
	return postEnergy.Cmp(preEnergy) <= 0
}

func (w *World) threeDynamicFrictionEvents(pre, post State, sweeps [3]*decad.SweepReport,
	normal, tangent, spin, mass float64) []ContactEvent {
	before, after := pre.Entries(), post.Entries()
	intermediate := before[0]
	intermediate.LinearVelocity = spherePairQuantityVelocity(r3.Vec{
		X: before[0].LinearVelocity.X.Base() - normal/mass,
		Y: before[0].LinearVelocity.Y.Base() - tangent/mass})
	intermediate.AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(-spin)}
	zero := units.KilogramMillimetersPerSecond(0)
	impulses := [2]QuantityVec{
		{X: zero, Y: units.KilogramMillimetersPerSecond(tangent), Z: zero},
		{X: units.KilogramMillimetersPerSecond(tangent), Y: zero, Z: zero},
	}
	events := make([]ContactEvent, 0, 2)
	for key := range 2 {
		bodyBefore, bodyAfter := before[0], intermediate
		outerBefore, outerAfter := before[key+1], after[key+1]
		if key == 1 {
			bodyBefore, bodyAfter = intermediate, after[0]
		}
		at := sweeps[key].Event.At
		pair := w.three.pairs[key]
		events = append(events, ContactEvent{Kind: ContactImpact,
			Pair:    BodyPair{A: pair.bodies[0].definition.Body, B: pair.bodies[1].definition.Body},
			Bracket: decad.SweepInterval{From: at, To: at}, Time: at.Elapsed.Value,
			Manifold:      cloneManifold(*sweeps[key].Event.Manifold),
			NormalImpulse: units.KilogramMillimetersPerSecond(normal), TangentImpulse: impulses[key],
			PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(normal),
				Tangent: impulses[key]}},
			PreVelocity: outerBefore.LinearVelocity, PostVelocity: outerAfter.LinearVelocity,
			PreVelocityA: bodyBefore.LinearVelocity, PostVelocityA: bodyAfter.LinearVelocity,
			PreVelocityB: outerBefore.LinearVelocity, PostVelocityB: outerAfter.LinearVelocity,
			PreAngularVelocityA:  bodyBefore.AngularVelocity,
			PostAngularVelocityA: bodyAfter.AngularVelocity,
			PreAngularVelocityB:  outerBefore.AngularVelocity,
			PostAngularVelocityB: outerAfter.AngularVelocity,
			PoseA:                bodyBefore.Pose, PoseB: outerBefore.Pose,
			Solver: &ContactSolverReport{NormalResidual: w.step.VelocityResidual,
				TangentResidual: w.step.VelocityResidual, ConeResidual: w.step.ImpulseResidual,
				PenetrationResidual: w.step.PenetrationResidual,
				AngularUpper:        units.RadiansPerSecond(spin), Iterations: 1},
		})
	}
	return events
}

// Three freely drifting source spheres can continue after the frictional
// island while every real pair sweep certifies strict clear motion.
func (w *World) stepThreeDynamicSphereClear(ctx context.Context, from, kicked State,
	input StepInput, dt units.Value) (*StepReport, bool, error) {
	if w.three.dynamicCount != 3 || w.hasExcluded() {
		return nil, false, nil
	}
	spinning := false
	for i, entry := range kicked.Entries() {
		spinning = spinning || !zeroAngularVelocity(entry.AngularVelocity)
		if w.bodies[i].definition.Supplied == nil {
			return nil, false, nil
		}
		faces := entry.Body.Faces()
		if len(faces) != 1 {
			return nil, false, nil
		}
		sphere, ok := faces[0].Surface().(decad.Sphere)
		if !ok || sphere.Center != (r3.Vec{}) {
			return nil, false, nil
		}
		if _, _, _, ok := exactSphereFloorMass(w.bodies[i].mass, sphere); !ok {
			return nil, false, nil
		}
	}
	if !spinning {
		return nil, false, nil
	}
	var proofs [3]*decad.SweepReport
	poses := make(map[*decad.Body]r3.Transform, 3)
	for key, pair := range w.three.pairs {
		proof, err := pair.sweep(ctx, pairState(kicked, pair), dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, true, err
		}
		if proof.Outcome != decad.SweepClear || !proof.HasAffineReplayProof() {
			return nil, false, nil
		}
		poseA, poseB, err := proof.CertifiedPosesAt(dt)
		if err != nil {
			// A missing replay certificate is a supported refusal.
			//nolint:nilerr
			return w.threeUndecided(key, "rotating clear sphere endpoint lacks replay proof"), true, nil
		}
		for side, pose := range [2]r3.Transform{poseA, poseB} {
			body := pair.bodies[side].definition.Body
			if held, seen := poses[body]; seen && held != pose {
				return w.threeUndecided(key, "rotating clear sphere replay poses disagree"), true, nil
			}
			poses[body] = pose
		}
		proofs[key] = proof
	}
	if len(poses) != 3 {
		return w.threeUndecided(0, "rotating clear sphere lacks a body proof"), true, nil
	}
	end := kicked
	for body, pose := range poses {
		entry, _ := end.Body(body)
		entry.Pose = pose
		end = withBodyState(end, entry)
	}
	trace := Trace{start: from, pre: kicked, post: kicked, end: end,
		duration: dt, threeSweeps: proofs}
	conservation, ok := w.threeAllDynamicConservation(from, kicked, end, trace, nil, input, dt)
	if !ok {
		return w.threeUndecided(0, "rotating clear sphere conservation is not finite"), true, nil
	}
	return &StepReport{Status: Advanced, Next: &end, Excluded: w.Excluded(),
		Trace: trace, Conservation: &conservation}, true, nil
}
