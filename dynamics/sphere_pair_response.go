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

func isObliqueSpherePairEvent(manifold *decad.ContactManifold) bool {
	if manifold == nil || len(manifold.Points) != 1 {
		return false
	}
	point := manifold.Points[0]
	if point.FaceA == nil || point.FaceB == nil {
		return false
	}
	if _, ok := point.FaceA.Surface().(decad.Sphere); !ok {
		return false
	}
	if _, ok := point.FaceB.Surface().(decad.Sphere); !ok {
		return false
	}
	_, _, cardinal := axisNormal(point.Normal.Value)
	return !cardinal
}

func spherePairVelocity(v QuantityVec) r3.Vec {
	return r3.Vec{X: v.X.Base(), Y: v.Y.Base(), Z: v.Z.Base()}
}

func spherePairQuantityVelocity(v r3.Vec) QuantityVec {
	return QuantityVec{X: units.MillimetersPerSecond(v.X),
		Y: units.MillimetersPerSecond(v.Y), Z: units.MillimetersPerSecond(v.Z)}
}

// stepObliqueSpherePair consumes the sphere sweep's bounded center-line
// witness. A bounded offset between the source and mass centers is admitted
// only when its omitted angular response fits the configured residual.
func (w *World) stepObliqueSpherePair(ctx context.Context, from, kicked, pre State,
	dt, eventAt units.Value, impactTime float64, first, roundedPrefix *decad.SweepReport) (*StepReport, error) {
	point := first.Event.Manifold.Points[0]
	if roundedPrefix == nil || roundedPrefix.Event == nil || roundedPrefix.Event.Manifold == nil ||
		len(roundedPrefix.Event.Manifold.Points) != 1 || !first.HasAffineReplayProof() ||
		!roundedImpactPrefixAtEnd(roundedPrefix, first, w.step.PenetrationResidual) {
		return undecided(w, "sphere impact lacks a rounded contact witness"), nil
	}
	roundedPoint := roundedPrefix.Event.Manifold.Points[0]
	if w.parts[0].definition.Role != Dynamic || w.parts[1].definition.Role != Dynamic ||
		w.friction.lower.Sign() != 0 || !zeroAngularVelocity(pre.entries[0].AngularVelocity) ||
		!zeroAngularVelocity(pre.entries[1].AngularVelocity) {
		return undecided(w, "oblique sphere impact needs two frictionless nonspinning dynamic bodies"), nil
	}
	n := point.Normal.Value
	if !finite(n.X, n.Y, n.Z, point.Normal.Bound.Base(), point.NormalAngle.Base(),
		point.Separation.Value.Base(), point.Separation.Bound.Base()) ||
		point.Normal.Bound.Base() < 0 || point.Normal.Bound.Base() > w.step.Contact.NormalResolution.Base() ||
		point.NormalAngle.Base() > w.step.Contact.NormalResolution.Base() ||
		point.OnA.Bound.Base() > w.step.Contact.PointResolution.Base() ||
		point.OnB.Bound.Base() > w.step.Contact.PointResolution.Base() {
		return undecided(w, "sphere center-line normal or witnesses exceed contact resolution"), nil
	}
	var inverse [2]float64
	var spheres [2]decad.Sphere
	for i, face := range [2]*decad.Face{point.FaceA, point.FaceB} {
		sphere, ok := face.Surface().(decad.Sphere)
		if !ok {
			return undecided(w, "sphere impact lacks a source sphere face"), nil
		}
		spheres[i] = sphere
		mass := w.parts[i].mass
		if mass.Mass.Value.Base()-mass.Mass.Bound.Base() <= 0 ||
			!finite(mass.Mass.Value.Base(), mass.Mass.Bound.Base()) {
			return undecided(w, "sphere mass interval is not positive and finite"), nil
		}
		inverse[i] = 1 / mass.Mass.Value.Base()
	}
	vA, vB := spherePairVelocity(pre.entries[0].LinearVelocity),
		spherePairVelocity(pre.entries[1].LinearVelocity)
	relative := vB.Sub(vA)
	closing := relative.Dot(n)
	preNorm, ok := sphereNormUpper(relative)
	if !finite(closing) || !ok {
		return undecided(w, "sphere closing speed cannot be bounded"), nil
	}
	preDot := sphereDotExact(relative, n)
	preError := new(big.Rat).Mul(ratFloat(preNorm), exactBase(point.Normal.Bound))
	closingUpper := new(big.Rat).Add(preDot, preError)
	if closingUpper.Cmp(new(big.Rat).Neg(exactBase(w.step.VelocityResidual))) >= 0 {
		return undecided(w, "sphere impact has no bounded closing normal speed"), nil
	}
	e := 0.0
	approachLow := new(big.Rat).Neg(new(big.Rat).Add(preDot, preError))
	approachHigh := new(big.Rat).Add(new(big.Rat).Neg(preDot), preError)
	if approachLow.Cmp(exactBase(w.step.ImpactSpeed)) > 0 {
		e = w.restitution.Base()
	} else if approachHigh.Cmp(exactBase(w.step.ImpactSpeed)) > 0 {
		return undecided(w, "sphere impact speed crosses the restitution threshold"), nil
	}
	impulse := -(1 + e) * closing / (inverse[0] + inverse[1])
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "sphere normal impulse is not finite and positive"), nil
	}
	for i, sphere := range spheres {
		if !sphereOmittedSpinWithin(roundedPoint, point, i, pre.entries[i].Pose, w.parts[i].mass,
			sphere.Radius, impulse, dt, w.step) {
			return undecided(w, "sphere omitted angular response exceeds its residual"), nil
		}
	}
	post := pre
	postA := vA.Sub(n.Scale(impulse * inverse[0]))
	postB := vB.Add(n.Scale(impulse * inverse[1]))
	if !finite(postA.X, postA.Y, postA.Z, postB.X, postB.Y, postB.Z) {
		return undecided(w, "sphere response velocity is not finite"), nil
	}
	post.entries[0].LinearVelocity = spherePairQuantityVelocity(postA)
	post.entries[1].LinearVelocity = spherePairQuantityVelocity(postB)
	if !spherePairResponseWithin([2]r3.Vec{vA, vB}, [2]r3.Vec{postA, postB},
		[2]decad.MassProperties{w.parts[0].mass, w.parts[1].mass},
		n, point.Normal.Bound.Base(), impulse, e, w.step) {
		return undecided(w, "sphere response normal residual or departure exceeds limit"), nil
	}
	travel, ok := boundBracketTravel(*first.Bracket, math.Hypot(relative.X,
		math.Hypot(relative.Y, relative.Z)))
	if !ok {
		return undecided(w, "sphere impact bracket travel is not bounded"), nil
	}
	allowance := outwardSum(travel, point.Separation.Bound.Base(), w.step.ContactSlop.Base())
	depth := -point.Separation.Value.Base() + w.step.ContactSlop.Base()/4
	if !finite(allowance, depth) || depth < 0 || depth > allowance {
		return undecided(w, "sphere impact penetration exceeds correction allowance"), nil
	}
	post, err := correctPair(post, n, depth, inverse)
	if err != nil {
		return undecidedArithmetic(w, "sphere position correction is not finite", err)
	}
	if !sphereCorrectionWithin(pre, post, allowance) {
		return undecided(w, "sphere position correction exceeds its allowance"), nil
	}
	contact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if contact.Relation != decad.ContactSeparated || contact.Gap == nil ||
		contact.Gap.Value.Base()+contact.Gap.Bound.Base() > allowance {
		return undecided(w, fmt.Sprintf("sphere corrected impact has relation %v", contact.Relation)), nil
	}
	remaining := dt.Base() - impactTime
	var actual *decad.SweepReport
	if remaining > 0 {
		certified, err := w.sweep(ctx, post, units.Seconds(remaining), decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if certified.Outcome != decad.SweepClear {
			return undecided(w, fmt.Sprintf("sphere separating remainder returned %v", certified.Outcome)), nil
		}
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "sphere response endpoint is not finite", err)
	}
	if remaining > 0 {
		actual, err = w.sweepPoses(ctx, post, end, units.Seconds(remaining), decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if actual.Outcome != decad.SweepClear {
			return undecided(w, fmt.Sprintf("rounded sphere remainder returned %v", actual.Outcome)), nil
		}
	}
	last, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if last.Relation != decad.ContactSeparated {
		return undecided(w, "sphere response endpoint is not proven separated"), nil
	}
	changeA := post.entries[0].Pose.Translation().Sub(pre.entries[0].Pose.Translation())
	changeB := post.entries[1].Pose.Translation().Sub(pre.entries[1].Pose.Translation())
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: *first.Bracket, Time: eventAt, Manifold: cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulse), TangentImpulse: zeroImpulseVec(),
		PreVelocity: kicked.entries[0].LinearVelocity, PostVelocity: post.entries[0].LinearVelocity,
		PositionChange: changeA, PreVelocityA: kicked.entries[0].LinearVelocity,
		PreVelocityB: kicked.entries[1].LinearVelocity, PostVelocityA: post.entries[0].LinearVelocity,
		PostVelocityB: post.entries[1].LinearVelocity, PositionChangeA: changeA,
		PositionChangeB: changeB}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: pre, post: post, end: end, duration: dt,
			eventAt: eventAt, hasEvent: true, preSweep: roundedPrefix, postSweep: actual}}, nil
}

func sphereDotExact(a, b r3.Vec) *big.Rat {
	out := new(big.Rat)
	for _, component := range [][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		out.Add(out, new(big.Rat).Mul(ratFloat(component[0]), ratFloat(component[1])))
	}
	return out
}

// sphereNormUpper checks the proposed float length against the exact square
// of the held vector before using it to charge normal-direction uncertainty.
func sphereNormUpper(v r3.Vec) (float64, bool) {
	squared := new(big.Rat)
	for _, component := range []float64{v.X, v.Y, v.Z} {
		if !finite(component) {
			return 0, false
		}
		q := ratFloat(component)
		squared.Add(squared, new(big.Rat).Mul(q, q))
	}
	if squared.Sign() == 0 {
		return 0, true
	}
	length := math.Hypot(v.X, math.Hypot(v.Y, v.Z))
	for range 16 {
		if !finite(length) {
			return 0, false
		}
		candidate := ratFloat(length)
		if new(big.Rat).Mul(candidate, candidate).Cmp(squared) >= 0 {
			return length, true
		}
		length = math.Nextafter(length, math.Inf(1))
	}
	return 0, false
}

func spherePairResponseWithin(pre, post [2]r3.Vec, mass [2]decad.MassProperties,
	n r3.Vec, normalBound, impulse, restitution float64, step StepConfig) bool {
	preRelative, postRelative := pre[1].Sub(pre[0]), post[1].Sub(post[0])
	postNorm, ok := sphereNormUpper(postRelative)
	if !ok {
		return false
	}
	preNorm, ok := sphereNormUpper(preRelative)
	if !ok {
		return false
	}
	preDot, postDot := sphereDotExact(preRelative, n), sphereDotExact(postRelative, n)
	nErr := ratFloat(normalBound)
	postUncertainty := new(big.Rat).Mul(ratFloat(postNorm), nErr)
	responseError := new(big.Rat).Add(postDot,
		new(big.Rat).Mul(ratFloat(restitution), preDot))
	responseError.Abs(responseError)
	responseError.Add(responseError, postUncertainty)
	responseError.Add(responseError, new(big.Rat).Mul(
		new(big.Rat).Mul(ratFloat(restitution), ratFloat(preNorm)), nErr))
	velocityLimit := exactBase(step.VelocityResidual)
	if responseError.Cmp(velocityLimit) > 0 ||
		new(big.Rat).Sub(postDot, postUncertainty).Cmp(velocityLimit) <= 0 {
		return false
	}
	impulseLimit := exactBase(step.ImpulseResidual)
	uncertainty := new(big.Rat).Mul(ratFloat(impulse), nErr)
	for i := range pre {
		center, bound := exactBase(mass[i].Mass.Value), exactBase(mass[i].Mass.Bound)
		if center == nil || bound == nil || bound.Sign() < 0 {
			return false
		}
		low, high := new(big.Rat).Sub(center, bound), new(big.Rat).Add(center, bound)
		if low.Sign() <= 0 {
			return false
		}
		sign := int64(1)
		if i == 0 {
			sign = -1
		}
		before := [3]float64{pre[i].X, pre[i].Y, pre[i].Z}
		after := [3]float64{post[i].X, post[i].Y, post[i].Z}
		normal := [3]float64{n.X, n.Y, n.Z}
		for axis := range 3 {
			change := new(big.Rat).Sub(ratFloat(after[axis]), ratFloat(before[axis]))
			applied := new(big.Rat).Mul(ratFloat(impulse), ratFloat(normal[axis]))
			applied.Mul(applied, big.NewRat(sign, 1))
			atLow := absRat(new(big.Rat).Sub(new(big.Rat).Mul(low, change), applied))
			atHigh := absRat(new(big.Rat).Sub(new(big.Rat).Mul(high, change), applied))
			residual := atLow
			if atHigh.Cmp(residual) > 0 {
				residual = atHigh
			}
			residual.Add(residual, uncertainty)
			limit := new(big.Rat).Add(impulseLimit, new(big.Rat).Mul(high, velocityLimit))
			if residual.Cmp(limit) > 0 {
				return false
			}
		}
	}
	return true
}

// sphereOmittedSpinWithin encloses the torque from the bounded witness and
// mass center, including the uncertain normal. The row-dominance inertia
// floor turns that torque into an upper angular-speed bound.
func sphereOmittedSpinWithin(witnessPoint, impulsePoint decad.ContactPoint, side int, pose r3.Transform,
	mass decad.MassProperties, radius units.Value, impulse float64, duration units.Value, step StepConfig) bool {
	if side < 0 || side > 1 || !finite(impulse) || impulse <= 0 {
		return false
	}
	witness := witnessPoint.OnA
	if side == 1 {
		witness = witnessPoint.OnB
	}
	center, centerError, ok := worldCenterReading(pose, mass.Center)
	lower := certifiedInertiaLower(mass)
	pointError, normalError := exactBase(witness.Bound), exactBase(impulsePoint.Normal.Bound)
	roundedNormalError := exactBase(witnessPoint.Normal.Bound)
	impulseLimit, angularLimit := exactBase(step.ImpulseResidual), exactBase(step.AngularVelocityResidual)
	if !ok || lower == nil || lower.Sign() <= 0 || pointError == nil || normalError == nil ||
		roundedNormalError == nil || pointError.Sign() < 0 || normalError.Sign() < 0 ||
		roundedNormalError.Sign() < 0 || impulseLimit == nil || angularLimit == nil {
		return false
	}
	normalError.Add(normalError, roundedNormalError)
	witnessValue := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
	normalValue := [3]float64{impulsePoint.Normal.Value.X, impulsePoint.Normal.Value.Y,
		impulsePoint.Normal.Value.Z}
	roundedNormal := [3]float64{witnessPoint.Normal.Value.X, witnessPoint.Normal.Value.Y,
		witnessPoint.Normal.Value.Z}
	var lever, leverError, normal [3]*big.Rat
	armUpper := new(big.Rat)
	for axis := range 3 {
		if !finite(witnessValue[axis], normalValue[axis], roundedNormal[axis]) {
			return false
		}
		normalError.Add(normalError, absRat(new(big.Rat).Sub(ratFloat(normalValue[axis]),
			ratFloat(roundedNormal[axis]))))
		lever[axis] = new(big.Rat).Sub(ratFloat(witnessValue[axis]), center[axis])
		leverError[axis] = new(big.Rat).Add(pointError, centerError[axis])
		normal[axis] = ratFloat(normalValue[axis])
		armUpper.Add(armUpper, absRat(new(big.Rat).Set(lever[axis])))
		armUpper.Add(armUpper, leverError[axis])
	}
	torqueLever := new(big.Rat)
	for axis := range 3 {
		j, k := (axis+1)%3, (axis+2)%3
		cross := new(big.Rat).Sub(new(big.Rat).Mul(lever[j], normal[k]),
			new(big.Rat).Mul(lever[k], normal[j]))
		term := absRat(cross)
		term.Add(term, new(big.Rat).Mul(leverError[j], absRat(new(big.Rat).Set(normal[k]))))
		term.Add(term, new(big.Rat).Mul(leverError[k], absRat(new(big.Rat).Set(normal[j]))))
		for _, tangent := range []int{j, k} {
			arm := new(big.Rat).Add(absRat(new(big.Rat).Set(lever[tangent])), leverError[tangent])
			term.Add(term, new(big.Rat).Mul(arm, normalError))
		}
		torqueLever.Add(torqueLever, term)
	}
	upperImpulse := new(big.Rat).Add(ratFloat(impulse), impulseLimit)
	spin := new(big.Rat).Quo(new(big.Rat).Mul(upperImpulse, torqueLever), lower)
	if spin.Cmp(angularLimit) > 0 {
		return false
	}
	velocityLimit := exactBase(step.VelocityResidual)
	inertiaUpper := inertiaRowCeiling(mass.Inertia)
	if velocityLimit == nil || inertiaUpper == nil ||
		new(big.Rat).Mul(spin, armUpper).Cmp(velocityLimit) > 0 {
		return false
	}
	// The 1/2 factors on rotational energy and impulse times the allowed
	// contact-point speed cancel in this comparison.
	energy := new(big.Rat).Mul(inertiaUpper, new(big.Rat).Mul(spin, spin))
	if energy.Cmp(new(big.Rat).Mul(upperImpulse, velocityLimit)) > 0 {
		return false
	}
	// The contact witness can lie on the near side of the center of mass.
	// Adding two radii also covers the opposite extreme of the source ball.
	armUpper.Add(armUpper, new(big.Rat).Mul(big.NewRat(2, 1), exactBase(radius)))
	travel := new(big.Rat).Mul(spin, exactBase(duration))
	travel.Mul(travel, armUpper)
	return travel.Cmp(exactBase(step.Contact.PointResolution)) <= 0
}

func sphereCorrectionWithin(before, after State, allowance float64) bool {
	travel := 0.0
	for i := range before.entries {
		delta := after.entries[i].Pose.Translation().Sub(before.entries[i].Pose.Translation())
		travel = outwardSum(travel, math.Hypot(delta.X, math.Hypot(delta.Y, delta.Z)))
	}
	return finite(travel) && travel <= allowance
}
