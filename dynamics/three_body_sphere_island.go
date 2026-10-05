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

// A sphere between a plane and another sphere has coupled normal impulses:
// each impulse changes the velocity seen at both contacts.
type sphereIslandContact struct {
	key         int
	pair        *World
	sweep       *decad.SweepReport
	point       decad.ContactPoint
	normal      r3.Vec
	outward     r3.Vec
	dynamic     int
	normalError float64
	impulse     float64
}

func threeHasSpherePair(sweeps [3]*decad.SweepReport) bool {
	for _, sweep := range sweeps {
		if sweep != nil && sweep.Event != nil && isObliqueSpherePairEvent(sweep.Event.Manifold) {
			return true
		}
	}
	return false
}

func (w *World) stepThreeSphereIsland(ctx context.Context, from, kicked State,
	gravity QuantityVec, loads [2]*BodyLoad, dt units.Value, sweeps [3]*decad.SweepReport,
	reference *World) (*StepReport, error) {
	contacts := make([]sphereIslandContact, 0, 2)
	spherePairs := 0
	for key, sweep := range sweeps {
		if sweep == nil {
			continue
		}
		pair := w.three.pairs[key]
		if sweep.Outcome != decad.SweepInitiallyTouching || sweep.Event == nil ||
			sweep.Event.Relation != decad.ContactTouching || sweep.Event.Manifold == nil ||
			len(sweep.Event.Manifold.Points) != 1 || pair.friction.upper.Sign() != 0 ||
			pair.restitution.Base() != 0 {
			return w.threeUndecided(key, "sphere island needs one frictionless zero-restitution point per pair"), nil
		}
		dynamic := 0
		if pair.parts[1].definition.Role == Dynamic {
			dynamic = 1
		}
		point := sweep.Event.Manifold.Points[0]
		if !sphereFaceOnBody([]*decad.Face{point.FaceA, point.FaceB}[dynamic],
			pair.parts[dynamic].definition.Body) {
			return w.threeUndecided(key, "sphere island lacks the dynamic source sphere witness"), nil
		}
		isPair := isObliqueSpherePairEvent(sweep.Event.Manifold)
		var normal r3.Vec
		var separation, bound float64
		var ok bool
		if isPair {
			spherePairs++
			normal, separation, bound, ok = boundedObliqueSphereContact(point, w.step.Contact)
		} else {
			fixedFace := []*decad.Face{point.FaceB, point.FaceA}[dynamic]
			if !boxFaceOnBody(fixedFace, pair.parts[1-dynamic].definition.Body) {
				return w.threeUndecided(key, "sphere island needs a source box face"), nil
			}
			normal, separation, bound, ok = reducedContact(sweep.Event.Manifold, w.step.Contact)
			_, _, cardinal := axisNormal(normal)
			ok = ok && cardinal
		}
		if !ok || !finite(separation, bound) ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
			return w.threeUndecided(key, "sphere island point exceeds contact bounds"), nil
		}
		outward := normal
		if dynamic == 0 {
			outward = outward.Scale(-1)
		}
		contacts = append(contacts, sphereIslandContact{key: key, pair: pair, sweep: sweep,
			point: point, normal: normal, outward: outward, dynamic: dynamic,
			normalError: outwardSum(point.Normal.Bound.Base(), point.NormalAngle.Base())})
	}
	if len(contacts) != 2 || spherePairs != 1 {
		return w.threeUndecided(0, "sphere island needs one sphere pair and one sphere face pair"), nil
	}
	preDynamic, _ := kicked.Body(w.three.parts[w.three.dynamic].Body)
	if !zeroAngularVelocity(preDynamic.AngularVelocity) {
		return w.threeUndecided(contacts[0].key, "sphere island starts with unresolved spin"), nil
	}
	mass := contacts[0].pair.parts[contacts[0].dynamic].mass
	massValue, massBound := mass.Mass.Value.Base(), mass.Mass.Bound.Base()
	if !finite(massValue, massBound) || massValue-massBound <= 0 {
		return w.threeUndecided(contacts[0].key, "sphere island mass is not positive and finite"), nil
	}
	before := spherePairVelocity(preDynamic.LinearVelocity)
	after, impulse, ok := solveSphereIslandNormals(before, massValue, contacts,
		w.step.VelocityResidual.Base(), w.step.ImpulseResidual.Base())
	if !ok || !sphereIslandMomentumWithin(before, after, massValue, massBound,
		contacts, impulse, w.step.ImpulseResidual.Base()) ||
		!sphereIslandEnergyWithin(before, after, massValue, massBound,
			w.step.VelocityResidual.Base(), w.step.ImpulseResidual.Base()) {
		return w.threeUndecided(contacts[0].key, "sphere island normal or impulse residual is unproved"), nil
	}
	angularBudget, ok := simultaneousAngularBudget(w, mass, dt, preDynamic.Pose, len(contacts))
	if !ok {
		return w.threeUndecided(contacts[0].key, "sphere island angular travel cannot be bounded"), nil
	}
	spinStep := w.step
	spinStep.AngularVelocityResidual = angularBudget
	totalTravel, totalPointSpeed := new(big.Rat), new(big.Rat)
	spins := make([]sphereOmittedBounds, 0, len(contacts))
	for i := range contacts {
		contacts[i].impulse = impulse[i]
		if impulse[i] == 0 {
			continue
		}
		face := []*decad.Face{contacts[i].point.FaceA, contacts[i].point.FaceB}[contacts[i].dynamic]
		sphere, sphereOK := face.Surface().(decad.Sphere)
		if !sphereOK {
			return w.threeUndecided(contacts[i].key, "sphere island loses its source sphere face"), nil
		}
		spin, spinOK := sphereOmittedSpinBounds(contacts[i].point, contacts[i].point,
			contacts[i].dynamic, preDynamic.Pose, mass, sphere.Radius, impulse[i], dt, spinStep)
		if !spinOK {
			return w.threeUndecided(contacts[i].key, "sphere island omitted spin exceeds its residual"), nil
		}
		totalTravel.Add(totalTravel, spin.travel)
		totalPointSpeed.Add(totalPointSpeed, spin.pointSpeed)
		spins = append(spins, spin)
	}
	totalAngularSpeed, totalTwiceEnergy, spinOK := sphereIslandCombinedSpin(mass, spins)
	if !spinOK || totalAngularSpeed.Cmp(exactBase(w.step.AngularVelocityResidual)) > 0 ||
		totalTravel.Cmp(exactBase(w.step.Contact.PointResolution)) > 0 ||
		totalTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 ||
		!sphereIslandSpinWithin(after, contacts, impulse, totalPointSpeed,
			totalTwiceEnergy, w.step) {
		return w.threeUndecided(contacts[0].key, "sphere island omitted spin exceeds response residuals"), nil
	}
	post := kicked
	preDynamic.LinearVelocity = spherePairQuantityVelocity(after)
	post = withBodyState(post, preDynamic)
	var events []ContactEvent
	var totalImpulse [3]*big.Rat
	for axis := range totalImpulse {
		totalImpulse[axis] = new(big.Rat)
	}
	lastActive := -1
	for i, contact := range contacts {
		if contact.impulse > 0 {
			lastActive = i
		}
	}
	current := before
	for i, contact := range contacts {
		if contact.impulse == 0 {
			continue
		}
		previous := current
		current = current.Add(contact.outward.Scale(contact.impulse / massValue))
		if i == lastActive {
			current = after
		}
		beforePair := pairState(kicked, contact.pair)
		afterPair := beforePair
		beforePair.entries[contact.dynamic].LinearVelocity = spherePairQuantityVelocity(previous)
		afterPair.entries[contact.dynamic].LinearVelocity = spherePairQuantityVelocity(current)
		impulseValue := units.KilogramMillimetersPerSecond(contact.impulse)
		instant := contact.sweep.Event.At
		events = append(events, ContactEvent{
			Kind: ContactImpact, Pair: BodyPair{A: contact.pair.parts[0].definition.Body,
				B: contact.pair.parts[1].definition.Body},
			Bracket: decad.SweepInterval{From: instant, To: instant},
			Time:    instant.Elapsed.Value, Manifold: cloneManifold(*contact.sweep.Event.Manifold),
			NormalImpulse: impulseValue, TangentImpulse: zeroImpulseVec(),
			PointImpulses: []ContactPointImpulse{{Normal: impulseValue, Tangent: zeroImpulseVec()}},
			PreVelocity:   beforePair.entries[contact.dynamic].LinearVelocity,
			PostVelocity:  afterPair.entries[contact.dynamic].LinearVelocity,
			PreVelocityA:  beforePair.entries[0].LinearVelocity,
			PreVelocityB:  beforePair.entries[1].LinearVelocity,
			PostVelocityA: afterPair.entries[0].LinearVelocity,
			PostVelocityB: afterPair.entries[1].LinearVelocity,
			Solver: &ContactSolverReport{NormalResidual: w.step.VelocityResidual,
				TangentResidual:     units.MillimetersPerSecond(0),
				ConeResidual:        units.KilogramMillimetersPerSecond(0),
				PenetrationResidual: w.step.PenetrationResidual,
				AngularUpper:        angularBudget, Iterations: 1},
		})
		for axis, value := range []float64{contact.outward.X, contact.outward.Y, contact.outward.Z} {
			totalImpulse[axis].Add(totalImpulse[axis], new(big.Rat).Mul(
				exactBase(impulseValue), ratFloat(value)))
		}
	}
	if len(events) >= w.step.MaxEvents {
		return w.threeUndecided(contacts[0].key, "sphere island reaches the event limit"), nil
	}
	endPair, err := driftState(pairState(post, reference), dt.Base())
	if err != nil {
		return w.threeUndecidedArithmetic(contacts[0].key, "sphere island final pose is not finite", err)
	}
	end := withPairState(post, endPair)
	var rounded [3]*decad.SweepReport
	for _, contact := range contacts {
		mode := decad.ContinueCertifiedTouch
		if contact.outward.Dot(after) > w.step.VelocityResidual.Base() {
			mode = decad.ContinueSeparatingTouch
		}
		startPair, endPair := pairState(post, contact.pair), pairState(end, contact.pair)
		ideal, sweepErr := contact.pair.sweep(ctx, startPair, dt, mode)
		if sweepErr != nil {
			return nil, sweepErr
		}
		expected := ideal.Outcome
		if contact.impulse > 0 && expected != decad.SweepPersistentTouch ||
			!sphereIslandTrackWithin(ideal, contact, expected, w.step) {
			return w.threeUndecided(contact.key,
				fmt.Sprintf("sphere island ideal remainder returned %v", ideal.Outcome)), nil
		}
		actual, sweepErr := contact.pair.sweepPoses(ctx, startPair, endPair, dt, mode)
		if sweepErr != nil {
			return nil, sweepErr
		}
		if !sphereIslandTrackWithin(actual, contact, expected, w.step) {
			return w.threeUndecided(contact.key,
				fmt.Sprintf("sphere island rounded remainder returned %v", actual.Outcome)), nil
		}
		rounded[contact.key] = actual
		final, contactErr := w.doc.ContactPair(ctx, contact.pair.parts[0].definition.Body,
			contact.pair.parts[1].definition.Body, endPair.entries[0].Pose,
			endPair.entries[1].Pose, w.step.Contact)
		if contactErr != nil {
			return nil, contactErr
		}
		if expected == decad.SweepDepartedClear {
			if final.Relation != decad.ContactSeparated {
				return w.threeUndecided(contact.key, "sphere island departing endpoint is not separated"), nil
			}
			continue
		}
		if final.Relation != decad.ContactTouching || final.Manifold == nil ||
			len(final.Manifold.Points) != 1 {
			return w.threeUndecided(contact.key, "sphere island persistent endpoint lacks a point"), nil
		}
		var finalNormal r3.Vec
		var separation, bound float64
		var valid bool
		if isObliqueSpherePairEvent(final.Manifold) {
			finalNormal, separation, bound, valid = boundedObliqueSphereContact(
				final.Manifold.Points[0], w.step.Contact)
		} else {
			finalNormal, separation, bound, valid = reducedContact(final.Manifold, w.step.Contact)
		}
		if !valid || finalNormal != contact.normal ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
			return w.threeUndecided(contact.key, "sphere island endpoint exceeds its contact bounds"), nil
		}
	}
	trace := Trace{start: from, pre: kicked, post: post, end: end,
		duration: dt, eventAt: units.Seconds(0), hasEvent: len(events) > 0,
		threeSweeps: rounded}
	childTrace := Trace{start: pairState(from, reference), pre: pairState(kicked, reference),
		post: pairState(post, reference), end: pairState(end, reference), duration: dt,
		eventAt: units.Seconds(0), hasEvent: len(events) > 0}
	conservation, ok := reference.conservationReadings(childTrace.start, childTrace.pre,
		childTrace.end, childTrace, nil, gravity, loads, dt)
	if !ok {
		return w.threeUndecided(contacts[0].key, "sphere island conservation is not finite"), nil
	}
	contactImpulse, ok := boundedMomentum(totalImpulse, totalImpulse, totalImpulse)
	if !ok {
		return w.threeUndecided(contacts[0].key, "sphere island contact impulse is not finite"), nil
	}
	conservation.ContactImpulse = contactImpulse
	return &StepReport{Status: Advanced, Next: &end, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, nil
}

func withBodyState(state State, entry BodyState) State {
	out := state
	for i := range out.entries {
		if out.entries[i].Body == entry.Body {
			out.entries[i] = entry
		}
	}
	if out.third.Body == entry.Body {
		out.third = entry
	}
	return out
}

// The four possible active sets are small enough to solve directly. A
// contact without impulse must be separating or resting after the solve.
func solveSphereIslandNormals(v r3.Vec, mass float64, contacts []sphereIslandContact,
	velocityLimit, impulseLimit float64) (r3.Vec, [2]float64, bool) {
	var impulse [2]float64
	if len(contacts) != 2 || !finite(mass, velocityLimit, impulseLimit) || mass <= 0 ||
		velocityLimit <= 0 || impulseLimit < 0 {
		return r3.Vec{}, impulse, false
	}
	q0, q1 := contacts[0].outward, contacts[1].outward
	k00, k11, k01 := q0.Dot(q0), q1.Dot(q1), q0.Dot(q1)
	det := k00*k11 - k01*k01
	errorBound := outwardSum(contacts[0].normalError, contacts[1].normalError)
	// One normal is cardinal and exact. Even after the other normal's
	// certified displacement, their dot product must stay strictly below 1.
	if !finite(k00, k11, k01, det, errorBound) ||
		new(big.Rat).Add(absRat(sphereDotExact(q0, q1)), ratFloat(errorBound)).Cmp(big.NewRat(1, 1)) >= 0 ||
		det <= 0 {
		return r3.Vec{}, impulse, false
	}
	u0, u1 := q0.Dot(v), q1.Dot(v)
	type candidate struct{ j0, j1 float64 }
	candidates := []candidate{
		{mass * (-u0*k11 + u1*k01) / det, mass * (-u1*k00 + u0*k01) / det},
		{-mass * u0 / k00, 0}, {0, -mass * u1 / k11}, {0, 0},
	}
	for _, candidate := range candidates {
		if !finite(candidate.j0, candidate.j1) || candidate.j0 < 0 || candidate.j1 < 0 ||
			(candidate.j0 > 0 && candidate.j0 <= impulseLimit) ||
			(candidate.j1 > 0 && candidate.j1 <= impulseLimit) {
			continue
		}
		post := v.Add(q0.Scale(candidate.j0 / mass)).Add(q1.Scale(candidate.j1 / mass))
		for axis, value := range []float64{post.X, post.Y, post.Z} {
			if math.Abs(value) <= velocityLimit/4 {
				setVecComponent(&post, axis, 0)
			}
		}
		speed, valid := sphereNormUpper(post)
		for i, j := range [2]float64{candidate.j0, candidate.j1} {
			normalSpeed := sphereDotExact(contacts[i].outward, post)
			uncertainty := new(big.Rat).Mul(ratFloat(speed), ratFloat(contacts[i].normalError))
			if j > 0 && new(big.Rat).Add(absRat(new(big.Rat).Set(normalSpeed)),
				uncertainty).Cmp(ratFloat(velocityLimit)) > 0 {
				valid = false
			}
			if j == 0 && new(big.Rat).Sub(normalSpeed, uncertainty).Cmp(
				new(big.Rat).Neg(ratFloat(velocityLimit))) < 0 {
				valid = false
			}
		}
		if valid {
			return post, [2]float64{candidate.j0, candidate.j1}, true
		}
	}
	return r3.Vec{}, impulse, false
}

func setVecComponent(v *r3.Vec, axis int, value float64) {
	switch axis {
	case 0:
		v.X = value
	case 1:
		v.Y = value
	default:
		v.Z = value
	}
}

func sphereIslandMomentumWithin(before, after r3.Vec, mass, massBound float64,
	contacts []sphereIslandContact, impulse [2]float64, limit float64) bool {
	if !finite(mass, massBound, limit) || massBound < 0 || limit < 0 {
		return false
	}
	uncertainty := new(big.Rat)
	for i, contact := range contacts {
		uncertainty.Add(uncertainty, new(big.Rat).Mul(ratFloat(impulse[i]),
			ratFloat(contact.normalError)))
	}
	beforeAxes := [3]float64{before.X, before.Y, before.Z}
	afterAxes := [3]float64{after.X, after.Y, after.Z}
	for axis := range beforeAxes {
		change := new(big.Rat).Sub(ratFloat(afterAxes[axis]), ratFloat(beforeAxes[axis]))
		applied := new(big.Rat)
		for i, contact := range contacts {
			component := [3]float64{contact.outward.X, contact.outward.Y, contact.outward.Z}[axis]
			applied.Add(applied, new(big.Rat).Mul(ratFloat(impulse[i]), ratFloat(component)))
		}
		residual := absRat(new(big.Rat).Sub(new(big.Rat).Mul(ratFloat(mass), change), applied))
		residual.Add(residual, new(big.Rat).Mul(ratFloat(massBound),
			absRat(new(big.Rat).Set(change))))
		residual.Add(residual, uncertainty)
		if residual.Cmp(ratFloat(limit)) > 0 {
			return false
		}
	}
	return true
}

func sphereIslandEnergyWithin(before, after r3.Vec, mass, massBound,
	velocityLimit, impulseLimit float64) bool {
	if !finite(mass, massBound, velocityLimit, impulseLimit) ||
		massBound < 0 || velocityLimit < 0 || impulseLimit < 0 {
		return false
	}
	difference, allowance := new(big.Rat), new(big.Rat)
	upperMass := new(big.Rat).Add(ratFloat(mass), ratFloat(massBound))
	for _, pair := range [3][2]float64{{before.X, after.X}, {before.Y, after.Y}, {before.Z, after.Z}} {
		initial, final := ratFloat(pair[0]), ratFloat(pair[1])
		difference.Add(difference, new(big.Rat).Sub(
			new(big.Rat).Mul(final, final), new(big.Rat).Mul(initial, initial)))
		speedSum := new(big.Rat).Add(absRat(new(big.Rat).Set(initial)),
			absRat(new(big.Rat).Set(final)))
		speedSum.Add(speedSum, ratFloat(velocityLimit))
		budget := new(big.Rat).Add(new(big.Rat).Mul(upperMass, ratFloat(velocityLimit)),
			ratFloat(impulseLimit))
		allowance.Add(allowance, new(big.Rat).Mul(budget, speedSum))
	}
	if difference.Sign() <= 0 {
		return true
	}
	gain := new(big.Rat).Mul(upperMass, difference)
	gain.Quo(gain, big.NewRat(2, 1))
	return gain.Cmp(allowance) <= 0
}

// The two torque bounds act on the same body. Their angular speeds add before
// the inertia ceiling is applied; summing separate energies loses the cross term.
func sphereIslandCombinedSpin(mass decad.MassProperties,
	spins []sphereOmittedBounds) (*big.Rat, *big.Rat, bool) {
	upper := inertiaRowCeiling(mass.Inertia)
	if upper == nil {
		return nil, nil, false
	}
	angular := new(big.Rat)
	for _, spin := range spins {
		if spin.angularSpeed == nil || spin.angularSpeed.Sign() < 0 {
			return nil, nil, false
		}
		angular.Add(angular, spin.angularSpeed)
	}
	twiceEnergy := new(big.Rat).Mul(upper, new(big.Rat).Mul(angular, angular))
	return angular, twiceEnergy, true
}

func sphereIslandSpinWithin(after r3.Vec, contacts []sphereIslandContact,
	impulse [2]float64, pointSpeed, twiceEnergy *big.Rat, step StepConfig) bool {
	if pointSpeed == nil || twiceEnergy == nil || pointSpeed.Sign() < 0 || twiceEnergy.Sign() < 0 {
		return false
	}
	speed, ok := sphereNormUpper(after)
	if !ok {
		return false
	}
	limit := exactBase(step.VelocityResidual)
	maxResidual, totalImpulse := new(big.Rat), new(big.Rat)
	for i, contact := range contacts {
		normalSpeed := sphereDotExact(contact.outward, after)
		uncertainty := new(big.Rat).Mul(ratFloat(speed), ratFloat(contact.normalError))
		residual := new(big.Rat)
		if impulse[i] > 0 {
			residual.Add(absRat(normalSpeed), uncertainty)
			totalImpulse.Add(totalImpulse, ratFloat(impulse[i]))
		} else {
			residual.Sub(uncertainty, normalSpeed)
			if residual.Sign() < 0 {
				residual.SetInt64(0)
			}
		}
		if residual.Cmp(maxResidual) > 0 {
			maxResidual = residual
		}
	}
	remaining := new(big.Rat).Sub(limit, maxResidual)
	if remaining.Sign() < 0 || pointSpeed.Cmp(remaining) > 0 {
		return false
	}
	remaining.Sub(remaining, pointSpeed)
	allowance := new(big.Rat).Add(totalImpulse, exactBase(step.ImpulseResidual))
	allowance.Mul(allowance, remaining)
	return twiceEnergy.Cmp(allowance) <= 0
}

func sphereIslandTrackWithin(sweep *decad.SweepReport, contact sphereIslandContact,
	expected decad.SweepOutcome, step StepConfig) bool {
	if sweep == nil || !sweep.HasAffineReplayProof() {
		return false
	}
	if expected == decad.SweepDepartedClear {
		return sweep.Outcome == decad.SweepDepartedClear && sweep.Departure != nil &&
			sweep.Departure.GapAtUntil.Value.Base()-sweep.Departure.GapAtUntil.Bound.Base() > 0
	}
	if expected != decad.SweepPersistentTouch {
		return false
	}
	if sweep.Outcome != decad.SweepPersistentTouch || sweep.ContactTrack == nil ||
		sweep.ContactTrack.Start().Fraction.Base() != 0 ||
		sweep.ContactTrack.End().Fraction.Base() != 1 {
		return false
	}
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(.5), units.Scalar(1)} {
		manifold, err := sweep.ContactTrack.ManifoldAt(fraction)
		if err != nil || manifold == nil || len(manifold.Points) != 1 {
			return false
		}
		var normal r3.Vec
		var separation, bound float64
		var ok bool
		if isObliqueSpherePairEvent(manifold) {
			normal, separation, bound, ok = boundedObliqueSphereContact(manifold.Points[0], step.Contact)
		} else {
			normal, separation, bound, ok = reducedContact(manifold, step.Contact)
		}
		if !ok || normal != contact.normal ||
			outwardSum(math.Abs(separation), bound) > step.PenetrationResidual.Base() {
			return false
		}
	}
	return true
}
