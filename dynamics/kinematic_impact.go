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

// stepKinematicImpact consumes the geometry producer's bracket-right pose.
// The driver stays prescribed; only the dynamic body receives correction and impulse.
func (w *World) stepKinematicImpact(ctx context.Context, from, kicked State, dt units.Value,
	motion kinematicMotion, first *decad.SweepReport) (*StepReport, error) {
	if first.Bracket == nil || first.Event == nil {
		return undecided(w, "kinematic impact lacks a bracket and event"), nil
	}
	chosen := first.Bracket.To.Elapsed.Value.Base()
	if !finite(chosen) || chosen <= 0 || chosen >= dt.Base() {
		return undecided(w, "kinematic impact time is outside the interior step"), nil
	}
	eventAt := first.Bracket.To.Elapsed.Value
	fraction := exactBase(first.Bracket.To.Fraction)
	if fraction == nil || fraction.Sign() <= 0 || fraction.Cmp(big.NewRat(1, 1)) >= 0 ||
		exactBase(eventAt).Cmp(exactBase(dt)) >= 0 {
		return undecided(w, "kinematic impact exceeds the exact step duration"), nil
	}
	pre := kicked
	found := false
	for _, sample := range first.Samples {
		if exactBase(sample.At.Fraction).Cmp(exactBase(first.Bracket.To.Fraction)) == 0 {
			pre.entries[0].Pose, pre.entries[1].Pose = sample.PoseA, sample.PoseB
			found = true
			break
		}
	}
	if !found {
		return undecided(w, "kinematic impact has no bracket-right pose sample"), nil
	}
	prefixMotion := motion
	prefixMotion.path = decad.PoseSegment{From: kicked.entries[motion.index].Pose,
		To: pre.entries[motion.index].Pose, Duration: units.Seconds(chosen)}
	roundedPrefix, err := w.sweepKinematicPoses(ctx, kicked, pre, units.Seconds(chosen),
		prefixMotion, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if first.HasAffineReplayProof() &&
		!roundedImpactPrefixAtEnd(roundedPrefix, first, w.step.PenetrationResidual) {
		return undecided(w, "published kinematic impact prefix lacks a rounded endpoint bracket"), nil
	}
	contactAtRight, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		pre.entries[0].Pose, pre.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if contactAtRight.Relation != first.Event.Relation || contactAtRight.Manifold == nil ||
		len(contactAtRight.Manifold.Points) == 0 {
		return undecided(w, "kinematic bracket-right pose lacks its reported contact"), nil
	}
	manifold := contactAtRight.Manifold
	normal, separation, bound, valid := reducedContact(manifold, w.step.Contact)
	if !valid || !finite(separation, bound) || separation > bound {
		return undecided(w, "kinematic impact normal or penetration is not certified"), nil
	}
	axis, sign, valid := axisNormal(normal)
	if !valid {
		return undecided(w, "kinematic impact normal is unsupported"), nil
	}
	dynamic := 1 - motion.index
	preSpeed := [2]units.Value{
		velocityComponent(kicked.entries[0].LinearVelocity, axis),
		velocityComponent(kicked.entries[1].LinearVelocity, axis),
	}
	preSpeed[motion.index] = velocityComponent(motion.effective, axis)
	relative := (preSpeed[1].Base() - preSpeed[0].Base()) * sign
	if !finite(relative) || relative >= -w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic bracket is not certified closing"), nil
	}
	if w.step.MaxEvents <= 1 {
		return undecided(w, "kinematic impact reaches the event limit with time remaining"), nil
	}
	inverseMass := 1 / w.parts[dynamic].mass.Mass.Value.Base()
	if !finite(inverseMass) || inverseMass <= 0 {
		return undecided(w, "kinematic impact effective mass is invalid"), nil
	}
	coefficient := w.restitution
	idealRelative := new(big.Rat).Sub(exactBase(preSpeed[1]), exactBase(preSpeed[0]))
	idealRelative.Mul(idealRelative, big.NewRat(int64(sign), 1))
	target := 0.0
	effectiveCoefficient := units.Scalar(0)
	if new(big.Rat).Neg(idealRelative).Cmp(exactBase(w.step.ImpactSpeed)) > 0 {
		effectiveCoefficient = coefficient
		target = -coefficient.Base() * relative
	}
	impulse := (target - relative) / inverseMass
	if !finite(target, impulse) || impulse <= 0 {
		return undecided(w, "kinematic impact impulse is not finite and positive"), nil
	}
	postSpeed := [2]float64{preSpeed[0].Base(), preSpeed[1].Base()}
	if dynamic == 0 {
		postSpeed[0] -= impulse * sign * inverseMass
	} else {
		postSpeed[1] += impulse * sign * inverseMass
	}
	if !responsePairResidualsWithin(preSpeed, sign, effectiveCoefficient, w.parts,
		target, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) ||
		!omittedSpinWithin(manifold, pre.entries[dynamic].Pose, w.parts[dynamic].mass,
			dynamic, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
		return undecided(w, "kinematic impact exceeds velocity, impulse, or spin residual"), nil
	}
	bracketTravel, valid := boundBracketTravel(*first.Bracket, preSpeed[1].Base()-preSpeed[0].Base())
	if !valid {
		return undecided(w, "kinematic bracket travel is not bounded"), nil
	}
	allowance := outwardSum(bracketTravel, w.step.ContactSlop.Base(), bound)
	if !finite(allowance) || -separation > allowance {
		return undecided(w, "kinematic impact penetration exceeds the bracket allowance"), nil
	}
	inverse := [2]float64{}
	inverse[dynamic] = inverseMass
	post, err := correctPair(pre, normal, -separation, inverse)
	if err != nil {
		return undecidedArithmetic(w, "kinematic impact correction is not finite", err)
	}
	if !pairCorrectionWithin(pre, post, axis, allowance) {
		return undecided(w, "kinematic impact correction exceeds its allowance"), nil
	}
	setVelocityComponent(&post.entries[dynamic].LinearVelocity, axis,
		units.MillimetersPerSecond(postSpeed[dynamic]))
	contact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	for iteration := 1; iteration < w.step.MaxIterations && contact.Relation == decad.ContactOverlapping; iteration++ {
		if contact.Manifold == nil {
			break
		}
		contactNormal, contactSeparation, contactBound, valid := reducedContact(contact.Manifold, w.step.Contact)
		if !valid || contactNormal != normal || contactSeparation >= 0 ||
			!finite(contactSeparation, contactBound) {
			break
		}
		candidate, err := correctPair(post, normal, -contactSeparation, inverse)
		if err != nil {
			return undecidedArithmetic(w, "kinematic impact correction is not finite", err)
		}
		allowance = outwardSum(allowance, contactBound)
		if !finite(allowance) || !pairCorrectionWithin(pre, candidate, axis, allowance) {
			break
		}
		post = candidate
		contact, err = w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
			post.entries[0].Pose, post.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
	}
	if contact.Relation != decad.ContactTouching {
		return undecided(w, fmt.Sprintf("kinematic corrected impact has relation %v", contact.Relation)), nil
	}
	if motion.screw != nil {
		contactNormal, contactSeparation, contactBound, ok := reducedContact(contact.Manifold, w.step.Contact)
		if !ok || contactNormal != normal ||
			math.Abs(contactSeparation)+contactBound > w.step.PenetrationResidual.Base() {
			return undecided(w, "rotating impact lacks an axis-normal touching support face"), nil
		}
	}
	remaining := dt.Base() - chosen
	postRelative := (postSpeed[1] - postSpeed[0]) * sign
	if !finite(remaining, postRelative) || remaining <= 0 ||
		postRelative < -w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic impact response remains closing"), nil
	}
	persistent := postRelative <= w.step.VelocityResidual.Base()
	if motion.screw != nil && persistent {
		return undecided(w, "rotating driver lacks a persistent-contact certificate"), nil
	}
	if motion.screw != nil {
		// The original prescribed screw and the sliced sweep share a cardinal
		// support axis. Rotation about it cannot change either face's normal
		// coordinate, so this exact gap rate certifies the original remainder.
		pairSpeed := [2]units.Value{
			velocityComponent(post.entries[0].LinearVelocity, axis),
			velocityComponent(post.entries[1].LinearVelocity, axis),
		}
		pairSpeed[motion.index] = velocityComponent(motion.effective, axis)
		gapRate := new(big.Rat).Sub(exactBase(pairSpeed[1]), exactBase(pairSpeed[0]))
		gapRate.Mul(gapRate, big.NewRat(int64(sign), 1))
		if gapRate.Cmp(exactBase(w.step.VelocityResidual)) <= 0 {
			return undecided(w, "original rotating driver lacks a strict departure rate"), nil
		}
		for other := range 3 {
			if other != axis && exactBase(velocityComponent(post.entries[dynamic].AngularVelocity,
				other)).Sign() != 0 {
				return undecided(w, "rotating response changes its support-plane axis"), nil
			}
		}
	}
	policy := decad.ContinueSeparatingTouch
	if persistent {
		policy = decad.ContinueCertifiedTouch
	}
	remainderMotion, valid := w.sliceKinematicMotion(motion, post.entries[motion.index].Pose,
		units.Seconds(remaining), normal, contact.Manifold)
	if !valid {
		return undecided(w, "kinematic driver remainder differs from its admitted speed"), nil
	}
	ideal, err := w.sweepKinematic(ctx, post, units.Seconds(remaining), remainderMotion,
		policy)
	if err != nil {
		return nil, err
	}
	if (persistent && !w.persistentTrackWithin(ideal, normal)) ||
		(!persistent && ideal.Outcome != decad.SweepDepartedClear) {
		return undecided(w, fmt.Sprintf("kinematic continuation returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, remaining)
	if err != nil {
		return undecidedArithmetic(w, "kinematic rebound pose is not finite", err)
	}
	end.entries[motion.index].Pose = motion.path.To
	rounded, err := w.sweepKinematicPoses(ctx, post, end, units.Seconds(remaining), remainderMotion,
		policy)
	if err != nil {
		return nil, err
	}
	if (persistent && !w.persistentTrackWithin(rounded, normal)) ||
		(!persistent && rounded.Outcome != decad.SweepDepartedClear) {
		return undecided(w, fmt.Sprintf("rounded kinematic continuation returned %v", rounded.Outcome)), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if persistent {
		if finalContact.Relation != decad.ContactTouching || finalContact.Manifold == nil {
			return undecided(w, "kinematic persistent endpoint lacks contact"), nil
		}
		endNormal, endSeparation, endBound, ok := reducedContact(finalContact.Manifold, w.step.Contact)
		if !ok || endNormal != normal || !finite(endSeparation, endBound) ||
			math.Abs(endSeparation)+endBound > w.step.PenetrationResidual.Base() {
			return undecided(w, "kinematic persistent endpoint exceeds penetration residual"), nil
		}
	} else if finalContact.Relation != decad.ContactSeparated {
		return undecided(w, "kinematic impact endpoint is not separated"), nil
	}
	effectivePre := [2]QuantityVec{kicked.entries[0].LinearVelocity, kicked.entries[1].LinearVelocity}
	effectivePost := [2]QuantityVec{post.entries[0].LinearVelocity, post.entries[1].LinearVelocity}
	contactVelocity, ok := motion.contactVelocity(manifold, w.step.VelocityResidual)
	if !ok {
		return undecided(w, "kinematic impact contact-point derivative is not bounded"), nil
	}
	effectivePre[motion.index], effectivePost[motion.index] = contactVelocity, contactVelocity
	change := post.entries[dynamic].Pose.Translation().Sub(pre.entries[dynamic].Pose.Translation())
	changes := [2]r3.Vec{}
	changes[dynamic] = change
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:            ContactImpact,
		Pair:            BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:         *first.Bracket,
		Time:            eventAt,
		Manifold:        cloneManifold(*manifold),
		NormalImpulse:   units.KilogramMillimetersPerSecond(impulse),
		TangentImpulse:  zeroImpulseVec(),
		PreVelocity:     effectivePre[dynamic],
		PostVelocity:    effectivePost[dynamic],
		PositionChange:  change,
		PreVelocityA:    effectivePre[0],
		PreVelocityB:    effectivePre[1],
		PostVelocityA:   effectivePost[0],
		PostVelocityB:   effectivePost[1],
		PositionChangeA: changes[0],
		PositionChangeB: changes[1],
	}}
	report.Trace = Trace{start: from, pre: pre, post: post, end: end, duration: dt,
		eventAt: eventAt, hasEvent: true,
		preSweep: roundedPrefix, postSweep: rounded}
	return report, nil
}

// sliceKinematicMotion checks the remainder's read derivative against the
// admitted full-step derivative before it is used for departure.
func (w *World) sliceKinematicMotion(motion kinematicMotion, from r3.Transform, duration units.Value,
	normal r3.Vec, manifold *decad.ContactManifold) (
	kinematicMotion, bool) {
	if !validQuantity(duration, units.Time, true) {
		return kinematicMotion{}, false
	}
	sliced := motion
	sliced.path = decad.PoseSegment{From: from, To: motion.path.To, Duration: duration}
	if motion.screw != nil {
		axis, _, supported := axisNormal(normal)
		if !supported || [3]float64{motion.screw.Axis.X, motion.screw.Axis.Y,
			motion.screw.Axis.Z}[axis] == 0 {
			return kinematicMotion{}, false
		}
		inverse, err := from.Inverse()
		if err != nil {
			return kinematicMotion{}, false
		}
		relative, err := inverse.Then(motion.path.To)
		if err != nil {
			return kinematicMotion{}, false
		}
		screw, err := relative.Screw()
		if err != nil || screw.Axis != motion.screw.Axis {
			return kinematicMotion{}, false
		}
		fullPoint := [3]float64{motion.screw.Point.X, motion.screw.Point.Y, motion.screw.Point.Z}
		slicedPoint := [3]float64{screw.Point.X, screw.Point.Y, screw.Point.Z}
		// Reading a screw from two rounded endpoint poses can shift its axis
		// line. Bound that lateral displacement and the velocity it induces.
		lineBound := new(big.Rat)
		for coordinate := range 3 {
			if !finite(fullPoint[coordinate], slicedPoint[coordinate]) {
				return kinematicMotion{}, false
			}
			if coordinate == axis {
				continue
			}
			delta := new(big.Rat).Sub(new(big.Rat).SetFloat64(slicedPoint[coordinate]),
				new(big.Rat).SetFloat64(fullPoint[coordinate]))
			lineBound.Add(lineBound, delta.Abs(delta))
		}
		if lineBound.Cmp(exactBase(w.step.Contact.PointResolution)) > 0 {
			return kinematicMotion{}, false
		}
		lineSpeedError := new(big.Rat).Mul(lineBound,
			new(big.Rat).Abs(exactBase(velocityComponent(motion.angular, axis))))
		if lineSpeedError.Cmp(exactBase(w.step.VelocityResidual)) > 0 {
			return kinematicMotion{}, false
		}
		linear, angular, ok := roundedScrewDriverRates(screw, duration)
		if !ok {
			return kinematicMotion{}, false
		}
		exactLinear := new(big.Rat).Quo(new(big.Rat).SetFloat64(screw.Slide), exactBase(duration))
		exactAngular := new(big.Rat).Quo(exactBase(screw.Angle), exactBase(duration))
		for axis := range 3 {
			component := [3]float64{screw.Axis.X, screw.Axis.Y, screw.Axis.Z}[axis]
			actualLinear := new(big.Rat).Mul(exactLinear, new(big.Rat).SetFloat64(component))
			actualAngular := new(big.Rat).Mul(exactAngular, new(big.Rat).SetFloat64(component))
			linearError := new(big.Rat).Abs(new(big.Rat).Sub(actualLinear,
				exactBase(velocityComponent(motion.effective, axis))))
			linearError.Add(linearError, new(big.Rat).Abs(new(big.Rat).Sub(actualLinear,
				exactBase(velocityComponent(linear, axis)))))
			angularError := new(big.Rat).Abs(new(big.Rat).Sub(actualAngular,
				exactBase(velocityComponent(motion.angular, axis))))
			angularError.Add(angularError, new(big.Rat).Abs(new(big.Rat).Sub(actualAngular,
				exactBase(velocityComponent(angular, axis)))))
			if linearError.Cmp(exactBase(w.step.VelocityResidual)) > 0 ||
				angularError.Cmp(exactBase(w.step.AngularVelocityResidual)) > 0 {
				return kinematicMotion{}, false
			}
		}
		sliced.effective, sliced.angular, sliced.screw = linear, angular, &screw
		originalPointSpeed, originalOK := motion.contactVelocity(manifold, w.step.VelocityResidual)
		slicedPointSpeed, slicedOK := sliced.contactVelocity(manifold, w.step.VelocityResidual)
		if !originalOK || !slicedOK {
			return kinematicMotion{}, false
		}
		for axis := range 3 {
			difference := new(big.Rat).Sub(exactBase(velocityComponent(slicedPointSpeed, axis)),
				exactBase(velocityComponent(originalPointSpeed, axis)))
			if new(big.Rat).Abs(difference).Cmp(exactBase(w.step.VelocityResidual)) > 0 {
				return kinematicMotion{}, false
			}
		}
		return sliced, true
	}
	start, end := from.Translation(), motion.path.To.Translation()
	starts, ends := [3]float64{start.X, start.Y, start.Z}, [3]float64{end.X, end.Y, end.Z}
	effective := [3]units.Value{motion.effective.X, motion.effective.Y, motion.effective.Z}
	for axis := range 3 {
		actual := new(big.Rat).Sub(new(big.Rat).SetFloat64(ends[axis]),
			new(big.Rat).SetFloat64(starts[axis]))
		actual.Quo(actual, exactBase(duration))
		if actual.Cmp(exactBase(effective[axis])) != 0 {
			return kinematicMotion{}, false
		}
	}
	return sliced, true
}
