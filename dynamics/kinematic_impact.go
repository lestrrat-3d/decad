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
	if first.Bracket == nil || first.Event == nil || first.Event.Manifold == nil ||
		len(first.Event.Manifold.Points) == 0 {
		return undecided(w, "kinematic impact lacks a bracket and manifold"), nil
	}
	chosen := first.Bracket.To.Elapsed.Value.Base()
	if !finite(chosen) || chosen <= 0 || chosen >= dt.Base() {
		return undecided(w, "kinematic impact time is outside the interior step"), nil
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
	if !roundedImpactPrefixAtEnd(roundedPrefix, first, w.step.PenetrationResidual) {
		return undecided(w, "published kinematic impact prefix lacks a rounded endpoint bracket"), nil
	}
	contactAtRight, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		pre.entries[0].Pose, pre.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if contactAtRight.Relation != first.Event.Relation || contactAtRight.Manifold == nil {
		return undecided(w, "kinematic bracket-right pose lacks its reported contact"), nil
	}
	normal, separation, bound, valid := reducedContact(first.Event.Manifold, w.step.Contact)
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
		!omittedSpinWithin(first.Event.Manifold, pre.entries[dynamic].Pose, w.parts[dynamic].mass,
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
	remaining := dt.Base() - chosen
	postRelative := (postSpeed[1] - postSpeed[0]) * sign
	if !finite(remaining, postRelative) || remaining <= 0 ||
		postRelative < -w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic impact response remains closing"), nil
	}
	persistent := postRelative <= w.step.VelocityResidual.Base()
	policy := decad.ContinueSeparatingTouch
	if persistent {
		policy = decad.ContinueCertifiedTouch
	}
	remainderMotion, valid := sliceKinematicMotion(motion, post.entries[motion.index].Pose,
		units.Seconds(remaining))
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
	effectivePre[motion.index], effectivePost[motion.index] = motion.effective, motion.effective
	change := post.entries[dynamic].Pose.Translation().Sub(pre.entries[dynamic].Pose.Translation())
	changes := [2]r3.Vec{}
	changes[dynamic] = change
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:            ContactImpact,
		Pair:            BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:         *first.Bracket,
		Time:            first.Bracket.To.Elapsed.Value,
		Manifold:        cloneManifold(*first.Event.Manifold),
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
		eventAt: first.Bracket.To.Elapsed.Value, hasEvent: true,
		preSweep: roundedPrefix, postSweep: rounded}
	return report, nil
}

// sliceKinematicMotion checks the published affine remainder against the
// driver's admitted full-step derivative before it is used for departure.
func sliceKinematicMotion(motion kinematicMotion, from r3.Transform, duration units.Value) (
	kinematicMotion, bool) {
	if !validQuantity(duration, units.Time, true) {
		return kinematicMotion{}, false
	}
	sliced := motion
	sliced.path = decad.PoseSegment{From: from, To: motion.path.To, Duration: duration}
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
