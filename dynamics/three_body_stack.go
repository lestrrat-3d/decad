package dynamics

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// stepThreeBoxStack resolves two centered, vertical face contacts together.
// The first return flag reports whether the real sweeps identified this stack.
func (w *World) stepThreeBoxStack(ctx context.Context, from, kicked State,
	input StepInput, dt units.Value) (*StepReport, bool, error) {
	if w.hasExcluded() {
		return nil, false, nil
	}
	var first [3]*decad.SweepReport
	var touching, clearPairs []int
	for key, pair := range w.three.pairs {
		if pair == nil {
			return nil, false, nil
		}
		sweep, err := pair.sweep(ctx, pairState(kicked, pair), dt, decad.StopAtInitialContact)
		if err != nil {
			return nil, true, err
		}
		first[key] = sweep
		switch sweep.Outcome {
		case decad.SweepInitiallyTouching:
			touching = append(touching, key)
		case decad.SweepClear:
			clearPairs = append(clearPairs, key)
		default:
			return nil, false, nil
		}
	}
	if len(touching) != 2 || len(clearPairs) != 1 {
		return nil, false, nil
	}
	fixed := -1
	for i, part := range w.bodies {
		if part.definition.Role == Fixed {
			fixed = i
			break
		}
	}
	if fixed < 0 {
		return nil, false, nil
	}
	lower := -1
	for _, key := range touching {
		indices := threePairs[key]
		if indices[0] == fixed {
			lower = indices[1]
		} else if indices[1] == fixed {
			lower = indices[0]
		}
	}
	if lower < 0 || w.bodies[lower].definition.Role != Dynamic {
		return nil, false, nil
	}
	upper := -1
	for i, part := range w.bodies {
		if i != fixed && i != lower && part.definition.Role == Dynamic {
			upper = i
		}
	}
	if upper < 0 || clearPairs[0] != stackPairKey(fixed, upper) {
		return nil, false, nil
	}
	floorKey, pairKey := stackPairKey(fixed, lower), stackPairKey(lower, upper)
	floor, okFloor := w.stackContact(first[floorKey], floorKey, fixed, lower)
	between, okBetween := w.stackContact(first[pairKey], pairKey, lower, upper)
	if !okFloor || !okBetween {
		return w.threeUndecided(-1, "stack needs two bounded upward box-face contacts"), true, nil
	}
	for _, key := range touching {
		pair := w.three.pairs[key]
		if pair.pairs[0].restitution.Base() != 0 || pair.pairs[0].friction.upper.Sign() != 0 {
			return w.threeUndecided(key, "stack needs zero restitution and friction"), true, nil
		}
	}
	for _, entry := range kicked.Entries() {
		if !zeroAngularVelocity(entry.AngularVelocity) {
			return w.threeUndecided(-1, "stack starts with spin"), true, nil
		}
	}
	lowerState, _ := kicked.Body(w.bodies[lower].definition.Body)
	upperState, _ := kicked.Body(w.bodies[upper].definition.Body)
	for _, entry := range []BodyState{lowerState, upperState} {
		if entry.LinearVelocity.X.Base() != 0 || entry.LinearVelocity.Y.Base() != 0 {
			return w.threeUndecided(-1, "stack needs vertical center motion"), true, nil
		}
	}
	vl, vu := lowerState.LinearVelocity.Z.Base(), upperState.LinearVelocity.Z.Base()
	ml, mu := w.bodies[lower].mass.Mass.Value.Base(), w.bodies[upper].mass.Mass.Value.Base()
	ju, jf := -mu*vu, -ml*vl-mu*vu
	mid := ju / ml
	if !finite(vl, vu, ml, mu, ju, jf, mid) || ml <= 0 || mu <= 0 ||
		vl >= -w.step.VelocityResidual.Base() || vu >= -w.step.VelocityResidual.Base() ||
		ju <= 0 || jf <= 0 || len(touching) >= w.step.MaxEvents ||
		!w.stackImpulseWithin(lower, upper, vl, vu, jf, ju) {
		return w.threeUndecided(-1, "stack coupled impulse exceeds its bounds or event limit"), true, nil
	}
	postFloor := kicked
	lowerState.LinearVelocity.Z = units.MillimetersPerSecond(mid)
	postFloor = withBodyState(postFloor, lowerState)
	post := postFloor
	lowerState.LinearVelocity.Z = units.MillimetersPerSecond(0)
	upperState.LinearVelocity.Z = units.MillimetersPerSecond(0)
	post = withBodyState(withBodyState(post, lowerState), upperState)
	events := []ContactEvent{
		w.stackEvent(floor, pairState(kicked, floor.pair), pairState(postFloor, floor.pair), jf),
		w.stackEvent(between, pairState(postFloor, between.pair), pairState(post, between.pair), ju),
	}
	angularLimits := [3]units.Value{}
	for _, index := range []int{lower, upper} {
		count := 1
		if index == lower {
			count = 2
		}
		entry, _ := kicked.Body(w.bodies[index].definition.Body)
		limit, bounded := simultaneousAngularBudgetForBody(w, entry.Body,
			w.bodies[index].mass, dt, entry.Pose, count)
		if !bounded {
			return w.threeUndecided(-1, "stack omitted spin travel cannot be bounded"), true, nil
		}
		angularLimits[index] = limit
	}
	for i, contact := range []stackFaceContact{floor, between} {
		if !w.stackEventMomentumWithin(events[i], contact.pair) {
			return w.threeUndecided(contact.key, "stack point or linear impulse exceeds its bounds"), true, nil
		}
		for side, part := range contact.pair.bodies {
			if part.definition.Role != Dynamic {
				continue
			}
			event := events[i]
			worldIndex := w.bodyIndex(part.definition.Body)
			if worldIndex < 0 || !omittedSpinWithin(contact.sweep.Event.Manifold,
				[]r3.Transform{event.PoseA, event.PoseB}[side], part.mass, side, 2,
				event.NormalImpulse.Base(), w.step.ImpulseResidual, angularLimits[worldIndex]) {
				return w.threeUndecided(contact.key, "stack omitted spin exceeds its bound"), true, nil
			}
			if reason := contact.pair.eventAngularImpulseFailure(event, side,
				eventAngularVelocity(event, side, true), eventAngularVelocity(event, side, false),
				[]r3.Transform{event.PoseA, event.PoseB}[side], exactBase(w.step.ImpulseResidual)); reason != "" {
				return w.threeUndecided(contact.key, reason), true, nil
			}
		}
	}
	end, err := w.threeDriftState(post, dt.Base())
	if err != nil {
		report, stepErr := w.threeUndecidedArithmetic(-1, "stack final pose is not finite", err)
		return report, true, stepErr
	}
	var rounded [3]*decad.SweepReport
	for key, pair := range w.three.pairs {
		policy := decad.StopAtInitialContact
		if key == floorKey || key == pairKey {
			policy = decad.ContinueCertifiedTouch
		}
		startPair, endPair := pairState(post, pair), pairState(end, pair)
		ideal, sweepErr := pair.sweep(ctx, startPair, dt, policy)
		if sweepErr != nil {
			return nil, true, sweepErr
		}
		actual, sweepErr := pair.sweepPoses(ctx, startPair, endPair, dt, policy)
		if sweepErr != nil {
			return nil, true, sweepErr
		}
		if !actual.HasAffineReplayProof() || ideal.Outcome != actual.Outcome {
			return w.threeUndecided(key, "stack rounded path lacks replay proof"), true, nil
		}
		if key == clearPairs[0] {
			if ideal.Outcome != decad.SweepClear || actual.Outcome != decad.SweepClear {
				return w.threeUndecided(key, "stack floor-to-upper path is not clear"), true, nil
			}
		} else {
			contact := floor
			if key == pairKey {
				contact = between
			}
			if !pair.persistentTrackWithin(ideal, contact.normal) ||
				!pair.persistentTrackWithin(actual, contact.normal) {
				return w.threeUndecided(key, "stack contact track is not persistent"), true, nil
			}
		}
		rounded[key] = actual
		endpoint, contactErr := w.doc.ContactPair(ctx, pair.bodies[0].definition.Body,
			pair.bodies[1].definition.Body, endPair.entries[0].Pose, endPair.entries[1].Pose,
			w.step.Contact)
		if contactErr != nil {
			return nil, true, contactErr
		}
		if key == clearPairs[0] && endpoint.Relation != decad.ContactSeparated ||
			key != clearPairs[0] && (endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil) {
			return w.threeUndecided(key, "stack endpoint loses its pair relation"), true, nil
		}
		if key != clearPairs[0] {
			contact := floor
			if key == pairKey {
				contact = between
			}
			normal, separation, bound, contactOK := reducedContact(endpoint.Manifold, w.step.Contact)
			if !contactOK || normal != contact.normal ||
				outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
				return w.threeUndecided(key, "stack endpoint contact exceeds its bounds"), true, nil
			}
		}
	}
	trace := Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: units.Seconds(0), hasEvent: true, threeSweeps: rounded}
	conservation, valid := w.threeTwoDynamicConservation(from, kicked, end, trace, events, input, dt)
	if !valid {
		return w.threeUndecided(-1, "stack conservation cannot be bounded"), true, nil
	}
	beforeEnergy := new(big.Rat).Sub(exactBase(conservation.AfterKick.KineticEnergy.Value),
		exactBase(conservation.AfterKick.KineticEnergy.Bound))
	afterEnergy := new(big.Rat).Add(exactBase(conservation.Completion.KineticEnergy.Value),
		exactBase(conservation.Completion.KineticEnergy.Bound))
	if afterEnergy.Cmp(beforeEnergy) > 0 {
		return w.threeUndecided(-1, "stack coupled response may gain kinetic energy"), true, nil
	}
	return &StepReport{Status: Advanced, Next: &end, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, true, nil
}

type stackFaceContact struct {
	key    int
	pair   *World
	sweep  *decad.SweepReport
	normal r3.Vec
}

func stackPairKey(a, b int) int {
	for key, pair := range threePairs {
		if pair == [2]int{a, b} || pair == [2]int{b, a} {
			return key
		}
	}
	return -1
}

func (w *World) stackContact(sweep *decad.SweepReport, key, below, above int) (stackFaceContact, bool) {
	contact := stackFaceContact{key: key, pair: w.three.pairs[key], sweep: sweep}
	if sweep == nil || sweep.Outcome != decad.SweepInitiallyTouching || sweep.Event == nil ||
		sweep.Event.Relation != decad.ContactTouching || sweep.Event.Manifold == nil ||
		len(sweep.Event.Manifold.Points) != 4 ||
		sweep.Event.At.Fraction.Base() != 0 || sweep.Event.At.Elapsed.Value.Base() != 0 {
		return contact, false
	}
	normal, separation, bound, ok := reducedContact(sweep.Event.Manifold, w.step.Contact)
	if !ok || !finite(separation, bound) ||
		outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
		return contact, false
	}
	indices := threePairs[key]
	if indices[0] == below && indices[1] == above && normal != (r3.Vec{Z: 1}) ||
		indices[1] == below && indices[0] == above && normal != (r3.Vec{Z: -1}) {
		return contact, false
	}
	for _, point := range sweep.Event.Manifold.Points {
		if !boxFaceOnBody(point.FaceA, contact.pair.bodies[0].definition.Body) ||
			!boxFaceOnBody(point.FaceB, contact.pair.bodies[1].definition.Body) {
			return contact, false
		}
	}
	contact.normal = normal
	return contact, true
}

func (w *World) stackImpulseWithin(lower, upper int, vl, vu, jf, ju float64) bool {
	ml, bl := exactBase(w.bodies[lower].mass.Mass.Value), exactBase(w.bodies[lower].mass.Mass.Bound)
	mu, bu := exactBase(w.bodies[upper].mass.Mass.Value), exactBase(w.bodies[upper].mass.Mass.Bound)
	if ml == nil || bl == nil || mu == nil || bu == nil || bl.Sign() < 0 || bu.Sign() < 0 ||
		new(big.Rat).Sub(ml, bl).Sign() <= 0 || new(big.Rat).Sub(mu, bu).Sign() <= 0 {
		return false
	}
	lv, uv := new(big.Rat).Neg(ratFloat(vl)), new(big.Rat).Neg(ratFloat(vu))
	jULow := new(big.Rat).Mul(new(big.Rat).Sub(mu, bu), uv)
	jUHigh := new(big.Rat).Mul(new(big.Rat).Add(mu, bu), uv)
	jFLow := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Sub(ml, bl), lv), jULow)
	jFHigh := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Add(ml, bl), lv), jUHigh)
	limit := exactBase(w.step.ImpulseResidual)
	return intervalDeviation(ratFloat(ju), jULow, jUHigh).Cmp(limit) <= 0 &&
		intervalDeviation(ratFloat(jf), jFLow, jFHigh).Cmp(limit) <= 0
}

// The two events share one instant. Check each body's impulse equation here;
// a sequential energy check would incorrectly reject an intermediate speed.
func (w *World) stackEventMomentumWithin(event ContactEvent, pair *World) bool {
	applied, ok := eventAppliedImpulse(event)
	if !ok || len(event.PointImpulses) != len(event.Manifold.Points) {
		return false
	}
	impulseLimit, velocityLimit := exactBase(w.step.ImpulseResidual), exactBase(w.step.VelocityResidual)
	if impulseLimit == nil || velocityLimit == nil {
		return false
	}
	normalError := new(big.Rat)
	for _, point := range event.Manifold.Points {
		bound := new(big.Rat).Add(exactBase(point.Normal.Bound), exactBase(point.NormalAngle))
		if bound.Cmp(normalError) > 0 {
			normalError = bound
		}
	}
	for side, part := range pair.bodies {
		if part.definition.Role != Dynamic {
			continue
		}
		mass, bound := exactBase(part.mass.Mass.Value), exactBase(part.mass.Mass.Bound)
		if mass == nil || bound == nil || bound.Sign() < 0 {
			return false
		}
		low, high := new(big.Rat).Sub(mass, bound), new(big.Rat).Add(mass, bound)
		if low.Sign() <= 0 {
			return false
		}
		before, after := eventBodyVelocities(event, side)
		sign := int64(1)
		if side == 0 {
			sign = -1
		}
		for axis := range 3 {
			change := new(big.Rat).Sub(exactBase(velocityComponent(after, axis)),
				exactBase(velocityComponent(before, axis)))
			impulse := new(big.Rat).Mul(applied[axis], big.NewRat(sign, 1))
			residual := absRat(new(big.Rat).Sub(new(big.Rat).Mul(low, change), impulse))
			if other := absRat(new(big.Rat).Sub(new(big.Rat).Mul(high, change), impulse)); other.Cmp(residual) > 0 {
				residual = other
			}
			allowance := new(big.Rat).Add(impulseLimit, new(big.Rat).Mul(high, velocityLimit))
			allowance.Add(allowance, new(big.Rat).Mul(exactBase(event.NormalImpulse), normalError))
			if residual.Cmp(allowance) > 0 {
				return false
			}
		}
	}
	return true
}

func (w *World) stackEvent(contact stackFaceContact, before, after State, impulse float64) ContactEvent {
	count := len(contact.sweep.Event.Manifold.Points)
	pointImpulses := make([]ContactPointImpulse, count)
	for i := range pointImpulses {
		pointImpulses[i] = ContactPointImpulse{Normal: units.KilogramMillimetersPerSecond(impulse / float64(count)),
			Tangent: zeroImpulseVec()}
	}
	instant := contact.sweep.Event.At
	reportSide := 1
	if contact.pair.bodies[1].definition.Role == Fixed {
		reportSide = 0
	}
	return ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{A: contact.pair.bodies[0].definition.Body, B: contact.pair.bodies[1].definition.Body},
		Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
		Manifold:      cloneManifold(*contact.sweep.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulse), TangentImpulse: zeroImpulseVec(),
		PointImpulses: pointImpulses, PreVelocity: before.entries[reportSide].LinearVelocity,
		PostVelocity: after.entries[reportSide].LinearVelocity,
		PreVelocityA: before.entries[0].LinearVelocity, PreVelocityB: before.entries[1].LinearVelocity,
		PostVelocityA: after.entries[0].LinearVelocity, PostVelocityB: after.entries[1].LinearVelocity,
		PreAngularVelocityA:  before.entries[0].AngularVelocity,
		PreAngularVelocityB:  before.entries[1].AngularVelocity,
		PostAngularVelocityA: after.entries[0].AngularVelocity,
		PostAngularVelocityB: after.entries[1].AngularVelocity,
		PoseA:                before.entries[0].Pose, PoseB: before.entries[1].Pose,
		Solver: &ContactSolverReport{NormalResidual: w.step.VelocityResidual,
			TangentResidual:     units.MillimetersPerSecond(0),
			ConeResidual:        units.KilogramMillimetersPerSecond(0),
			PenetrationResidual: w.step.PenetrationResidual,
			AngularUpper:        w.step.AngularVelocityResidual, Iterations: 1}}
}

func eventAngularVelocity(event ContactEvent, side int, pre bool) QuantityVec {
	if pre {
		if side == 0 {
			return event.PreAngularVelocityA
		}
		return event.PreAngularVelocityB
	}
	if side == 0 {
		return event.PostAngularVelocityA
	}
	return event.PostAngularVelocityB
}
