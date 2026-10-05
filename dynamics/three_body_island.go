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

// initialConstraint is one certified face contact with an impulse direction
// pointing toward the dynamic body. Orthogonal directions decouple the normal
// response, but every pair is gathered and checked before any state is returned.
type initialConstraint struct {
	key     int
	pair    *World
	sweep   *decad.SweepReport
	normal  r3.Vec
	axis    int
	sign    float64
	dynamic int
	impulse float64
}

func (w *World) stepThreeSimultaneousInitial(ctx context.Context, from, kicked State,
	gravity QuantityVec, loads [2]*BodyLoad, dt units.Value, sweeps [3]*decad.SweepReport,
	reference *World) (*StepReport, error) {
	if threeHasSpherePair(sweeps) {
		return w.stepThreeSphereIsland(ctx, from, kicked, gravity, loads, dt, sweeps, reference)
	}
	constraints := make([]initialConstraint, 0, 2)
	for key, sweep := range sweeps {
		if sweep == nil {
			continue
		}
		pair := w.three.pairs[key]
		if sweep.Outcome != decad.SweepInitiallyTouching || sweep.Event == nil ||
			sweep.Event.Relation != decad.ContactTouching || sweep.Event.Manifold == nil ||
			len(sweep.Event.Manifold.Points) == 0 {
			return w.threeUndecided(key, "simultaneous contact lacks an initial face manifold"), nil
		}
		normal, separation, bound, ok := reducedContact(sweep.Event.Manifold, w.step.Contact)
		axis, sign, axisOK := axisNormal(normal)
		if !ok || !axisOK || !finite(separation, bound) ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
			return w.threeUndecided(key, "simultaneous manifold exceeds contact bounds"), nil
		}
		if pair.friction.lower.Sign() != 0 {
			return w.threeUndecided(key, "simultaneous friction needs a joint tangent solve"), nil
		}
		dynamic := 0
		if pair.parts[1].definition.Role == Dynamic {
			dynamic = 1
		}
		if dynamic == 0 {
			sign = -sign
		}
		constraints = append(constraints, initialConstraint{
			key: key, pair: pair, sweep: sweep, normal: normal,
			axis: axis, sign: sign, dynamic: dynamic,
		})
	}
	if len(constraints) != 2 {
		return w.threeUndecided(constraints[0].key, "simultaneous contact count is unsupported"), nil
	}
	if constraints[0].axis == constraints[1].axis {
		return w.threeUndecided(constraints[1].key, "coupled contact normals need a wider island solve"), nil
	}
	post := kicked
	preDynamic, _ := kicked.Body(w.three.parts[w.three.dynamic].Body)
	if !zeroAngularVelocity(preDynamic.AngularVelocity) {
		return w.threeUndecided(constraints[0].key, "simultaneous rotating response is not certified"), nil
	}
	mass := constraints[0].pair.parts[constraints[0].dynamic].mass
	massValue := mass.Mass.Value.Base()
	if !finite(massValue) || massValue <= 0 {
		return w.threeUndecided(constraints[0].key, "simultaneous mass is not finite"), nil
	}
	angularBudget, ok := simultaneousAngularBudget(w, mass, dt, preDynamic.Pose, len(constraints))
	if !ok {
		return w.threeUndecided(constraints[0].key, "simultaneous omitted-spin travel is not bounded"), nil
	}
	events := make([]ContactEvent, 0, len(constraints))
	var impulseVector [3]*big.Rat
	for i := range impulseVector {
		impulseVector[i] = new(big.Rat)
	}
	for i := range constraints {
		c := &constraints[i]
		beforePair := pairState(post, c.pair)
		speed := velocityComponent(beforePair.entries[c.dynamic].LinearVelocity, c.axis).Base()
		closing := speed * c.sign
		if !finite(closing) || closing > w.step.VelocityResidual.Base() {
			return w.threeUndecided(c.key, "simultaneous separating contact needs departure proof"), nil
		}
		if closing >= -w.step.VelocityResidual.Base() {
			continue
		}
		c.impulse = -closing * massValue
		postSpeed := speed + c.sign*c.impulse/massValue
		if !finite(c.impulse, postSpeed) || c.impulse <= 0 ||
			math.Abs(postSpeed*c.sign) > w.step.VelocityResidual.Base() {
			return w.threeUndecided(c.key, "simultaneous normal response exceeds velocity residual"), nil
		}
		var preSpeed [2]units.Value
		var afterSpeed [2]float64
		for side := range 2 {
			preSpeed[side] = velocityComponent(beforePair.entries[side].LinearVelocity, c.axis)
			afterSpeed[side] = preSpeed[side].Base()
		}
		afterSpeed[c.dynamic] = postSpeed
		originalSign := c.normal.X + c.normal.Y + c.normal.Z
		if !responsePairResidualsWithin(preSpeed, originalSign, units.Scalar(0), c.pair.parts,
			0, c.impulse, afterSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) ||
			!omittedSpinWithin(c.sweep.Event.Manifold, beforePair.entries[c.dynamic].Pose,
				mass, c.dynamic, c.axis, c.impulse, w.step.ImpulseResidual, angularBudget) {
			return w.threeUndecided(c.key, "simultaneous impulse or omitted spin exceeds residual"), nil
		}
		afterPair := beforePair
		setVelocityComponent(&afterPair.entries[c.dynamic].LinearVelocity, c.axis,
			units.MillimetersPerSecond(postSpeed))
		post = withPairState(post, afterPair)
		impulseValue := units.KilogramMillimetersPerSecond(c.impulse)
		instant := c.sweep.Event.At
		reportBody := c.dynamic
		event := ContactEvent{
			Kind: ContactImpact, Pair: BodyPair{A: c.pair.parts[0].definition.Body,
				B: c.pair.parts[1].definition.Body},
			Bracket: decad.SweepInterval{From: instant, To: instant},
			Time:    instant.Elapsed.Value, Manifold: cloneManifold(*c.sweep.Event.Manifold),
			NormalImpulse: impulseValue, TangentImpulse: zeroImpulseVec(),
			PreVelocity:   beforePair.entries[reportBody].LinearVelocity,
			PostVelocity:  afterPair.entries[reportBody].LinearVelocity,
			PreVelocityA:  beforePair.entries[0].LinearVelocity,
			PreVelocityB:  beforePair.entries[1].LinearVelocity,
			PostVelocityA: afterPair.entries[0].LinearVelocity,
			PostVelocityB: afterPair.entries[1].LinearVelocity,
			Solver: &ContactSolverReport{NormalResidual: w.step.VelocityResidual,
				TangentResidual:     units.MillimetersPerSecond(0),
				ConeResidual:        units.KilogramMillimetersPerSecond(0),
				PenetrationResidual: w.step.PenetrationResidual,
				AngularUpper:        w.step.AngularVelocityResidual, Iterations: 1},
		}
		if reason := c.pair.eventConservationFailure(event); reason != "" {
			return w.threeUndecided(c.key, reason), nil
		}
		events = append(events, event)
		impulseVector[c.axis].Add(impulseVector[c.axis],
			new(big.Rat).Mul(exactBase(impulseValue), ratFloat(c.sign)))
	}
	if len(events) >= w.step.MaxEvents {
		return w.threeUndecided(constraints[0].key, "simultaneous event count reaches the step limit"), nil
	}
	for _, c := range constraints {
		pairPost := pairState(post, c.pair)
		ideal, err := c.pair.sweep(ctx, pairPost, dt, decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !c.pair.persistentTrackWithin(ideal, c.normal) {
			return w.threeUndecided(c.key, fmt.Sprintf("simultaneous continuation returned %v", ideal.Outcome)), nil
		}
	}
	pairEnd, err := driftState(pairState(post, reference), dt.Base())
	if err != nil {
		return w.threeUndecidedArithmetic(constraints[0].key,
			"simultaneous final pose is not finite", err)
	}
	end := withPairState(post, pairEnd)
	for _, c := range constraints {
		pairPost, pairEnd := pairState(post, c.pair), pairState(end, c.pair)
		actual, err := c.pair.sweepPoses(ctx, pairPost, pairEnd, dt, decad.ContinueCertifiedTouch)
		if err != nil {
			return nil, err
		}
		if !c.pair.persistentTrackWithin(actual, c.normal) {
			return w.threeUndecided(c.key, fmt.Sprintf("simultaneous rounded path returned %v", actual.Outcome)), nil
		}
		contact, err := w.doc.ContactPair(ctx, c.pair.parts[0].definition.Body,
			c.pair.parts[1].definition.Body, pairEnd.entries[0].Pose,
			pairEnd.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
		if contact.Relation != decad.ContactTouching || contact.Manifold == nil {
			return w.threeUndecided(c.key, "simultaneous endpoint lacks certified contact"), nil
		}
		finalNormal, separation, bound, ok := reducedContact(contact.Manifold, w.step.Contact)
		if !ok || finalNormal != c.normal ||
			outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() {
			return w.threeUndecided(c.key, "simultaneous endpoint exceeds penetration residual"), nil
		}
	}
	trace := Trace{start: from, pre: kicked, post: post, end: end,
		duration: dt, eventAt: units.Seconds(0), hasEvent: len(events) > 0}
	childTrace := Trace{start: pairState(from, reference),
		pre: pairState(kicked, reference), post: pairState(post, reference),
		end: pairState(end, reference), duration: dt,
		eventAt: units.Seconds(0), hasEvent: len(events) > 0}
	conservation, ok := reference.conservationReadings(childTrace.start, childTrace.pre,
		childTrace.end, childTrace, nil, gravity, loads, dt)
	if !ok {
		return w.threeUndecided(constraints[0].key, "simultaneous conservation is not finite"), nil
	}
	contactImpulse, ok := boundedMomentum(impulseVector, impulseVector, impulseVector)
	if !ok {
		return w.threeUndecided(constraints[0].key, "simultaneous contact impulse is not finite"), nil
	}
	conservation.ContactImpulse = contactImpulse
	return &StepReport{Status: Advanced, Next: &end, Events: events,
		Excluded: w.Excluded(), Trace: trace, Conservation: &conservation}, nil
}

func (w *World) threeUndecidedArithmetic(key int, reason string, err error) (*StepReport, error) {
	return w.threeUndecided(key, reason+": "+err.Error()), nil
}

// Divide the allowed spin and whole-step point travel among every contact.
// The source box and center bounds enclose the largest body-point lever.
func simultaneousAngularBudget(w *World, mass decad.MassProperties, dt units.Value,
	pose r3.Transform, count int) (units.Value, bool) {
	return simultaneousAngularBudgetForBody(w, w.three.parts[w.three.dynamic].Body,
		mass, dt, pose, count)
}

func simultaneousAngularBudgetForBody(w *World, body *decad.Body,
	mass decad.MassProperties, dt units.Value, pose r3.Transform, count int) (units.Value, bool) {
	if pose.Basis() != r3.Identity().Basis() || count <= 0 {
		return units.Value{}, false
	}
	box, err := body.Bounds()
	if err != nil || box.Bound.Kind() != units.Length ||
		!finite(box.Min.X, box.Min.Y, box.Min.Z, box.Max.X, box.Max.Y, box.Max.Z,
			box.Bound.Base(), mass.Center.Value.X, mass.Center.Value.Y, mass.Center.Value.Z,
			mass.Center.Bound.Base()) || box.Bound.Base() < 0 {
		return units.Value{}, false
	}
	minimum := [3]float64{box.Min.X, box.Min.Y, box.Min.Z}
	maximum := [3]float64{box.Max.X, box.Max.Y, box.Max.Z}
	center := [3]float64{mass.Center.Value.X, mass.Center.Value.Y, mass.Center.Value.Z}
	lever := new(big.Rat)
	for axis := range 3 {
		low := absRat(new(big.Rat).Sub(ratFloat(minimum[axis]), ratFloat(center[axis])))
		high := absRat(new(big.Rat).Sub(ratFloat(maximum[axis]), ratFloat(center[axis])))
		if high.Cmp(low) > 0 {
			low = high
		}
		lever.Add(lever, low)
	}
	uncertainty := new(big.Rat).Add(exactBase(box.Bound), exactBase(mass.Center.Bound))
	lever.Add(lever, uncertainty.Mul(uncertainty, big.NewRat(3, 1)))
	if lever.Sign() <= 0 {
		return units.Value{}, false
	}
	travel := new(big.Rat).Quo(exactBase(w.step.PenetrationResidual),
		new(big.Rat).Mul(exactBase(dt), lever))
	limit := exactBase(w.step.AngularVelocityResidual)
	if travel.Cmp(limit) < 0 {
		limit = travel
	}
	limit.Quo(limit, big.NewRat(int64(count), 1))
	value, _ := limit.Float64()
	value = math.Nextafter(value, math.Inf(-1))
	if !finite(value) || value <= 0 {
		return units.Value{}, false
	}
	return units.RadiansPerSecond(value), true
}
