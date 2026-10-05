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

type kinematicMotion struct {
	index     int
	path      decad.PoseSegment
	effective QuantityVec
	angular   QuantityVec
	screw     *r3.Screw
}

// validateDriver admits exact affine translation or a cardinal rotating screw
// with representable full-step rates. Other paths lack the response proof.
func (w *World) validateDriver(from State, drivers []KinematicDriver,
	dt units.Value) (kinematicMotion, error) {
	kinematic := -1
	for i, part := range w.bodies {
		if part.definition.Role == Kinematic {
			kinematic = i
		}
	}
	if kinematic < 0 {
		if len(drivers) != 0 {
			return kinematicMotion{}, fmt.Errorf("%w: driver body is not kinematic", ErrInvalidInput)
		}
		return kinematicMotion{index: -1}, nil
	}
	if len(drivers) != 1 || drivers[0].Body != w.bodies[kinematic].definition.Body {
		return kinematicMotion{}, fmt.Errorf("%w: exactly one driver is required for the kinematic body", ErrInvalidInput)
	}
	if drivers[0].Path == nil {
		return kinematicMotion{}, fmt.Errorf("%w: nil kinematic path", ErrInvalidInput)
	}
	var path decad.PoseSegment
	switch supplied := drivers[0].Path.(type) {
	case decad.PoseSegment:
		path = supplied
	case *decad.PoseSegment:
		if supplied == nil {
			return kinematicMotion{}, fmt.Errorf("%w: nil kinematic path", ErrInvalidInput)
		}
		path = *supplied
	default:
		return kinematicMotion{}, fmt.Errorf("%w: only PoseSegment kinematic drivers are implemented", ErrUnsupported)
	}
	if !validQuantity(path.Duration, units.Time, true) ||
		exactBase(path.Duration).Cmp(exactBase(dt)) != 0 ||
		!path.From.IsValid() || !path.To.IsValid() ||
		path.From.IsReflection() || path.To.IsReflection() {
		return kinematicMotion{}, fmt.Errorf("%w: invalid driver duration or pose", ErrInvalidInput)
	}
	statePose := from.entries[kinematic].Pose
	if path.From.Translation() != statePose.Translation() || !sameOrientation(path.From, statePose) {
		return kinematicMotion{}, fmt.Errorf("%w: driver start differs from the state pose", ErrInvalidInput)
	}
	if !sameOrientation(path.From, path.To) {
		inverse, err := path.From.Inverse()
		if err != nil {
			return kinematicMotion{}, fmt.Errorf("%w: invalid rotating driver start", ErrInvalidInput)
		}
		relative, err := inverse.Then(path.To)
		if err != nil {
			return kinematicMotion{}, fmt.Errorf("%w: invalid rotating driver path", ErrInvalidInput)
		}
		screw, err := relative.Screw()
		if err != nil {
			return kinematicMotion{}, fmt.Errorf("%w: invalid rotating driver screw", ErrInvalidInput)
		}
		if !cardinalDriverAxis(screw.Axis) || screw.Angle.Base() <= 0 ||
			!finite(screw.Point.X, screw.Point.Y, screw.Point.Z, screw.Slide) {
			return kinematicMotion{}, fmt.Errorf("%w: rotating driver axis lacks a current contact proof", ErrUnsupported)
		}
		linear, angular, ok := screwDriverRates(screw, dt)
		if !ok {
			return kinematicMotion{}, fmt.Errorf("%w: rotating driver derivative exceeds the current proof", ErrUnsupported)
		}
		return kinematicMotion{index: kinematic, path: path, effective: linear,
			angular: angular, screw: &screw}, nil
	}
	start, end := path.From.Translation(), path.To.Translation()
	starts, ends := [3]float64{start.X, start.Y, start.Z}, [3]float64{end.X, end.Y, end.Z}
	var components [3]units.Value
	for axis := range 3 {
		delta := new(big.Rat).Sub(new(big.Rat).SetFloat64(ends[axis]),
			new(big.Rat).SetFloat64(starts[axis]))
		exactSpeed := new(big.Rat).Quo(delta, exactBase(dt))
		speed, _ := exactSpeed.Float64()
		if !finite(speed) {
			return kinematicMotion{}, fmt.Errorf("%w: driver derivative exceeds the current exact-speed proof", ErrUnsupported)
		}
		components[axis] = units.MillimetersPerSecond(speed)
		if exactBase(components[axis]).Cmp(exactSpeed) != 0 {
			return kinematicMotion{}, fmt.Errorf("%w: driver derivative exceeds the current exact-speed proof", ErrUnsupported)
		}
	}
	return kinematicMotion{index: kinematic, path: path,
		effective: QuantityVec{X: components[0], Y: components[1], Z: components[2]}}, nil
}

func cardinalDriverAxis(axis r3.Vec) bool {
	switch axis {
	case r3.Vec{X: 1}, r3.Vec{X: -1}, r3.Vec{Y: 1}, r3.Vec{Y: -1},
		r3.Vec{Z: 1}, r3.Vec{Z: -1}:
		return true
	}
	return false
}

func screwDriverRates(screw r3.Screw, dt units.Value) (QuantityVec, QuantityVec, bool) {
	angle := new(big.Rat).Quo(exactBase(screw.Angle), exactBase(dt))
	slide := new(big.Rat).Quo(new(big.Rat).SetFloat64(screw.Slide), exactBase(dt))
	omega, _ := angle.Float64()
	speed, _ := slide.Float64()
	if !finite(omega, speed) || new(big.Rat).SetFloat64(omega).Cmp(angle) != 0 ||
		new(big.Rat).SetFloat64(speed).Cmp(slide) != 0 {
		return QuantityVec{}, QuantityVec{}, false
	}
	linear := QuantityVec{X: units.MillimetersPerSecond(screw.Axis.X * speed),
		Y: units.MillimetersPerSecond(screw.Axis.Y * speed),
		Z: units.MillimetersPerSecond(screw.Axis.Z * speed)}
	angular := QuantityVec{X: units.RadiansPerSecond(screw.Axis.X * omega),
		Y: units.RadiansPerSecond(screw.Axis.Y * omega),
		Z: units.RadiansPerSecond(screw.Axis.Z * omega)}
	return linear, angular, true
}

func roundedScrewDriverRates(screw r3.Screw, dt units.Value) (QuantityVec, QuantityVec, bool) {
	if !validQuantity(dt, units.Time, true) || !cardinalDriverAxis(screw.Axis) ||
		!finite(screw.Slide, screw.Angle.Base()) {
		return QuantityVec{}, QuantityVec{}, false
	}
	omega := screw.Angle.Base() / dt.Base()
	speed := screw.Slide / dt.Base()
	if !finite(omega, speed) {
		return QuantityVec{}, QuantityVec{}, false
	}
	return QuantityVec{X: units.MillimetersPerSecond(screw.Axis.X * speed),
			Y: units.MillimetersPerSecond(screw.Axis.Y * speed),
			Z: units.MillimetersPerSecond(screw.Axis.Z * speed)},
		QuantityVec{X: units.RadiansPerSecond(screw.Axis.X * omega),
			Y: units.RadiansPerSecond(screw.Axis.Y * omega),
			Z: units.RadiansPerSecond(screw.Axis.Z * omega)}, true
}

func (motion kinematicMotion) contactVelocity(manifold *decad.ContactManifold,
	limit units.Value) (QuantityVec, bool) {
	if motion.screw == nil {
		return motion.effective, true
	}
	if manifold == nil || len(manifold.Points) != 1 || !validQuantity(limit, units.Velocity, true) {
		return QuantityVec{}, false
	}
	witness := manifold.Points[0].OnA
	if motion.index == 1 {
		witness = manifold.Points[0].OnB
	}
	if !validQuantity(witness.Bound, units.Length, false) {
		return QuantityVec{}, false
	}
	coords := [3]float64{witness.Value.X, witness.Value.Y, witness.Value.Z}
	axisPoint := [3]float64{motion.screw.Point.X, motion.screw.Point.Y, motion.screw.Point.Z}
	linear := [3]units.Value{motion.effective.X, motion.effective.Y, motion.effective.Z}
	angular := [3]units.Value{motion.angular.X, motion.angular.Y, motion.angular.Z}
	var lever, omega [3]*big.Rat
	for axis := range 3 {
		if !finite(coords[axis], axisPoint[axis]) {
			return QuantityVec{}, false
		}
		lever[axis] = new(big.Rat).Sub(new(big.Rat).SetFloat64(coords[axis]),
			new(big.Rat).SetFloat64(axisPoint[axis]))
		omega[axis] = exactBase(angular[axis])
	}
	pointBound := exactBase(witness.Bound)
	velocityLimit := exactBase(limit)
	var values [3]units.Value
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		first := new(big.Rat).Mul(omega[i], lever[j])
		second := new(big.Rat).Mul(omega[j], lever[i])
		exact := new(big.Rat).Add(exactBase(linear[axis]),
			new(big.Rat).Sub(first, second))
		held, _ := exact.Float64()
		if !finite(held) {
			return QuantityVec{}, false
		}
		values[axis] = units.MillimetersPerSecond(held)
		errorBound := new(big.Rat).Abs(new(big.Rat).Sub(exact, exactBase(values[axis])))
		pointError := new(big.Rat).Add(new(big.Rat).Abs(omega[i]), new(big.Rat).Abs(omega[j]))
		errorBound.Add(errorBound, pointError.Mul(pointError, pointBound))
		if errorBound.Cmp(velocityLimit) > 0 {
			return QuantityVec{}, false
		}
	}
	return QuantityVec{X: values[0], Y: values[1], Z: values[2]}, true
}

func sameOrientation(a, b r3.Transform) bool {
	for _, basis := range []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}} {
		if a.ApplyDir(basis) != b.ApplyDir(basis) {
			return false
		}
	}
	return true
}

func (w *World) sweepKinematic(ctx context.Context, state State, dt units.Value,
	motion kinematicMotion, policy decad.SweepStartPolicy) (*decad.SweepReport, error) {
	var paths [2]decad.PairPath
	for i, entry := range state.entries {
		if i == motion.index {
			paths[i] = motion.path
			continue
		}
		paths[i] = decad.RigidDriftSegment{From: entry.Pose,
			Center:         entry.Pose.Apply(w.bodies[i].mass.Center.Value),
			LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity, Duration: dt}
	}
	return w.doc.SweepPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		paths[0], paths[1], w.sweepRequest(dt, policy))
}

func (w *World) sweepKinematicPoses(ctx context.Context, from, to State, dt units.Value,
	motion kinematicMotion, policy decad.SweepStartPolicy) (*decad.SweepReport, error) {
	var paths [2]decad.PairPath
	for i := range from.entries {
		if i == motion.index {
			paths[i] = motion.path
			continue
		}
		paths[i] = decad.PoseSegment{From: from.entries[i].Pose, To: to.entries[i].Pose, Duration: dt}
	}
	return w.doc.SweepPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		paths[0], paths[1], w.sweepRequest(dt, policy))
}

func (w *World) stepKinematicPush(ctx context.Context, from, kicked State, dt units.Value,
	motion kinematicMotion) (*StepReport, error) {
	for _, entry := range kicked.entries {
		if !sameOrientation(entry.Pose, r3.Identity()) {
			return nil, fmt.Errorf("%w: rotated poses are not implemented", ErrUnsupported)
		}
	}
	first, err := w.sweepKinematic(ctx, kicked, dt, motion, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if first.Outcome == decad.SweepClear {
		end, err := driftState(kicked, dt.Base())
		if err != nil {
			return undecidedArithmetic(w, "non-finite clear kinematic drift", err)
		}
		end.entries[motion.index].Pose = motion.path.To
		rounded, err := w.sweepKinematicPoses(ctx, kicked, end, dt, motion, decad.StopAtInitialContact)
		if err != nil {
			return nil, err
		}
		if rounded.Outcome != decad.SweepClear {
			return undecided(w, "rounded kinematic drift lacks a clear path"), nil
		}
		finalContact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
			end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
		if finalContact.Relation != decad.ContactSeparated {
			return undecided(w, "kinematic clear endpoint is not separated"), nil
		}
		return &StepReport{Status: Advanced, Next: &end,
			Trace: Trace{start: from, end: end, duration: dt, preSweep: rounded}}, nil
	}
	if first.Outcome == decad.SweepImpactBracket {
		return w.stepKinematicImpact(ctx, from, kicked, dt, motion, first)
	}
	if first.Outcome != decad.SweepInitiallyTouching || first.Event == nil ||
		first.Event.Relation != decad.ContactTouching || first.Event.Manifold == nil {
		return undecided(w, fmt.Sprintf("kinematic first sweep returned %v", first.Outcome)), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok || !finite(separation, bound) ||
		math.Abs(separation)+bound > w.step.PenetrationResidual.Base() {
		return undecided(w, "kinematic initial contact exceeds penetration residual"), nil
	}
	axis, sign, valid := axisNormal(normal)
	if !valid {
		return undecided(w, "kinematic contact normal is unsupported"), nil
	}
	dynamic := 1 - motion.index
	preSpeed := [2]units.Value{
		velocityComponent(kicked.entries[0].LinearVelocity, axis),
		velocityComponent(kicked.entries[1].LinearVelocity, axis),
	}
	preSpeed[motion.index] = velocityComponent(motion.effective, axis)
	relativeExact := new(big.Rat).Sub(exactBase(preSpeed[1]), exactBase(preSpeed[0]))
	relativeExact.Mul(relativeExact, big.NewRat(int64(sign), 1))
	if relativeExact.Cmp(exactBase(w.step.VelocityResidual)) > 0 {
		return w.stepKinematicDeparture(ctx, from, kicked, dt, motion)
	}
	closing := (preSpeed[1].Base() - preSpeed[0].Base()) * sign
	if !finite(closing) || closing >= -w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic contact is not certified closing"), nil
	}
	if w.step.MaxEvents <= 1 {
		return undecided(w, "kinematic contact reaches the event limit with time remaining"), nil
	}
	inverseMass := 1 / w.bodies[dynamic].mass.Mass.Value.Base()
	impulse := -closing / inverseMass
	if !finite(inverseMass, impulse) || inverseMass <= 0 || impulse <= 0 {
		return undecided(w, "kinematic response has invalid effective mass"), nil
	}
	postSpeed := [2]float64{preSpeed[0].Base(), preSpeed[1].Base()}
	if dynamic == 0 {
		postSpeed[0] -= float64(impulse * sign * inverseMass)
	} else {
		postSpeed[1] += float64(impulse * sign * inverseMass)
	}
	if !responsePairResidualsWithin(preSpeed, sign, units.Scalar(0), w.bodies,
		0, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) ||
		!omittedSpinWithin(first.Event.Manifold, kicked.entries[dynamic].Pose, w.bodies[dynamic].mass,
			dynamic, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
		return undecided(w, "kinematic response exceeds velocity, impulse, or spin residual"), nil
	}
	post := kicked.clone()
	setVelocityComponent(&post.entries[dynamic].LinearVelocity, axis, units.MillimetersPerSecond(postSpeed[dynamic]))
	if math.Abs((postSpeed[1]-postSpeed[0])*sign) > w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic post-contact speed exceeds residual"), nil
	}
	ideal, err := w.sweepKinematic(ctx, post, dt, motion, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(ideal, normal) {
		return undecided(w, fmt.Sprintf("kinematic continuation returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite kinematic drift", err)
	}
	end.entries[motion.index].Pose = motion.path.To
	rounded, err := w.sweepKinematicPoses(ctx, post, end, dt, motion, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(rounded, normal) {
		return undecided(w, fmt.Sprintf("rounded kinematic continuation returned %v", rounded.Outcome)), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if finalContact.Relation != decad.ContactTouching || finalContact.Manifold == nil {
		return undecided(w, "kinematic endpoint lacks certified contact"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(finalContact.Manifold, w.step.Contact)
	if !valid || finalNormal != normal ||
		math.Abs(finalSeparation)+finalBound > w.step.PenetrationResidual.Base() {
		return undecided(w, "kinematic endpoint exceeds penetration residual"), nil
	}
	effectivePre := [2]QuantityVec{kicked.entries[0].LinearVelocity, kicked.entries[1].LinearVelocity}
	effectivePost := [2]QuantityVec{post.entries[0].LinearVelocity, post.entries[1].LinearVelocity}
	contactVelocity, ok := motion.contactVelocity(first.Event.Manifold, w.step.VelocityResidual)
	if !ok {
		return undecided(w, "kinematic contact-point derivative is not bounded"), nil
	}
	effectivePre[motion.index], effectivePost[motion.index] = contactVelocity, contactVelocity
	instant := first.Event.At
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:           ContactImpact,
		Pair:           BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
		Bracket:        decad.SweepInterval{From: instant, To: instant},
		Time:           instant.Elapsed.Value,
		Manifold:       cloneManifold(*first.Event.Manifold),
		NormalImpulse:  units.KilogramMillimetersPerSecond(impulse),
		TangentImpulse: zeroImpulseVec(),
		PreVelocity:    effectivePre[dynamic],
		PostVelocity:   effectivePost[dynamic],
		PreVelocityA:   effectivePre[0],
		PreVelocityB:   effectivePre[1],
		PostVelocityA:  effectivePost[0],
		PostVelocityB:  effectivePost[1],
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true, postSweep: rounded}
	return report, nil
}

// stepKinematicDeparture requires geometry to certify the entire path after
// an initially touching pair starts separating. It applies no contact impulse.
func (w *World) stepKinematicDeparture(ctx context.Context, from, kicked State,
	dt units.Value, motion kinematicMotion) (*StepReport, error) {
	ideal, err := w.sweepKinematic(ctx, kicked, dt, motion, decad.ContinueSeparatingTouch)
	if err != nil {
		return nil, err
	}
	if ideal.Outcome != decad.SweepDepartedClear {
		return undecided(w, fmt.Sprintf("kinematic departure returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(kicked, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite kinematic departure drift", err)
	}
	end.entries[motion.index].Pose = motion.path.To
	rounded, err := w.sweepKinematicPoses(ctx, kicked, end, dt, motion, decad.ContinueSeparatingTouch)
	if err != nil {
		return nil, err
	}
	if rounded.Outcome != decad.SweepDepartedClear {
		return undecided(w, fmt.Sprintf("rounded kinematic departure returned %v", rounded.Outcome)), nil
	}
	finalContact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if finalContact.Relation != decad.ContactSeparated {
		return undecided(w, "kinematic departure endpoint is not separated"), nil
	}
	return &StepReport{Status: Advanced, Next: &end,
		Trace: Trace{start: from, end: end, duration: dt, preSweep: rounded}}, nil
}
