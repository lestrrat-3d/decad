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
}

// validateDriver admits one whole-step affine translation whose derivative is
// exactly representable as a held Velocity. Other derivatives need an interval
// response proof before they can act as collision inputs.
func (w *World) validateDriver(from State, drivers []KinematicDriver,
	dt units.Value) (kinematicMotion, error) {
	kinematic := -1
	for i, part := range w.parts {
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
	if len(drivers) != 1 || drivers[0].Body != w.parts[kinematic].definition.Body {
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
		return kinematicMotion{}, fmt.Errorf("%w: rotating kinematic drivers are not implemented", ErrUnsupported)
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
			Center:         entry.Pose.Apply(w.parts[i].mass.Center.Value),
			LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity, Duration: dt}
	}
	return w.doc.SweepPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
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
	return w.doc.SweepPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
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
		finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
			end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
		if err != nil {
			return nil, err
		}
		if finalContact.Relation != decad.ContactSeparated {
			return undecided(w, "kinematic clear endpoint is not separated"), nil
		}
		return &StepReport{Status: Advanced, Next: &end,
			Trace: Trace{start: from, end: end, duration: dt}}, nil
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
	closing := (preSpeed[1].Base() - preSpeed[0].Base()) * sign
	if !finite(closing) || closing >= -w.step.VelocityResidual.Base() {
		return undecided(w, "kinematic contact is not certified closing"), nil
	}
	if w.step.MaxEvents <= 1 {
		return undecided(w, "kinematic contact reaches the event limit with time remaining"), nil
	}
	inverseMass := 1 / w.parts[dynamic].mass.Mass.Value.Base()
	impulse := -closing / inverseMass
	if !finite(inverseMass, impulse) || inverseMass <= 0 || impulse <= 0 {
		return undecided(w, "kinematic response has invalid effective mass"), nil
	}
	postSpeed := [2]float64{preSpeed[0].Base(), preSpeed[1].Base()}
	if dynamic == 0 {
		postSpeed[0] -= impulse * sign * inverseMass
	} else {
		postSpeed[1] += impulse * sign * inverseMass
	}
	if !responsePairResidualsWithin(preSpeed, sign, units.Scalar(0), w.parts,
		0, impulse, postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) ||
		!omittedSpinWithin(first.Event.Manifold, kicked.entries[dynamic].Pose, w.parts[dynamic].mass,
			dynamic, axis, impulse, w.step.ImpulseResidual, w.step.AngularVelocityResidual) {
		return undecided(w, "kinematic response exceeds velocity, impulse, or spin residual"), nil
	}
	post := kicked
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
	finalContact, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
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
	effectivePre[motion.index], effectivePost[motion.index] = motion.effective, motion.effective
	instant := first.Event.At
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:          ContactImpact,
		Pair:          BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:       decad.SweepInterval{From: instant, To: instant},
		Time:          instant.Elapsed.Value,
		Manifold:      cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(impulse),
		PreVelocity:   effectivePre[dynamic],
		PostVelocity:  effectivePost[dynamic],
		PreVelocityA:  effectivePre[0],
		PreVelocityB:  effectivePre[1],
		PostVelocityA: effectivePost[0],
		PostVelocityB: effectivePost[1],
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true}
	return report, nil
}
