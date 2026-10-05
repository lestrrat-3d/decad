package dynamics

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// stepGrazingTouch records a zero-impulse event only after both full-step
// affine paths certify the same isolated sphere-pair touch.
func (w *World) stepGrazingTouch(ctx context.Context, from, kicked State, dt units.Value,
	first *decad.SweepReport) (*StepReport, error) {
	if w.pairs[0].friction.upper.Sign() != 0 || first == nil || first.Event == nil ||
		first.Event.Relation != decad.ContactTouching || first.Event.Manifold == nil ||
		len(first.Event.Manifold.Points) != 1 || first.Bracket != nil {
		return undecided(w, "grazing touch lacks a frictionless point event"), nil
	}
	fraction, duration := exactBase(first.Event.At.Fraction), exactBase(dt)
	if fraction == nil || duration == nil || fraction.Sign() <= 0 ||
		fraction.Cmp(big.NewRat(1, 1)) >= 0 {
		return undecided(w, "grazing event fraction is invalid"), nil
	}
	exactTime := new(big.Rat).Mul(fraction, duration)
	eventAt := units.Seconds(dt.Base() * first.Event.At.Fraction.Base())
	if !finite(eventAt.Base()) || exactBase(eventAt) == nil || exactBase(eventAt).Cmp(exactTime) != 0 {
		return undecided(w, "grazing event time is not exactly representable"), nil
	}
	end, err := driftState(kicked, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite grazing endpoint", err)
	}
	rounded, err := w.sweepPoses(ctx, kicked, end, dt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if rounded.Outcome != decad.SweepGrazingTouch || rounded.Event == nil ||
		rounded.Event.At.Fraction != first.Event.At.Fraction || rounded.Event.Manifold == nil ||
		len(rounded.Event.Manifold.Points) != 1 || !rounded.HasAffineReplayProof() ||
		!grazingManifoldsMatch(first.Event.Manifold.Points[0], rounded.Event.Manifold.Points[0],
			w.step.Contact, w.step.PenetrationResidual) {
		return undecided(w, fmt.Sprintf("rounded grazing path returned %v", rounded.Outcome)), nil
	}
	if !grazingNormalSpeedWithin(kicked, rounded.Event.Manifold.Points[0], w.step.VelocityResidual) {
		return undecided(w, "grazing normal speed exceeds velocity residual"), nil
	}
	eventState, err := driftState(kicked, eventAt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite grazing event pose", err)
	}
	poseA, poseB, err := rounded.CertifiedPosesAtInterval(eventAt, units.Seconds(0), dt)
	if err != nil {
		return undecidedArithmetic(w, "rounded grazing event pose has no proof", err)
	}
	if poseA != eventState.entries[0].Pose || poseB != eventState.entries[1].Pose {
		return undecided(w, "rounded grazing event pose differs from drift"), nil
	}
	poseA, poseB, err = rounded.CertifiedPosesAtInterval(dt, units.Seconds(0), dt)
	if err != nil {
		return undecidedArithmetic(w, "rounded grazing endpoint has no proof", err)
	}
	if poseA != end.entries[0].Pose || poseB != end.entries[1].Pose {
		return undecided(w, "rounded grazing endpoint differs from drift"), nil
	}
	final, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if final.Relation != decad.ContactSeparated {
		return undecided(w, "grazing endpoint is not separated"), nil
	}
	zero := units.KilogramMillimetersPerSecond(0)
	reportBody := 1
	if w.bodies[1].definition.Role == Fixed {
		reportBody = 0
	}
	event := ContactEvent{
		Kind: ContactGraze, Pair: BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
		Bracket: decad.SweepInterval{From: rounded.Event.At, To: rounded.Event.At}, Time: eventAt,
		Manifold:      cloneManifold(*rounded.Event.Manifold),
		NormalImpulse: zero, TangentImpulse: zeroImpulseVec(),
		PointImpulses:        []ContactPointImpulse{{Normal: zero, Tangent: zeroImpulseVec()}},
		PreVelocity:          eventState.entries[reportBody].LinearVelocity,
		PostVelocity:         eventState.entries[reportBody].LinearVelocity,
		PreVelocityA:         eventState.entries[0].LinearVelocity,
		PreVelocityB:         eventState.entries[1].LinearVelocity,
		PostVelocityA:        eventState.entries[0].LinearVelocity,
		PostVelocityB:        eventState.entries[1].LinearVelocity,
		PreAngularVelocityA:  eventState.entries[0].AngularVelocity,
		PreAngularVelocityB:  eventState.entries[1].AngularVelocity,
		PostAngularVelocityA: eventState.entries[0].AngularVelocity,
		PostAngularVelocityB: eventState.entries[1].AngularVelocity,
		PoseA:                eventState.entries[0].Pose, PoseB: eventState.entries[1].Pose,
	}
	trace := Trace{start: from, pre: eventState, post: eventState, end: end,
		duration: dt, eventAt: eventAt, hasEvent: true, grazingSweep: rounded}
	if event.PoseA != trace.pre.entries[0].Pose || event.PoseA != trace.post.entries[0].Pose ||
		event.PoseB != trace.pre.entries[1].Pose || event.PoseB != trace.post.entries[1].Pose {
		return undecided(w, "grazing event poses differ from trace"), nil
	}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event}, Trace: trace}, nil
}

func grazingManifoldsMatch(a, b decad.ContactPoint, req decad.ContactRequest,
	penetration units.Value) bool {
	if a.FaceA != b.FaceA || a.FaceB != b.FaceB || a.FeatureA != b.FeatureA ||
		a.FeatureB != b.FeatureB {
		return false
	}
	for _, witness := range [][2]decad.VecMeasurement{{a.OnA, b.OnA}, {a.OnB, b.OnB}} {
		difference := witness[0].Value.Sub(witness[1].Value)
		bound := math.Abs(difference.X) + math.Abs(difference.Y) + math.Abs(difference.Z) +
			witness[0].Bound.Base() + witness[1].Bound.Base()
		if !finite(bound) || bound > req.PointResolution.Base() {
			return false
		}
	}
	normal := a.Normal.Value.Sub(b.Normal.Value)
	normalBound := math.Abs(normal.X) + math.Abs(normal.Y) + math.Abs(normal.Z) +
		a.Normal.Bound.Base() + b.Normal.Bound.Base() + a.NormalAngle.Base() + b.NormalAngle.Base()
	separation := math.Abs(a.Separation.Value.Base()-b.Separation.Value.Base()) +
		a.Separation.Bound.Base() + b.Separation.Bound.Base()
	return finite(normalBound, separation) && normalBound <= req.NormalResolution.Base() &&
		separation <= penetration.Base()
}

func grazingNormalSpeedWithin(state State, point decad.ContactPoint, residual units.Value) bool {
	normal := [3]float64{point.Normal.Value.X, point.Normal.Value.Y, point.Normal.Value.Z}
	nominal, totalSpeed := new(big.Rat), new(big.Rat)
	for axis, direction := range normal {
		a := exactBase(velocityComponent(state.entries[0].LinearVelocity, axis))
		b := exactBase(velocityComponent(state.entries[1].LinearVelocity, axis))
		n := new(big.Rat).SetFloat64(direction)
		if a == nil || b == nil || n == nil {
			return false
		}
		relative := new(big.Rat).Sub(b, a)
		nominal.Add(nominal, new(big.Rat).Mul(relative, n))
		totalSpeed.Add(totalSpeed, absRat(relative))
	}
	bound := exactBase(point.Normal.Bound)
	angle := exactBase(point.NormalAngle)
	limit := exactBase(residual)
	if bound == nil || angle == nil || limit == nil || bound.Sign() < 0 || angle.Sign() < 0 {
		return false
	}
	errorSpeed := new(big.Rat).Mul(totalSpeed, new(big.Rat).Add(bound, angle))
	return new(big.Rat).Add(absRat(nominal), errorSpeed).Cmp(limit) <= 0
}
