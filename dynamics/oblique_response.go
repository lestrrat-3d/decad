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

// stepObliqueSupport admits a centered frictionless face impact only when the
// incoming velocity is normal to the bounded contact and no spin is needed.
func (w *World) stepObliqueSupport(ctx context.Context, from, kicked State,
	dt units.Value) (*StepReport, error) {
	if w.friction.lower.Sign() != 0 || w.parts[0].definition.Role == w.parts[1].definition.Role {
		return undecided(w, "tilted contact needs a frictionless fixed/dynamic support pair"), nil
	}
	first, err := w.sweep(ctx, kicked, dt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if first.Outcome == decad.SweepClear {
		end, driftErr := driftState(kicked, dt.Base())
		if driftErr != nil {
			return undecidedArithmetic(w, "non-finite tilted clear endpoint", driftErr)
		}
		rounded, sweepErr := w.sweepPoses(ctx, kicked, end, dt, decad.StopAtInitialContact)
		if sweepErr != nil {
			return nil, sweepErr
		}
		if rounded.Outcome != decad.SweepClear {
			return undecided(w, fmt.Sprintf("rounded tilted clear path returned %v", rounded.Outcome)), nil
		}
		return &StepReport{Status: Advanced, Next: &end, Trace: Trace{
			start: from, pre: kicked, end: end, duration: dt, preSweep: rounded}}, nil
	}
	if first.Outcome == decad.SweepImpactBracket {
		return w.stepObliqueSphereImpact(ctx, from, kicked, dt, first)
	}
	if first.Outcome != decad.SweepInitiallyTouching || first.Event == nil || first.Event.Manifold == nil {
		return undecided(w, fmt.Sprintf("tilted initial contact returned %v", first.Outcome)), nil
	}
	if normal, _, _, ok := reducedContact(first.Event.Manifold, w.step.Contact); ok {
		if _, _, axis := axisNormal(normal); axis && w.restitution.Base() == 0 &&
			w.friction.upper.Sign() == 0 {
			return w.stepInitialTouch(ctx, from, kicked, dt, first)
		}
	}
	normal, separation, uncertainty, ok := boundedObliqueContact(first.Event.Manifold, w.step.Contact)
	if !ok || math.Abs(separation)+uncertainty > w.step.PenetrationResidual.Base() {
		return undecided(w, "tilted initial contact exceeds its geometry residual"), nil
	}
	dynamic := 0
	if w.parts[1].definition.Role == Dynamic {
		dynamic = 1
	}
	pre := kicked.entries[dynamic].LinearVelocity
	velocity := r3.Vec{X: pre.X.Base(), Y: pre.Y.Base(), Z: pre.Z.Base()}
	closing := velocity.X*normal.X + velocity.Y*normal.Y + velocity.Z*normal.Z
	if dynamic == 0 {
		closing = -closing
	}
	if !finite(closing) || closing >= -w.step.VelocityResidual.Base() {
		return undecided(w, "tilted contact has no bounded closing speed"), nil
	}
	mass := w.parts[dynamic].mass.Mass
	stopImpulse := -closing * mass.Value.Base()
	if !finite(stopImpulse) || stopImpulse <= 0 ||
		!obliqueStopMomentumWithin(pre, normal, first.Event.Manifold, mass,
			stopImpulse, dynamic, w.step) {
		return undecided(w, "tilted support needs tangent velocity or exceeds impulse residual"), nil
	}
	coefficient := 0.0
	if -closing > w.step.ImpactSpeed.Base() {
		coefficient = w.restitution.Base()
	}
	impulse := stopImpulse * (1 + coefficient)
	if !finite(impulse) || impulse <= 0 {
		return undecided(w, "tilted impact impulse is not finite"), nil
	}
	post := kicked
	post.entries[dynamic].LinearVelocity = QuantityVec{
		X: units.MillimetersPerSecond(-coefficient * pre.X.Base()),
		Y: units.MillimetersPerSecond(-coefficient * pre.Y.Base()),
		Z: units.MillimetersPerSecond(-coefficient * pre.Z.Base()),
	}
	policy := decad.ContinueCertifiedTouch
	if coefficient > 0 {
		policy = decad.ContinueSeparatingTouch
	}
	ideal, err := w.sweep(ctx, post, dt, policy)
	if err != nil {
		return nil, err
	}
	if coefficient == 0 && !w.obliqueTrackWithin(ideal, normal) ||
		coefficient > 0 && !obliqueDepartureWithin(ideal) {
		return undecided(w, fmt.Sprintf("tilted support continuation returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite tilted support endpoint", err)
	}
	rounded, err := w.sweepPoses(ctx, post, end, dt, policy)
	if err != nil {
		return nil, err
	}
	if coefficient == 0 && !w.obliqueTrackWithin(rounded, normal) ||
		coefficient > 0 && !obliqueDepartureWithin(rounded) {
		return undecided(w, fmt.Sprintf("rounded tilted support returned %v", rounded.Outcome)), nil
	}
	last, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if coefficient == 0 {
		lastNormal, lastSeparation, lastBound, lastOK := boundedObliqueContact(last.Manifold, w.step.Contact)
		if last.Relation != decad.ContactTouching || !lastOK || lastNormal != normal ||
			math.Abs(lastSeparation)+lastBound > w.step.PenetrationResidual.Base() {
			return undecided(w, "tilted support endpoint lacks a bounded face contact"), nil
		}
	} else if last.Relation != decad.ContactSeparated || last.Gap == nil ||
		last.Gap.Value.Base()-last.Gap.Bound.Base() <= 0 {
		return undecided(w, "tilted rebound endpoint lacks a positive bounded gap"), nil
	}
	instant := first.Event.At
	points := make([]ContactPointImpulse, len(first.Event.Manifold.Points))
	share := impulse / float64(len(points))
	for i := range points {
		points[i] = ContactPointImpulse{Normal: units.KilogramMillimetersPerSecond(share),
			Tangent: zeroImpulseVec()}
	}
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
		Manifold:       cloneManifold(*first.Event.Manifold),
		NormalImpulse:  units.KilogramMillimetersPerSecond(impulse),
		TangentImpulse: zeroImpulseVec(), PointImpulses: points,
		PreVelocity: pre, PostVelocity: post.entries[dynamic].LinearVelocity,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                kicked.entries[0].Pose, PoseB: kicked.entries[1].Pose}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
			eventAt: instant.Elapsed.Value, hasEvent: true, postSweep: rounded}}, nil
}

func obliqueDepartureWithin(sweep *decad.SweepReport) bool {
	return sweep != nil && sweep.Outcome == decad.SweepDepartedClear && sweep.Departure != nil &&
		sweep.Departure.GapAtUntil.Value.Base()-sweep.Departure.GapAtUntil.Bound.Base() > 0
}

func boundedObliqueContact(manifold *decad.ContactManifold,
	req decad.ContactRequest) (r3.Vec, float64, float64, bool) {
	if manifold == nil || len(manifold.Points) != 4 {
		return r3.Vec{}, 0, 0, false
	}
	normal := manifold.Points[0].Normal.Value
	if _, _, axis := axisNormal(normal); axis || !finite(normal.X, normal.Y, normal.Z) {
		return r3.Vec{}, 0, 0, false
	}
	var separation, bound float64
	for _, point := range manifold.Points {
		if point.Normal.Value != normal || !finite(point.Normal.Bound.Base(), point.NormalAngle.Base(),
			point.OnA.Bound.Base(), point.OnB.Bound.Base(), point.Separation.Value.Base(),
			point.Separation.Bound.Base()) || point.Normal.Bound.Base() < 0 ||
			point.Normal.Bound.Base() > req.NormalResolution.Base() ||
			point.NormalAngle.Base() > req.NormalResolution.Base() ||
			point.OnA.Bound.Base() > req.PointResolution.Base() ||
			point.OnB.Bound.Base() > req.PointResolution.Base() {
			return r3.Vec{}, 0, 0, false
		}
		separation += point.Separation.Value.Base()
		bound = math.Max(bound, outwardSum(point.Separation.Bound.Base(),
			point.OnA.Bound.Base(), point.OnB.Bound.Base()))
	}
	return normal, separation / float64(len(manifold.Points)), bound, finite(separation, bound)
}

func obliqueStopMomentumWithin(pre QuantityVec, normal r3.Vec, manifold *decad.ContactManifold,
	mass decad.Measurement, impulse float64, dynamic int, step StepConfig) bool {
	m, massBound, j := exactBase(mass.Value), exactBase(mass.Bound), new(big.Rat).SetFloat64(impulse)
	limit, velocityLimit := exactBase(step.ImpulseResidual), exactBase(step.VelocityResidual)
	if m == nil || massBound == nil || j == nil || limit == nil || velocityLimit == nil ||
		massBound.Sign() < 0 || new(big.Rat).Sub(m, massBound).Sign() <= 0 {
		return false
	}
	maximumNormalError := new(big.Rat)
	for _, point := range manifold.Points {
		errorBound := new(big.Rat).Add(exactBase(point.Normal.Bound), exactBase(point.NormalAngle))
		if errorBound.Cmp(maximumNormalError) > 0 {
			maximumNormalError = errorBound
		}
	}
	sign := int64(1)
	if dynamic == 0 {
		sign = -1
	}
	for axis, component := range []float64{normal.X, normal.Y, normal.Z} {
		v, n := exactBase(velocityComponent(pre, axis)), new(big.Rat).SetFloat64(component)
		if v == nil || n == nil {
			return false
		}
		residual := new(big.Rat).Mul(m, v)
		residual.Add(residual, new(big.Rat).Mul(j, new(big.Rat).Mul(n, big.NewRat(sign, 1))))
		residual = absRat(residual)
		residual.Add(residual, new(big.Rat).Mul(massBound, absRat(v)))
		residual.Add(residual, new(big.Rat).Mul(new(big.Rat).Add(j, limit), maximumNormalError))
		allowance := new(big.Rat).Add(limit,
			new(big.Rat).Mul(new(big.Rat).Add(m, massBound), velocityLimit))
		if residual.Cmp(allowance) > 0 {
			return false
		}
	}
	return true
}

func (w *World) obliqueTrackWithin(sweep *decad.SweepReport, normal r3.Vec) bool {
	if sweep == nil || sweep.Outcome != decad.SweepPersistentTouch || sweep.ContactTrack == nil ||
		sweep.ContactTrack.Start().Fraction.Base() != 0 ||
		sweep.ContactTrack.End().Fraction.Base() != 1 ||
		sweep.ContactTrack.Normal().Value != normal {
		return false
	}
	for _, fraction := range []units.Value{units.Scalar(0), units.Scalar(.5), units.Scalar(1)} {
		manifold, err := sweep.ContactTrack.ManifoldAt(fraction)
		if err != nil {
			return false
		}
		sampleNormal, separation, bound, ok := boundedObliqueContact(manifold, w.step.Contact)
		if !ok || sampleNormal != normal || math.Abs(separation)+bound > w.step.PenetrationResidual.Base() {
			return false
		}
	}
	return true
}
