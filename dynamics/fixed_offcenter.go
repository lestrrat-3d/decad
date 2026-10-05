package dynamics

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// fixedOffcenterPatch identifies the narrow initial face impact handled here.
// The later response gate checks all four witnesses and the supplied inertia.
func (w *World) fixedOffcenterPatch(event *decad.SweepEvent) bool {
	if event == nil || event.Manifold == nil || len(event.Manifold.Points) != 4 ||
		w.parts[0].definition.Role != Fixed || w.parts[1].definition.Role != Dynamic ||
		w.parts[1].definition.Supplied == nil || w.friction.upper.Sign() != 0 ||
		w.restitution.Base() <= 0 || event.Manifold.Points[0].Normal.Value != (r3.Vec{Z: 1}) {
		return false
	}
	center := w.parts[1].mass.Center.Value.X
	maximum := event.Manifold.Points[0].OnB.Value.X
	for _, point := range event.Manifold.Points {
		maximum = math.Max(maximum, point.OnB.Value.X)
	}
	return maximum < center
}

// stepFixedOffcenter resolves an initial horizontal patch with two active
// vertices on its edge nearest the dynamic center. Every other manifold
// vertex must have a certified separating normal speed after the impulse.
func (w *World) stepFixedOffcenter(ctx context.Context, from, kicked State, dt units.Value,
	first *decad.SweepReport) (*StepReport, error) {
	manifold := first.Event.Manifold
	if first.Event.Relation != decad.ContactTouching ||
		kicked.entries[0].Pose != r3.Identity() || kicked.entries[1].Pose != r3.Identity() ||
		!zeroAngularVelocity(kicked.entries[0].AngularVelocity) ||
		!zeroAngularVelocity(kicked.entries[1].AngularVelocity) ||
		kicked.entries[0].LinearVelocity != zeroVelocityVec() ||
		kicked.entries[1].LinearVelocity.X.Base() != 0 ||
		kicked.entries[1].LinearVelocity.Y.Base() != 0 ||
		kicked.entries[1].LinearVelocity.Z.Base() >= 0 || w.step.MaxEvents <= 1 {
		return undecided(w, "fixed off-center impact exceeds the initial face path"), nil
	}
	normal, separation, separationBound, ok := reducedContact(manifold, w.step.Contact)
	if !ok || normal != (r3.Vec{Z: 1}) || !finite(separation, separationBound) ||
		math.Abs(separation)+separationBound > w.step.PenetrationResidual.Base() ||
		!w.fixedFloorPatchWitnesses(manifold) {
		return undecided(w, "fixed off-center manifold exceeds its witness bounds"), nil
	}
	mass := w.parts[1].mass
	if mass.Inertia.XY.Value.Base() != 0 || mass.Inertia.XZ.Value.Base() != 0 ||
		mass.Inertia.YZ.Value.Base() != 0 || mass.Inertia.YY.Value.Base() <= 0 {
		return undecided(w, "fixed off-center mass needs a bounded principal Y inertia"), nil
	}
	center := mass.Center.Value
	maximum := manifold.Points[0].OnB.Value.X
	for _, point := range manifold.Points[1:] {
		maximum = math.Max(maximum, point.OnB.Value.X)
	}
	arm := center.X - maximum
	if !finite(arm) || arm <= 0 {
		return undecided(w, "fixed off-center mass center does not permit an edge response"), nil
	}
	var active [2]int
	count := 0
	var points [4]pairPatchPoint
	for i, witness := range manifold.Points {
		if witness.OnB.Value.Z != manifold.Points[0].OnB.Value.Z ||
			witness.OnB.Value.X > maximum || witness.OnB.Value.X >= center.X ||
			witness.OnB.Bound.Base() > w.step.Contact.PointResolution.Base() {
			return undecided(w, "fixed off-center patch has an unproved edge"), nil
		}
		points[i].exactArm[1] = exactPatchLever(witness.OnB.Value, center)
		points[i].bound[1] = exactBase(witness.OnB.Bound)
		if witness.OnB.Value.X == maximum {
			if count == len(active) {
				return undecided(w, "fixed off-center patch has too many active vertices"), nil
			}
			active[count] = i
			count++
		}
	}
	if count != 2 || manifold.Points[active[0]].OnB.Value.Y ==
		manifold.Points[active[1]].OnB.Value.Y ||
		new(big.Rat).Add(ratFloat(manifold.Points[active[0]].OnB.Value.Y),
			ratFloat(manifold.Points[active[1]].OnB.Value.Y)).Cmp(
			new(big.Rat).Mul(big.NewRat(2, 1), ratFloat(center.Y))) != 0 {
		return undecided(w, "fixed off-center active edge is not symmetric about the mass center"), nil
	}
	preZ := kicked.entries[1].LinearVelocity.Z.Base()
	if new(big.Rat).Neg(exactBase(kicked.entries[1].LinearVelocity.Z)).Cmp(
		exactBase(w.step.ImpactSpeed)) <= 0 {
		return undecided(w, "fixed off-center closing speed does not exceed impact threshold"), nil
	}
	inverseMass, inverseInertia := 1/mass.Mass.Value.Base(), 1/mass.Inertia.YY.Value.Base()
	effective := inverseMass + arm*arm*inverseInertia
	impulse := -(1 + w.restitution.Base()) * preZ / effective
	postZ := preZ + impulse*inverseMass
	spinY := impulse * arm * inverseInertia
	if !finite(effective, impulse, postZ, spinY) || effective <= 0 || impulse <= 0 || spinY <= 0 {
		return undecided(w, "fixed off-center response is not finite and positive"), nil
	}
	for _, index := range active {
		points[index].jn = impulse / 2
	}
	angularError, normalResidual, valid := fixedOffcenterResiduals(mass,
		manifold, points, postZ, spinY, kicked.entries[1].LinearVelocity.Z,
		w.restitution, w.step)
	if !valid {
		return undecided(w, "fixed off-center response exceeds solver residuals"), nil
	}
	post := kicked
	post.entries[1].LinearVelocity.Z = units.MillimetersPerSecond(postZ)
	post.entries[1].AngularVelocity.Y = units.RadiansPerSecond(spinY)
	ideal, err := w.sweep(ctx, post, dt, decad.ContinueSeparatingTouch)
	if err != nil {
		return nil, err
	}
	poses, valid := w.fixedOffcenterEndpoint(post, dt, ideal)
	if !valid {
		return undecided(w, "fixed off-center spin needs a certified departure remainder"), nil
	}
	end := post
	end.entries[0].Pose, end.entries[1].Pose = poses[0], poses[1]
	pointImpulses := make([]ContactPointImpulse, 4)
	for i := range pointImpulses {
		pointImpulses[i] = ContactPointImpulse{Normal: units.KilogramMillimetersPerSecond(points[i].jn),
			Tangent: zeroImpulseVec()}
	}
	instant := first.Event.At
	angularUpper := new(big.Rat).Add(absRat(ratFloat(spinY)), angularError)
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:           ContactImpact,
		Pair:           BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:        decad.SweepInterval{From: instant, To: instant},
		Time:           instant.Elapsed.Value,
		Manifold:       cloneManifold(*manifold),
		NormalImpulse:  units.KilogramMillimetersPerSecond(impulse),
		TangentImpulse: zeroImpulseVec(),
		PointImpulses:  pointImpulses,
		Solver: &ContactSolverReport{
			NormalResidual:      units.MillimetersPerSecond(outwardRatFloat(normalResidual)),
			TangentResidual:     units.MillimetersPerSecond(0),
			ConeResidual:        units.KilogramMillimetersPerSecond(0),
			PenetrationResidual: units.Millimeters(math.Abs(separation) + separationBound),
			AngularUpper:        units.RadiansPerSecond(outwardRatFloat(angularUpper)), Iterations: 1},
		PreVelocity: kicked.entries[1].LinearVelocity, PostVelocity: post.entries[1].LinearVelocity,
		PreVelocityA: kicked.entries[0].LinearVelocity, PreVelocityB: kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                kicked.entries[0].Pose, PoseB: kicked.entries[1].Pose,
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true, rotationalRemainder: ideal}
	return report, nil
}

func zeroVelocityVec() QuantityVec {
	zero := units.MillimetersPerSecond(0)
	return QuantityVec{X: zero, Y: zero, Z: zero}
}

func fixedOffcenterResiduals(mass decad.MassProperties, manifold *decad.ContactManifold,
	points [4]pairPatchPoint, postZ, spinY float64, preZ, restitution units.Value,
	cfg StepConfig) (*big.Rat, *big.Rat, bool) {
	low := new(big.Rat).Sub(exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound))
	high := new(big.Rat).Add(exactBase(mass.Mass.Value), exactBase(mass.Mass.Bound))
	if low.Sign() <= 0 {
		return nil, nil, false
	}
	impulse := new(big.Rat)
	for _, point := range points {
		impulse.Add(impulse, ratFloat(point.jn))
	}
	responseLow := new(big.Rat).Add(exactBase(preZ), new(big.Rat).Quo(impulse, low))
	responseHigh := new(big.Rat).Add(exactBase(preZ), new(big.Rat).Quo(impulse, high))
	if responseLow.Cmp(responseHigh) > 0 {
		responseLow, responseHigh = responseHigh, responseLow
	}
	linearError := intervalDeviation(ratFloat(postZ), responseLow, responseHigh)
	angularError := pairAngularLawError(mass, points, 1, [3]float64{0, spinY, 0}, 1)
	if angularError == nil || linearError.Cmp(exactBase(cfg.VelocityResidual)) > 0 ||
		angularError.Cmp(exactBase(cfg.AngularVelocityResidual)) > 0 {
		return nil, nil, false
	}
	target := new(big.Rat).Mul(new(big.Rat).Neg(exactBase(preZ)), exactBase(restitution))
	maxResidual := new(big.Rat)
	centerX := ratFloat(mass.Center.Value.X)
	for i, point := range manifold.Points {
		arm := new(big.Rat).Sub(centerX, ratFloat(point.OnB.Value.X))
		velocity := new(big.Rat).Add(ratFloat(postZ), new(big.Rat).Mul(ratFloat(spinY), arm))
		uncertainty := new(big.Rat).Add(linearError,
			new(big.Rat).Mul(angularError, absRat(new(big.Rat).Set(arm))))
		positionBound := new(big.Rat).Add(exactBase(mass.Center.Bound), exactBase(point.OnB.Bound))
		uncertainty.Add(uncertainty, new(big.Rat).Mul(positionBound,
			new(big.Rat).Add(absRat(ratFloat(spinY)), angularError)))
		residual := new(big.Rat).Sub(target, velocity)
		if points[i].jn > 0 {
			residual = absRat(residual)
		} else if residual.Sign() < 0 {
			residual.SetInt64(0)
		}
		residual.Add(residual, uncertainty)
		if residual.Cmp(exactBase(cfg.VelocityResidual)) > 0 {
			return nil, nil, false
		}
		if residual.Cmp(maxResidual) > 0 {
			maxResidual = residual
		}
	}
	return angularError, maxResidual, true
}

func (w *World) fixedOffcenterEndpoint(post State, dt units.Value,
	sweep *decad.SweepReport) ([2]r3.Transform, bool) {
	var poses [2]r3.Transform
	if sweep == nil || sweep.Outcome != decad.SweepDepartedClear ||
		sweep.Departure == nil || sweep.InitialEvent == nil ||
		sweep.InitialEvent.Relation != decad.ContactTouching {
		return poses, false
	}
	fixed, fixedOK := sweep.PathA.(decad.PoseSegment)
	drift, driftOK := sweep.PathB.(decad.RigidDriftSegment)
	if !fixedOK || !driftOK || fixed.From != post.entries[0].Pose || fixed.To != fixed.From ||
		fixed.Duration != dt || drift.From != post.entries[1].Pose ||
		drift.Center != post.entries[1].Pose.Apply(w.parts[1].mass.Center.Value) ||
		drift.LinearVelocity != post.entries[1].LinearVelocity ||
		drift.AngularVelocity != post.entries[1].AngularVelocity || drift.Duration != dt {
		return poses, false
	}
	found := false
	for _, sample := range sweep.Samples {
		if sample.At.Fraction.Base() != 1 {
			continue
		}
		if found || sample.Ideal.Relation != decad.ContactSeparated ||
			sample.Ideal.Gap == nil || sample.FloatContact == nil ||
			sample.FloatContact.Relation != decad.ContactSeparated {
			return poses, false
		}
		value, bound := exactBase(sample.Ideal.Gap.Value), exactBase(sample.Ideal.Gap.Bound)
		if value == nil || bound == nil || value.Cmp(bound) <= 0 {
			return poses, false
		}
		poses, found = [2]r3.Transform{sample.PoseA, sample.PoseB}, true
	}
	return poses, found
}
