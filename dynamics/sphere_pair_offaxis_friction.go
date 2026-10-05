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

type sphereRatVec2 struct{ x, y *big.Rat }

func sphereVec2(x, y *big.Rat) sphereRatVec2 { return sphereRatVec2{x, y} }

func (v sphereRatVec2) dot(other sphereRatVec2) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(v.x, other.x), new(big.Rat).Mul(v.y, other.y))
}

func (v sphereRatVec2) scale(factor *big.Rat) sphereRatVec2 {
	return sphereVec2(new(big.Rat).Mul(v.x, factor), new(big.Rat).Mul(v.y, factor))
}

func (v sphereRatVec2) add(other sphereRatVec2) sphereRatVec2 {
	return sphereVec2(new(big.Rat).Add(v.x, other.x), new(big.Rat).Add(v.y, other.y))
}

func (v sphereRatVec2) sub(other sphereRatVec2) sphereRatVec2 {
	return sphereVec2(new(big.Rat).Sub(v.x, other.x), new(big.Rat).Sub(v.y, other.y))
}

func sphereRatPointWithin(value r3.Vec, bound units.Value, expected sphereRatVec2) bool {
	if !finite(value.X, value.Y, value.Z, bound.Base()) || bound.Base() < 0 || value.Z != 0 {
		return false
	}
	difference := sphereVec2(ratFloat(value.X), ratFloat(value.Y)).sub(expected)
	return difference.dot(difference).Cmp(new(big.Rat).Mul(exactBase(bound), exactBase(bound))) <= 0
}

// stepInitialOffAxisSpherePairFriction uses the source radii and exact center
// offset to resolve one planar Coulomb impulse. Bounded float witnesses may
// enclose this exact geometry, but they never choose its response direction.
func (w *World) stepInitialOffAxisSpherePairFriction(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	if first == nil || first.Event == nil || first.Event.Manifold == nil ||
		first.Outcome != decad.SweepInitiallyTouching || first.Event.Relation != decad.ContactTouching ||
		first.Event.At.Fraction != units.Scalar(0) || len(first.Event.Manifold.Points) != 1 ||
		len(first.Samples) == 0 || first.Samples[0].FloatContact == nil ||
		first.Samples[0].FloatContact.Relation != decad.ContactTouching ||
		first.Samples[0].FloatContact.Manifold == nil ||
		len(first.Samples[0].FloatContact.Manifold.Points) != 1 ||
		w.parts[0].definition.Role != Dynamic || w.parts[1].definition.Role != Dynamic ||
		w.step.MaxEvents <= 1 || w.restitution.Base() <= 0 ||
		w.friction.lower == nil || w.friction.upper == nil ||
		w.friction.lower.Sign() <= 0 || w.friction.lower.Cmp(w.friction.upper) != 0 {
		return undecided(w, "off-axis sphere friction needs an initial exact dynamic point and coefficient"), nil
	}
	point := first.Event.Manifold.Points[0]
	rounded := first.Samples[0].FloatContact.Manifold.Points[0]
	if !isSourceSpherePairEvent(first.Event.Manifold) ||
		rounded.FaceA != point.FaceA || rounded.FaceB != point.FaceB ||
		rounded.FeatureA != point.FeatureA || rounded.FeatureB != point.FeatureB ||
		outwardSum(math.Abs(point.Separation.Value.Base()), point.Separation.Bound.Base(),
			math.Abs(rounded.Separation.Value.Base()), rounded.Separation.Bound.Base()) >
			w.step.PenetrationResidual.Base() {
		return undecided(w, "off-axis sphere friction lacks matching source witnesses"), nil
	}
	var mass, moment, radius [2]*big.Rat
	var center [2]sphereRatVec2
	for i, face := range [2]*decad.Face{point.FaceA, point.FaceB} {
		if face == nil || w.parts[i].definition.Supplied == nil ||
			kicked.entries[i].Pose.Basis() != r3.Identity().Basis() ||
			!zeroAngularVelocity(kicked.entries[i].AngularVelocity) ||
			kicked.entries[i].LinearVelocity.Z.Base() != 0 ||
			kicked.entries[i].Pose.Translation().Z != 0 {
			return undecided(w, "off-axis sphere friction needs planar translating source spheres"), nil
		}
		sphere, source := face.Surface().(decad.Sphere)
		if !source || sphere.Center != (r3.Vec{}) {
			return undecided(w, "off-axis sphere friction needs centered source spheres"), nil
		}
		var ok bool
		mass[i], moment[i], radius[i], ok = exactSphereFloorMass(w.parts[i].mass, sphere)
		if !ok {
			return undecided(w, "off-axis sphere friction needs exact centered isotropic mass"), nil
		}
		translation := kicked.entries[i].Pose.Translation()
		center[i] = sphereVec2(ratFloat(translation.X), ratFloat(translation.Y))
	}
	radiusSum := new(big.Rat).Add(radius[0], radius[1])
	offset := center[1].sub(center[0])
	if offset.dot(offset).Cmp(new(big.Rat).Mul(radiusSum, radiusSum)) != 0 ||
		offset.x.Sign() == 0 || offset.y.Sign() == 0 {
		return undecided(w, "off-axis sphere friction center distance is not an exact touch"), nil
	}
	normal := sphereVec2(new(big.Rat).Quo(offset.x, radiusSum),
		new(big.Rat).Quo(offset.y, radiusSum))
	tangent := sphereVec2(new(big.Rat).Neg(normal.y), new(big.Rat).Set(normal.x))
	witness := center[0].add(normal.scale(radius[0]))
	for _, candidate := range []decad.ContactPoint{point, rounded} {
		if !sphereRatPointWithin(candidate.OnA.Value, candidate.OnA.Bound, witness) ||
			!sphereRatPointWithin(candidate.OnB.Value, candidate.OnB.Bound, witness) ||
			!sphereRatPointWithin(candidate.Normal.Value, candidate.Normal.Bound, normal) ||
			candidate.NormalAngle.Base() > w.step.Contact.NormalResolution.Base() {
			return undecided(w, "off-axis sphere friction witness does not enclose the source point"), nil
		}
	}
	pre := [2]QuantityVec{kicked.entries[0].LinearVelocity, kicked.entries[1].LinearVelocity}
	velocity := [2]sphereRatVec2{
		sphereVec2(exactBase(pre[0].X), exactBase(pre[0].Y)),
		sphereVec2(exactBase(pre[1].X), exactBase(pre[1].Y)),
	}
	relative := velocity[1].sub(velocity[0])
	preNormal, slip := relative.dot(normal), relative.dot(tangent)
	if preNormal.Cmp(new(big.Rat).Neg(exactBase(w.step.ImpactSpeed))) >= 0 ||
		preNormal.Cmp(new(big.Rat).Neg(exactBase(w.step.VelocityResidual))) >= 0 {
		return undecided(w, "off-axis sphere friction lacks closing speed"), nil
	}
	inverse := [2]*big.Rat{new(big.Rat).Inv(mass[0]), new(big.Rat).Inv(mass[1])}
	inverseSum := new(big.Rat).Add(inverse[0], inverse[1])
	restitution := exactBase(w.restitution)
	jn := new(big.Rat).Neg(new(big.Rat).Quo(new(big.Rat).Mul(
		new(big.Rat).Add(big.NewRat(1, 1), restitution), preNormal), inverseSum))
	tangentInverse := new(big.Rat).Add(inverseSum,
		new(big.Rat).Add(new(big.Rat).Quo(new(big.Rat).Mul(radius[0], radius[0]), moment[0]),
			new(big.Rat).Quo(new(big.Rat).Mul(radius[1], radius[1]), moment[1])))
	jt := new(big.Rat).Neg(new(big.Rat).Quo(slip, tangentInverse))
	cone := new(big.Rat).Mul(w.friction.lower, jn)
	sliding := absRat(new(big.Rat).Set(jt)).Cmp(cone) > 0
	if sliding {
		jt.Set(cone)
		if slip.Sign() > 0 {
			jt.Neg(jt)
		}
	}
	impulse := normal.scale(jn).add(tangent.scale(jt))
	var postIdeal [2]sphereRatVec2
	var spinIdeal [2]*big.Rat
	for i := range postIdeal {
		signed := inverse[i]
		if i == 0 {
			signed = new(big.Rat).Neg(signed)
		}
		postIdeal[i] = velocity[i].add(impulse.scale(signed))
		spinIdeal[i] = new(big.Rat).Neg(new(big.Rat).Quo(new(big.Rat).Mul(radius[i], jt), moment[i]))
	}
	jnFloat, jtFloat := sphereRatFloat(jn), sphereRatFloat(jt)
	if !finite(jnFloat, jtFloat) || jnFloat <= 0 ||
		!rationalRoundedWithin(jn, jnFloat, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(jt, jtFloat, w.step.ImpulseResidual) {
		return undecided(w, "off-axis sphere friction impulse exceeds rounding residual"), nil
	}
	post := kicked
	for i := range post.entries {
		vx, vy, spin := sphereRatFloat(postIdeal[i].x), sphereRatFloat(postIdeal[i].y),
			sphereRatFloat(spinIdeal[i])
		if !finite(vx, vy, spin) ||
			!rationalRoundedWithin(postIdeal[i].x, vx, w.step.VelocityResidual) ||
			!rationalRoundedWithin(postIdeal[i].y, vy, w.step.VelocityResidual) ||
			!rationalRoundedWithin(spinIdeal[i], spin, w.step.AngularVelocityResidual) {
			return undecided(w, "off-axis sphere friction response exceeds rounding residual"), nil
		}
		post.entries[i].LinearVelocity = spherePairQuantityVelocity(r3.Vec{X: vx, Y: vy})
		post.entries[i].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(spin)}
	}
	actual := sphereVec2(exactBase(post.entries[1].LinearVelocity.X),
		exactBase(post.entries[1].LinearVelocity.Y)).sub(sphereVec2(
		exactBase(post.entries[0].LinearVelocity.X), exactBase(post.entries[0].LinearVelocity.Y)))
	normalResidual := absRat(new(big.Rat).Add(actual.dot(normal),
		new(big.Rat).Mul(restitution, preNormal)))
	slipAfter := actual.dot(tangent)
	for i := range post.entries {
		slipAfter.Sub(slipAfter, new(big.Rat).Mul(radius[i],
			exactBase(post.entries[i].AngularVelocity.Z)))
	}
	idealSlip := new(big.Rat).Add(slip, new(big.Rat).Mul(tangentInverse, jt))
	tangentResidual := absRat(new(big.Rat).Sub(slipAfter, idealSlip))
	if sliding && slip.Sign()*slipAfter.Sign() <= 0 ||
		!sliding && absRat(slipAfter).Cmp(exactBase(w.step.VelocityResidual)) > 0 {
		return undecided(w, "off-axis sphere friction slip direction is unresolved"), nil
	}
	if sliding {
		impulseResidual := absRat(new(big.Rat).Add(ratFloat(jtFloat),
			new(big.Rat).Mul(w.friction.lower, new(big.Rat).Mul(ratFloat(jnFloat),
				big.NewRat(int64(slip.Sign()), 1)))))
		if impulseResidual.Cmp(exactBase(w.step.ImpulseResidual)) > 0 {
			return undecided(w, "off-axis sphere friction sliding impulse exceeds residual"), nil
		}
	}
	coneResidual := new(big.Rat).Sub(absRat(ratFloat(jtFloat)),
		new(big.Rat).Mul(w.friction.lower, ratFloat(jnFloat)))
	if coneResidual.Sign() < 0 {
		coneResidual.SetInt64(0)
	}
	if normalResidual.Cmp(exactBase(w.step.VelocityResidual)) > 0 ||
		tangentResidual.Cmp(exactBase(w.step.VelocityResidual)) > 0 ||
		coneResidual.Cmp(exactBase(w.step.ImpulseResidual)) > 0 {
		return undecided(w, "off-axis sphere friction contact residual exceeds its limit"), nil
	}
	ideal, err := w.sweep(ctx, post, dt, decad.ContinueSeparatingTouch)
	if err != nil {
		return nil, err
	}
	if !spherePairContinuationWithin(ideal, decad.SweepDepartedClear, point, w.step) {
		return undecided(w, fmt.Sprintf("off-axis sphere friction departure returned %v", ideal.Outcome)), nil
	}
	poseA, poseB, err := ideal.CertifiedPosesAt(dt)
	if err != nil {
		//nolint:nilerr // A missing replay proof is a supported Undecided outcome.
		return undecided(w, "off-axis sphere friction endpoint lacks replay proof"), nil
	}
	end := post
	end.entries[0].Pose, end.entries[1].Pose = poseA, poseB
	lastSample := ideal.Samples[len(ideal.Samples)-1]
	if lastSample.At.Fraction != units.Scalar(1) || lastSample.FloatContact == nil ||
		lastSample.FloatContact.Relation != decad.ContactSeparated ||
		lastSample.Ideal.Relation != decad.ContactSeparated ||
		lastSample.PoseA != poseA || lastSample.PoseB != poseB {
		return undecided(w, "off-axis sphere friction rounded endpoint lacks separation proof"), nil
	}
	last, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		poseA, poseB, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if last.Relation != decad.ContactSeparated || last.Gap == nil ||
		last.Gap.Value.Base()-last.Gap.Bound.Base() <= 0 {
		return undecided(w, "off-axis sphere friction endpoint is not separated"), nil
	}
	angularUpper := new(big.Rat)
	for i := range post.entries {
		for _, value := range []*big.Rat{spinIdeal[i], exactBase(post.entries[i].AngularVelocity.Z)} {
			if magnitude := absRat(new(big.Rat).Set(value)); magnitude.Cmp(angularUpper) > 0 {
				angularUpper.Set(magnitude)
			}
		}
	}
	idealTangent := tangent.scale(jt)
	impulseX, impulseY := sphereRatFloat(idealTangent.x), sphereRatFloat(idealTangent.y)
	if !finite(impulseX, impulseY) ||
		!rationalRoundedWithin(idealTangent.x, impulseX, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(idealTangent.y, impulseY, w.step.ImpulseResidual) {
		return undecided(w, "off-axis sphere friction tangent impulse exceeds rounding residual"), nil
	}
	tangentImpulse := QuantityVec{X: units.KilogramMillimetersPerSecond(impulseX),
		Y: units.KilogramMillimetersPerSecond(impulseY), Z: units.KilogramMillimetersPerSecond(0)}
	instant := first.Event.At
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
		Manifold:      cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(jnFloat), TangentImpulse: tangentImpulse,
		PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(jnFloat),
			Tangent: tangentImpulse}},
		Solver: &ContactSolverReport{NormalResidual: units.MillimetersPerSecond(outwardRatFloat(normalResidual)),
			TangentResidual: units.MillimetersPerSecond(outwardRatFloat(tangentResidual)),
			ConeResidual:    units.KilogramMillimetersPerSecond(outwardRatFloat(coneResidual)),
			PenetrationResidual: units.Millimeters(outwardSum(math.Abs(point.Separation.Value.Base()),
				point.Separation.Bound.Base())),
			AngularUpper: units.RadiansPerSecond(outwardRatFloat(angularUpper)), Iterations: 1},
		PreVelocity: pre[1], PostVelocity: post.entries[1].LinearVelocity,
		PreVelocityA: pre[0], PreVelocityB: pre[1],
		PostVelocityA: post.entries[0].LinearVelocity, PostVelocityB: post.entries[1].LinearVelocity,
		PreAngularVelocityA:  kicked.entries[0].AngularVelocity,
		PreAngularVelocityB:  kicked.entries[1].AngularVelocity,
		PostAngularVelocityA: post.entries[0].AngularVelocity,
		PostAngularVelocityB: post.entries[1].AngularVelocity,
		PoseA:                kicked.entries[0].Pose, PoseB: kicked.entries[1].Pose}
	return &StepReport{Status: Advanced, Next: &end, Events: []ContactEvent{event},
		Trace: Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
			eventAt: instant.Elapsed.Value, hasEvent: true, rotationalRemainder: ideal}}, nil
}
