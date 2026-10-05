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

// stepInitialSpherePairFriction resolves one exact cardinal point. The exact
// centered mass and witness gates keep the normal and tangent laws scalar.
func (w *World) stepInitialSpherePairFriction(ctx context.Context, from, kicked State,
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
		w.friction.lower.Sign() < 0 || w.friction.lower.Cmp(w.friction.upper) != 0 {
		return undecided(w, "sphere-pair friction needs an initial exact dynamic point and coefficient"), nil
	}
	point := first.Event.Manifold.Points[0]
	rounded := first.Samples[0].FloatContact.Manifold.Points[0]
	if rounded.FaceA != point.FaceA || rounded.FaceB != point.FaceB ||
		rounded.FeatureA != point.FeatureA || rounded.FeatureB != point.FeatureB ||
		rounded.Normal.Value != point.Normal.Value ||
		outwardSum(math.Abs(rounded.Separation.Value.Base()), rounded.Separation.Bound.Base(),
			rounded.OnA.Bound.Base(), rounded.OnB.Bound.Base()) > w.step.PenetrationResidual.Base() {
		return undecided(w, "sphere-pair friction changes its rounded source features"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	if !ok || math.Abs(normal.X) != 1 || normal.Y != 0 || normal.Z != 0 ||
		outwardSum(math.Abs(separation), bound) > w.step.PenetrationResidual.Base() ||
		point.OnA.Value != point.OnB.Value || point.OnA.Bound.Base() != 0 ||
		point.OnB.Bound.Base() != 0 || point.Separation.Bound.Base() != 0 ||
		point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return undecided(w, "sphere-pair friction needs an exact cardinal point"), nil
	}
	var mass, moment, radius [2]*big.Rat
	for i, face := range [2]*decad.Face{point.FaceA, point.FaceB} {
		if face == nil || w.parts[i].definition.Supplied == nil ||
			kicked.entries[i].Pose.Basis() != r3.Identity().Basis() ||
			!zeroAngularVelocity(kicked.entries[i].AngularVelocity) ||
			kicked.entries[i].LinearVelocity.Z.Base() != 0 {
			return undecided(w, "sphere-pair friction needs translating planar source spheres"), nil
		}
		sphere, source := face.Surface().(decad.Sphere)
		if !source || sphere.Center != (r3.Vec{}) {
			return undecided(w, "sphere-pair friction needs a centered source sphere"), nil
		}
		mass[i], moment[i], radius[i], ok = exactSphereFloorMass(w.parts[i].mass, sphere)
		if !ok {
			return undecided(w, "sphere-pair friction needs exact centered isotropic mass"), nil
		}
		center := kicked.entries[i].Pose.Translation()
		witness := point.OnA.Value
		if i == 1 {
			witness = point.OnB.Value
		}
		expected := new(big.Rat).Mul(radius[i], ratFloat(normal.X))
		if i == 1 {
			expected.Neg(expected)
		}
		expected.Add(expected, ratFloat(center.X))
		if ratFloat(witness.X).Cmp(expected) != 0 || witness.Y != center.Y || witness.Z != center.Z {
			return undecided(w, "sphere-pair friction witness misses a source radius"), nil
		}
	}
	pre := [2]QuantityVec{kicked.entries[0].LinearVelocity, kicked.entries[1].LinearVelocity}
	preNormal := new(big.Rat).Mul(ratFloat(normal.X),
		new(big.Rat).Sub(exactBase(pre[1].X), exactBase(pre[0].X)))
	if preNormal.Cmp(new(big.Rat).Neg(exactBase(w.step.ImpactSpeed))) >= 0 ||
		preNormal.Cmp(new(big.Rat).Neg(exactBase(w.step.VelocityResidual))) >= 0 {
		return undecided(w, "sphere-pair friction lacks closing speed above the impact threshold"), nil
	}
	inverseA, inverseB := new(big.Rat).Inv(mass[0]), new(big.Rat).Inv(mass[1])
	inverseSum := new(big.Rat).Add(inverseA, inverseB)
	restitution := exactBase(w.restitution)
	jn := new(big.Rat).Neg(new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Add(big.NewRat(1, 1), restitution), preNormal), inverseSum))
	slip := new(big.Rat).Sub(exactBase(pre[1].Y), exactBase(pre[0].Y))
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
	postIdeal := [2][3]*big.Rat{}
	for i := range postIdeal {
		sign := int64(1)
		if i == 0 {
			sign = -1
		}
		inverse := inverseA
		if i == 1 {
			inverse = inverseB
		}
		postIdeal[i][0] = new(big.Rat).Add(exactBase(pre[i].X), new(big.Rat).Mul(
			big.NewRat(sign, 1), new(big.Rat).Mul(ratFloat(normal.X), new(big.Rat).Mul(jn, inverse))))
		postIdeal[i][1] = new(big.Rat).Add(exactBase(pre[i].Y), new(big.Rat).Mul(
			big.NewRat(sign, 1), new(big.Rat).Mul(jt, inverse)))
		postIdeal[i][2] = new(big.Rat).Neg(new(big.Rat).Quo(
			new(big.Rat).Mul(ratFloat(normal.X), new(big.Rat).Mul(radius[i], jt)), moment[i]))
	}
	jnFloat, jtFloat := sphereRatFloat(jn), sphereRatFloat(jt)
	if !finite(jnFloat, jtFloat) || jnFloat <= 0 ||
		!rationalRoundedWithin(jn, jnFloat, w.step.ImpulseResidual) ||
		!rationalRoundedWithin(jt, jtFloat, w.step.ImpulseResidual) {
		return undecided(w, "sphere-pair friction impulse exceeds its rounding residual"), nil
	}
	post := kicked
	for i := range post.entries {
		vx, vy, spin := sphereRatFloat(postIdeal[i][0]), sphereRatFloat(postIdeal[i][1]),
			sphereRatFloat(postIdeal[i][2])
		if !finite(vx, vy, spin) ||
			!rationalRoundedWithin(postIdeal[i][0], vx, w.step.VelocityResidual) ||
			!rationalRoundedWithin(postIdeal[i][1], vy, w.step.VelocityResidual) ||
			!rationalRoundedWithin(postIdeal[i][2], spin, w.step.AngularVelocityResidual) {
			return undecided(w, "sphere-pair friction response exceeds its rounding residual"), nil
		}
		post.entries[i].LinearVelocity = spherePairQuantityVelocity(r3.Vec{X: vx, Y: vy})
		post.entries[i].AngularVelocity = QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(spin)}
	}
	normalAfter := new(big.Rat).Mul(ratFloat(normal.X),
		new(big.Rat).Sub(exactBase(post.entries[1].LinearVelocity.X),
			exactBase(post.entries[0].LinearVelocity.X)))
	normalResidual := absRat(new(big.Rat).Add(normalAfter, new(big.Rat).Mul(restitution, preNormal)))
	slipAfter := new(big.Rat).Sub(exactBase(post.entries[1].LinearVelocity.Y),
		exactBase(post.entries[0].LinearVelocity.Y))
	for i := range post.entries {
		term := new(big.Rat).Mul(ratFloat(normal.X),
			new(big.Rat).Mul(radius[i], exactBase(post.entries[i].AngularVelocity.Z)))
		slipAfter.Sub(slipAfter, term)
	}
	idealSlip := new(big.Rat).Add(slip, new(big.Rat).Mul(tangentInverse, jt))
	tangentResidual := absRat(new(big.Rat).Sub(slipAfter, idealSlip))
	if sliding {
		if slip.Sign()*slipAfter.Sign() <= 0 {
			return undecided(w, "sphere-pair friction sliding direction is unresolved"), nil
		}
		impulseResidual := absRat(new(big.Rat).Add(ratFloat(jtFloat),
			new(big.Rat).Mul(w.friction.lower, new(big.Rat).Mul(ratFloat(jnFloat),
				big.NewRat(int64(slip.Sign()), 1)))))
		if impulseResidual.Cmp(exactBase(w.step.ImpulseResidual)) > 0 {
			return undecided(w, "sphere-pair friction sliding impulse exceeds residual"), nil
		}
	} else {
		if absRat(slipAfter).Cmp(exactBase(w.step.VelocityResidual)) > 0 {
			return undecided(w, "sphere-pair friction sticking slip exceeds residual"), nil
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
		return undecided(w, "sphere-pair friction contact residual exceeds its limit"), nil
	}
	ideal, err := w.sweep(ctx, post, dt, decad.ContinueSeparatingTouch)
	if err != nil {
		return nil, err
	}
	if !spherePairContinuationWithin(ideal, decad.SweepDepartedClear, point, w.step) {
		return undecided(w, fmt.Sprintf("sphere-pair friction ideal departure returned %v", ideal.Outcome)), nil
	}
	poseA, poseB, err := ideal.CertifiedPosesAt(dt)
	if err != nil {
		// A missing replay proof is a supported Undecided outcome.
		//nolint:nilerr
		return undecided(w, "sphere-pair friction endpoint lacks replay proof"), nil
	}
	end := post
	end.entries[0].Pose, end.entries[1].Pose = poseA, poseB
	lastSample := ideal.Samples[len(ideal.Samples)-1]
	if lastSample.At.Fraction != units.Scalar(1) || lastSample.FloatContact == nil ||
		lastSample.FloatContact.Relation != decad.ContactSeparated ||
		lastSample.Ideal.Relation != decad.ContactSeparated ||
		lastSample.PoseA != poseA || lastSample.PoseB != poseB {
		return undecided(w, "sphere-pair friction rounded endpoint lacks separation proof"), nil
	}
	last, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		poseA, poseB, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if last.Relation != decad.ContactSeparated || last.Gap == nil ||
		last.Gap.Value.Base()-last.Gap.Bound.Base() <= 0 {
		return undecided(w, "sphere-pair friction endpoint is not separated"), nil
	}
	zero := units.KilogramMillimetersPerSecond(0)
	tangent := QuantityVec{X: zero, Y: units.KilogramMillimetersPerSecond(jtFloat), Z: zero}
	angularUpper := new(big.Rat)
	for i := range post.entries {
		for _, value := range []*big.Rat{postIdeal[i][2],
			exactBase(post.entries[i].AngularVelocity.Z)} {
			if magnitude := absRat(new(big.Rat).Set(value)); magnitude.Cmp(angularUpper) > 0 {
				angularUpper.Set(magnitude)
			}
		}
	}
	instant := first.Event.At
	event := ContactEvent{Kind: ContactImpact,
		Pair:    BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket: decad.SweepInterval{From: instant, To: instant}, Time: instant.Elapsed.Value,
		Manifold:      cloneManifold(*first.Event.Manifold),
		NormalImpulse: units.KilogramMillimetersPerSecond(jnFloat), TangentImpulse: tangent,
		PointImpulses: []ContactPointImpulse{{Normal: units.KilogramMillimetersPerSecond(jnFloat),
			Tangent: tangent}},
		Solver: &ContactSolverReport{NormalResidual: units.MillimetersPerSecond(outwardRatFloat(normalResidual)),
			TangentResidual:     units.MillimetersPerSecond(outwardRatFloat(tangentResidual)),
			ConeResidual:        units.KilogramMillimetersPerSecond(outwardRatFloat(coneResidual)),
			PenetrationResidual: units.Millimeters(outwardSum(math.Abs(separation), bound)),
			AngularUpper:        units.RadiansPerSecond(outwardRatFloat(angularUpper)), Iterations: 1},
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
