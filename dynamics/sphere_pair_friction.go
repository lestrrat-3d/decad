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

// The zero-friction route uses this narrow gate so unsupported mass and
// source geometry continue through the existing frictionless response.
func (w *World) exactSpherePairFrictionCandidate(kicked State,
	manifold *decad.ContactManifold) bool {
	if !isSourceSpherePairEvent(manifold) || w.bodies[0].definition.Role != Dynamic ||
		w.bodies[1].definition.Role != Dynamic {
		return false
	}
	point := manifold.Points[0]
	if math.Abs(point.Normal.Value.X) != 1 || point.Normal.Value.Y != 0 ||
		point.Normal.Value.Z != 0 || point.Normal.Bound.Base() != 0 ||
		point.NormalAngle.Base() != 0 || point.OnA.Bound.Base() != 0 ||
		point.OnB.Bound.Base() != 0 || point.OnA.Value != point.OnB.Value {
		return false
	}
	for i, face := range [2]*decad.Face{point.FaceA, point.FaceB} {
		if w.bodies[i].definition.Supplied == nil ||
			kicked.entries[i].Pose.Basis() != r3.Identity().Basis() ||
			!zeroAngularVelocity(kicked.entries[i].AngularVelocity) ||
			kicked.entries[i].LinearVelocity.Z.Base() != 0 {
			return false
		}
		sphere, ok := face.Surface().(decad.Sphere)
		if !ok || sphere.Center != (r3.Vec{}) {
			return false
		}
		_, _, _, ok = exactSphereFloorMass(w.bodies[i].mass, sphere)
		if !ok {
			return false
		}
	}
	return true
}

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
		w.bodies[0].definition.Role != Dynamic || w.bodies[1].definition.Role != Dynamic ||
		w.step.MaxEvents <= 1 || w.pairs[0].restitution.Base() <= 0 ||
		w.pairs[0].friction.lower == nil || w.pairs[0].friction.upper == nil ||
		w.pairs[0].friction.lower.Sign() < 0 || w.pairs[0].friction.lower.Cmp(w.pairs[0].friction.upper) != 0 {
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
		if face == nil || w.bodies[i].definition.Supplied == nil ||
			kicked.entries[i].Pose.Basis() != r3.Identity().Basis() ||
			!zeroAngularVelocity(kicked.entries[i].AngularVelocity) ||
			kicked.entries[i].LinearVelocity.Z.Base() != 0 {
			return undecided(w, "sphere-pair friction needs translating planar source spheres"), nil
		}
		sphere, source := face.Surface().(decad.Sphere)
		if !source || sphere.Center != (r3.Vec{}) {
			return undecided(w, "sphere-pair friction needs a centered source sphere"), nil
		}
		mass[i], moment[i], radius[i], ok = exactSphereFloorMass(w.bodies[i].mass, sphere)
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
	restitution := exactBase(w.pairs[0].restitution)
	jn := new(big.Rat).Neg(new(big.Rat).Quo(
		new(big.Rat).Mul(new(big.Rat).Add(big.NewRat(1, 1), restitution), preNormal), inverseSum))
	slip := new(big.Rat).Sub(exactBase(pre[1].Y), exactBase(pre[0].Y))
	tangentInverse := new(big.Rat).Add(inverseSum,
		new(big.Rat).Add(new(big.Rat).Quo(new(big.Rat).Mul(radius[0], radius[0]), moment[0]),
			new(big.Rat).Quo(new(big.Rat).Mul(radius[1], radius[1]), moment[1])))
	jt := new(big.Rat).Neg(new(big.Rat).Quo(slip, tangentInverse))
	cone := new(big.Rat).Mul(w.pairs[0].friction.lower, jn)
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
	post := kicked.clone()
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
			new(big.Rat).Mul(w.pairs[0].friction.lower, new(big.Rat).Mul(ratFloat(jnFloat),
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
		new(big.Rat).Mul(w.pairs[0].friction.lower, ratFloat(jnFloat)))
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
	end := post.clone()
	end.entries[0].Pose, end.entries[1].Pose = poseA, poseB
	lastSample := ideal.Samples[len(ideal.Samples)-1]
	if lastSample.At.Fraction != units.Scalar(1) || lastSample.FloatContact == nil ||
		lastSample.FloatContact.Relation != decad.ContactSeparated ||
		lastSample.Ideal.Relation != decad.ContactSeparated ||
		lastSample.PoseA != poseA || lastSample.PoseB != poseB {
		return undecided(w, "sphere-pair friction rounded endpoint lacks separation proof"), nil
	}
	last, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body, w.bodies[1].definition.Body,
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
		Pair:    BodyPair{w.bodies[0].definition.Body, w.bodies[1].definition.Body},
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

// stepInteriorSpherePairFriction admits a bracket containing an exactly
// representable source-sphere touch. The new contact and prefix sweeps certify
// that point before the initial-touch Coulomb response runs on the remainder.
func (w *World) stepInteriorSpherePairFriction(ctx context.Context, from, kicked State,
	dt units.Value, first *decad.SweepReport) (*StepReport, error) {
	if first == nil || first.Outcome != decad.SweepImpactBracket || first.Bracket == nil ||
		first.Event == nil || first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 1 ||
		!first.HasAffineReplayProof() || w.pairs[0].restitution.Base() <= 0 ||
		!zeroAngularVelocity(kicked.entries[0].AngularVelocity) ||
		!zeroAngularVelocity(kicked.entries[1].AngularVelocity) {
		return undecided(w, "sphere-pair friction lacks a bounded interior impact"), nil
	}
	fraction, ok := firstInteriorDyadic(*first.Bracket)
	if !ok {
		return undecided(w, "sphere-pair friction bracket has no representable contact candidate"), nil
	}
	eventAt := units.Seconds(dt.Base() * fraction.Base())
	remaining := units.Seconds(dt.Base() - eventAt.Base())
	if !finite(eventAt.Base(), remaining.Base()) || eventAt.Base() <= 0 ||
		remaining.Base() <= 0 ||
		exactBase(eventAt).Cmp(new(big.Rat).Mul(exactBase(dt), exactBase(fraction))) != 0 ||
		exactBase(remaining).Cmp(new(big.Rat).Sub(exactBase(dt), exactBase(eventAt))) != 0 {
		return undecided(w, "sphere-pair friction impact time cannot be replayed exactly"), nil
	}
	contactState, err := driftState(kicked, eventAt.Base())
	if err != nil {
		return undecidedArithmetic(w, "sphere-pair friction impact pose is not finite", err)
	}
	prefix, err := w.sweepPoses(ctx, kicked, contactState, eventAt, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if !prefix.HasAffineReplayProof() ||
		!roundedImpactPrefixAtEnd(prefix, first, w.step.PenetrationResidual) {
		return undecided(w, "sphere-pair friction impact prefix lacks a matching source point"), nil
	}
	contact, err := w.doc.ContactPair(ctx, w.bodies[0].definition.Body,
		w.bodies[1].definition.Body, contactState.entries[0].Pose,
		contactState.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if contact.Relation != decad.ContactTouching || contact.Manifold == nil ||
		len(contact.Manifold.Points) != 1 {
		return undecided(w, "sphere-pair friction candidate is not a certified point touch"), nil
	}
	initial, err := w.sweep(ctx, contactState, remaining, decad.StopAtInitialContact)
	if err != nil {
		return nil, err
	}
	if initial.Outcome != decad.SweepInitiallyTouching || initial.Event == nil ||
		initial.Event.Manifold == nil || len(initial.Event.Manifold.Points) != 1 ||
		initial.Event.Manifold.Points[0].FaceA != contact.Manifold.Points[0].FaceA ||
		initial.Event.Manifold.Points[0].FaceB != contact.Manifold.Points[0].FaceB {
		return undecided(w, "sphere-pair friction impact changes its source point"), nil
	}
	var report *StepReport
	if isObliqueSpherePairEvent(initial.Event.Manifold) {
		report, err = w.stepInitialOffAxisSpherePairFriction(ctx,
			contactState, contactState, remaining, initial)
	} else {
		report, err = w.stepInitialSpherePairFriction(ctx, contactState, contactState, remaining, initial)
	}
	if err != nil || report == nil || report.Status != Advanced {
		return report, err
	}
	event := &report.Events[0]
	event.Bracket, event.Time = *first.Bracket, eventAt
	report.Trace = Trace{start: from, pre: contactState, post: report.Trace.post,
		end: *report.Next, duration: dt, eventAt: eventAt, hasEvent: true,
		preSweep: prefix, rotationalRemainder: report.Trace.rotationalRemainder}
	return report, nil
}

// The coarsest interior dyadic is a deterministic candidate. ContactPair and
// SweepPair must independently prove it is the true source-sphere touch.
func firstInteriorDyadic(bracket decad.SweepInterval) (units.Value, bool) {
	left, right := exactBase(bracket.From.Fraction), exactBase(bracket.To.Fraction)
	if left == nil || right == nil || left.Sign() < 0 || left.Cmp(right) >= 0 ||
		right.Cmp(big.NewRat(1, 1)) > 0 {
		return units.Value{}, false
	}
	for exponent := uint(1); exponent <= 53; exponent++ {
		denominator := new(big.Int).Lsh(big.NewInt(1), exponent)
		scaled := new(big.Rat).Mul(left, new(big.Rat).SetInt(denominator))
		numerator := new(big.Int).Quo(scaled.Num(), scaled.Denom())
		numerator.Add(numerator, big.NewInt(1))
		candidate := new(big.Rat).SetFrac(numerator, denominator)
		if candidate.Cmp(right) >= 0 || candidate.Cmp(big.NewRat(1, 1)) >= 0 {
			continue
		}
		value, _ := candidate.Float64()
		if finite(value) && ratFloat(value).Cmp(candidate) == 0 {
			return units.Scalar(value), true
		}
	}
	return units.Value{}, false
}
