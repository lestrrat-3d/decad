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

// stepInitialFriction admits a fixed-floor, dynamic-box patch only
// when both complete continuations certify the same touching face.
func (w *World) stepInitialFriction(ctx context.Context, from, kicked State, dt units.Value,
	first *decad.SweepReport) (*StepReport, error) {
	if w.parts[0].definition.Role == Dynamic && w.parts[1].definition.Role == Dynamic {
		return w.stepInitialTwoDynamicFriction(ctx, from, kicked, dt, first)
	}
	dynamic := 1
	if w.parts[0].definition.Role == Dynamic {
		dynamic = 0
	}
	if first.Event == nil || first.Event.Relation != decad.ContactTouching ||
		first.Event.Manifold == nil || len(first.Event.Manifold.Points) != 4 ||
		kicked.entries[1-dynamic].Pose != r3.Identity() ||
		kicked.entries[dynamic].Pose.Basis() != r3.Identity().Basis() ||
		!zeroAngularVelocity(kicked.entries[dynamic].AngularVelocity) ||
		kicked.entries[dynamic].LinearVelocity.Y.Mag() != 0 ||
		kicked.entries[dynamic].LinearVelocity.Z.Base() >= -w.step.VelocityResidual.Base() {
		return undecided(w, "frictional contact is outside the fixed-floor patch"), nil
	}
	normal, separation, bound, ok := reducedContact(first.Event.Manifold, w.step.Contact)
	penetration := outwardSum(math.Abs(separation), bound)
	expectedNormal := r3.Vec{Z: 1}
	if dynamic == 0 {
		expectedNormal.Z = -1
	}
	patch := floorToBoxManifold(first.Event.Manifold, dynamic)
	if !ok || normal != expectedNormal || !finite(separation, bound) ||
		!finite(penetration) || penetration > w.step.PenetrationResidual.Base() ||
		!w.fixedFloorPatchWitnesses(&patch) {
		return undecided(w, "frictional initial manifold exceeds its bounds"), nil
	}
	if w.step.MaxEvents <= 1 {
		return undecided(w, "frictional contact reaches the event limit with time remaining"), nil
	}
	pre := kicked.entries[dynamic].LinearVelocity
	if pre.X.Mag() == 0 {
		return w.stepFrictionStaticSupport(ctx, from, kicked, dt, first, &patch, dynamic)
	}
	if pre.X.Base() <= 0 {
		return undecided(w, "frictional contact has no admitted positive X slip"), nil
	}
	maximumLever, ok := w.frictionWholeBodyLeverWithin(&patch, dynamic, kicked.entries[dynamic].Pose)
	if !ok {
		return undecided(w, "frictional track can move the patch beyond audited corners"), nil
	}
	response, ok := solveFixedFloorFrictionPatch(&patch, w.parts[dynamic].mass,
		kicked.entries[dynamic].Pose, pre, w.friction, w.step)
	if !ok {
		return undecided(w, "frictional patch residuals exceed their limits"), nil
	}
	spinTravel := new(big.Rat).Mul(exactBase(response.AngularUpper), exactBase(dt))
	spinTravel.Mul(spinTravel, maximumLever)
	if spinTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 {
		return undecided(w, "omitted spin can move the frictional patch beyond penetration residual"), nil
	}
	normalImpulse, tangentImpulse, points, ok := aggregatePatchImpulses(response.Points, w.step.ImpulseResidual)
	if !ok {
		return undecided(w, "frictional aggregate impulse cannot be published"), nil
	}
	if dynamic == 0 {
		tangentImpulse, points = reversePatchTangent(tangentImpulse, points)
	}
	post := kicked
	post.entries[dynamic].LinearVelocity = response.Post
	ideal, err := w.sweep(ctx, post, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(ideal, normal) {
		return undecided(w, fmt.Sprintf("frictional ideal continuation returned %v", ideal.Outcome)), nil
	}
	end, err := driftState(post, dt.Base())
	if err != nil {
		return undecidedArithmetic(w, "non-finite frictional drift", err)
	}
	rounded, err := w.sweepPoses(ctx, post, end, dt, decad.ContinueCertifiedTouch)
	if err != nil {
		return nil, err
	}
	if !w.persistentTrackWithin(rounded, normal) {
		return undecided(w, fmt.Sprintf("frictional rounded continuation returned %v", rounded.Outcome)), nil
	}
	endpoint, err := w.doc.ContactPair(ctx, w.parts[0].definition.Body, w.parts[1].definition.Body,
		end.entries[0].Pose, end.entries[1].Pose, w.step.Contact)
	if err != nil {
		return nil, err
	}
	if endpoint.Relation != decad.ContactTouching || endpoint.Manifold == nil {
		return undecided(w, "frictional endpoint lacks certified touch"), nil
	}
	finalNormal, finalSeparation, finalBound, valid := reducedContact(endpoint.Manifold, w.step.Contact)
	if !valid || finalNormal != normal || !finite(finalSeparation, finalBound) ||
		outwardSum(math.Abs(finalSeparation), finalBound) > w.step.PenetrationResidual.Base() {
		return undecided(w, "frictional endpoint exceeds penetration residual"), nil
	}
	penetration = math.Max(penetration, outwardSum(math.Abs(finalSeparation), finalBound))
	instant := first.Event.At
	manifold := cloneManifold(*first.Event.Manifold)
	report := &StepReport{Status: Advanced, Next: &end}
	report.Events = []ContactEvent{{
		Kind:           ContactImpact,
		Pair:           BodyPair{w.parts[0].definition.Body, w.parts[1].definition.Body},
		Bracket:        decad.SweepInterval{From: instant, To: instant},
		Time:           instant.Elapsed.Value,
		Manifold:       manifold,
		NormalImpulse:  normalImpulse,
		TangentImpulse: tangentImpulse,
		PointImpulses:  points,
		Solver: &ContactSolverReport{NormalResidual: response.NormalResidual,
			TangentResidual: response.TangentResidual, ConeResidual: response.ConeResidual,
			PenetrationResidual: units.Millimeters(penetration),
			AngularUpper:        response.AngularUpper, Iterations: response.Iterations},
		PreVelocity:   pre,
		PostVelocity:  response.Post,
		PreVelocityA:  kicked.entries[0].LinearVelocity,
		PreVelocityB:  kicked.entries[1].LinearVelocity,
		PostVelocityA: post.entries[0].LinearVelocity,
		PostVelocityB: post.entries[1].LinearVelocity,
	}}
	report.Trace = Trace{start: from, pre: kicked, post: post, end: end, duration: dt,
		eventAt: instant.Elapsed.Value, hasEvent: true}
	return report, nil
}

// The joint solver uses a floor-to-box +Z normal. Keep the producer's point
// order and bounds, changing only which original body each witness names.
func floorToBoxManifold(manifold *decad.ContactManifold, dynamic int) decad.ContactManifold {
	patch := cloneManifold(*manifold)
	if dynamic == 1 {
		return patch
	}
	for i := range patch.Points {
		point := &patch.Points[i]
		point.OnA, point.OnB = point.OnB, point.OnA
		point.FaceA, point.FaceB = point.FaceB, point.FaceA
		point.FeatureA, point.FeatureB = point.FeatureB, point.FeatureA
		point.Normal.Value = point.Normal.Value.Scale(-1)
	}
	return patch
}

// The solver's tangent is the impulse on the box. World-order B is the floor
// when the box is A, so its event tangent has the opposite sign.
func reversePatchTangent(tangent QuantityVec, points []ContactPointImpulse) (QuantityVec, []ContactPointImpulse) {
	tangent.X = units.KilogramMillimetersPerSecond(-tangent.X.Base())
	tangent.Y = units.KilogramMillimetersPerSecond(-tangent.Y.Base())
	for i := range points {
		points[i].Tangent.X = units.KilogramMillimetersPerSecond(-points[i].Tangent.X.Base())
		points[i].Tangent.Y = units.KilogramMillimetersPerSecond(-points[i].Tangent.Y.Base())
	}
	return tangent, points
}

func (w *World) fixedFloorPatchWitnesses(manifold *decad.ContactManifold) bool {
	for _, point := range manifold.Points {
		if point.Normal.Value != (r3.Vec{Z: 1}) || point.Normal.Bound.Mag() != 0 ||
			point.NormalAngle.Mag() != 0 || point.OnA.Value != point.OnB.Value ||
			!finite(point.OnA.Value.X, point.OnA.Value.Y, point.OnA.Value.Z,
				point.OnB.Bound.Base(), point.OnA.Bound.Base(), point.Separation.Value.Base(),
				point.Separation.Bound.Base()) ||
			point.OnA.Bound.Base() > w.step.Contact.PointResolution.Base() ||
			point.OnB.Bound.Base() > w.step.Contact.PointResolution.Base() {
			return false
		}
	}
	return true
}

// frictionWholeBodyLeverWithin ensures any later box point has no greater
// omitted-spin speed error than the initial points audited by the solver.
func (w *World) frictionWholeBodyLeverWithin(manifold *decad.ContactManifold,
	dynamic int, pose r3.Transform) (*big.Rat, bool) {
	mass := w.parts[dynamic].mass
	box, err := w.parts[dynamic].definition.Body.Bounds()
	if err != nil || box.Bound.Kind() != units.Length || !finite(box.Bound.Base(),
		box.Min.X, box.Min.Y, box.Min.Z, box.Max.X, box.Max.Y, box.Max.Z) ||
		box.Bound.Base() < 0 {
		return nil, false
	}
	center := [3]float64{mass.Center.Value.X, mass.Center.Value.Y, mass.Center.Value.Z}
	minimum := [3]float64{box.Min.X, box.Min.Y, box.Min.Z}
	maximum := [3]float64{box.Max.X, box.Max.Y, box.Max.Z}
	full := new(big.Rat)
	for axis := range 3 {
		if minimum[axis] > maximum[axis] {
			return nil, false
		}
		low := absRat(new(big.Rat).Sub(ratFloat(minimum[axis]), ratFloat(center[axis])))
		high := absRat(new(big.Rat).Sub(ratFloat(maximum[axis]), ratFloat(center[axis])))
		if high.Cmp(low) > 0 {
			low = high
		}
		full.Add(full, low)
	}
	uncertainty := new(big.Rat).Add(exactBase(box.Bound), exactBase(mass.Center.Bound))
	full.Add(full, uncertainty.Mul(uncertainty, big.NewRat(3, 1)))
	initialMaximum := new(big.Rat)
	for _, point := range manifold.Points {
		lever := exactTranslatedPatchLever(point.OnB.Value, mass.Center.Value, pose.Translation())
		norm := new(big.Rat)
		for axis := range 3 {
			norm.Add(norm, absRat(lever[axis]))
		}
		pointUncertainty := new(big.Rat).Add(exactBase(point.OnB.Bound), exactBase(mass.Center.Bound))
		norm.Add(norm, pointUncertainty.Mul(pointUncertainty, big.NewRat(3, 1)))
		if norm.Cmp(initialMaximum) > 0 {
			initialMaximum = norm
		}
	}
	return full, full.Cmp(initialMaximum) <= 0
}

func (w *World) stepFrictionStaticSupport(ctx context.Context, from, kicked State, dt units.Value,
	first *decad.SweepReport, patch *decad.ContactManifold, dynamic int) (*StepReport, error) {
	mass := w.parts[dynamic].mass
	pose := kicked.entries[dynamic].Pose
	report, err := w.stepInitialTouch(ctx, from, kicked, dt, first)
	if err != nil || report.Status != Advanced || len(report.Events) != 1 {
		return report, err
	}
	event := &report.Events[0]
	perPoint := event.NormalImpulse.Base() / float64(len(first.Event.Manifold.Points))
	if !finite(perPoint) || perPoint <= 0 {
		return undecided(w, "static support impulse cannot be divided among corners"), nil
	}
	impulses := make([]frictionPointImpulse, len(first.Event.Manifold.Points))
	zero := units.KilogramMillimetersPerSecond(0)
	for i := range impulses {
		impulses[i] = frictionPointImpulse{Normal: units.KilogramMillimetersPerSecond(perPoint),
			TangentX: zero, TangentY: zero}
	}
	normal, tangent, points, ok := aggregatePatchImpulses(impulses, w.step.ImpulseResidual)
	if !ok || intervalDeviation(ratFloat(normal.Base()), exactBase(event.NormalImpulse),
		exactBase(event.NormalImpulse)).Cmp(exactBase(w.step.ImpulseResidual)) > 0 {
		return undecided(w, "static support corners exceed impulse residual"), nil
	}
	angularUpper, ok := staticSupportAngularUpper(patch, mass, pose, normal, w.step.ImpulseResidual)
	if !ok || angularUpper.Cmp(exactBase(w.step.AngularVelocityResidual)) > 0 {
		return undecided(w, "static support corner torque exceeds angular residual"), nil
	}
	maximumLever, ok := w.frictionWholeBodyLeverWithin(patch, dynamic, pose)
	if !ok {
		return undecided(w, "static support can move the patch beyond audited corners"), nil
	}
	spinTravel := new(big.Rat).Mul(angularUpper, exactBase(dt))
	spinTravel.Mul(spinTravel, maximumLever)
	if spinTravel.Cmp(exactBase(w.step.PenetrationResidual)) > 0 {
		return undecided(w, "static support omitted spin exceeds penetration residual"), nil
	}
	preSpeed := [2]units.Value{kicked.entries[0].LinearVelocity.Z, kicked.entries[1].LinearVelocity.Z}
	postSpeed := [2]float64{event.PostVelocityA.Z.Base(), event.PostVelocityB.Z.Base()}
	normalSign := float64(1)
	if dynamic == 0 {
		normalSign = -1
	}
	if !responsePairResidualsWithin(preSpeed, normalSign, units.Scalar(0), w.parts,
		0, normal.Base(), postSpeed, w.step.VelocityResidual, w.step.ImpulseResidual) {
		return undecided(w, "static support rounded corner sum exceeds response residual"), nil
	}
	event.NormalImpulse, event.TangentImpulse, event.PointImpulses = normal, tangent, points
	return report, nil
}

// staticSupportAngularUpper bounds the zero-spin error from equal normal
// corner impulses. Each witness and the center may move anywhere within its
// published spatial bound, so both horizontal torque components are widened.
func staticSupportAngularUpper(manifold *decad.ContactManifold, mass decad.MassProperties,
	pose r3.Transform, normal, impulseResidual units.Value) (*big.Rat, bool) {
	if manifold == nil || len(manifold.Points) != 4 ||
		normal.Kind() != units.Impulse || impulseResidual.Kind() != units.Impulse ||
		!finite(normal.Base(), impulseResidual.Base()) || normal.Base() <= 0 || impulseResidual.Base() < 0 {
		return nil, false
	}
	lower := certifiedInertiaLower(mass)
	centerBound := exactBase(mass.Center.Bound)
	if lower == nil || lower.Sign() <= 0 || centerBound == nil || centerBound.Sign() < 0 {
		return nil, false
	}
	var xTotal, yTotal big.Rat
	pointBound := new(big.Rat)
	for _, point := range manifold.Points {
		bound := exactBase(point.OnB.Bound)
		if bound == nil || bound.Sign() < 0 || !finite(point.OnB.Value.X, point.OnB.Value.Y) {
			return nil, false
		}
		lever := exactTranslatedPatchLever(point.OnB.Value, mass.Center.Value, pose.Translation())
		xTotal.Add(&xTotal, lever[0])
		yTotal.Add(&yTotal, lever[1])
		pointBound.Add(pointBound, bound)
	}
	leverUpper := new(big.Rat).Add(absRat(&xTotal), absRat(&yTotal))
	pointBound.Add(pointBound, new(big.Rat).Mul(centerBound, big.NewRat(4, 1)))
	leverUpper.Add(leverUpper, new(big.Rat).Mul(pointBound, big.NewRat(2, 1)))
	impulseUpper := new(big.Rat).Add(exactBase(normal), exactBase(impulseResidual))
	angularUpper := new(big.Rat).Mul(impulseUpper, leverUpper)
	angularUpper.Quo(angularUpper, big.NewRat(4, 1))
	angularUpper.Quo(angularUpper, lower)
	return angularUpper, true
}

func zeroAngularVelocity(v QuantityVec) bool {
	return v.X.Mag() == 0 && v.Y.Mag() == 0 && v.Z.Mag() == 0
}

func aggregatePatchImpulses(impulses []frictionPointImpulse,
	limit units.Value) (units.Value, QuantityVec, []ContactPointImpulse, bool) {
	if len(impulses) != 4 {
		return units.Value{}, QuantityVec{}, nil, false
	}
	var sums [3]*big.Rat
	for axis := range 3 {
		sums[axis] = new(big.Rat)
	}
	points := make([]ContactPointImpulse, len(impulses))
	for i, impulse := range impulses {
		values := [3]units.Value{impulse.TangentX, impulse.TangentY, impulse.Normal}
		for axis, value := range values {
			if value.Kind() != units.Impulse || !finite(value.Base()) {
				return units.Value{}, QuantityVec{}, nil, false
			}
			sums[axis].Add(sums[axis], exactBase(value))
		}
		points[i] = ContactPointImpulse{Normal: impulse.Normal,
			Tangent: QuantityVec{X: impulse.TangentX, Y: impulse.TangentY,
				Z: units.KilogramMillimetersPerSecond(0)}}
	}
	var published [3]float64
	for axis, exact := range sums {
		published[axis], _ = exact.Float64()
		if !finite(published[axis]) ||
			intervalDeviation(ratFloat(published[axis]), exact, exact).Cmp(exactBase(limit)) > 0 {
			return units.Value{}, QuantityVec{}, nil, false
		}
	}
	return units.KilogramMillimetersPerSecond(published[2]), QuantityVec{
		X: units.KilogramMillimetersPerSecond(published[0]),
		Y: units.KilogramMillimetersPerSecond(published[1]),
		Z: units.KilogramMillimetersPerSecond(0)}, points, true
}
