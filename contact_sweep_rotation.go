package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type rotationalSweepPath struct {
	body       *Body
	path       affinePairPath
	fullTravel *big.Rat
	startBox   orientedSourceBox
	sourceBox  orientedSourceBox
	frame      motionFrame
	fromRot    ivMat
	fromT      ratVec
	velocity   ratVec
	omegaLow   *big.Rat
	omegaHigh  *big.Rat
}

func prepareRotationalSweepPath(body *Body, path affinePairPath) (rotationalSweepPath, bool) {
	if path.screw != nil {
		axis := path.screw.Axis
		angle, ok := exactBaseValue(path.screw.Angle)
		if !ok || path.duration.Sign() <= 0 {
			return rotationalSweepPath{}, false
		}
		angular := new(big.Rat).Quo(angle, path.duration)
		linear := new(big.Rat).Quo(floatRat(path.screw.Slide), path.duration)
		omega, speed := ratFloatNearest(angular), ratFloatNearest(linear)
		if !finiteMeasurementValues(omega, speed) {
			return rotationalSweepPath{}, false
		}
		path.drift = &RigidDriftSegment{From: path.from, Center: path.screw.Point,
			LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(axis.X * speed),
				Y: units.MillimetersPerSecond(axis.Y * speed),
				Z: units.MillimetersPerSecond(axis.Z * speed)},
			AngularVelocity: QuantityVec{X: units.RadiansPerSecond(axis.X * omega),
				Y: units.RadiansPerSecond(axis.Y * omega),
				Z: units.RadiansPerSecond(axis.Z * omega)},
			Duration: units.Seconds(ratFloatNearest(path.duration))}
	}
	startBox, ok := sourceOrientedBoxAtPose(body, path.from)
	if !ok {
		return rotationalSweepPath{}, false
	}
	sourceBox, ok := sourceOrientedBoxAtPose(body, r3.Identity())
	if !ok {
		return rotationalSweepPath{}, false
	}
	fromRot, fromT, ok := exactTransform(path.from)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared := rotationalSweepPath{body: body, path: path, fromRot: fromRot, fromT: fromT,
		startBox: startBox, sourceBox: sourceBox}
	if path.drift == nil {
		travelSquared := new(big.Rat)
		for _, component := range path.delta {
			value := component.rat()
			travelSquared.Add(travelSquared, new(big.Rat).Mul(value, value))
		}
		bound := ratSqrtUp(travelSquared)
		if !finiteMeasurementValues(bound) {
			return rotationalSweepPath{}, false
		}
		prepared.fullTravel = floatRat(bound)
		return prepared, true
	}
	drift := path.drift
	velocity := [3]units.Value{drift.LinearVelocity.X, drift.LinearVelocity.Y, drift.LinearVelocity.Z}
	angular := [3]units.Value{drift.AngularVelocity.X, drift.AngularVelocity.Y, drift.AngularVelocity.Z}
	prepared.velocity = ratVec{}
	omega := ratVec{}
	vSquared, omegaSquared := new(big.Rat), new(big.Rat)
	for axis := range 3 {
		prepared.velocity[axis], _ = exactBaseValue(velocity[axis])
		omega[axis], _ = exactBaseValue(angular[axis])
		vSquared.Add(vSquared, new(big.Rat).Mul(prepared.velocity[axis], prepared.velocity[axis]))
		omegaSquared.Add(omegaSquared, new(big.Rat).Mul(omega[axis], omega[axis]))
	}
	if omegaSquared.Sign() <= 0 {
		return rotationalSweepPath{}, false
	}
	prepared.omegaLow = floatRat(ratSqrtDown(omegaSquared))
	prepared.omegaHigh = floatRat(ratSqrtUp(omegaSquared))
	if prepared.omegaLow == nil || prepared.omegaHigh == nil || prepared.omegaHigh.Sign() <= 0 {
		return rotationalSweepPath{}, false
	}
	center, ok := ratVecOf(drift.Center)
	if !ok {
		return rotationalSweepPath{}, false
	}
	unit, ok := unitScaleInterval(omega)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.frame = motionFrame{kind: motionRevolute, axis: omega, unit: unit, center: center}
	radius, ok := rotationalSweepRadius(body, path.from, drift.Center, omega)
	if !ok {
		return rotationalSweepPath{}, false
	}
	vUp := ratSqrtUp(vSquared)
	if !finiteMeasurementValues(vUp) {
		return rotationalSweepPath{}, false
	}
	speed := new(big.Rat).Add(floatRat(vUp), new(big.Rat).Mul(radius, prepared.omegaHigh))
	prepared.fullTravel = new(big.Rat).Mul(speed, path.duration)
	if path.screw != nil {
		angle, _ := exactBaseValue(path.screw.Angle)
		angular := new(big.Rat).Quo(angle, path.duration)
		linear := new(big.Rat).Quo(floatRat(path.screw.Slide), path.duration)
		for axis, component := range [3]float64{path.screw.Axis.X, path.screw.Axis.Y, path.screw.Axis.Z} {
			prepared.velocity[axis] = new(big.Rat).Mul(floatRat(component), linear)
		}
		prepared.omegaLow, prepared.omegaHigh = angular, angular
		prepared.fullTravel = new(big.Rat).Mul(new(big.Rat).Add(new(big.Rat).Abs(linear),
			new(big.Rat).Mul(radius, angular)), path.duration)
	}
	return prepared, true
}

func rotationalSweepRadius(body *Body, from r3.Transform, center r3.Vec,
	axis ratVec) (*big.Rat, bool) {
	box, err := body.Bounds()
	if err != nil || box.Bound.Kind() != units.Length ||
		!finiteMeasurementValues(box.Bound.Base(), box.Min.X, box.Min.Y, box.Min.Z,
			box.Max.X, box.Max.Y, box.Max.Z) || box.Bound.Base() < 0 {
		return nil, false
	}
	minimum := [3]float64{box.Min.X, box.Min.Y, box.Min.Z}
	maximum := [3]float64{box.Max.X, box.Max.Y, box.Max.Z}
	var extremes [3][2]dyadic
	bound := mustDyOf(box.Bound.Base())
	for axis := range 3 {
		if minimum[axis] > maximum[axis] {
			return nil, false
		}
		extremes[axis] = [2]dyadic{dySubScalar(mustDyOf(minimum[axis]), bound),
			dyAdd(mustDyOf(maximum[axis]), bound)}
	}
	pivot := dyVec(center)
	axisSquared := new(big.Rat)
	for k := range 3 {
		axisSquared.Add(axisSquared, new(big.Rat).Mul(axis[k], axis[k]))
	}
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	best := new(big.Rat)
	for index := range 8 {
		var corner dyV3
		for axis := range 3 {
			corner[axis] = extremes[axis][(index>>axis)&1]
		}
		mapped := exactContactTransform(from, corner)
		delta := dvSub(mapped, pivot)
		cross := [3]*big.Rat{
			new(big.Rat).Sub(new(big.Rat).Mul(delta[1].rat(), axis[2]),
				new(big.Rat).Mul(delta[2].rat(), axis[1])),
			new(big.Rat).Sub(new(big.Rat).Mul(delta[2].rat(), axis[0]),
				new(big.Rat).Mul(delta[0].rat(), axis[2])),
			new(big.Rat).Sub(new(big.Rat).Mul(delta[0].rat(), axis[1]),
				new(big.Rat).Mul(delta[1].rat(), axis[0])),
		}
		squared := new(big.Rat)
		for k := range 3 {
			squared.Add(squared, new(big.Rat).Mul(cross[k], cross[k]))
		}
		squared.Quo(squared, axisSquared)
		if squared.Cmp(best) > 0 {
			best = squared
		}
	}
	radius := ratSqrtUp(best)
	if !finiteMeasurementValues(radius) {
		return nil, false
	}
	return floatRat(radius), true
}

func (p rotationalSweepPath) poseAt(f *big.Rat) (r3.Transform, error) {
	if p.path.screw != nil {
		if f.Sign() == 0 {
			return p.path.from, nil
		}
		if f.Cmp(big.NewRat(1, 1)) == 0 {
			return p.path.to, nil
		}
		step, err := p.path.screw.At(ratFloatNearest(f))
		if err != nil {
			return r3.Transform{}, err
		}
		return p.path.from.Then(step)
	}
	if p.path.drift == nil {
		return p.path.poseAt(f)
	}
	if f.Sign() == 0 {
		return p.path.from, nil
	}
	drift := p.path.drift
	elapsed := ratFloatNearest(new(big.Rat).Mul(p.path.duration, f))
	axis := r3.Vec{X: drift.AngularVelocity.X.Base(), Y: drift.AngularVelocity.Y.Base(),
		Z: drift.AngularVelocity.Z.Base()}
	norm := math.Hypot(axis.X, math.Hypot(axis.Y, axis.Z))
	turn, err := r3.RotationAround(drift.Center, axis, units.Radians(norm*elapsed))
	if err != nil {
		return r3.Transform{}, err
	}
	pose, err := p.path.from.Then(turn)
	if err != nil {
		return r3.Transform{}, err
	}
	shift, err := r3.Translation(r3.Vec{X: drift.LinearVelocity.X.Base() * elapsed,
		Y: drift.LinearVelocity.Y.Base() * elapsed, Z: drift.LinearVelocity.Z.Base() * elapsed})
	if err != nil {
		return r3.Transform{}, err
	}
	return pose.Then(shift)
}

func (p rotationalSweepPath) idealAt(f *big.Rat) idealPose {
	zero := pointInterval(new(big.Rat))
	if p.path.drift == nil {
		shift := pointVec(p.fromT)
		for axis := range 3 {
			shift[axis] = intervalAdd(shift[axis],
				pointInterval(new(big.Rat).Mul(p.path.delta[axis].rat(), f)))
		}
		return idealPose{rot: p.fromRot, pivot: ivVec{zero, zero, zero}, shift: shift}
	}
	elapsed := new(big.Rat).Mul(p.path.duration, f)
	angleLow := new(big.Rat).Mul(p.omegaLow, elapsed)
	angleHigh := new(big.Rat).Mul(p.omegaHigh, elapsed)
	sin, cos := radianSinCos(angleLow)
	width := new(big.Rat).Sub(angleHigh, angleLow)
	sin = intervalOwned(new(big.Rat).Sub(sin.lo, width), new(big.Rat).Add(sin.hi, width))
	cos = intervalOwned(new(big.Rat).Sub(cos.lo, width), new(big.Rat).Add(cos.hi, width))
	rot := p.frame.rotation(sin, cos)
	center := pointVec(p.frame.center)
	shift := ivVecAdd(rot.apply(ivVecSub(pointVec(p.fromT), center)), center)
	for axis := range 3 {
		shift[axis] = intervalAdd(shift[axis],
			pointInterval(new(big.Rat).Mul(p.velocity[axis], elapsed)))
	}
	return idealPose{rot: rot.mul(p.fromRot), pivot: ivVec{zero, zero, zero}, shift: shift}
}

// roundedAt compares the exact staged source corners used by ContactPair with
// their ideal path positions. The distance between two affine images over a
// box is bounded by the largest corner distance, including placement and
// pose-composition rounding without relying on a rounded composed transform.
func (p rotationalSweepPath) roundedAt(pose r3.Transform, f *big.Rat) (orientedSourceBox, float64, bool) {
	if !pose.IsValid() {
		return orientedSourceBox{}, 0, false
	}
	ideal := p.idealAt(f)
	var box orientedSourceBox
	maxSquared := new(big.Rat)
	for i, corner := range p.sourceBox.corner {
		actual := exactContactTransform(pose, corner)
		box.corner[i] = actual
		point := pointVec(ratVec{corner[0].rat(), corner[1].rat(), corner[2].rat()})
		idealPoint := ivVecAdd(ivVecAdd(ideal.rot.apply(ivVecSub(point, ideal.pivot)), ideal.pivot), ideal.shift)
		observed := pointVec(ratVec{actual[0].rat(), actual[1].rat(), actual[2].rat()})
		difference := ivVecSub(observed, idealPoint)
		squared := magnitudeSquaredUpper(difference[:]...)
		if squared.Cmp(maxSquared) > 0 {
			maxSquared = squared
		}
	}
	box.faces = p.startBox.faces
	box.edge = [3]dyV3{dvSub(box.corner[1], box.corner[0]),
		dvSub(box.corner[2], box.corner[0]), dvSub(box.corner[4], box.corner[0])}
	for _, edge := range box.edge {
		if dvIsZero(edge) {
			return orientedSourceBox{}, 0, false
		}
	}
	bound := ratSqrtUp(maxSquared)
	return box, bound, finiteMeasurementValues(bound)
}

type rotationalPairSweep struct {
	doc        *Document
	a, b       rotationalSweepPath
	req        SweepRequest
	resolution *big.Rat
	report     *SweepReport
}

func (d *Document) sweepRotatingPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, error) {
	aPath, okA := prepareRotationalSweepPath(a, pa)
	bPath, okB := prepareRotationalSweepPath(b, pb)
	if !okA || !okB {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, nil
	}
	run := rotationalPairSweep{doc: d, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, err
	}
	if result.Outcome == SweepClear || result.Outcome == SweepPersistentTouch &&
		result.ContactTrack != nil && result.ContactTrack.orientedA != nil &&
		result.ContactTrack.orientedB != nil && aPath.path.drift == nil && bPath.path.drift == nil {
		result.replay = &sweepReplayProof{rotation: &[2]rotationalSweepPath{aPath, bPath},
			track: result.ContactTrack, request: req.ContactRequest}
		result.replay.snapshot(result)
	}
	return result, nil
}

func (r *rotationalPairSweep) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.a.poseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.b.poseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a.body, r.b.body, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	boxA, etaA, okA := r.a.roundedAt(poseA, f)
	boxB, etaB, okB := r.b.roundedAt(poseB, f)
	at := sweepInstant(f, r.a.path.duration)
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	if okA && okB {
		switch contact.Relation {
		case ContactSeparated:
			if contact.Gap != nil {
				value, _ := exactBaseValue(contact.Gap.Value)
				bound, _ := exactBaseValue(contact.Gap.Bound)
				charge := new(big.Rat).Add(floatRat(etaA), floatRat(etaB))
				newBound := new(big.Rat).Add(bound, charge)
				if new(big.Rat).Sub(value, newBound).Sign() > 0 {
					published := ratFloatUp(newBound)
					if finiteMeasurementValues(published) && value.Cmp(floatRat(published)) > 0 {
						gap := *contact.Gap
						gap.Bound = units.Millimeters(published)
						gap.Exactness = exactnessFromBound(published)
						event.Relation, event.Gap, event.Reason = ContactSeparated, &gap, ContactNoReason
					}
				}
			}
		case ContactOverlapping:
			if orientedInteriorWitness(boxA, boxB, floatRat(etaA), floatRat(etaB)) {
				event.Relation, event.Reason = ContactOverlapping, contact.Reason
			}
			if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				floatRat(etaA), floatRat(etaB)); ok {
				event = proof
			}
		case ContactTouching:
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = ContactTouching, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				floatRat(etaA), floatRat(etaB)); ok {
				event = proof
			}
		}
	}
	sample := &SweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: contact, Ideal: event, exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

// horizontalSpinContact uses the invariant support height of a Z-axis spin.
// The read face patch must remain strictly inside the stationary face after
// charging both pose deviations. Exact ideal support heights decide the event.
func (r *rotationalPairSweep) horizontalSpinContact(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, etaA, etaB *big.Rat) (SweepEvent, bool) {
	if contact.Manifold == nil || len(contact.Manifold.Points) != 4 {
		return SweepEvent{}, false
	}
	stationary, spinning := -1, -1
	paths := [2]rotationalSweepPath{r.a, r.b}
	for i, path := range paths {
		if path.path.drift == nil && path.path.delta == [3]dyadic{} {
			stationary = i
		}
		if path.path.drift != nil && path.frame.axis[0].Sign() == 0 &&
			path.frame.axis[1].Sign() == 0 && path.frame.axis[2].Sign() != 0 &&
			path.velocity[0].Sign() == 0 && path.velocity[1].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return SweepEvent{}, false
	}
	poses := [2]r3.Transform{poseA, poseB}
	base, okBase := sourceBoxAtPose(paths[stationary].body, poses[stationary])
	startA, okA := sourceBoxAtPose(r.a.body, r.a.path.from)
	startB, okB := sourceBoxAtPose(r.b.body, r.b.path.from)
	if !okBase || !okA || !okB {
		return SweepEvent{}, false
	}
	normal := contact.Manifold.Points[0].Normal.Value
	if normal != (r3.Vec{Z: 1}) && normal != (r3.Vec{Z: -1}) {
		return SweepEvent{}, false
	}
	elapsed := new(big.Rat).Mul(r.a.path.duration, f)
	travel := new(big.Rat).Mul(paths[spinning].velocity[2], elapsed)
	loA, hiA := startA.lo[2].rat(), startA.hi[2].rat()
	loB, hiB := startB.lo[2].rat(), startB.hi[2].rat()
	if spinning == 0 {
		loA.Add(loA, travel)
		hiA.Add(hiA, travel)
	} else {
		loB.Add(loB, travel)
		hiB.Add(hiB, travel)
	}
	gap := new(big.Rat)
	if normal.Z > 0 {
		gap.Sub(loB, hiA)
		if loA.Cmp(loB) >= 0 || hiA.Cmp(hiB) >= 0 {
			return SweepEvent{}, false
		}
	} else {
		gap.Sub(loA, hiB)
		if loB.Cmp(loA) >= 0 || hiB.Cmp(hiA) >= 0 {
			return SweepEvent{}, false
		}
	}
	if gap.Sign() > 0 {
		return SweepEvent{}, false
	}
	value := ratFloatNearest(gap)
	bound := rationalFloatError(gap, value)
	if !finiteMeasurementValues(value, bound) {
		return SweepEvent{}, false
	}
	deviation := new(big.Rat).Add(etaA, etaB)
	points := append([]ContactPoint(nil), contact.Manifold.Points...)
	for i := range points {
		point := &points[i]
		if point.Normal.Value != normal || point.Normal.Bound.Base() != 0 ||
			point.NormalAngle.Base() != 0 {
			return SweepEvent{}, false
		}
		witness := point.OnA
		if spinning == 1 {
			witness = point.OnB
		}
		margin := new(big.Rat).Add(deviation, floatRat(witness.Bound.Base()))
		margin.Sub(margin, gap)
		coordinates := [2]float64{witness.Value.X, witness.Value.Y}
		for axis := range 2 {
			value := floatRat(coordinates[axis])
			if value.Cmp(new(big.Rat).Add(base.lo[axis].rat(), margin)) <= 0 ||
				value.Cmp(new(big.Rat).Sub(base.hi[axis].rat(), margin)) >= 0 {
				return SweepEvent{}, false
			}
		}
		for _, position := range []*VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(floatRat(position.Bound.Base()), deviation)
			if bound.Cmp(floatRat(r.req.PointResolution.Base())) > 0 {
				return SweepEvent{}, false
			}
			published := ratFloatUp(bound)
			if !finiteMeasurementValues(published) || published > r.req.PointResolution.Base() {
				return SweepEvent{}, false
			}
			position.Bound = units.Millimeters(published)
			position.Exactness = exactnessFromBound(position.Bound.Base())
		}
		point.Separation = Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessFromBound(bound)}
	}
	zero := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
	event := SweepEvent{At: sweepInstant(f, r.a.path.duration), Manifold: &ContactManifold{Points: points}}
	if gap.Sign() == 0 {
		event.Relation, event.Gap = ContactTouching, &zero
	} else {
		event.Relation = ContactOverlapping
	}
	return event, true
}

func (r *rotationalPairSweep) execute(ctx context.Context) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case ContactOverlapping:
		r.report.Outcome, r.report.InitialEvent, r.report.Event =
			SweepInitiallyOverlapping, &first.Ideal, &first.Ideal
		return r.report, nil
	case ContactTouching:
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if r.req.StartPolicy == ContinueCertifiedTouch && r.coMovingOrientedTouch(first) {
			last, sampleErr := r.sample(ctx, one)
			if errors.Is(sampleErr, errSweepPoseBudget) {
				return r.undecided(zero, one, SweepPoseBudget), nil
			}
			if sampleErr != nil {
				return nil, sampleErr
			}
			if last.Ideal.Relation != ContactTouching || last.Ideal.Manifold == nil {
				return r.undecided(zero, one, SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack = &SweepContactTrack{orientedA: &r.a.startBox,
				orientedB: &r.b.startBox, orientedDelta: r.a.path.delta,
				start: zero, end: one,
				duration: r.a.path.duration, request: r.req.ContactRequest,
				features: [2]ContactFeature{first.Ideal.Manifold.Points[0].FeatureA,
					first.Ideal.Manifold.Points[0].FeatureB},
				normal: first.Ideal.Manifold.Points[0].Normal}
			r.report.Outcome = SweepPersistentTouch
			r.sortSamples()
			return r.report, nil
		}
		if departure, ok := r.rotationalDepartureFraction(first); ok {
			left, sampleErr := r.sample(ctx, departure)
			if errors.Is(sampleErr, errSweepPoseBudget) {
				return r.undecided(zero, departure, SweepPoseBudget), nil
			}
			if sampleErr != nil {
				return nil, sampleErr
			}
			if left.Ideal.Relation != ContactSeparated || left.Ideal.Gap == nil {
				return r.undecided(zero, departure, SweepDepartureUnproved), nil
			}
			r.report.Departure = &SweepDeparture{Until: left.At, GapAtUntil: *left.Ideal.Gap}
			last, sampleErr := r.sample(ctx, one)
			if errors.Is(sampleErr, errSweepPoseBudget) {
				return r.undecided(departure, one, SweepPoseBudget), nil
			}
			if sampleErr != nil {
				return nil, sampleErr
			}
			done, refineErr := r.refine(ctx, left, last, 0)
			if refineErr != nil {
				return nil, refineErr
			}
			if !done {
				r.report.Outcome = SweepDepartedClear
			}
			r.sortSamples()
			return r.report, nil
		}
		cause := SweepDepartureUnproved
		if r.req.StartPolicy == ContinueCertifiedTouch {
			cause = SweepContactTrackUnproved
		}
		return r.undecided(zero, one, cause), nil
	case ContactSeparated:
	default:
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, one, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	done, err := r.refine(ctx, first, last, 0)
	if err != nil {
		return nil, err
	}
	if !done {
		r.report.Outcome = SweepClear
	}
	r.sortSamples()
	return r.report, nil
}

func (r *rotationalPairSweep) coMovingOrientedTouch(first *SweepSample) bool {
	if r.a.path.drift != nil || r.b.path.drift != nil || first.Ideal.Manifold == nil {
		return false
	}
	for axis := range 3 {
		if dyCmp(r.a.path.delta[axis], r.b.path.delta[axis]) != 0 {
			return false
		}
	}
	return true
}

func translatedOrientedBox(box orientedSourceBox, delta [3]dyadic, fraction *big.Rat) (orientedSourceBox, bool) {
	for axis := range 3 {
		step, ok := dyOfRat(new(big.Rat).Mul(delta[axis].rat(), fraction))
		if !ok {
			return orientedSourceBox{}, false
		}
		for corner := range box.corner {
			box.corner[corner][axis] = dyAdd(box.corner[corner][axis], step)
		}
	}
	return box, true
}

// The admitted departure paths each prove an open-at-zero whole-body gap.
// For a common rotation, Taylor's theorem bounds the support-plane gap below
// by c*u - K*u²/2, using exact c and outward K over the whole step.
func (r *rotationalPairSweep) rotationalDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if fraction, ok := r.obliqueAffineDepartureFraction(first); ok {
		return fraction, true
	}
	if fraction, ok := r.horizontalSpinDepartureFraction(first); ok {
		return fraction, true
	}
	if fraction, ok := r.axisFaceDepartureFraction(first); ok {
		return fraction, true
	}
	if r.a.path.drift == nil || r.b.path.drift == nil || first.Ideal.Manifold == nil ||
		len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	point := first.Ideal.Manifold.Points[0]
	axis, side, ok := signedAxis(point.Normal.Value)
	if !ok || point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return nil, false
	}
	boxA, okA := sourceBoxAtPose(r.a.body, r.a.path.from)
	boxB, okB := sourceBoxAtPose(r.b.body, r.b.path.from)
	if !okA || !okB ||
		(side == 1 && dyCmp(boxA.hi[axis], boxB.lo[axis]) != 0) ||
		(side == 0 && dyCmp(boxB.hi[axis], boxA.lo[axis]) != 0) {
		return nil, false
	}
	var omega, difference, velocity [3]*big.Rat
	for i := range 3 {
		if r.a.frame.axis[i].Cmp(r.b.frame.axis[i]) != 0 {
			return nil, false
		}
		omega[i] = r.a.frame.axis[i]
		centers := [2]float64{r.a.path.drift.Center.X, r.b.path.drift.Center.X}
		switch i {
		case 1:
			centers = [2]float64{r.a.path.drift.Center.Y, r.b.path.drift.Center.Y}
		case 2:
			centers = [2]float64{r.a.path.drift.Center.Z, r.b.path.drift.Center.Z}
		}
		difference[i] = new(big.Rat).Sub(floatRat(centers[1]), floatRat(centers[0]))
		velocity[i] = new(big.Rat).Sub(r.b.velocity[i], r.a.velocity[i])
	}
	sign := int64(1)
	if side == 0 {
		sign = -1
	}
	derivative := new(big.Rat).Mul(velocity[axis], big.NewRat(sign, 1))
	normal := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	normal[axis] = big.NewRat(sign, 1)
	cross := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(omega[1], normal[2]),
			new(big.Rat).Mul(omega[2], normal[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(omega[2], normal[0]),
			new(big.Rat).Mul(omega[0], normal[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(omega[0], normal[1]),
			new(big.Rat).Mul(omega[1], normal[0])),
	}
	for i := range 3 {
		derivative.Add(derivative, new(big.Rat).Mul(cross[i], difference[i]))
	}
	if derivative.Sign() <= 0 {
		return nil, false
	}
	normUpper := func(vector [3]*big.Rat) *big.Rat {
		squared := new(big.Rat)
		for i := range 3 {
			squared.Add(squared, new(big.Rat).Mul(vector[i], vector[i]))
		}
		root := ratSqrtUp(squared)
		if !finiteMeasurementValues(root) {
			return nil
		}
		return floatRat(root)
	}
	dNorm, vNorm := normUpper(difference), normUpper(velocity)
	if dNorm == nil || vNorm == nil {
		return nil, false
	}
	omegaUpper := r.a.omegaHigh
	omegaSquared := new(big.Rat).Mul(omegaUpper, omegaUpper)
	curvature := new(big.Rat).Add(new(big.Rat).Mul(omegaSquared, dNorm),
		new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(omegaUpper, vNorm)))
	curvature.Add(curvature, new(big.Rat).Mul(omegaSquared,
		new(big.Rat).Mul(vNorm, r.a.path.duration)))
	fraction := big.NewRat(1, 1)
	for range 60 {
		until := new(big.Rat).Mul(fraction, r.a.path.duration)
		if new(big.Rat).Mul(curvature, until).Cmp(derivative) < 0 {
			return fraction, true
		}
		fraction = new(big.Rat).Quo(fraction, big.NewRat(2, 1))
	}
	return nil, false
}

// obliqueAffineDepartureFraction proves an open-at-zero gap between the
// complete source boxes. Their common support plane is exact even though its
// published unit normal is rounded. Translation makes the signed plane gap
// affine, so a positive slope keeps the entire pair clear after time zero.
func (r *rotationalPairSweep) obliqueAffineDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if r.a.path.drift != nil || r.b.path.drift != nil || first.Ideal.Manifold == nil ||
		len(first.Ideal.Manifold.Points) != 4 {
		return nil, false
	}
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := dvCross(r.a.startBox.edge[i], r.a.startBox.edge[j])
		if dvIsZero(normal) {
			continue
		}
		alo, ahi := orientedProjection(r.a.startBox, normal)
		blo, bhi := orientedProjection(r.b.startBox, normal)
		side := 0
		switch {
		case dyCmp(ahi, blo) == 0:
			side = 1
		case dyCmp(bhi, alo) == 0:
			side = -1
		default:
			continue
		}
		relative := dyV3{}
		for k := range 3 {
			relative[k] = dySubScalar(r.b.path.delta[k], r.a.path.delta[k])
		}
		slope := dvDot(relative, normal)
		if side < 0 {
			slope = dyNeg(slope)
		}
		if slope.sign() > 0 {
			return big.NewRat(1, 2), true
		}
	}
	return nil, false
}

// A spin around the contact normal leaves both bodies' Z supports unchanged.
// Positive relative Z travel therefore opens a whole-body gap from exact touch.
func (r *rotationalPairSweep) horizontalSpinDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	paths := [2]rotationalSweepPath{r.a, r.b}
	stationary, spinning := -1, -1
	for i, path := range paths {
		if path.path.drift == nil && path.path.delta == [3]dyadic{} {
			stationary = i
		}
		if path.path.drift != nil && path.frame.axis[0].Sign() == 0 &&
			path.frame.axis[1].Sign() == 0 && path.frame.axis[2].Sign() != 0 &&
			path.velocity[0].Sign() == 0 && path.velocity[1].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return nil, false
	}
	normal := first.Ideal.Manifold.Points[0].Normal.Value
	if normal != (r3.Vec{Z: 1}) && normal != (r3.Vec{Z: -1}) {
		return nil, false
	}
	a, okA := sourceOrientedBoxAtPose(r.a.body, r.a.path.from)
	b, okB := sourceOrientedBoxAtPose(r.b.body, r.b.path.from)
	if !okA || !okB {
		return nil, false
	}
	z := dyV3{dyZero(), dyZero(), mustDyOf(1)}
	alo, ahi := orientedProjection(a, z)
	blo, bhi := orientedProjection(b, z)
	gap := dySubScalar(blo, ahi)
	if normal.Z < 0 {
		gap = dySubScalar(alo, bhi)
	}
	if gap.sign() != 0 {
		return nil, false
	}
	speed := new(big.Rat).Set(paths[spinning].velocity[2])
	if spinning == 0 {
		speed.Neg(speed)
	}
	if normal.Z < 0 {
		speed.Neg(speed)
	}
	if speed.Sign() <= 0 {
		return nil, false
	}
	return big.NewRat(1, 2), true
}

// A rotation about an axis-normal contact face does not move either support
// plane along that axis. A strictly increasing relative plane gap certifies
// immediate departure even when only one body rotates.
func (r *rotationalPairSweep) axisFaceDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	point := first.Ideal.Manifold.Points[0]
	axis, side, ok := signedAxis(point.Normal.Value)
	if !ok || point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return nil, false
	}
	sideA, sideB := 1, 0
	if side == 0 {
		sideA, sideB = 0, 1
	}
	var faceA, faceB orientedFace
	if !orientedAxisFace(&r.a.startBox, axis, sideA, &faceA) ||
		!orientedAxisFace(&r.b.startBox, axis, sideB, &faceB) ||
		dyCmp(faceA.origin[axis], faceB.origin[axis]) != 0 {
		return nil, false
	}
	velocity := func(path rotationalSweepPath) (*big.Rat, bool) {
		if path.path.drift == nil {
			return new(big.Rat).Quo(path.path.delta[axis].rat(), path.path.duration), true
		}
		for other := range 3 {
			if other != axis && path.frame.axis[other].Sign() != 0 {
				return nil, false
			}
		}
		return path.velocity[axis], true
	}
	vA, validA := velocity(r.a)
	vB, validB := velocity(r.b)
	if !validA || !validB {
		return nil, false
	}
	derivative := new(big.Rat).Sub(vB, vA)
	if side == 0 {
		derivative.Neg(derivative)
	}
	if derivative.Sign() <= 0 {
		return nil, false
	}
	return big.NewRat(1, 2), true
}

func (r *rotationalPairSweep) refine(ctx context.Context, left, right *SweepSample,
	depth int) (bool, error) {
	lf, rf := floatRat(left.At.Fraction.Base()), floatRat(right.At.Fraction.Base())
	if left.Ideal.Relation == ContactSeparated && right.Ideal.Relation == ContactSeparated &&
		r.intervalClear(left, right, lf, rf) {
		return false, nil
	}
	width := new(big.Rat).Mul(new(big.Rat).Sub(rf, lf), r.a.path.duration)
	if width.Cmp(r.resolution) <= 0 {
		if left.Ideal.Relation == ContactSeparated &&
			(right.Ideal.Relation == ContactTouching || right.Ideal.Relation == ContactOverlapping) {
			r.report.Outcome, r.report.Event = SweepImpactBracket, &right.Ideal
			r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
			r.report.bracketRight = new(big.Rat).Set(right.exactFraction)
			return true, nil
		}
		r.undecided(lf, rf, SweepTimeFloor)
		return true, nil
	}
	if depth >= 60 {
		r.undecided(lf, rf, SweepFractionFloor)
		return true, nil
	}
	middle := new(big.Rat).Quo(new(big.Rat).Add(lf, rf), big.NewRat(2, 1))
	if ratFloatNearest(middle) == left.At.Fraction.Base() ||
		ratFloatNearest(middle) == right.At.Fraction.Base() {
		r.undecided(lf, rf, SweepFractionFloor)
		return true, nil
	}
	sample, err := r.sample(ctx, middle)
	if errors.Is(err, errSweepPoseBudget) {
		r.undecided(lf, rf, SweepPoseBudget)
		return true, nil
	}
	if err != nil {
		return false, err
	}
	done, err := r.refine(ctx, left, sample, depth+1)
	if done || err != nil {
		return done, err
	}
	return r.refine(ctx, sample, right, depth+1)
}

func (r *rotationalPairSweep) intervalClear(left, right *SweepSample, lf, rf *big.Rat) bool {
	if r.obliqueAffineIntervalClear(lf, rf) {
		return true
	}
	if r.intervalAxisSeparated(lf, rf) {
		return true
	}
	gapLower := func(sample *SweepSample) *big.Rat {
		if sample.Ideal.Gap == nil {
			return nil
		}
		value, okValue := exactBaseValue(sample.Ideal.Gap.Value)
		bound, okBound := exactBaseValue(sample.Ideal.Gap.Bound)
		if !okValue || !okBound {
			return nil
		}
		return new(big.Rat).Sub(value, bound)
	}
	a, b := gapLower(left), gapLower(right)
	if a == nil || b == nil || a.Sign() <= 0 || b.Sign() <= 0 {
		return false
	}
	travel := new(big.Rat).Mul(new(big.Rat).Sub(rf, lf),
		new(big.Rat).Add(r.a.fullTravel, r.b.fullTravel))
	return new(big.Rat).Add(a, b).Cmp(travel) > 0
}

// An exact positive support gap at both ends of an affine interval is
// positive throughout it. This also covers an oblique outward drift whose
// world-axis hulls overlap after the boxes have separated.
func (r *rotationalPairSweep) obliqueAffineIntervalClear(from, to *big.Rat) bool {
	if r.a.path.drift != nil || r.b.path.drift != nil {
		return false
	}
	a0, okA := translatedOrientedBox(r.a.startBox, r.a.path.delta, from)
	b0, okB := translatedOrientedBox(r.b.startBox, r.b.path.delta, from)
	a1, okC := translatedOrientedBox(r.a.startBox, r.a.path.delta, to)
	b1, okD := translatedOrientedBox(r.b.startBox, r.b.path.delta, to)
	if !okA || !okB || !okC || !okD {
		return false
	}
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := dvCross(r.a.startBox.edge[i], r.a.startBox.edge[j])
		if dvIsZero(normal) {
			continue
		}
		alo0, ahi0 := orientedProjection(a0, normal)
		blo0, bhi0 := orientedProjection(b0, normal)
		alo1, ahi1 := orientedProjection(a1, normal)
		blo1, bhi1 := orientedProjection(b1, normal)
		if dyCmp(ahi0, blo0) < 0 && dyCmp(ahi1, blo1) < 0 ||
			dyCmp(bhi0, alo0) < 0 && dyCmp(bhi1, alo1) < 0 {
			return true
		}
	}
	return false
}

// intervalAxisSeparated encloses all eight ideal corner paths over the whole
// fraction span. Strict separation of their coordinate hulls proves clear.
func (r *rotationalPairSweep) intervalAxisSeparated(from, to *big.Rat) bool {
	a := r.a.cornerSpan(from, to)
	b := r.b.cornerSpan(from, to)
	for axis := range 3 {
		aLow, aHigh := a[0][axis].lo, a[0][axis].hi
		bLow, bHigh := b[0][axis].lo, b[0][axis].hi
		for corner := 1; corner < 8; corner++ {
			if a[corner][axis].lo.Cmp(aLow) < 0 {
				aLow = a[corner][axis].lo
			}
			if a[corner][axis].hi.Cmp(aHigh) > 0 {
				aHigh = a[corner][axis].hi
			}
			if b[corner][axis].lo.Cmp(bLow) < 0 {
				bLow = b[corner][axis].lo
			}
			if b[corner][axis].hi.Cmp(bHigh) > 0 {
				bHigh = b[corner][axis].hi
			}
		}
		if aHigh.Cmp(bLow) < 0 || bHigh.Cmp(aLow) < 0 {
			return true
		}
	}
	return false
}

func (p rotationalSweepPath) cornerSpan(from, to *big.Rat) [8]ivVec {
	var output [8]ivVec
	if p.path.drift == nil {
		for index, corner := range p.startBox.corner {
			for axis := range 3 {
				start := corner[axis].rat()
				lo := new(big.Rat).Mul(p.path.delta[axis].rat(), from)
				hi := new(big.Rat).Mul(p.path.delta[axis].rat(), to)
				if lo.Cmp(hi) > 0 {
					lo, hi = hi, lo
				}
				output[index][axis] = interval(new(big.Rat).Add(start, lo), new(big.Rat).Add(start, hi))
			}
		}
		return output
	}
	lowTime := new(big.Rat).Mul(p.path.duration, from)
	highTime := new(big.Rat).Mul(p.path.duration, to)
	lowAngle := new(big.Rat).Mul(p.omegaLow, lowTime)
	highAngle := new(big.Rat).Mul(p.omegaHigh, highTime)
	sin, cos := rotationalSinCosSpan(lowAngle, highAngle)
	rotationSpan := p.frame.rotation(sin, cos)
	midTime := new(big.Rat).Quo(new(big.Rat).Add(lowTime, highTime), big.NewRat(2, 1))
	angleAtMidLow := new(big.Rat).Mul(p.omegaLow, midTime)
	angleAtMidHigh := new(big.Rat).Mul(p.omegaHigh, midTime)
	midSin, midCos := rotationalSinCosSpan(angleAtMidLow, angleAtMidHigh)
	rotationMid := p.frame.rotation(midSin, midCos)
	halfDuration := new(big.Rat).Quo(new(big.Rat).Sub(highTime, lowTime), big.NewRat(2, 1))
	pivot := pointVec(p.frame.center)
	for index, corner := range p.startBox.corner {
		start := ratVec{corner[0].rat(), corner[1].rat(), corner[2].rat()}
		relative := ivVecSub(pointVec(start), pivot)
		spanRelative := rotationSpan.apply(relative)
		point := ivVecAdd(rotationMid.apply(relative), pivot)
		for axis := range 3 {
			point[axis] = intervalAdd(point[axis],
				pointInterval(new(big.Rat).Mul(p.velocity[axis], midTime)))
			following, preceding := (axis+1)%3, (axis+2)%3
			derivative := intervalAdd(pointInterval(p.velocity[axis]), intervalSub(
				intervalScale(spanRelative[preceding], p.frame.axis[following]),
				intervalScale(spanRelative[following], p.frame.axis[preceding])))
			maximum := new(big.Rat).Abs(derivative.lo)
			if other := new(big.Rat).Abs(derivative.hi); other.Cmp(maximum) > 0 {
				maximum = other
			}
			travel := new(big.Rat).Mul(maximum, halfDuration)
			output[index][axis] = intervalOwned(new(big.Rat).Sub(point[axis].lo, travel),
				new(big.Rat).Add(point[axis].hi, travel))
		}
	}
	return output
}

func rotationalSinCosSpan(low, high *big.Rat) (ratInterval, ratInterval) {
	loSin, loCos := radianSinCos(low)
	hiSin, hiCos := radianSinCos(high)
	if low.Sign() >= 0 && high.Cmp(halfPiInterval().lo) <= 0 {
		return interval(loSin.lo, hiSin.hi), interval(hiCos.lo, loCos.hi)
	}
	width := new(big.Rat).Sub(high, low)
	return intervalOwned(new(big.Rat).Sub(loSin.lo, width),
			new(big.Rat).Add(loSin.hi, width)),
		intervalOwned(new(big.Rat).Sub(loCos.lo, width), new(big.Rat).Add(loCos.hi, width))
}

func (r *rotationalPairSweep) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

func (r *rotationalPairSweep) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.a.path.duration),
		To: sweepInstant(to, r.a.path.duration)}
	r.sortSamples()
	return r.report
}
