package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/clearance"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rotationalSweepPath is one body's prepared path: its ideal motion and travel
// bound, and the exact source points whose images bound the float-to-ideal
// pose deviation. A source box carries its eight corners; an exact planar body
// (contact_sweep_faceted.go) carries every vertex of its snapshot.
type rotationalSweepPath struct {
	body         *Body
	path         affinePairPath
	fullTravel   *big.Rat
	startBox     orientedSourceBox
	sourceBox    orientedSourceBox
	startPoints  []proofarith.DyV3 // exact source points under the path's From
	sourcePoints []proofarith.DyV3 // exact source points at the identity query pose
	solid        *pair.PlanarSolid // the identity-pose planar snapshot, planar paths only
	delta        proofarith.Dyadic // the snapshot's held displacement δ (§10.4), planar paths only
	frame        motionbound.MotionFrame
	fromRot      motionbound.IvMat
	fromT        motionbound.RatVec
	velocity     motionbound.RatVec
	omegaLow     *big.Rat
	omegaHigh    *big.Rat
	memo         *sweepPathMemo // the run's memo (contact_sweep_memo.go); nil outside a run
}

func prepareRotationalSweepPath(body *Body, path affinePairPath) (rotationalSweepPath, bool) {
	startBox, ok := sourceOrientedBoxAtPose(body, path.from)
	if !ok {
		return rotationalSweepPath{}, false
	}
	sourceBox, ok := sourceOrientedBoxAtPose(body, r3.Identity())
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared, ok := prepareSweepMotion(body, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.startBox, prepared.sourceBox = startBox, sourceBox
	prepared.startPoints = append([]proofarith.DyV3(nil), startBox.corner[:]...)
	prepared.sourcePoints = append([]proofarith.DyV3(nil), sourceBox.corner[:]...)
	return prepared, true
}

// prepareSweepMotion builds the ideal path and the §4.1 travel bound of one
// body. The travel radius reads the body's inflated bounds, so it holds for
// any body; the caller supplies the source points.
func prepareSweepMotion(body *Body, path affinePairPath) (rotationalSweepPath, bool) {
	if path.screw != nil {
		axis := path.screw.Axis
		angle, ok := exactBaseValue(path.screw.Angle)
		if !ok || path.duration.Sign() <= 0 {
			return rotationalSweepPath{}, false
		}
		angular := new(big.Rat).Quo(angle, path.duration)
		linear := new(big.Rat).Quo(proofarith.FloatRat(path.screw.Slide), path.duration)
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
	fromRot, fromT, ok := motionbound.ExactTransform(path.from)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared := rotationalSweepPath{body: body, path: path, fromRot: fromRot, fromT: fromT}
	if path.drift == nil {
		travelSquared := new(big.Rat)
		for _, component := range path.delta {
			value := component.Rat()
			travelSquared.Add(travelSquared, new(big.Rat).Mul(value, value))
		}
		bound := proofbound.RatSqrtUp(travelSquared)
		if !finiteMeasurementValues(bound) {
			return rotationalSweepPath{}, false
		}
		prepared.fullTravel = proofarith.FloatRat(bound)
		return prepared, true
	}
	drift := path.drift
	velocity := [3]units.Value{drift.LinearVelocity.X, drift.LinearVelocity.Y, drift.LinearVelocity.Z}
	angular := [3]units.Value{drift.AngularVelocity.X, drift.AngularVelocity.Y, drift.AngularVelocity.Z}
	prepared.velocity = motionbound.RatVec{}
	omega := motionbound.RatVec{}
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
	prepared.omegaLow = proofarith.FloatRat(proofbound.RatSqrtDown(omegaSquared))
	prepared.omegaHigh = proofarith.FloatRat(proofbound.RatSqrtUp(omegaSquared))
	if prepared.omegaLow == nil || prepared.omegaHigh == nil || prepared.omegaHigh.Sign() <= 0 {
		return rotationalSweepPath{}, false
	}
	center, ok := motionbound.RatVecOf(drift.Center)
	if !ok {
		return rotationalSweepPath{}, false
	}
	unit, ok := motionbound.UnitScaleInterval(omega)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.frame = motionbound.MotionFrame{Kind: motionbound.MotionRevolute, Axis: omega, Unit: unit, Center: center}
	radius, ok := rotationalSweepRadius(body, path.from, drift.Center, omega)
	if !ok {
		return rotationalSweepPath{}, false
	}
	vUp := proofbound.RatSqrtUp(vSquared)
	if !finiteMeasurementValues(vUp) {
		return rotationalSweepPath{}, false
	}
	speed := new(big.Rat).Add(proofarith.FloatRat(vUp), new(big.Rat).Mul(radius, prepared.omegaHigh))
	prepared.fullTravel = new(big.Rat).Mul(speed, path.duration)
	if path.screw != nil {
		angle, _ := exactBaseValue(path.screw.Angle)
		angular := new(big.Rat).Quo(angle, path.duration)
		linear := new(big.Rat).Quo(proofarith.FloatRat(path.screw.Slide), path.duration)
		for axis, component := range [3]float64{path.screw.Axis.X, path.screw.Axis.Y, path.screw.Axis.Z} {
			prepared.velocity[axis] = new(big.Rat).Mul(proofarith.FloatRat(component), linear)
		}
		prepared.omegaLow, prepared.omegaHigh = angular, angular
		prepared.fullTravel = new(big.Rat).Mul(new(big.Rat).Add(new(big.Rat).Abs(linear),
			new(big.Rat).Mul(radius, angular)), path.duration)
	}
	return prepared, true
}

// rotationalSweepRadius is §4.1's ρ: an upper bound, rounded to a float, on
// the largest distance of the From-mapped inflated bounds corners from the
// axis line through center, the root of max |(corner − center) × axis|² /
// |axis|². Every squared cross product shares the one denominator |axis|², so
// the largest is chosen by its numerator, which is exact dyadic arithmetic
// for the dyadic corners, pivot and axis every caller holds; one division
// then gives the same rational the per-corner quotients' maximum is. A body's
// readings are memoized on it (sweepRadiusMemo), since a step reads each
// path's radius both for its swept box and for its sweep.
func rotationalSweepRadius(body *Body, from r3.Transform, center r3.Vec,
	axis motionbound.RatVec) (*big.Rat, bool) {
	var dyAxis proofarith.DyV3
	for k := range 3 {
		component, ok := proofarith.DyOfRat(axis[k])
		if !ok {
			return rationalSweepRadius(body, from, center, axis)
		}
		dyAxis[k] = component
	}
	if !motionbound.MemosOn() {
		return dyadicSweepRadius(body, from, center, dyAxis)
	}
	key := newSweepRadiusKey(from, center, dyAxis)
	if radius, ok, hit := body.sweepRadii.load(key); hit {
		if !ok {
			return nil, false
		}
		return proofarith.FloatRat(radius), true
	}
	radius, ok := dyadicSweepRadius(body, from, center, dyAxis)
	held := 0.0
	if ok {
		held, _ = radius.Float64()
	}
	body.sweepRadii.store(key, held, ok)
	return radius, ok
}

// dyadicSweepRadius is rotationalSweepRadius over a dyadic axis, computed
// afresh.
func dyadicSweepRadius(body *Body, from r3.Transform, center r3.Vec, axis proofarith.DyV3) (*big.Rat, bool) {
	corners, ok := inflatedBoundsCorners(body)
	if !ok {
		return nil, false
	}
	axisSquared := proofarith.DvDot(axis, axis)
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	pivot := proofarith.DyVec(center)
	place := newExactContactMap(from)
	best := proofarith.DyZero()
	for _, corner := range corners {
		cross := proofarith.DvCross(proofarith.DvSub(place.apply(corner), pivot), axis)
		if squared := proofarith.DvDot(cross, cross); proofarith.DyCmp(squared, best) > 0 {
			best = squared
		}
	}
	return sweepRadiusOf(new(big.Rat).Quo(best.Rat(), axisSquared.Rat()))
}

// rationalSweepRadius is rotationalSweepRadius over an axis with a
// non-dyadic component, which no caller holds today.
func rationalSweepRadius(body *Body, from r3.Transform, center r3.Vec,
	axis motionbound.RatVec) (*big.Rat, bool) {
	corners, ok := inflatedBoundsCorners(body)
	if !ok {
		return nil, false
	}
	axisSquared := new(big.Rat)
	for k := range 3 {
		axisSquared.Add(axisSquared, new(big.Rat).Mul(axis[k], axis[k]))
	}
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	pivot := proofarith.DyVec(center)
	best := new(big.Rat)
	for _, corner := range corners {
		delta := proofarith.DvSub(exactContactTransform(from, corner), pivot)
		squared := new(big.Rat)
		for k := range 3 {
			i, j := (k+1)%3, (k+2)%3
			component := new(big.Rat).Sub(new(big.Rat).Mul(delta[i].Rat(), axis[j]),
				new(big.Rat).Mul(delta[j].Rat(), axis[i]))
			squared.Add(squared, component.Mul(component, component))
		}
		if squared.Cmp(best) > 0 {
			best = squared
		}
	}
	return sweepRadiusOf(best.Quo(best, axisSquared))
}

// sweepRadiusOf rounds the exact squared radius's root up to a float.
func sweepRadiusOf(squared *big.Rat) (*big.Rat, bool) {
	radius := proofbound.RatSqrtUp(squared)
	if !finiteMeasurementValues(radius) {
		return nil, false
	}
	return proofarith.FloatRat(radius), true
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

// sweepIdealPose is a sweep path's ideal pose x ↦ rot·x + shift, its linear
// part in the common-denominator form (motionbound.ScaledIvMat).
type sweepIdealPose struct {
	rot   motionbound.ScaledIvMat
	shift motionbound.IvVec
}

// idealAt is the ideal pose at fraction f. A rotating path's rotation
// encloses the turn ω·t over both rate bounds, and its shift is the pivot's
// image R·(t(From) − c) + c + v·t. ok is false only when the From basis does
// not read as points, which motionbound.ExactTransform never produces.
func (p rotationalSweepPath) idealAt(f *big.Rat) (sweepIdealPose, bool) {
	if !p.memo.open() {
		return p.idealAtAfresh(f)
	}
	key := ratKey(f)
	if ideal, ok := p.memo.loadIdeal(key); ok {
		return ideal, true
	}
	ideal, ok := p.idealAtAfresh(f)
	if ok {
		p.memo.storeIdeal(key, ideal)
	}
	return ideal, ok
}

// idealAtAfresh is idealAt computed without the memo.
func (p rotationalSweepPath) idealAtAfresh(f *big.Rat) (sweepIdealPose, bool) {
	if p.path.drift == nil {
		shift := motionbound.PointVec(p.fromT)
		for axis := range 3 {
			shift[axis] = proofbound.IntervalAdd(shift[axis],
				proofbound.PointInterval(new(big.Rat).Mul(p.path.delta[axis].Rat(), f)))
		}
		return sweepIdealPose{rot: motionbound.NewScaledIvMat(p.fromRot), shift: shift}, true
	}
	elapsed := new(big.Rat).Mul(p.path.duration, f)
	angleLow := new(big.Rat).Mul(p.omegaLow, elapsed)
	angleHigh := new(big.Rat).Mul(p.omegaHigh, elapsed)
	sin, cos := motionbound.RadianSinCos(angleLow)
	width := new(big.Rat).Sub(angleHigh, angleLow)
	sin = proofbound.IntervalOwned(new(big.Rat).Sub(sin.Lo, width), new(big.Rat).Add(sin.Hi, width))
	cos = proofbound.IntervalOwned(new(big.Rat).Sub(cos.Lo, width), new(big.Rat).Add(cos.Hi, width))
	rot := p.frame.ScaledRotation(sin, cos)
	offset := make([]*big.Rat, 3)
	for axis := range 3 {
		offset[axis] = new(big.Rat).Sub(p.fromT[axis], p.frame.Center[axis])
	}
	q := proofarith.CommonDenom(offset...)
	lo, hi := rot.ApplyScaled([3]*big.Int{proofarith.ScaledNum(offset[0], q), proofarith.ScaledNum(offset[1], q), proofarith.ScaledNum(offset[2], q)})
	rotDen := new(big.Int).Mul(rot.Den, q)
	var shift motionbound.IvVec
	for axis := range 3 {
		pivot := new(big.Rat).Add(p.frame.Center[axis], new(big.Rat).Mul(p.velocity[axis], elapsed))
		den := proofarith.LcmInt(rotDen, pivot.Denom())
		multiplier := new(big.Int).Quo(den, rotDen)
		pivotN := proofarith.ScaledNum(pivot, den)
		low := lo[axis].Mul(lo[axis], multiplier)
		high := hi[axis].Mul(hi[axis], multiplier)
		shift[axis] = proofbound.IntervalOwned(new(big.Rat).SetFrac(low.Add(low, pivotN), den),
			new(big.Rat).SetFrac(high.Add(high, pivotN), den))
	}
	linear, ok := rot.MulPoints(p.fromRot)
	return sweepIdealPose{rot: linear, shift: shift}, ok
}

// roundedAt compares the exact staged source corners used by ContactPair with
// their ideal path positions and returns the rounded pose's source box.
func (p rotationalSweepPath) roundedAt(pose r3.Transform, f *big.Rat) (orientedSourceBox, float64, bool) {
	points, bound, ok, _ := p.pointDeviation(pose, f, noSweepPoll)
	if !ok {
		return orientedSourceBox{}, 0, false
	}
	var box orientedSourceBox
	copy(box.corner[:], points)
	box.faces = p.startBox.faces
	box.edge = [3]proofarith.DyV3{proofarith.DvSub(box.corner[1], box.corner[0]),
		proofarith.DvSub(box.corner[2], box.corner[0]), proofarith.DvSub(box.corner[4], box.corner[0])}
	for _, edge := range box.edge {
		if proofarith.DvIsZero(edge) {
			return orientedSourceBox{}, 0, false
		}
	}
	return box, bound, true
}

func noSweepPoll() error { return nil }

// replayDeviation is what replay charges one rounded pose at f: the staged
// points' distance bound from the ideal path (pointDeviation), and for a
// positive-displacement planar body the §10.4 transfer charge (transferCharge).
// A source-box path holds δ zero and charges nothing.
func (p rotationalSweepPath) replayDeviation(pose r3.Transform, f *big.Rat) (*big.Rat, *big.Rat, bool) {
	ideal, ok := p.idealAt(f)
	if !ok {
		return nil, nil, false
	}
	var bound float64
	if p.memo.open() {
		_, bound, ok, _ = p.pointDeviation(pose, f, noSweepPoll)
	} else {
		_, bound, ok, _ = p.pointDeviationFrom(pose, ideal, noSweepPoll)
	}
	if !ok {
		return nil, nil, false
	}
	charge, ok := p.transferCharge(pose, ideal)
	if !ok {
		return nil, nil, false
	}
	return proofarith.FloatRat(bound), charge, true
}

// transferCharge is the §10.4 transfer charge ‖R_r − R_i‖_F·δ of one rounded
// pose: R_r is the pose's float basis, the linear part exactContactTransform
// stages a point through, and R_i the ideal rotation's interval enclosure at
// the pose's fraction (idealAt). A true point is x + e with |e| <= δ in the
// body's frame, so its rounded image differs from its ideal one by the held
// deviation plus (R_r − R_i)·e, whose length is at most the Frobenius norm of
// R_r − R_i times δ. The norm is the upper bound over every member of the
// enclosure (motionbound.MagnitudeSquaredUpper). A translating path's enclosure is its
// From basis exactly, which the rounded pose keeps, so it charges zero.
func (p rotationalSweepPath) transferCharge(pose r3.Transform, ideal sweepIdealPose) (*big.Rat, bool) {
	if p.delta.Sign() == 0 {
		return new(big.Rat), true
	}
	rounded, _, ok := motionbound.ExactTransform(pose)
	if !ok {
		return nil, false
	}
	// Each entry of R_r − R_i is [r − hi, r − lo] for the point r and the
	// enclosure [lo, hi]; its farther endpoint, squared and summed, is
	// motionbound.MagnitudeSquaredUpper's bound, here over one denominator.
	values := make([]*big.Rat, 0, 9)
	for i := range 3 {
		for k := range 3 {
			values = append(values, rounded[i][k].Lo)
		}
	}
	den := proofarith.LcmInt(proofarith.CommonDenom(values...), ideal.rot.Den)
	scale := new(big.Int).Quo(den, ideal.rot.Den)
	squared := new(big.Int)
	for i := range 3 {
		for k := range 3 {
			r := proofarith.ScaledNum(values[3*i+k], den)
			below := new(big.Int).Mul(ideal.rot.Hi[i][k], scale)
			below.Sub(r, below)
			above := new(big.Int).Mul(ideal.rot.Lo[i][k], scale)
			above.Sub(r, above)
			farther := below.Abs(below)
			if above.Abs(above).Cmp(farther) > 0 {
				farther = above
			}
			squared.Add(squared, farther.Mul(farther, farther))
		}
	}
	norm := proofbound.RatSqrtUp(new(big.Rat).SetFrac(squared, new(big.Int).Mul(den, den)))
	if !finiteMeasurementValues(norm) {
		return nil, false
	}
	return new(big.Rat).Mul(proofarith.FloatRat(norm), p.delta.Rat()), true
}

// pointDeviation maps the exact source points through the read query pose,
// staged exactly as ContactPair stages them, and bounds their distance from
// their ideal path positions (contact-sweep §3). The rounded and ideal bodies
// are two affine images of one source, so their difference is affine and the
// largest point distance bounds every point of the points' hull. This charges
// placement and pose-composition rounding without trusting a rounded composed
// transform. poll is charged once per point.
//
// Each point's squared distance bound is Σ max(|o − hi|, |o − lo|)² over the
// axes, o the staged coordinate and [lo, hi] the ideal one,
// rot·p + shift, evaluated exactly in the common-denominator form (see
// motionbound.ScaledIvMat): the source and staged points share one q, and every point's
// sum shares one denominator, so points compare by numerator.
//
// A memo hit charges poll once per point, as the reading it replaces did.
func (p rotationalSweepPath) pointDeviation(pose r3.Transform, f *big.Rat,
	poll func() error) ([]proofarith.DyV3, float64, bool, error) {
	if !pose.IsValid() {
		return nil, 0, false, nil
	}
	var key sweepDeviationKey
	if p.memo.open() {
		key = sweepDeviationKey{points: pointSetOf(p.sourcePoints), pose: poseBits(pose), f: ratKey(f)}
		if entry, ok := p.memo.loadDeviation(key); ok {
			for range p.sourcePoints {
				if err := poll(); err != nil {
					return nil, 0, false, err
				}
			}
			return entry.points, entry.bound, entry.ok, nil
		}
	}
	ideal, ok := p.idealAt(f)
	if !ok {
		return nil, 0, false, nil
	}
	points, bound, ok, err := p.pointDeviationFrom(pose, ideal, poll)
	if err == nil && p.memo.open() {
		p.memo.storeDeviation(key, sweepDeviation{points: points, bound: bound, ok: ok})
	}
	return points, bound, ok, err
}

// pointDeviationFrom is pointDeviation against an already enclosed ideal pose.
func (p rotationalSweepPath) pointDeviationFrom(pose r3.Transform, ideal sweepIdealPose,
	poll func() error) ([]proofarith.DyV3, float64, bool, error) {
	if !pose.IsValid() {
		return nil, 0, false, nil
	}
	actual, squared, err := p.pointDeviationSquared(pose, ideal, poll)
	if err != nil {
		return nil, 0, false, err
	}
	bound := proofbound.RatSqrtUp(squared)
	return actual, bound, finiteMeasurementValues(bound), nil
}

// pointDeviationSquared stages the source points through pose and returns
// them with the exact largest squared distance bound, maxSquared/whole², the
// value pointDeviationFrom rounds up.
func (p rotationalSweepPath) pointDeviationSquared(pose r3.Transform, ideal sweepIdealPose,
	poll func() error) ([]proofarith.DyV3, *big.Rat, error) {
	actual := make([]proofarith.DyV3, len(p.sourcePoints))
	place := newExactContactMap(pose)
	for i, source := range p.sourcePoints {
		if err := poll(); err != nil {
			return nil, nil, err
		}
		actual[i] = place.apply(source)
	}
	// Every coordinate is dyadic, so the common denominator q of the source
	// and staged points is 2^shift, shift their largest denominator exponent,
	// and each coordinate's numerator over q is its mantissa shifted.
	shift := 0
	for i, source := range p.sourcePoints {
		for axis := range 3 {
			shift = max(shift, dyDenominatorExp(source[axis]), dyDenominatorExp(actual[i][axis]))
		}
	}
	q := new(big.Int).Lsh(big.NewInt(1), uint(shift))
	rot := ideal.rot
	rotDen := new(big.Int).Mul(rot.Den, q)
	// Axis i's interval endpoints over den[i]; the squared sum over whole².
	var den, rotMultiplier, observedMultiplier, shiftLo, shiftHi, toWhole [3]*big.Int
	whole := big.NewInt(1)
	for axis := range 3 {
		shift := ideal.shift[axis]
		den[axis] = proofarith.LcmInt(proofarith.LcmInt(rotDen, shift.Lo.Denom()), shift.Hi.Denom())
		rotMultiplier[axis] = new(big.Int).Quo(den[axis], rotDen)
		observedMultiplier[axis] = new(big.Int).Quo(den[axis], q)
		shiftLo[axis], shiftHi[axis] = proofarith.ScaledNum(shift.Lo, den[axis]), proofarith.ScaledNum(shift.Hi, den[axis])
		whole = proofarith.LcmInt(whole, den[axis])
	}
	for axis := range 3 {
		toWhole[axis] = new(big.Int).Quo(whole, den[axis])
	}
	maxSquared := new(big.Int)
	for i := range p.sourcePoints {
		var point [3]*big.Int
		for axis := range 3 {
			point[axis] = dyScaledNum(p.sourcePoints[i][axis], shift)
		}
		lo, hi := rot.ApplyScaled(point)
		squared := new(big.Int)
		for axis := range 3 {
			observed := dyScaledNum(actual[i][axis], shift)
			observed.Mul(observed, observedMultiplier[axis])
			low := lo[axis].Mul(lo[axis], rotMultiplier[axis])
			low.Add(low, shiftLo[axis])
			high := hi[axis].Mul(hi[axis], rotMultiplier[axis])
			high.Add(high, shiftHi[axis])
			// observed − ideal is [o − hi, o − lo].
			below := high.Sub(observed, high)
			above := low.Sub(observed, low)
			maximum := below.Abs(below)
			if above.Abs(above).Cmp(maximum) > 0 {
				maximum = above
			}
			maximum.Mul(maximum, toWhole[axis])
			squared.Add(squared, maximum.Mul(maximum, maximum))
		}
		if squared.Cmp(maxSquared) > 0 {
			maxSquared = squared
		}
	}
	return actual, new(big.Rat).SetFrac(maxSquared, new(big.Int).Mul(whole, whole)), nil
}

// dyDenominatorExp is the exponent k of d's reduced denominator 2^k, the
// denominator d.Rat() carries: a nonzero Dyadic's mantissa is odd, so k is
// −exp for a negative exponent and zero otherwise.
func dyDenominatorExp(d proofarith.Dyadic) int {
	if d.Sign() == 0 {
		return 0
	}
	return max(0, -d.Exp())
}

// dyScaledNum is d·2^shift as an integer, proofarith.ScaledNum(d.Rat(), q)
// for q = 2^shift, which shift at least dyDenominatorExp(d) makes whole.
func dyScaledNum(d proofarith.Dyadic, shift int) *big.Int {
	if d.Sign() == 0 {
		return new(big.Int)
	}
	mant := d.MantInto(new(big.Int))
	return mant.Lsh(mant, uint(d.Exp()+shift))
}

type rotationalPairSweep struct {
	doc        *Document
	a, b       rotationalSweepPath
	req        SweepRequest
	resolution *big.Rat
	report     *SweepReport
	planar     bool                  // both paths carry exact planar vertex sets (contact_sweep_faceted.go)
	departure  *planarDepartureProof // a planar pair's §10.2 departure, when proved
}

func (d *Document) sweepRotatingPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, error) {
	aPath, okA := prepareRotationalSweepPath(a, pa)
	bPath, okB := prepareRotationalSweepPath(b, pb)
	if !okA || !okB {
		if result, admitted, err := d.sweepPlanarPair(ctx, a, b, pa, pb, req, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, nil
	}
	attachSweepMemos(&aPath, &bPath)
	defer closeSweepMemos(&aPath, &bPath)
	run := rotationalPairSweep{doc: d, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, err
	}
	if planar, ok, planarErr := d.planarContinuation(ctx, result, pa, pb, resolution); planarErr != nil || ok {
		return planar, planarErr
	}
	if result.Outcome == SweepClear || result.Outcome == SweepDepartedClear ||
		result.Outcome == SweepImpactBracket && result.Bracket != nil ||
		result.Outcome == SweepPersistentTouch &&
			result.ContactTrack != nil && result.ContactTrack.orientedA != nil &&
			result.ContactTrack.orientedB != nil && aPath.path.drift == nil && bPath.path.drift == nil {
		result.replay = &sweepReplayProof{rotation: &[2]rotationalSweepPath{aPath, bPath},
			track: result.ContactTrack, request: req.ContactRequest}
		result.replay.snapshot(result)
		if result.Outcome == SweepImpactBracket {
			left, leftOK := exactBaseValue(result.Bracket.From.Fraction)
			right, rightOK := exactBaseValue(result.Bracket.To.Fraction)
			if !leftOK || !rightOK || left.Cmp(right) >= 0 {
				result.replay = nil
			} else {
				result.replay.setBracket(left, right)
				result.replay.setRotatingBracketGap(result, new(big.Rat).Add(aPath.fullTravel, bPath.fullTravel))
			}
		}
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
	at := sweepInstant(f, r.a.path.duration)
	var event SweepEvent
	if r.planar {
		event, err = r.planarIdealEvent(ctx, f, at, poseA, poseB, contact)
		if err != nil {
			return nil, err
		}
	} else {
		event = r.orientedIdealEvent(f, at, poseA, poseB, contact)
	}
	sample := &SweepSample{At: at, PoseA: poseA, PoseB: poseB,
		FloatContact: contact, Ideal: event, exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

// separatedIdealGap transfers a float-pose gap to the ideal path: its bound
// grows by both pose deviations, and the result must stay strictly positive.
func separatedIdealGap(contact *ContactReport, etaA, etaB float64) (*Measurement, bool) {
	if contact.Gap == nil {
		return nil, false
	}
	value, okValue := exactBaseValue(contact.Gap.Value)
	bound, okBound := exactBaseValue(contact.Gap.Bound)
	if !okValue || !okBound {
		return nil, false
	}
	charge := new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB))
	newBound := new(big.Rat).Add(bound, charge)
	if new(big.Rat).Sub(value, newBound).Sign() <= 0 {
		return nil, false
	}
	published := proofbound.RatFloatUp(newBound)
	if !finiteMeasurementValues(published) || value.Cmp(proofarith.FloatRat(published)) <= 0 {
		return nil, false
	}
	gap := *contact.Gap
	gap.Bound = units.Millimeters(published)
	gap.Exactness = exactnessFromBound(published)
	return &gap, true
}

// orientedIdealEvent transfers a source-box pair's float-pose relation to the
// ideal path through the rounded source boxes.
func (r *rotationalPairSweep) orientedIdealEvent(f *big.Rat, at SweepInstant,
	poseA, poseB r3.Transform, contact *ContactReport) SweepEvent {
	boxA, etaA, okA := r.a.roundedAt(poseA, f)
	boxB, etaB, okB := r.b.roundedAt(poseB, f)
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	if okA && okB {
		switch contact.Relation {
		case ContactSeparated:
			if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = ContactSeparated, gap, ContactNoReason
			}
		case ContactOverlapping:
			if orientedInteriorWitness(boxA, boxB, proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)) {
				event.Relation, event.Reason = ContactOverlapping, contact.Reason
			}
			if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case ContactTouching:
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = ContactTouching, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if proof, ok := r.horizontalSpinContact(f, poseA, poseB, contact,
				proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)); ok {
				event = proof
			}
		case ContactBand:
			// §10.5: the exact planar path's band of two boxes apart within the
			// SupportBand transfers whole at zero deviation, and otherwise, as
			// planarIdealEvent's does, with both deviations added to its width
			// and its manifold dropped.
			if etaA == 0 && etaB == 0 {
				event.Relation, event.Gap, event.Manifold = ContactBand, contact.Gap, contact.Manifold
				event.Reason = contact.Reason
			} else if gap, ok := widenedBand(contact, etaA, etaB); ok {
				event.Relation, event.Gap, event.Reason = ContactBand, gap, ContactNoNormalProof
			}
		}
	}
	return event
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
		if path.path.drift == nil && path.path.delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.path.drift != nil && path.frame.Axis[0].Sign() == 0 &&
			path.frame.Axis[1].Sign() == 0 && path.frame.Axis[2].Sign() != 0 &&
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
	loA, hiA := startA.lo[2].Rat(), startA.hi[2].Rat()
	loB, hiB := startB.lo[2].Rat(), startB.hi[2].Rat()
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
	bound := proofarith.RationalFloatError(gap, value)
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
		margin := new(big.Rat).Add(deviation, proofarith.FloatRat(witness.Bound.Base()))
		margin.Sub(margin, gap)
		coordinates := [2]float64{witness.Value.X, witness.Value.Y}
		for axis := range 2 {
			value := proofarith.FloatRat(coordinates[axis])
			if value.Cmp(new(big.Rat).Add(base.lo[axis].Rat(), margin)) <= 0 ||
				value.Cmp(new(big.Rat).Sub(base.hi[axis].Rat(), margin)) >= 0 {
				return SweepEvent{}, false
			}
		}
		for _, position := range []*VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), deviation)
			if bound.Cmp(proofarith.FloatRat(r.req.PointResolution.Base())) > 0 {
				return SweepEvent{}, false
			}
			published := proofbound.RatFloatUp(bound)
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
		if !r.planar && r.req.StartPolicy == ContinueCertifiedTouch && r.coMovingOrientedTouch(first) {
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
				pointCount: len(first.Ideal.Manifold.Points),
				features: [2]ContactFeature{first.Ideal.Manifold.Points[0].FeatureA,
					first.Ideal.Manifold.Points[0].FeatureB},
				normal: first.Ideal.Manifold.Points[0].Normal}
			r.report.Outcome = SweepPersistentTouch
			r.sortSamples()
			return r.report, nil
		}
		var departure *big.Rat
		var departs bool
		if r.planar {
			departure, departs, err = r.planarDepartureFraction(ctx)
			if err != nil {
				return nil, err
			}
		} else {
			departure, departs = r.rotationalDepartureFraction(first)
		}
		if departs {
			return r.depart(ctx, departure)
		}
		if r.planar && r.req.StartPolicy == ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
				if track.planar.band == nil {
					r.report.Outcome = SweepPersistentTouch
				}
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
	case ContactBand:
		// §10.4: a positive-displacement pair that starts inside its band is
		// an initial contact. No departure is published, since the true pair
		// may stay inside the band however fast the held pair separates; a
		// certified continuation is §10.3's band track over the held touch.
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		// §10.5: an exact pair apart within its SupportBand has every vertex
		// above the support plane, so it departs when §10.2's proof holds.
		if r.planar && r.req.StartPolicy == ContinueSeparatingTouch && r.a.delta.Sign() == 0 && r.b.delta.Sign() == 0 {
			departure, departs, err := r.planarDepartureFraction(ctx)
			if err != nil {
				return nil, err
			}
			if departs {
				return r.depart(ctx, departure)
			}
		}
		if r.planar && r.req.StartPolicy == ContinueCertifiedTouch {
			track, ok, bandErr := r.planarBand(ctx)
			if bandErr != nil {
				return nil, bandErr
			}
			if ok {
				r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
				r.sortSamples()
				return r.report, nil
			}
		}
		return r.undecided(zero, one, r.continuationCause()), nil
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

// depart publishes a proven departure through the given fraction, then
// continues the clear search from there to the end of the sweep.
func (r *rotationalPairSweep) depart(ctx context.Context, departure *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	left, err := r.sample(ctx, departure)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, departure, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	gap, ok, err := r.departureGap(ctx, departure, left)
	if err != nil {
		return nil, err
	}
	if !ok {
		return r.undecided(zero, departure, SweepDepartureUnproved), nil
	}
	r.report.Departure = &SweepDeparture{Until: left.At, GapAtUntil: *gap}
	if departure.Cmp(one) == 0 {
		r.report.Outcome = SweepDepartedClear
		r.sortSamples()
		return r.report, nil
	}
	last, err := r.sample(ctx, one)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(departure, one, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	done, err := r.refine(ctx, left, last, 0)
	if err != nil {
		return nil, err
	}
	if !done {
		r.report.Outcome = SweepDepartedClear
	}
	r.sortSamples()
	return r.report, nil
}

// departureGap is the proven gap of the sample that ends a departure. Under
// a positive SupportBand (§10.5) a pair apart by at most the band samples as
// ContactBand; its gap is then read again at the same rounded poses with a
// zero band, which publishes the exact relation's gap, and transferred to
// the ideal path as a separated sample's is. ok is false when no positive
// gap is proven.
func (r *rotationalPairSweep) departureGap(ctx context.Context, f *big.Rat, sample *SweepSample) (*Measurement, bool, error) {
	switch {
	case sample.Ideal.Relation == ContactSeparated && sample.Ideal.Gap != nil:
		return sample.Ideal.Gap, true, nil
	case sample.Ideal.Relation != ContactBand:
		return nil, false, nil
	}
	req := r.req.ContactRequest
	req.SupportBand = units.Value{}
	contact, err := r.doc.ContactPair(ctx, r.a.body, r.b.body, sample.PoseA, sample.PoseB, req)
	if err != nil {
		return nil, false, err
	}
	var event SweepEvent
	if r.planar {
		event, err = r.planarIdealEvent(ctx, f, sample.At, sample.PoseA, sample.PoseB, contact)
		if err != nil {
			return nil, false, err
		}
	} else {
		event = r.orientedIdealEvent(f, sample.At, sample.PoseA, sample.PoseB, contact)
	}
	if event.Relation != ContactSeparated || event.Gap == nil {
		return nil, false, nil
	}
	return event.Gap, true, nil
}

// continuationCause names the missing proof after an initial touch.
func (r *rotationalPairSweep) continuationCause() SweepCause {
	if r.req.StartPolicy == ContinueCertifiedTouch {
		return SweepContactTrackUnproved
	}
	return SweepDepartureUnproved
}

// coMovingOrientedTouch admits a touch of two co-translating source boxes
// whose manifold is a box face patch. A manifold the exact planar path
// completed (an edge or vertex on a face, or a support set) is left to the
// planar continuation, since the oriented track rereads face patches only.
func (r *rotationalPairSweep) coMovingOrientedTouch(first *SweepSample) bool {
	if r.a.path.drift != nil || r.b.path.drift != nil || first.Ideal.Manifold == nil {
		return false
	}
	for _, point := range first.Ideal.Manifold.Points {
		if point.FeatureA.Face == nil || point.FeatureB.Face == nil {
			return false
		}
	}
	for axis := range 3 {
		if proofarith.DyCmp(r.a.path.delta[axis], r.b.path.delta[axis]) != 0 {
			return false
		}
	}
	return true
}

func translatedOrientedBox(box orientedSourceBox, delta [3]proofarith.Dyadic, fraction *big.Rat) (orientedSourceBox, bool) {
	for axis := range 3 {
		step, ok := proofarith.DyOfRat(new(big.Rat).Mul(delta[axis].Rat(), fraction))
		if !ok {
			return orientedSourceBox{}, false
		}
		for corner := range box.corner {
			box.corner[corner][axis] = proofarith.DyAdd(box.corner[corner][axis], step)
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
	if fraction, ok := r.tangentAxisSpinDepartureFraction(first); ok {
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
	axis, side, ok := clearance.SignedAxis(point.Normal.Value)
	if !ok || point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return nil, false
	}
	boxA, okA := sourceBoxAtPose(r.a.body, r.a.path.from)
	boxB, okB := sourceBoxAtPose(r.b.body, r.b.path.from)
	if !okA || !okB ||
		(side == 1 && proofarith.DyCmp(boxA.hi[axis], boxB.lo[axis]) != 0) ||
		(side == 0 && proofarith.DyCmp(boxB.hi[axis], boxA.lo[axis]) != 0) {
		return nil, false
	}
	var omega, difference, velocity [3]*big.Rat
	for i := range 3 {
		if r.a.frame.Axis[i].Cmp(r.b.frame.Axis[i]) != 0 {
			return nil, false
		}
		omega[i] = r.a.frame.Axis[i]
		centers := [2]float64{r.a.path.drift.Center.X, r.b.path.drift.Center.X}
		switch i {
		case 1:
			centers = [2]float64{r.a.path.drift.Center.Y, r.b.path.drift.Center.Y}
		case 2:
			centers = [2]float64{r.a.path.drift.Center.Z, r.b.path.drift.Center.Z}
		}
		difference[i] = new(big.Rat).Sub(proofarith.FloatRat(centers[1]), proofarith.FloatRat(centers[0]))
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
		root := proofbound.RatSqrtUp(squared)
		if !finiteMeasurementValues(root) {
			return nil
		}
		return proofarith.FloatRat(root)
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
		normal := proofarith.DvCross(r.a.startBox.edge[i], r.a.startBox.edge[j])
		if proofarith.DvIsZero(normal) {
			continue
		}
		alo, ahi := orientedProjection(r.a.startBox, normal)
		blo, bhi := orientedProjection(r.b.startBox, normal)
		side := 0
		switch {
		case proofarith.DyCmp(ahi, blo) == 0:
			side = 1
		case proofarith.DyCmp(bhi, alo) == 0:
			side = -1
		default:
			continue
		}
		relative := proofarith.DyV3{}
		for k := range 3 {
			relative[k] = proofarith.DySubScalar(r.b.path.delta[k], r.a.path.delta[k])
		}
		slope := proofarith.DvDot(relative, normal)
		if side < 0 {
			slope = proofarith.DyNeg(slope)
		}
		if slope.Sign() > 0 {
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
		if path.path.drift == nil && path.path.delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.path.drift != nil && path.frame.Axis[0].Sign() == 0 &&
			path.frame.Axis[1].Sign() == 0 && path.frame.Axis[2].Sign() != 0 &&
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
	z := proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)}
	alo, ahi := orientedProjection(a, z)
	blo, bhi := orientedProjection(b, z)
	gap := proofarith.DySubScalar(blo, ahi)
	if normal.Z < 0 {
		gap = proofarith.DySubScalar(alo, bhi)
	}
	if gap.Sign() != 0 {
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

// A stationary horizontal source face and a source box spinning about Y
// separate when every moving corner has a positive outward derivative. The
// second derivative of each corner's height is bounded by
// omega²*(|corner.X-center.X|+|corner.Z-center.Z|). Taylor's theorem then
// proves one positive support gap for the complete bodies through the chosen
// duration, including every time immediately after zero.
func (r *rotationalPairSweep) tangentAxisSpinDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 4 {
		return nil, false
	}
	// Read the prepared paths through pointers so the duration and exact
	// corner storage stay in their original records.
	paths := [2]*rotationalSweepPath{&r.a, &r.b}
	stationary, spinning := -1, -1
	for i, path := range paths {
		if path.path.drift == nil && path.path.delta == [3]proofarith.Dyadic{} {
			stationary = i
		}
		if path.path.drift != nil && path.path.screw == nil &&
			path.frame.Axis[0].Sign() == 0 && path.frame.Axis[1].Sign() != 0 &&
			path.frame.Axis[2].Sign() == 0 {
			spinning = i
		}
	}
	if stationary < 0 || spinning < 0 || stationary == spinning {
		return nil, false
	}
	static, moving := &paths[stationary].startBox, &paths[spinning].startBox
	staticLow, staticHigh := static.corner[0][2], static.corner[0][2]
	movingLow, movingHigh := moving.corner[0][2], moving.corner[0][2]
	for i := 1; i < len(static.corner); i++ {
		staticLow = dyMin(staticLow, static.corner[i][2])
		staticHigh = dyMax(staticHigh, static.corner[i][2])
		movingLow = dyMin(movingLow, moving.corner[i][2])
		movingHigh = dyMax(movingHigh, moving.corner[i][2])
	}
	sign := int64(0)
	if proofarith.DyCmp(staticHigh, movingLow) == 0 {
		sign = 1
	} else if proofarith.DyCmp(movingHigh, staticLow) == 0 {
		sign = -1
	}
	if sign == 0 {
		return nil, false
	}
	wantNormal := float64(sign)
	if spinning == 0 {
		wantNormal = -wantNormal
	}
	for _, point := range first.Ideal.Manifold.Points {
		if point.Normal.Value != (r3.Vec{Z: wantNormal}) ||
			point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
			return nil, false
		}
	}
	path := paths[spinning]
	omega := path.frame.Axis[1]
	omegaSquared := new(big.Rat).Mul(omega, omega)
	minimum, curvature := new(big.Rat), new(big.Rat)
	for i := range moving.corner {
		corner := &moving.corner[i]
		dx := new(big.Rat).Sub(corner[0].Rat(), path.frame.Center[0])
		dz := new(big.Rat).Sub(corner[2].Rat(), path.frame.Center[2])
		derivative := new(big.Rat).Sub(path.velocity[2], new(big.Rat).Mul(omega, dx))
		derivative.Mul(derivative, big.NewRat(sign, 1))
		if i == 0 || derivative.Cmp(minimum) < 0 {
			minimum = derivative
		}
		cornerCurvature := new(big.Rat).Mul(omegaSquared,
			new(big.Rat).Add(new(big.Rat).Abs(dx), new(big.Rat).Abs(dz)))
		if cornerCurvature.Cmp(curvature) > 0 {
			curvature = cornerCurvature
		}
	}
	if minimum.Sign() <= 0 {
		return nil, false
	}
	fraction := big.NewRat(1, 1)
	for range 60 {
		until := new(big.Rat).Mul(fraction, r.a.path.duration)
		if new(big.Rat).Mul(curvature, until).Cmp(minimum) < 0 {
			return fraction, true
		}
		fraction.Quo(fraction, big.NewRat(2, 1))
	}
	return nil, false
}

// A rotation about an axis-normal contact face does not move either support
// plane along that axis. A strictly increasing relative plane gap certifies
// immediate departure even when only one body rotates.
func (r *rotationalPairSweep) axisFaceDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	point := first.Ideal.Manifold.Points[0]
	axis, side, ok := clearance.SignedAxis(point.Normal.Value)
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
		proofarith.DyCmp(faceA.origin[axis], faceB.origin[axis]) != 0 {
		return nil, false
	}
	velocity := func(path rotationalSweepPath) (*big.Rat, bool) {
		if path.path.drift == nil {
			return new(big.Rat).Quo(path.path.delta[axis].Rat(), path.path.duration), true
		}
		for other := range 3 {
			if other != axis && path.frame.Axis[other].Sign() != 0 {
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
	lf, rf := proofarith.FloatRat(left.At.Fraction.Base()), proofarith.FloatRat(right.At.Fraction.Base())
	if left.Ideal.Relation == ContactSeparated && right.Ideal.Relation == ContactSeparated &&
		r.intervalClear(left, right, lf, rf) {
		return false, nil
	}
	width := new(big.Rat).Mul(new(big.Rat).Sub(rf, lf), r.a.path.duration)
	if width.Cmp(r.resolution) <= 0 {
		if left.Ideal.Relation == ContactSeparated && meetingRelation(right.Ideal.Relation) {
			left, right, err := r.narrowBracket(ctx, left, right)
			if err != nil {
				return false, err
			}
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

// meetingRelation reports whether a sample's ideal relation closes an impact
// bracket on its right.
func meetingRelation(relation ContactRelation) bool {
	return relation == ContactTouching || relation == ContactOverlapping || relation == ContactBand
}

// narrowBracket halves an impact bracket already within TimeResolution while
// its right edge would not replay (bracketRightReplays). Replay inside a
// rotating bracket charges every point's travel from the left edge against
// that edge's lower gap, so a fast body's bracket can need a width below
// TimeResolution before its right edge, where a step cuts the impact, fits
// PointResolution. A separated midpoint the clear certificate joins to the
// left edge becomes the left edge, and a meeting midpoint the right edge. The
// current bracket stands at the float floor, at the pose budget, and at a
// midpoint that settles neither, so narrowing never turns a bracket into an
// undecided report.
func (r *rotationalPairSweep) narrowBracket(ctx context.Context, left, right *SweepSample) (
	*SweepSample, *SweepSample, error) {
	for !r.bracketRightReplays(left, right) {
		lf, rf := proofarith.FloatRat(left.At.Fraction.Base()), proofarith.FloatRat(right.At.Fraction.Base())
		middle := new(big.Rat).Quo(new(big.Rat).Add(lf, rf), big.NewRat(2, 1))
		if ratFloatNearest(middle) == left.At.Fraction.Base() ||
			ratFloatNearest(middle) == right.At.Fraction.Base() {
			return left, right, nil
		}
		sample, err := r.sample(ctx, middle)
		if errors.Is(err, errSweepPoseBudget) {
			return left, right, nil
		}
		if err != nil {
			return nil, nil, err
		}
		switch {
		case meetingRelation(sample.Ideal.Relation):
			right = sample
		case sample.Ideal.Relation == ContactSeparated && r.intervalClear(left, sample, lf, middle):
			left = sample
		default:
			return left, right, nil
		}
	}
	return left, right, nil
}

// bracketRightReplays reports whether replay accepts the bracket's right
// edge: (hi − lo)·T − g plus the rounded poses' deviation there fits
// PointResolution (sweepReplayProof.bracketDepthWithin), with g the left
// sample's proven lower gap and T both travel bounds per unit fraction. The
// deviation is the one replay charges, read at the same rounded poses. A left
// sample without a positive gap, or a right edge whose deviation has no finite
// bound, has nothing narrowing can settle and reports true.
func (r *rotationalPairSweep) bracketRightReplays(left, right *SweepSample) bool {
	gap := sampleLowerGap(left)
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if gap == nil || !ok {
		return true
	}
	lo, hi := proofarith.FloatRat(left.At.Fraction.Base()), proofarith.FloatRat(right.At.Fraction.Base())
	charge := new(big.Rat)
	for _, path := range [2]rotationalSweepPath{r.a, r.b} {
		pose, err := path.poseAt(hi)
		if err != nil {
			return true
		}
		deviation, displacement, ok := path.replayDeviation(pose, hi)
		if !ok {
			return true
		}
		charge.Add(charge, deviation)
		charge.Add(charge, displacement)
	}
	bracket := sweepReplayProof{bracketLo: lo, bracketHi: hi, bracketGap: gap,
		bracketTravel: new(big.Rat).Add(r.a.fullTravel, r.b.fullTravel)}
	return bracket.bracketDepthWithin(hi, charge, resolution)
}

func (r *rotationalPairSweep) intervalClear(left, right *SweepSample, lf, rf *big.Rat) bool {
	if !r.planar && r.obliqueAffineIntervalClear(lf, rf) {
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
		normal := proofarith.DvCross(r.a.startBox.edge[i], r.a.startBox.edge[j])
		if proofarith.DvIsZero(normal) {
			continue
		}
		alo0, ahi0 := orientedProjection(a0, normal)
		blo0, bhi0 := orientedProjection(b0, normal)
		alo1, ahi1 := orientedProjection(a1, normal)
		blo1, bhi1 := orientedProjection(b1, normal)
		if proofarith.DyCmp(ahi0, blo0) < 0 && proofarith.DyCmp(ahi1, blo1) < 0 ||
			proofarith.DyCmp(bhi0, alo0) < 0 && proofarith.DyCmp(bhi1, alo1) < 0 {
			return true
		}
	}
	return false
}

// intervalAxisSeparated encloses every ideal source-point path over the whole
// fraction span. Each body lies in the hull of its source points, so strict
// separation of the two coordinate hulls proves clear.
func (r *rotationalPairSweep) intervalAxisSeparated(from, to *big.Rat) bool {
	return r.intervalAxisGap(from, to) != nil
}

// intervalAxisGap is the widest strict gap between the two coordinate hulls
// over the span, less both held displacements (§10.4), a lower bound on the
// pair's separation throughout it, or nil when no such gap stays positive.
func (r *rotationalPairSweep) intervalAxisGap(from, to *big.Rat) *big.Rat {
	a := r.a.cornerSpan(from, to)
	b := r.b.cornerSpan(from, to)
	if a.len() == 0 || b.len() == 0 {
		return nil
	}
	displacement := proofarith.DyAdd(r.a.delta, r.b.delta).Rat()
	var widest *big.Rat
	for axis := range 3 {
		aLow, aHigh := a.hull(axis)
		bLow, bHigh := b.hull(axis)
		for _, gap := range []*big.Rat{new(big.Rat).Sub(bLow, aHigh), new(big.Rat).Sub(aLow, bHigh)} {
			gap.Sub(gap, displacement)
			if gap.Sign() > 0 && (widest == nil || gap.Cmp(widest) > 0) {
				widest = gap
			}
		}
	}
	return widest
}

// cornerSpans is cornerSpan's enclosure of every source point's ideal path,
// one coordinate interval per point and axis. Each axis's endpoints are held
// as integer numerators over that axis's one shared positive denominator (the
// common-denominator form beside motionbound.ScaledIvMat); hull and span convert them back
// to the exact rationals.
type cornerSpans struct {
	den    [3]*big.Int
	lo, hi [][3]*big.Int
}

func (c cornerSpans) len() int { return len(c.lo) }

// hull is the axis's coordinate hull over every point. Endpoints over one
// denominator compare as their numerators do.
func (c cornerSpans) hull(axis int) (*big.Rat, *big.Rat) {
	low, high := c.lo[0][axis], c.hi[0][axis]
	for index := 1; index < len(c.lo); index++ {
		if c.lo[index][axis].Cmp(low) < 0 {
			low = c.lo[index][axis]
		}
		if c.hi[index][axis].Cmp(high) > 0 {
			high = c.hi[index][axis]
		}
	}
	return new(big.Rat).SetFrac(low, c.den[axis]), new(big.Rat).SetFrac(high, c.den[axis])
}

// span is one point's coordinate interval on one axis.
func (c cornerSpans) span(index, axis int) proofbound.RatInterval {
	return proofbound.IntervalOwned(new(big.Rat).SetFrac(c.lo[index][axis], c.den[axis]),
		new(big.Rat).SetFrac(c.hi[index][axis], c.den[axis]))
}

// cornerSpan encloses each source point's ideal path over the fraction span
// [from, to]. An affine path moves a point by delta·f. A rotating one takes
// the point at the span's mid time, rotationMid·(p − c) + c + v·t_mid, grown
// on each axis by its largest speed over the span, |v + ω × rotationSpan·(p −
// c)| per axis, times the half span: the per-point arithmetic runs in the
// common-denominator form, so every endpoint is the exact rational the
// interval expression denotes.
func (p rotationalSweepPath) cornerSpan(from, to *big.Rat) cornerSpans {
	if !p.memo.open() {
		return p.cornerSpanAfresh(from, to)
	}
	key := spanKey(p.startPoints, from, to)
	if spans, ok := p.memo.loadSpans(key); ok {
		return spans
	}
	spans := p.cornerSpanAfresh(from, to)
	p.memo.storeSpans(key, spans)
	return spans
}

// cornerSpanAfresh is cornerSpan computed without the memo.
func (p rotationalSweepPath) cornerSpanAfresh(from, to *big.Rat) cornerSpans {
	count := len(p.startPoints)
	output := cornerSpans{lo: make([][3]*big.Int, count), hi: make([][3]*big.Int, count)}
	if p.path.drift == nil {
		starts := make([]*big.Rat, count)
		for axis := range 3 {
			lo := new(big.Rat).Mul(p.path.delta[axis].Rat(), from)
			hi := new(big.Rat).Mul(p.path.delta[axis].Rat(), to)
			if lo.Cmp(hi) > 0 {
				lo, hi = hi, lo
			}
			for index, corner := range p.startPoints {
				starts[index] = corner[axis].Rat()
			}
			den := proofarith.LcmInt(proofarith.LcmInt(proofarith.CommonDenom(starts...), lo.Denom()), hi.Denom())
			output.den[axis] = den
			loN, hiN := proofarith.ScaledNum(lo, den), proofarith.ScaledNum(hi, den)
			for index, start := range starts {
				startN := proofarith.ScaledNum(start, den)
				output.lo[index][axis] = new(big.Int).Add(startN, loN)
				output.hi[index][axis] = startN.Add(startN, hiN)
			}
		}
		return output
	}
	lowTime := new(big.Rat).Mul(p.path.duration, from)
	highTime := new(big.Rat).Mul(p.path.duration, to)
	lowAngle := new(big.Rat).Mul(p.omegaLow, lowTime)
	highAngle := new(big.Rat).Mul(p.omegaHigh, highTime)
	sin, cos := rotationalSinCosSpan(lowAngle, highAngle)
	rotationSpan := p.frame.ScaledRotation(sin, cos)
	midTime := new(big.Rat).Quo(new(big.Rat).Add(lowTime, highTime), big.NewRat(2, 1))
	angleAtMidLow := new(big.Rat).Mul(p.omegaLow, midTime)
	angleAtMidHigh := new(big.Rat).Mul(p.omegaHigh, midTime)
	midSin, midCos := rotationalSinCosSpan(angleAtMidLow, angleAtMidHigh)
	rotationMid := p.frame.ScaledRotation(midSin, midCos)
	halfDuration := new(big.Rat).Quo(new(big.Rat).Sub(highTime, lowTime), big.NewRat(2, 1))

	// Every point's offset p − c from the pivot, over one shared q.
	relative := make([]*big.Rat, 0, 3*count)
	for _, corner := range p.startPoints {
		for axis := range 3 {
			relative = append(relative, new(big.Rat).Sub(corner[axis].Rat(), p.frame.Center[axis]))
		}
	}
	q := proofarith.CommonDenom(relative...)
	spanLo, spanHi := make([][3]*big.Int, count), make([][3]*big.Int, count)
	midLo, midHi := make([][3]*big.Int, count), make([][3]*big.Int, count)
	for index := range count {
		var offset [3]*big.Int
		for axis := range 3 {
			offset[axis] = proofarith.ScaledNum(relative[3*index+axis], q)
		}
		spanLo[index], spanHi[index] = rotationSpan.ApplyScaled(offset)
		midLo[index], midHi[index] = rotationMid.ApplyScaled(offset)
	}
	spanDen := new(big.Int).Mul(rotationSpan.Den, q)
	midDen := new(big.Int).Mul(rotationMid.Den, q)
	for axis := range 3 {
		following, preceding := (axis+1)%3, (axis+2)%3
		velocity := p.velocity[axis]
		// The derivative v + rotationSpan·(p − c)[preceding]·ω[following] −
		// rotationSpan·(p − c)[following]·ω[preceding] over derivativeDen; each
		// scale carries its axis component's sign, which decides the endpoint
		// order exactly as proofbound.IntervalScale's does.
		scaleF, scaleP := p.frame.Axis[following], p.frame.Axis[preceding]
		scaleDen := proofarith.LcmInt(new(big.Int).Set(scaleF.Denom()), scaleP.Denom())
		derivativeDen := proofarith.LcmInt(new(big.Int).Mul(spanDen, scaleDen), velocity.Denom())
		multiplierF := new(big.Int).Quo(derivativeDen, new(big.Int).Mul(spanDen, scaleF.Denom()))
		multiplierF.Mul(multiplierF, scaleF.Num())
		multiplierP := new(big.Int).Quo(derivativeDen, new(big.Int).Mul(spanDen, scaleP.Denom()))
		multiplierP.Mul(multiplierP, scaleP.Num())
		velocityN := proofarith.ScaledNum(velocity, derivativeDen)
		// The midpoint rotationMid·(p − c) + (c + v·t_mid), less and plus the
		// travel max(|derivative|)·halfDuration, over den.
		shift := new(big.Rat).Add(p.frame.Center[axis], new(big.Rat).Mul(velocity, midTime))
		travelDen := new(big.Int).Mul(derivativeDen, halfDuration.Denom())
		den := proofarith.LcmInt(proofarith.LcmInt(midDen, shift.Denom()), travelDen)
		output.den[axis] = den
		midMultiplier := new(big.Int).Quo(den, midDen)
		travelMultiplier := new(big.Int).Quo(den, travelDen)
		travelMultiplier.Mul(travelMultiplier, halfDuration.Num())
		shiftN := proofarith.ScaledNum(shift, den)
		for index := range count {
			loF := new(big.Int).Mul(spanLo[index][preceding], multiplierF)
			hiF := new(big.Int).Mul(spanHi[index][preceding], multiplierF)
			if multiplierF.Sign() < 0 {
				loF, hiF = hiF, loF
			}
			loP := new(big.Int).Mul(spanLo[index][following], multiplierP)
			hiP := new(big.Int).Mul(spanHi[index][following], multiplierP)
			if multiplierP.Sign() < 0 {
				loP, hiP = hiP, loP
			}
			low := loF.Sub(loF, hiP)
			low.Add(low, velocityN)
			high := hiF.Sub(hiF, loP)
			high.Add(high, velocityN)
			maximum := low.Abs(low)
			if high.Abs(high).Cmp(maximum) > 0 {
				maximum = high
			}
			travel := maximum.Mul(maximum, travelMultiplier)
			outLo := new(big.Int).Mul(midLo[index][axis], midMultiplier)
			outLo.Add(outLo, shiftN)
			output.lo[index][axis] = outLo.Sub(outLo, travel)
			outHi := new(big.Int).Mul(midHi[index][axis], midMultiplier)
			outHi.Add(outHi, shiftN)
			output.hi[index][axis] = outHi.Add(outHi, travel)
		}
	}
	return output
}

func rotationalSinCosSpan(low, high *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	loSin, loCos := motionbound.RadianSinCos(low)
	// A path whose rate bounds agree (a screw's exact rate) reads its mid
	// span at one angle; the pair read at low is the pair at high.
	hiSin, hiCos := loSin, loCos
	if low.Cmp(high) != 0 {
		hiSin, hiCos = motionbound.RadianSinCos(high)
	}
	if low.Sign() >= 0 && high.Cmp(proofbound.HalfPiInterval().Lo) <= 0 {
		return proofbound.Interval(loSin.Lo, hiSin.Hi), proofbound.Interval(hiCos.Lo, loCos.Hi)
	}
	width := new(big.Rat).Sub(high, low)
	return proofbound.IntervalOwned(new(big.Rat).Sub(loSin.Lo, width),
			new(big.Rat).Add(loSin.Hi, width)),
		proofbound.IntervalOwned(new(big.Rat).Sub(loCos.Lo, width), new(big.Rat).Add(loCos.Hi, width))
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
