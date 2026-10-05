package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sweepReplayProof keeps the source geometry used by the continuous sweep. A
// replayed float pose is checked against it, rather than inferred from the
// sweep's endpoint samples.
type sweepReplayProof struct {
	pa, pb                 affinePairPath
	boxA, boxB             sourceBoxContactProof
	sphere                 *sourceSphereContactProof
	spherePair             *[2]sourceSphereContactProof
	cylinder               *sourceCylinderContactProof
	cylinderFirst          bool
	clearAxis, clearSign   int
	clearGap               *big.Rat
	sphereFirst            bool
	sphereAxis, sphereSide int
	sphereGap, sphereSlope dyadic
	rotation               *[2]rotationalSweepPath
	track                  *SweepContactTrack
	request                ContactRequest
	outcome                SweepOutcome
	bracketLo              *big.Rat
	bracketHi              *big.Rat
}

func (p *sweepReplayProof) snapshot(r *SweepReport) {
	p.outcome = r.Outcome
}

func (p *sweepReplayProof) setBracket(left, right *big.Rat) {
	p.bracketLo = new(big.Rat).Set(left)
	p.bracketHi = new(big.Rat).Set(right)
}

// HasAffineReplayProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, source-cylinder clear, or oriented
// face-track path.
func (r *SweepReport) HasAffineReplayProof() bool {
	if r == nil || r.replay == nil {
		return false
	}
	if r.replay.rotation == nil {
		switch r.replay.outcome {
		case SweepClear, SweepDepartedClear, SweepPersistentTouch,
			SweepImpactBracket, SweepContactTransitionBracket:
			return true
		default:
			return false
		}
	}
	return r.replay.outcome == SweepPersistentTouch && r.replay.track != nil &&
		r.replay.rotation[0].path.drift == nil && r.replay.rotation[1].path.drift == nil
}

// CertifiedPosesAt evaluates the recorded paths at elapsed time and checks
// their rounded placements against the sweep's exact source proof.
// It reads no Document geometry and performs no new contact query. A pose whose
// rounding can change the reported relation beyond PointResolution is refused.
func (r *SweepReport) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || r.replay == nil || elapsed.Kind() != units.Time ||
		!finiteMeasurementValues(elapsed.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", ErrUnsupported)
	}
	t, ok := exactBaseValue(elapsed)
	duration := r.replay.pa.duration
	if r.replay.rotation != nil {
		duration = r.replay.rotation[0].path.duration
	}
	if !ok || t.Sign() < 0 || t.Cmp(duration) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the sweep", ErrDegenerate)
	}
	return r.certifiedPosesAtFraction(new(big.Rat).Quo(t, duration))
}

// CertifiedPosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path. It permits a caller to replay a slice whose global
// clock endpoints differ slightly from the rounded sweep duration.
func (r *SweepReport) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || r.replay == nil || time.Kind() != units.Time || start.Kind() != units.Time ||
		end.Kind() != units.Time || !finiteMeasurementValues(time.Base(), start.Base(), end.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", ErrUnsupported)
	}
	timeValue, timeOK := exactBaseValue(time)
	startValue, startOK := exactBaseValue(start)
	endValue, endOK := exactBaseValue(end)
	if !timeOK || !startOK || !endOK {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is not finite", ErrDegenerate)
	}
	span := new(big.Rat).Sub(endValue, startValue)
	if span.Sign() <= 0 || timeValue.Cmp(startValue) < 0 || timeValue.Cmp(endValue) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the interval", ErrDegenerate)
	}
	f := new(big.Rat).Quo(new(big.Rat).Sub(timeValue, startValue), span)
	return r.certifiedPosesAtFraction(f)
}

func (r *SweepReport) certifiedPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	if !r.replayFractionCovered(f) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the certified sweep prefix", ErrUnsupported)
	}
	if r.replay.rotation != nil {
		return r.certifiedRotationalPosesAtFraction(f)
	}
	poseA, err := r.replay.pa.poseAt(f)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	poseB, err := r.replay.pb.poseAt(f)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	if r.replay.sphere != nil {
		return r.certifiedSpherePosesAtFraction(f, poseA, poseB)
	}
	if r.replay.cylinder != nil {
		return r.certifiedCylinderPosesAtFraction(f, poseA, poseB)
	}
	if r.replay.spherePair != nil {
		return r.certifiedSpherePairPosesAtFraction(f, poseA, poseB)
	}
	actualA, ok := translatedReplayBox(r.replay.boxA, r.replay.pa.from, poseA)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose A is not an affine translation", ErrUnsupported)
	}
	actualB, ok := translatedReplayBox(r.replay.boxB, r.replay.pb.from, poseB)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose B is not an affine translation", ErrUnsupported)
	}
	deviation := boxPoseDeviation(r.replay.boxA, actualA, r.replay.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(r.replay.boxB, actualB, r.replay.pb.delta, f))
	resolution, ok := exactBaseValue(r.replay.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	contact := &ContactReport{Request: r.replay.request}
	classifySourceBoxes(contact, actualA, actualB)
	if !r.replayRelationCovered(f, contact.Relation, actualA, actualB, resolution) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (r *SweepReport) certifiedCylinderPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	p := r.replay
	actualA, okA := translatedReplayBox(p.boxA, p.pa.from, poseA)
	actualB, okB := translatedReplayBox(p.boxB, p.pb.from, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay pose is not affine", ErrUnsupported)
	}
	resolution, ok := exactBaseValue(p.request.PointResolution)
	if !ok || p.clearGap == nil || p.clearGap.Cmp(resolution) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay has no clear margin", ErrUnsupported)
	}
	deviation := boxPoseDeviation(p.boxA, actualA, p.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(p.boxB, actualB, p.pb.delta, f))
	actualCylinder, actualBox := actualB, actualA
	if p.cylinderFirst {
		actualCylinder, actualBox = actualA, actualB
	}
	if deviation.Cmp(resolution) > 0 ||
		!cylinderInsideBoxFace(actualCylinder, actualBox, p.clearAxis) ||
		!outerBoxGapExceeds(actualA, actualB, p.clearAxis, p.clearSign, deviation) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its separated outer boxes", ErrUnsupported)
	}
	return poseA, poseB, nil
}

// The sphere-pair producer proves the ideal center-distance quadratic over
// its full certified interval. Replay checks the two rounded source centers
// against that same exact path and retains the producer's relation gate.
func (r *SweepReport) certifiedSpherePairPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	p := r.replay
	start := p.spherePair
	actualA, okA := translatedReplaySphere(start[0], p.pa.from, poseA)
	actualB, okB := translatedReplaySphere(start[1], p.pb.from, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay pose is not affine", ErrUnsupported)
	}
	idealA := spherePairIdealCenter(start[0], p.pa.delta, f)
	idealB := spherePairIdealCenter(start[1], p.pb.delta, f)
	resolution, ok := exactBaseValue(p.request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay resolution is invalid", ErrUnsupported)
	}
	deviation := new(big.Rat)
	for _, pair := range []struct {
		ideal    [3]*big.Rat
		observed sourceSphereContactProof
	}{{idealA, actualA}, {idealB, actualB}} {
		for axis := range 3 {
			difference := new(big.Rat).Sub(pair.observed.center[axis].rat(), pair.ideal[axis])
			deviation.Add(deviation, difference.Abs(difference))
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair pose exceeds point resolution", ErrUnsupported)
	}
	idealDistance2 := spherePairCenterDistance2(idealA, idealB)
	actualDistance2 := spherePairCenterDistance2(
		spherePairHeldCenter(actualA), spherePairHeldCenter(actualB))
	radius := dyAdd(start[0].radius, start[1].radius).rat()
	radius2 := new(big.Rat).Mul(radius, radius)
	idealRelation := idealDistance2.Cmp(radius2)
	actualRelation := actualDistance2.Cmp(radius2)
	clearMargin := new(big.Rat).Add(radius, deviation)
	clearMargin.Mul(clearMargin, clearMargin)
	strictClear := actualDistance2.Cmp(clearMargin) > 0
	switch p.outcome {
	case SweepClear:
		if idealRelation <= 0 || !strictClear {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair clear path loses its gap", ErrUnsupported)
		}
	case SweepDepartedClear:
		if f.Sign() == 0 {
			if idealRelation != 0 || actualRelation != 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair departure start is not touching", ErrUnsupported)
			}
		} else if idealRelation <= 0 || !strictClear {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair departure loses its gap", ErrUnsupported)
		}
	case SweepImpactBracket:
		if p.bracketLo == nil || p.bracketHi == nil {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair impact lacks its exact bracket", ErrUnsupported)
		}
		if f.Cmp(p.bracketLo) < 0 && (idealRelation <= 0 || !strictClear) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair impact prefix is not clear", ErrUnsupported)
		}
		if f.Cmp(p.bracketHi) == 0 && (idealRelation > 0 || actualDistance2.Cmp(clearMargin) > 0) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair impact right pose is not near contact", ErrUnsupported)
		}
	default:
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair outcome has no replay proof", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func spherePairIdealCenter(s sourceSphereContactProof, delta [3]dyadic, f *big.Rat) [3]*big.Rat {
	center := spherePairHeldCenter(s)
	for axis := range 3 {
		center[axis].Add(center[axis], new(big.Rat).Mul(delta[axis].rat(), f))
	}
	return center
}

func spherePairHeldCenter(s sourceSphereContactProof) [3]*big.Rat {
	var center [3]*big.Rat
	for axis := range 3 {
		center[axis] = s.center[axis].rat()
	}
	return center
}

func spherePairCenterDistance2(a, b [3]*big.Rat) *big.Rat {
	squared := new(big.Rat)
	for axis := range 3 {
		delta := new(big.Rat).Sub(b[axis], a[axis])
		squared.Add(squared, new(big.Rat).Mul(delta, delta))
	}
	return squared
}

// A clear rotating sweep certifies the ideal source boxes over every fraction.
// Replay only accepts the rounded poses when their exact held source corners
// remain separated after charging the producer's pose error bound.
func (r *SweepReport) certifiedRotationalPosesAtFraction(f *big.Rat) (
	r3.Transform, r3.Transform, error) {
	paths := r.replay.rotation
	var pose [2]r3.Transform
	var box [2]orientedSourceBox
	var deviation [2]float64
	for i := range paths {
		path := paths[i]
		var err error
		pose[i], err = path.poseAt(f)
		if err != nil {
			return r3.Transform{}, r3.Transform{}, err
		}
		var ok bool
		box[i], deviation[i], ok = path.roundedAt(pose[i], f)
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay pose has no finite error bound", ErrUnsupported)
		}
	}
	resolution, ok := exactBaseValue(r.replay.request.PointResolution)
	if !ok || new(big.Rat).Add(floatRat(deviation[0]), floatRat(deviation[1])).Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose exceeds point resolution", ErrUnsupported)
	}
	if r.replay.outcome == SweepPersistentTouch {
		if !r.certifiedOrientedTouchAtFraction(f, box[0], box[1]) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose loses the certified face track", ErrUnsupported)
		}
		return pose[0], pose[1], nil
	}
	relation, gap, normSquared := orientedBoxRelation(box[0], box[1])
	if relation != ContactSeparated {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose is not separated", ErrUnsupported)
	}
	norm := ratSqrtUp(normSquared.rat())
	if !finiteMeasurementValues(norm) || norm <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay gap has no finite bound", ErrUnsupported)
	}
	lower := new(big.Rat).Quo(gap.rat(), floatRat(norm))
	if lower.Cmp(new(big.Rat).Add(floatRat(deviation[0]), floatRat(deviation[1]))) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay gap does not exceed pose error", ErrUnsupported)
	}
	return pose[0], pose[1], nil
}

// The producer proves the ideal face track for every fraction. Replay checks
// the rounded source boxes against that same face pair and its point bounds.
func (r *SweepReport) certifiedOrientedTouchAtFraction(f *big.Rat, a, b orientedSourceBox) bool {
	track := r.replay.track
	if track == nil || track.orientedA == nil || track.orientedB == nil ||
		f.Cmp(track.start) < 0 || f.Cmp(track.end) > 0 ||
		r.replay.rotation[0].path.drift != nil || r.replay.rotation[1].path.drift != nil {
		return false
	}
	report := &ContactReport{Request: r.replay.request}
	report.Relation, _, _ = orientedBoxRelation(a, b)
	if report.Relation == ContactTouching {
		publishOrientedBoxPatch(report, a, b)
		if report.Manifold == nil {
			publishClippedHorizontalPatch(report, a, b)
		}
	}
	if report.Relation != ContactTouching || report.Manifold == nil ||
		len(report.Manifold.Points) != track.pointCount {
		return false
	}
	for _, point := range report.Manifold.Points {
		if point.FeatureA != track.features[0] || point.FeatureB != track.features[1] ||
			point.Normal.Value != track.normal.Value ||
			point.OnA.Bound.Base() > track.request.PointResolution.Base() ||
			point.OnB.Bound.Base() > track.request.PointResolution.Base() ||
			point.NormalAngle.Base() > track.request.NormalResolution.Base() {
			return false
		}
	}
	return true
}

func translatedReplayBox(box sourceBoxContactProof, from, at r3.Transform) (sourceBoxContactProof, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return sourceBoxContactProof{}, false
	}
	start, end := from.Translation(), at.Translation()
	before := [3]float64{start.X, start.Y, start.Z}
	after := [3]float64{end.X, end.Y, end.Z}
	for i := range 3 {
		if !finiteMeasurementValues(before[i], after[i]) {
			return sourceBoxContactProof{}, false
		}
		move := dySubScalar(mustDyOf(after[i]), mustDyOf(before[i]))
		box.lo[i], box.hi[i] = dyAdd(box.lo[i], move), dyAdd(box.hi[i], move)
	}
	return box, true
}

func translatedReplaySphere(sphere sourceSphereContactProof, from, at r3.Transform) (sourceSphereContactProof, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return sourceSphereContactProof{}, false
	}
	start, end := from.Translation(), at.Translation()
	before := [3]float64{start.X, start.Y, start.Z}
	after := [3]float64{end.X, end.Y, end.Z}
	for i := range 3 {
		if !finiteMeasurementValues(before[i], after[i]) {
			return sourceSphereContactProof{}, false
		}
		move := dySubScalar(mustDyOf(after[i]), mustDyOf(before[i]))
		sphere.center[i] = dyAdd(sphere.center[i], move)
	}
	return sphere, true
}

// The sphere producer proves one face corridor and an affine support gap.
// Replay checks the rounded placements against both claims at the requested
// fraction, including fractions that are not dyadic.
func (r *SweepReport) certifiedSpherePosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	p := r.replay
	spherePath, boxPath := p.pb, p.pa
	spherePose, boxPose := poseB, poseA
	box := p.boxA
	if p.sphereFirst {
		spherePath, boxPath = p.pa, p.pb
		spherePose, boxPose = poseA, poseB
		box = p.boxB
	}
	sphere, okSphere := translatedReplaySphere(*p.sphere, spherePath.from, spherePose)
	observedBox, okBox := translatedReplayBox(box, boxPath.from, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose is not an affine translation", ErrUnsupported)
	}
	resolution, ok := exactBaseValue(p.request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay point resolution is invalid", ErrUnsupported)
	}
	deviation := boxPoseDeviation(box, observedBox, boxPath.delta, f)
	for i := range 3 {
		ideal := new(big.Rat).Add(p.sphere.center[i].rat(), new(big.Rat).Mul(spherePath.delta[i].rat(), f))
		difference := new(big.Rat).Sub(sphere.center[i].rat(), ideal)
		deviation.Add(deviation, difference.Abs(difference))
		if i == p.sphereAxis {
			continue
		}
		if dyCmp(dySubScalar(sphere.center[i], sphere.radius), observedBox.lo[i]) <= 0 ||
			dyCmp(dyAdd(sphere.center[i], sphere.radius), observedBox.hi[i]) >= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere leaves the certified face corridor", ErrUnsupported)
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	var faceDistance, oppositeDistance dyadic
	if p.sphereSide == 1 {
		faceDistance = dySubScalar(sphere.center[p.sphereAxis], observedBox.hi[p.sphereAxis])
		oppositeDistance = dySubScalar(sphere.center[p.sphereAxis], observedBox.lo[p.sphereAxis])
	} else {
		faceDistance = dySubScalar(observedBox.lo[p.sphereAxis], sphere.center[p.sphereAxis])
		oppositeDistance = dySubScalar(observedBox.hi[p.sphereAxis], sphere.center[p.sphereAxis])
	}
	if faceDistance.sign() <= 0 || dyCmp(oppositeDistance, sphere.radius) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified face", ErrUnsupported)
	}
	observedGap := dySubScalar(faceDistance, sphere.radius).rat()
	idealGap := new(big.Rat).Add(p.sphereGap.rat(), new(big.Rat).Mul(p.sphereSlope.rat(), f))
	difference := new(big.Rat).Sub(observedGap, idealGap)
	if difference.Abs(difference).Cmp(resolution) > 0 || !p.sphereRelationCovered(f, idealGap, observedGap, resolution) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (p *sweepReplayProof) sphereRelationCovered(f, ideal, observed, resolution *big.Rat) bool {
	switch p.outcome {
	case SweepClear:
		return ideal.Sign() > 0 && observed.Sign() > 0
	case SweepDepartedClear:
		if f.Sign() == 0 {
			return ideal.Sign() == 0 && observed.Sign() == 0
		}
		return ideal.Sign() > 0 && observed.Sign() > 0
	case SweepPersistentTouch:
		return ideal.Sign() == 0 && new(big.Rat).Abs(observed).Cmp(resolution) <= 0
	case SweepImpactBracket:
		if p.bracketLo == nil || p.bracketHi == nil {
			return false
		}
		if f.Cmp(p.bracketLo) < 0 && ideal.Sign() <= 0 {
			return false
		}
		if f.Cmp(p.bracketHi) == 0 && ideal.Sign() > 0 {
			return false
		}
		return true
	default:
		return false
	}
}

func (r *SweepReport) replayFractionCovered(f *big.Rat) bool {
	if r.replay.rotation != nil && r.replay.outcome == SweepImpactBracket {
		// The producer proves clear intervals only through the left bracket edge.
		return r.replay.bracketLo != nil && f.Cmp(r.replay.bracketLo) <= 0
	}
	switch r.replay.outcome {
	case SweepClear, SweepDepartedClear, SweepPersistentTouch:
		return true
	case SweepImpactBracket, SweepContactTransitionBracket:
		return r.replay.bracketHi != nil && f.Cmp(r.replay.bracketHi) <= 0
	default:
		return false
	}
}

func (r *SweepReport) replayRelationCovered(f *big.Rat, relation ContactRelation,
	a, b sourceBoxContactProof, resolution *big.Rat) bool {
	switch r.replay.outcome {
	case SweepClear:
		return relation == ContactSeparated
	case SweepDepartedClear:
		if f.Sign() == 0 {
			return relation == ContactTouching
		}
		return relation == ContactSeparated
	case SweepPersistentTouch:
		return relation == ContactTouching ||
			(relation == ContactSeparated || relation == ContactOverlapping) &&
				boxRelationDistanceWithin(a, b, resolution)
	case SweepImpactBracket:
		if r.replay.bracketLo == nil {
			return false
		}
		return relation == ContactSeparated || relation == ContactTouching ||
			relation == ContactOverlapping && boxRelationDistanceWithin(a, b, resolution)
	case SweepContactTransitionBracket:
		return relation == ContactTouching || relation == ContactSeparated ||
			relation == ContactOverlapping && boxRelationDistanceWithin(a, b, resolution)
	default:
		return false
	}
}

// For axis-aligned boxes, the least support gap (separated case) or least
// penetration depth (overlap case) is bounded directly from exact endpoints.
func boxRelationDistanceWithin(a, b sourceBoxContactProof, limit *big.Rat) bool {
	var distance *big.Rat
	for axis := range 3 {
		for _, candidate := range []*big.Rat{
			new(big.Rat).Abs(new(big.Rat).Sub(a.hi[axis].rat(), b.lo[axis].rat())),
			new(big.Rat).Abs(new(big.Rat).Sub(b.hi[axis].rat(), a.lo[axis].rat())),
		} {
			if distance == nil || candidate.Cmp(distance) < 0 {
				distance = candidate
			}
		}
	}
	return distance != nil && distance.Cmp(limit) <= 0
}
