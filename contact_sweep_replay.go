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
	sphereFirst            bool
	sphereAxis, sphereSide int
	sphereGap, sphereSlope dyadic
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
// along its affine source-box or source-sphere path.
func (r *SweepReport) HasAffineReplayProof() bool { return r != nil && r.replay != nil }

// CertifiedPosesAt evaluates the recorded affine paths at elapsed time and
// checks their rounded placements against the sweep's exact source proof.
// It reads no Document geometry and performs no new contact query. A pose whose
// rounding can change the reported relation beyond PointResolution is refused.
func (r *SweepReport) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || r.replay == nil || elapsed.Kind() != units.Time ||
		!finiteMeasurementValues(elapsed.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no affine replay proof", ErrUnsupported)
	}
	t, ok := exactBaseValue(elapsed)
	if !ok || t.Sign() < 0 || t.Cmp(r.replay.pa.duration) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the sweep", ErrDegenerate)
	}
	return r.certifiedPosesAtFraction(new(big.Rat).Quo(t, r.replay.pa.duration))
}

// CertifiedPosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path. It permits a caller to replay a slice whose global
// clock endpoints differ slightly from the rounded sweep duration.
func (r *SweepReport) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || r.replay == nil || time.Kind() != units.Time || start.Kind() != units.Time ||
		end.Kind() != units.Time || !finiteMeasurementValues(time.Base(), start.Base(), end.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no affine replay proof", ErrUnsupported)
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
