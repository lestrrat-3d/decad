package decad

import (
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sweepReplayProof keeps the source geometry used by the continuous sweep. A
// replayed float pose is checked against it, rather than inferred from the
// sweep's endpoint samples.
type sweepReplayProof struct {
	pa, pb                     affinePairPath
	boxA, boxB                 sourceBoxContactProof
	sphere                     *sourceSphereContactProof
	spherePair                 *[2]sourceSphereContactProof
	facetedClear               *boundedFacetedExtent
	facetedFirst               bool
	cylinder                   *sourceCylinderContactProof
	cylinderFirst              bool
	clearAxis, clearSign       int
	clearGap                   *big.Rat
	cylinderGap, cylinderSlope proofarith.Dyadic
	cylinderSide               int
	orientedSphere             *sourceSphereContactProof
	orientedSphereBox          *orientedSourceBox
	sphereFirst                bool
	sphereAxis, sphereSide     int
	sphereGap, sphereSlope     proofarith.Dyadic
	rotation                   *[2]rotationalSweepPath
	planar                     *planarReplay // a general planar sweep (contact_sweep_faceted.go)
	track                      *SweepContactTrack
	request                    ContactRequest
	outcome                    SweepOutcome
	bracketLo                  *big.Rat
	bracketHi                  *big.Rat
	bracketGap                 *big.Rat // a rotating bracket's proven lower gap at its left edge
	bracketTravel              *big.Rat // both bodies' travel bound per unit fraction, rotating brackets
	grazingAt                  *big.Rat
}

func (p *sweepReplayProof) snapshot(r *SweepReport) {
	p.outcome = r.Outcome
}

func (p *sweepReplayProof) setBracket(left, right *big.Rat) {
	p.bracketLo = new(big.Rat).Set(left)
	p.bracketHi = new(big.Rat).Set(right)
}

// setRotatingBracketGap records what bracketDepthWithin needs from a
// rotating source-box impact: the left sample's certified lower gap and both
// bodies' travel bound per unit fraction. A left sample without a positive
// gap leaves the bracket replayable only through its left edge.
func (p *sweepReplayProof) setRotatingBracketGap(r *SweepReport, travel *big.Rat) {
	for i := range r.Samples {
		sample := &r.Samples[i]
		if sample.exactFraction == nil || sample.exactFraction.Cmp(p.bracketLo) != 0 {
			continue
		}
		if gap := sampleLowerGap(sample); gap != nil {
			p.bracketGap, p.bracketTravel = gap, travel
		}
		return
	}
}

// HasAffineReplayProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, faceted clear, extruded or
// revolved source-cylinder, or oriented face-track path.
func (r *SweepReport) HasAffineReplayProof() bool {
	if r == nil || r.replay == nil {
		return false
	}
	if r.replay.rotation == nil {
		switch r.replay.outcome {
		case SweepClear, SweepDepartedClear, SweepPersistentTouch, SweepGrazingTouch,
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
	poseAt := func(path affinePairPath) (r3.Transform, error) { return path.poseAt(f) }
	if r.replay.sphere != nil || r.replay.spherePair != nil {
		poseAt = func(path affinePairPath) (r3.Transform, error) {
			return sourceSpherePathPoseAt(path, f)
		}
	}
	poseA, err := poseAt(r.replay.pa)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	poseB, err := poseAt(r.replay.pb)
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
	if r.replay.orientedSphere != nil {
		return r.certifiedOrientedSpherePosesAtFraction(f, poseA, poseB)
	}
	if r.replay.facetedClear != nil {
		return r.certifiedBoundedFacetedPosesAtFraction(f, poseA, poseB)
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

func (r *SweepReport) certifiedBoundedFacetedPosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	p := r.replay
	floor := p.boxA
	if p.facetedFirst {
		floor = p.boxB
	}
	observedFloor, observedExtent, deviation, ok := boundedFacetedReplayBoxes(
		floor, *p.facetedClear, p.pa, p.pb, poseA, poseB, f, p.facetedFirst)
	resolution, valid := exactBaseValue(p.request.PointResolution)
	if !ok || !valid || deviation.Cmp(resolution) > 0 ||
		!boundedFacetedInsideFloor(observedExtent, observedFloor) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: bounded faceted replay leaves its clear corridor", ErrUnsupported)
	}
	if _, proved := boundedFacetedFloorGap(observedExtent, observedFloor); !proved {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded bounded faceted replay loses its clear gap", ErrUnsupported)
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
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay has no resolution", ErrUnsupported)
	}
	deviation := boxPoseDeviation(p.boxA, actualA, p.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(p.boxB, actualB, p.pb.delta, f))
	actualCylinder, actualBox := actualB, actualA
	if p.cylinderFirst {
		actualCylinder, actualBox = actualA, actualB
	}
	if deviation.Cmp(resolution) > 0 ||
		!cylinderInsideBoxFace(actualCylinder, actualBox, p.clearAxis) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its separated outer boxes", ErrUnsupported)
	}
	if p.outcome == SweepClear {
		if p.clearGap == nil || p.clearGap.Cmp(resolution) <= 0 ||
			!outerBoxGapExceeds(actualA, actualB, p.clearAxis, p.clearSign, deviation) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its clear gap", ErrUnsupported)
		}
		return poseA, poseB, nil
	}
	axis := p.clearAxis
	var actualGap *big.Rat
	if p.cylinderSide == 1 {
		actualGap = new(big.Rat).Sub(actualCylinder.lo[axis].Rat(), actualBox.hi[axis].Rat())
	} else {
		actualGap = new(big.Rat).Sub(actualBox.lo[axis].Rat(), actualCylinder.hi[axis].Rat())
	}
	idealGap := new(big.Rat).Add(p.cylinderGap.Rat(),
		new(big.Rat).Mul(p.cylinderSlope.Rat(), f))
	difference := new(big.Rat).Sub(actualGap, idealGap)
	if new(big.Rat).Abs(difference).Cmp(deviation) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay exceeds affine path", ErrUnsupported)
	}
	switch p.outcome {
	case SweepDepartedClear:
		if f.Sign() == 0 {
			if idealGap.Sign() != 0 || actualGap.Sign() != 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder departure does not start at touch", ErrUnsupported)
			}
		} else if idealGap.Sign() <= 0 || actualGap.Cmp(deviation) <= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder departure loses its gap", ErrUnsupported)
		}
	case SweepPersistentTouch:
		// The rounded gap lies within the deviation, itself within
		// PointResolution, of the track's identically zero ideal gap.
		if p.track == nil || p.track.cylinder == nil || idealGap.Sign() != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its disk track", ErrUnsupported)
		}
	case SweepImpactBracket:
		if p.bracketLo == nil || p.bracketHi == nil {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact has no bracket", ErrUnsupported)
		}
		if f.Cmp(p.bracketLo) < 0 {
			if idealGap.Sign() <= 0 || actualGap.Cmp(deviation) <= 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact prefix loses its gap", ErrUnsupported)
			}
		} else if new(big.Rat).Abs(idealGap).Cmp(resolution) > 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact bracket exceeds point resolution", ErrUnsupported)
		}
	default:
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay outcome is unsupported", ErrUnsupported)
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
	actualA, okA := rotatingReplaySphere(start[0], p.pa, poseA)
	actualB, okB := rotatingReplaySphere(start[1], p.pb, poseB)
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
			difference := new(big.Rat).Sub(pair.observed.center[axis].Rat(), pair.ideal[axis])
			deviation.Add(deviation, difference.Abs(difference))
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair pose exceeds point resolution", ErrUnsupported)
	}
	idealDistance2 := spherePairCenterDistance2(idealA, idealB)
	actualDistance2 := spherePairCenterDistance2(
		spherePairHeldCenter(actualA), spherePairHeldCenter(actualB))
	radius := proofarith.DyAdd(start[0].radius, start[1].radius).Rat()
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
	case SweepPersistentTouch:
		if p.track == nil || idealRelation != 0 || actualRelation != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay loses its exact touch", ErrUnsupported)
		}
		contact := &ContactReport{Request: p.request}
		classifySourceSpherePair(contact, actualA, actualB)
		if contact.Manifold == nil || len(contact.Manifold.Points) != 1 ||
			contact.Manifold.Points[0].FeatureA != p.track.features[0] ||
			contact.Manifold.Points[0].FeatureB != p.track.features[1] {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair point is unproved", ErrUnsupported)
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
	case SweepGrazingTouch:
		if p.grazingAt == nil {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair graze lacks exact time", ErrUnsupported)
		}
		if f.Cmp(p.grazingAt) == 0 {
			if idealRelation != 0 || actualRelation != 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair graze loses exact touch", ErrUnsupported)
			}
		} else if idealRelation <= 0 || actualRelation <= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair graze loses separation", ErrUnsupported)
		}
	default:
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair outcome has no replay proof", ErrUnsupported)
	}
	return poseA, poseB, nil
}

// The producer's face corridor makes one support distance affine. Replay
// compares the rounded cached source sets with that ideal support at any
// fraction, without asking the document to classify a new pair.
func (r *SweepReport) certifiedOrientedSpherePosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	p := r.replay
	spherePath, boxPath := p.pb, p.pa
	spherePose, boxPose := poseB, poseA
	if p.sphereFirst {
		spherePath, boxPath = p.pa, p.pb
		spherePose, boxPose = poseA, poseB
	}
	sphere, okSphere := translatedReplaySphere(*p.orientedSphere, spherePath.from, spherePose)
	box, okBox := translatedReplayOrientedBox(*p.orientedSphereBox, boxPath.from, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere replay is not affine", ErrUnsupported)
	}
	deviation := orientedBoxPoseDeviation(*p.orientedSphereBox, box, boxPath.delta, f)
	for k := range 3 {
		expected := new(big.Rat).Add(p.orientedSphere.center[k].Rat(),
			new(big.Rat).Mul(spherePath.delta[k].Rat(), f))
		difference := new(big.Rat).Sub(sphere.center[k].Rat(), expected)
		deviation.Add(deviation, difference.Abs(difference))
	}
	resolution, ok := exactBaseValue(p.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere pose exceeds point resolution", ErrUnsupported)
	}
	axis, side, outward, _, observed2, faceOK := orientedSphereFace(sphere, box)
	if !faceOK || axis != p.sphereAxis || side != p.sphereSide || observed2 == nil {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere leaves the face corridor", ErrUnsupported)
	}
	startOutward := orientedDual(*p.orientedSphereBox, p.sphereAxis)
	if p.sphereSide == 0 {
		for k := range 3 {
			startOutward[k] = proofarith.DyNeg(startOutward[k])
		}
	}
	if !sameDyV3(outward, startOutward) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere face normal changed", ErrUnsupported)
	}
	ideal := new(big.Rat).Add(p.sphereGap.Rat(), new(big.Rat).Mul(p.sphereSlope.Rat(), f))
	ideal2 := new(big.Rat).Mul(ideal, ideal)
	radius2 := proofarith.DyMul(p.orientedSphere.radius, p.orientedSphere.radius).Rat()
	threshold := new(big.Rat).Mul(radius2, proofarith.DvDot(outward, outward).Rat())
	idealSign := ideal2.Cmp(threshold)
	observedSign := observed2.Cmp(radius2)
	if ideal.Sign() <= 0 || !p.orientedSphereRelationCovered(f, idealSign, observedSign) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func spherePairIdealCenter(s sourceSphereContactProof, delta [3]proofarith.Dyadic, f *big.Rat) [3]*big.Rat {
	center := spherePairHeldCenter(s)
	for axis := range 3 {
		center[axis].Add(center[axis], new(big.Rat).Mul(delta[axis].Rat(), f))
	}
	return center
}

func spherePairHeldCenter(s sourceSphereContactProof) [3]*big.Rat {
	var center [3]*big.Rat
	for axis := range 3 {
		center[axis] = s.center[axis].Rat()
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

func (p *sweepReplayProof) orientedSphereRelationCovered(f *big.Rat, ideal, observed int) bool {
	switch p.outcome {
	case SweepClear:
		return ideal > 0 && observed > 0
	case SweepDepartedClear:
		if f.Sign() == 0 {
			return ideal == 0 && observed == 0
		}
		return ideal > 0 && observed > 0
	case SweepImpactBracket:
		if p.bracketLo == nil || p.bracketHi == nil {
			return false
		}
		if f.Cmp(p.bracketLo) < 0 && (ideal <= 0 || observed <= 0) {
			return false
		}
		if f.Cmp(p.bracketHi) == 0 && ideal > 0 {
			return false
		}
		return true
	default:
		return false
	}
}

// A clear rotating sweep certifies the ideal source boxes over every fraction.
// Replay only accepts the rounded poses when their exact held source corners
// remain separated after charging the producer's pose error bound.
func (r *SweepReport) certifiedRotationalPosesAtFraction(f *big.Rat) (
	r3.Transform, r3.Transform, error) {
	if r.replay.planar != nil {
		return r.certifiedPlanarPosesAtFraction(f)
	}
	if r.replay.track != nil && r.replay.track.rolling != nil {
		return r.certifiedRollingPosesAtFraction(f)
	}
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
	if !ok || new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1])).Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose exceeds point resolution", ErrUnsupported)
	}
	if r.replay.outcome == SweepImpactBracket && r.replay.bracketLo != nil && f.Cmp(r.replay.bracketLo) > 0 {
		deviation := new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1]))
		if !r.replay.bracketDepthWithin(f, deviation, resolution) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating impact pose leaves the point resolution of contact", ErrUnsupported)
		}
		return pose[0], pose[1], nil
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
	norm := ratSqrtUp(normSquared.Rat())
	if !finiteMeasurementValues(norm) || norm <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay gap has no finite bound", ErrUnsupported)
	}
	lower := new(big.Rat).Quo(gap.Rat(), proofarith.FloatRat(norm))
	if lower.Cmp(new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1]))) <= 0 {
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
		move := proofarith.DySubScalar(proofarith.MustDyOf(after[i]), proofarith.MustDyOf(before[i]))
		box.lo[i], box.hi[i] = proofarith.DyAdd(box.lo[i], move), proofarith.DyAdd(box.hi[i], move)
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
		move := proofarith.DySubScalar(proofarith.MustDyOf(after[i]), proofarith.MustDyOf(before[i]))
		sphere.center[i] = proofarith.DyAdd(sphere.center[i], move)
	}
	return sphere, true
}

func rotatingReplaySphere(sphere sourceSphereContactProof, path affinePairPath,
	at r3.Transform) (sourceSphereContactProof, bool) {
	if path.drift == nil {
		return translatedReplaySphere(sphere, path.from, at)
	}
	if !at.IsValid() || at.IsReflection() || !path.from.IsValid() {
		return sourceSphereContactProof{}, false
	}
	from := path.from.Translation()
	if proofarith.DyCmp(sphere.center[0], proofarith.MustDyOf(from.X)) != 0 ||
		proofarith.DyCmp(sphere.center[1], proofarith.MustDyOf(from.Y)) != 0 ||
		proofarith.DyCmp(sphere.center[2], proofarith.MustDyOf(from.Z)) != 0 {
		return sourceSphereContactProof{}, false
	}
	sphere.center = proofarith.DyVec(at.Translation())
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
	sphere, okSphere := rotatingReplaySphere(*p.sphere, spherePath, spherePose)
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
		ideal := new(big.Rat).Add(p.sphere.center[i].Rat(), new(big.Rat).Mul(spherePath.delta[i].Rat(), f))
		difference := new(big.Rat).Sub(sphere.center[i].Rat(), ideal)
		deviation.Add(deviation, difference.Abs(difference))
		if i == p.sphereAxis {
			continue
		}
		if proofarith.DyCmp(proofarith.DySubScalar(sphere.center[i], sphere.radius), observedBox.lo[i]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(sphere.center[i], sphere.radius), observedBox.hi[i]) >= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere leaves the certified face corridor", ErrUnsupported)
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	var faceDistance, oppositeDistance proofarith.Dyadic
	if p.sphereSide == 1 {
		faceDistance = proofarith.DySubScalar(sphere.center[p.sphereAxis], observedBox.hi[p.sphereAxis])
		oppositeDistance = proofarith.DySubScalar(sphere.center[p.sphereAxis], observedBox.lo[p.sphereAxis])
	} else {
		faceDistance = proofarith.DySubScalar(observedBox.lo[p.sphereAxis], sphere.center[p.sphereAxis])
		oppositeDistance = proofarith.DySubScalar(observedBox.hi[p.sphereAxis], sphere.center[p.sphereAxis])
	}
	if faceDistance.Sign() <= 0 || proofarith.DyCmp(oppositeDistance, sphere.radius) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified face", ErrUnsupported)
	}
	observedGap := proofarith.DySubScalar(faceDistance, sphere.radius).Rat()
	idealGap := new(big.Rat).Add(p.sphereGap.Rat(), new(big.Rat).Mul(p.sphereSlope.Rat(), f))
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
	if r.replay.track != nil && (r.replay.track.planar != nil || r.replay.track.rolling != nil) {
		return f.Sign() >= 0 && f.Cmp(r.replay.track.end) <= 0
	}
	if r.replay.rotation != nil && r.replay.outcome == SweepImpactBracket {
		// The producer proves clear intervals only through the left bracket
		// edge; the bracket itself, through its right edge, replays by the
		// travel from the left edge's proven gap (bracketDepthWithin).
		if r.replay.bracketGap != nil && r.replay.bracketHi != nil {
			return r.replay.bracketLo != nil && f.Cmp(r.replay.bracketHi) <= 0
		}
		return r.replay.bracketLo != nil && f.Cmp(r.replay.bracketLo) <= 0
	}
	switch r.replay.outcome {
	case SweepClear, SweepDepartedClear, SweepPersistentTouch, SweepGrazingTouch:
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
			new(big.Rat).Abs(new(big.Rat).Sub(a.hi[axis].Rat(), b.lo[axis].Rat())),
			new(big.Rat).Abs(new(big.Rat).Sub(b.hi[axis].Rat(), a.lo[axis].Rat())),
		} {
			if distance == nil || candidate.Cmp(distance) < 0 {
				distance = candidate
			}
		}
	}
	return distance != nil && distance.Cmp(limit) <= 0
}

// certifiedPlanarPosesAtFraction replays a general planar sweep without
// rerunning the pair relation. Both rounded poses must fit PointResolution
// of their ideal poses. A track replay checks the rounded vertex heights
// against the band; a clear or departing replay needs the proven lower gap at
// f to exceed the summed deviation, so the rounded pair is separated too; a
// departure from a face-local plane (§10.6) reads that gap no larger than its
// lateral clearance. The initial touch of a departure replays only at zero
// deviation.
//
// The vertex deviation bounds the held bodies' move from ideal to rounded. A
// true point of a positive-displacement body (§10.4) lies within δ of its held
// body, and the rounded and ideal linear parts move that offset apart by at
// most the transfer charge ‖R_r − R_i‖_F·δ (rotationalSweepPath.transferCharge),
// so the lower gap must also exceed that for each body. A translating path
// charges nothing, and a rotating one a few ulps of δ.
func (r *SweepReport) certifiedPlanarPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	p := r.replay
	var poses [2]r3.Transform
	deviation, displacement := new(big.Rat), new(big.Rat)
	for i, path := range p.rotation {
		pose, err := path.poseAt(f)
		if err != nil {
			return r3.Transform{}, r3.Transform{}, err
		}
		bound, moved, ok := path.replayDeviation(pose, f)
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar replay pose has no finite error bound", ErrUnsupported)
		}
		poses[i] = pose
		deviation.Add(deviation, bound)
		displacement.Add(displacement, moved)
	}
	resolution, ok := exactBaseValue(p.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose exceeds point resolution", ErrUnsupported)
	}
	if p.track != nil {
		if _, ok := p.track.planar.replayHeights(f); !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose leaves the certified band", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if f.Sign() == 0 && p.planar.departure != nil {
		if deviation.Sign() != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar departure start is not the exact touch", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if p.bracketLo != nil && f.Cmp(p.bracketLo) > 0 {
		if !p.bracketDepthWithin(f, new(big.Rat).Add(deviation, displacement), resolution) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar impact pose leaves the point resolution of contact", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	lower := p.planar.lowerGap(f)
	if lower == nil || lower.Cmp(new(big.Rat).Add(deviation, displacement)) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay gap does not exceed pose error", ErrUnsupported)
	}
	return poses[0], poses[1], nil
}

// bracketDepthWithin decides a fraction inside a rotating impact bracket,
// after its left edge lo and through its right one, where the pair meets. The
// left edge's certified lower gap g and the §4.3 travel bound T per unit
// fraction place every ideal point within (f − lo)·T of its position at the
// left edge, where the pair was separated by g; the rounded pose adds its
// deviation, which for a displaced body (§10.4) carries its charge. The
// rounded pair therefore lies within (f − lo)·T − g + deviation of a
// separated pair, and replay requires that within PointResolution, the claim
// the affine source-box replay makes inside its bracket.
func (p *sweepReplayProof) bracketDepthWithin(f, deviation, resolution *big.Rat) bool {
	if p.bracketGap == nil || p.bracketTravel == nil || p.bracketHi == nil || f.Cmp(p.bracketHi) > 0 {
		return false
	}
	depth := new(big.Rat).Mul(new(big.Rat).Sub(f, p.bracketLo), p.bracketTravel)
	depth.Sub(depth, p.bracketGap)
	depth.Add(depth, deviation)
	return depth.Cmp(resolution) <= 0
}
