package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	"github.com/lestrrat-3d/decad/internal/ringmemo"
	"github.com/lestrrat-3d/decad/internal/spherepath"
	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sweepReplayProof keeps the source geometry used by the continuous sweep. A
// replayed float pose is checked against it, rather than inferred from the
// sweep's endpoint samples.
type sweepReplayProof struct {
	pa, pb                     affinePairPath
	boxA, boxB                 contactbox.SourceBox
	sphere                     *contactround.SphereProof
	spherePair                 *[2]contactround.SphereProof
	facetedClear               *contactfaceted.BoundedExtent
	facetedFirst               bool
	cylinder                   *contactround.CylinderProof
	cylinderFirst              bool
	clearAxis, clearSign       int
	clearGap                   *big.Rat
	cylinderGap, cylinderSlope proofarith.Dyadic
	cylinderSide               int
	orientedSphere             *contactround.SphereProof
	orientedSphereBox          *contactbox.OrientedSourceBox
	sphereFirst                bool
	sphereAxis, sphereSide     int
	sphereGap, sphereSlope     proofarith.Dyadic
	rotation                   *[2]rotationalSweepPath
	planar                     *planarsweep.Replay // a general planar sweep (contact_sweep_planar.go)
	track                      *SweepContactTrack
	request                    ContactRequest
	outcome                    SweepOutcome
	bracketLo                  *big.Rat
	bracketHi                  *big.Rat
	bracketGap                 *big.Rat // a rotating bracket's proven lower gap at its left edge
	bracketTravel              *big.Rat // both bodies' travel bound per unit fraction, rotating brackets
	grazingAt                  *big.Rat
	poses                      ringmemo.Memo[string, sweepmemo.ReplayPoses] // certifiedPosesAtFraction's answers; changes no outcome
}

func (p *sweepReplayProof) snapshot(r *SweepReport) {
	p.outcome = r.Outcome
}

func (p *sweepReplayProof) setBracket(left, right *big.Rat) {
	p.bracketLo = new(big.Rat).Set(left)
	p.bracketHi = new(big.Rat).Set(right)
}

// setRotatingBracketGap records what sweepmemo.BracketDepthWithin needs from a
// rotating source-box impact: the left sample's certified lower gap and both
// bodies' travel bound per unit fraction. A left sample without a positive
// gap leaves the bracket replayable only through its left edge.
func (p *sweepReplayProof) setRotatingBracketGap(r *SweepReport, travel *big.Rat) {
	for i := range r.Samples {
		sample := &r.Samples[i]
		if sample.ExactFraction(seal.Key{}) == nil || sample.ExactFraction(seal.Key{}).Cmp(p.bracketLo) != 0 {
			continue
		}
		if gap := sampleLowerGap(sample); gap != nil {
			p.bracketGap, p.bracketTravel = gap, travel
		}
		return
	}
}

// Present reports whether the proof exists.
func (p *sweepReplayProof) Present() bool { return p != nil }

// HasAffineProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, faceted clear, extruded or
// revolved source-cylinder, or oriented face-track path.
func (p *sweepReplayProof) HasAffineProof() bool {
	if p.rotation == nil {
		switch p.outcome {
		case SweepClear, SweepDepartedClear, SweepPersistentTouch, SweepGrazingTouch,
			SweepImpactBracket, SweepContactTransitionBracket:
			return true
		default:
			return false
		}
	}
	return p.outcome == SweepPersistentTouch && p.track != nil &&
		p.rotation[0].Path.Drift == nil && p.rotation[1].Path.Drift == nil
}

// PosesAt evaluates the recorded paths at elapsed time and checks
// their rounded placements against the exact source proof.
func (p *sweepReplayProof) PosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if elapsed.Kind() != units.Time || !measurement.FiniteValues(elapsed.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", ErrUnsupported)
	}
	t, ok := sweeppath.ExactBaseValue(elapsed)
	duration := p.pa.Duration
	if p.rotation != nil {
		duration = p.rotation[0].Path.Duration
	}
	if !ok || t.Sign() < 0 || t.Cmp(duration) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the sweep", ErrDegenerate)
	}
	return p.posesAtFraction(new(big.Rat).Quo(t, duration))
}

// PosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path.
func (p *sweepReplayProof) PosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if time.Kind() != units.Time || start.Kind() != units.Time ||
		end.Kind() != units.Time || !measurement.FiniteValues(time.Base(), start.Base(), end.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", ErrUnsupported)
	}
	timeValue, timeOK := sweeppath.ExactBaseValue(time)
	startValue, startOK := sweeppath.ExactBaseValue(start)
	endValue, endOK := sweeppath.ExactBaseValue(end)
	if !timeOK || !startOK || !endOK {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is not finite", ErrDegenerate)
	}
	span := new(big.Rat).Sub(endValue, startValue)
	if span.Sign() <= 0 || timeValue.Cmp(startValue) < 0 || timeValue.Cmp(endValue) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the interval", ErrDegenerate)
	}
	f := new(big.Rat).Quo(new(big.Rat).Sub(timeValue, startValue), span)
	return p.posesAtFraction(f)
}

// certifiedPosesAtFraction answers from the replay's memo when it holds f,
// and replays f otherwise.
func (p *sweepReplayProof) posesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	key := sweepmemo.ReplayFractionKey(f)
	if value, ok := p.poses.Load(key); ok {
		return value.A, value.B, value.Err
	}
	a, b, err := p.replayPosesAtFraction(f)
	p.poses.Store(key, sweepmemo.ReplayPoses{A: a, B: b, Err: err}, sweepmemo.ReplayPoseCap)
	return a, b, err
}

func (p *sweepReplayProof) replayPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	var trackEnd *big.Rat
	if p.track != nil && (p.track.planar != nil || p.track.rolling != nil) {
		trackEnd = p.track.end
	}
	if !sweepmemo.FractionCovered(p.outcome, f, trackEnd, p.bracketLo,
		p.bracketHi, p.bracketGap, p.rotation != nil) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the certified sweep prefix", ErrUnsupported)
	}
	if p.rotation != nil {
		return p.certifiedRotationalPosesAtFraction(f)
	}
	poseAt := func(path affinePairPath) (r3.Transform, error) { return path.PoseAt(f) }
	if p.sphere != nil || p.spherePair != nil {
		poseAt = func(path affinePairPath) (r3.Transform, error) {
			return path.SpherePoseAt(f)
		}
	}
	poseA, err := poseAt(p.pa)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	poseB, err := poseAt(p.pb)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	if p.sphere != nil {
		return p.certifiedSpherePosesAtFraction(f, poseA, poseB)
	}
	if p.cylinder != nil {
		return p.certifiedCylinderPosesAtFraction(f, poseA, poseB)
	}
	if p.spherePair != nil {
		return p.certifiedSpherePairPosesAtFraction(f, poseA, poseB)
	}
	if p.orientedSphere != nil {
		return p.certifiedOrientedSpherePosesAtFraction(f, poseA, poseB)
	}
	if p.facetedClear != nil {
		return p.certifiedBoundedFacetedPosesAtFraction(f, poseA, poseB)
	}
	actualA, ok := translatedReplayBox(p.boxA, p.pa.From, poseA)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose A is not an affine translation", ErrUnsupported)
	}
	actualB, ok := translatedReplayBox(p.boxB, p.pb.From, poseB)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose B is not an affine translation", ErrUnsupported)
	}
	deviation := boxPoseDeviation(p.boxA, actualA, p.pa.Delta, f)
	deviation.Add(deviation, boxPoseDeviation(p.boxB, actualB, p.pb.Delta, f))
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	contact := &nodeContactReport{Request: p.request}
	contactbox.ClassifySourceBoxes(contact, actualA, actualB)
	if !sweepmemo.RelationCovered(p.outcome, f, p.bracketLo,
		resolution, contact.Relation, actualA.AxisBox(), actualB.AxisBox()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (p *sweepReplayProof) certifiedBoundedFacetedPosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	floor := p.boxA
	if p.facetedFirst {
		floor = p.boxB
	}
	observedFloor, observedExtent, deviation, ok := boundedFacetedReplayBoxes(
		floor, *p.facetedClear, p.pa, p.pb, poseA, poseB, f, p.facetedFirst)
	resolution, valid := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok || !valid || deviation.Cmp(resolution) > 0 ||
		!contactfaceted.InsideFloor(observedExtent, observedFloor) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: bounded faceted replay leaves its clear corridor", ErrUnsupported)
	}
	if _, proved := contactfaceted.FloorGap(observedExtent, observedFloor); !proved {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded bounded faceted replay loses its clear gap", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (p *sweepReplayProof) certifiedCylinderPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	actualA, okA := translatedReplayBox(p.boxA, p.pa.From, poseA)
	actualB, okB := translatedReplayBox(p.boxB, p.pb.From, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay pose is not affine", ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay has no resolution", ErrUnsupported)
	}
	deviation := boxPoseDeviation(p.boxA, actualA, p.pa.Delta, f)
	deviation.Add(deviation, boxPoseDeviation(p.boxB, actualB, p.pb.Delta, f))
	actualCylinder, actualBox := actualB, actualA
	if p.cylinderFirst {
		actualCylinder, actualBox = actualA, actualB
	}
	if deviation.Cmp(resolution) > 0 ||
		!pairbox.CylinderInsideFace(actualCylinder.AxisBox(), actualBox.AxisBox(), p.clearAxis) {
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
		actualGap = new(big.Rat).Sub(actualCylinder.Lo[axis].Rat(), actualBox.Hi[axis].Rat())
	} else {
		actualGap = new(big.Rat).Sub(actualBox.Lo[axis].Rat(), actualCylinder.Hi[axis].Rat())
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
func (p *sweepReplayProof) certifiedSpherePairPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	start := p.spherePair
	actualA, okA := rotatingReplaySphere(start[0], p.pa, poseA)
	actualB, okB := rotatingReplaySphere(start[1], p.pb, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay pose is not affine", ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay resolution is invalid", ErrUnsupported)
	}
	motion := spherepath.PairMotion{
		CenterA: start[0].Center, CenterB: start[1].Center,
		RadiusA: start[0].Radius, RadiusB: start[1].Radius,
		DeltaA: p.pa.Delta, DeltaB: p.pb.Delta,
	}
	if detail := spherepath.PairReplay(motion, actualA.Center, actualB.Center,
		p.outcome, f, resolution, p.bracketLo, p.bracketHi,
		p.grazingAt, p.track != nil); detail != "" {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: %s", ErrUnsupported, detail)
	}
	if p.outcome == SweepPersistentTouch {
		contact := &nodeContactReport{Request: p.request}
		contactround.ClassifySpherePair(contact, actualA, actualB)
		if contact.Manifold == nil || len(contact.Manifold.Points) != 1 ||
			contact.Manifold.Points[0].FeatureA != p.track.features[0] ||
			contact.Manifold.Points[0].FeatureB != p.track.features[1] {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair point is unproved", ErrUnsupported)
		}
	}
	return poseA, poseB, nil
}

// The producer's face corridor makes one support distance affine. Replay
// compares the rounded cached source sets with that ideal support at any
// fraction, without asking the document to classify a new pair.
func (p *sweepReplayProof) certifiedOrientedSpherePosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	spherePath, boxPath := p.pb, p.pa
	spherePose, boxPose := poseB, poseA
	if p.sphereFirst {
		spherePath, boxPath = p.pa, p.pb
		spherePose, boxPose = poseA, poseB
	}
	sphere, okSphere := translatedReplaySphere(*p.orientedSphere, spherePath.From, spherePose)
	box, okBox := spherepath.TranslateObservedBox(p.orientedSphereBox.OrientedBox, boxPath.From, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere replay is not affine", ErrUnsupported)
	}
	deviation := spherepath.OrientedSpherePoseDeviation(p.orientedSphereBox.OrientedBox, box,
		boxPath.Delta, p.orientedSphere.Center, sphere.Center, spherePath.Delta, f)
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere pose exceeds point resolution", ErrUnsupported)
	}
	axis, side, outward, _, observed2, faceOK := pairbox.OrientedSphereFace(
		sphere.Center, sphere.Radius, box)
	if !faceOK || axis != p.sphereAxis || side != p.sphereSide || observed2 == nil {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere leaves the face corridor", ErrUnsupported)
	}
	startOutward := pairbox.OrientedDual(p.orientedSphereBox.OrientedBox, p.sphereAxis)
	if p.sphereSide == 0 {
		for k := range 3 {
			startOutward[k] = proofarith.DyNeg(startOutward[k])
		}
	}
	if !proofarith.DvEqual(outward, startOutward) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere face normal changed", ErrUnsupported)
	}
	ideal := new(big.Rat).Add(p.sphereGap.Rat(), new(big.Rat).Mul(p.sphereSlope.Rat(), f))
	ideal2 := new(big.Rat).Mul(ideal, ideal)
	radius2 := proofarith.DyMul(p.orientedSphere.Radius, p.orientedSphere.Radius).Rat()
	threshold := new(big.Rat).Mul(radius2, proofarith.DvDot(outward, outward).Rat())
	idealSign := ideal2.Cmp(threshold)
	observedSign := observed2.Cmp(radius2)
	if ideal.Sign() <= 0 || !sweepmemo.OrientedSphereRelationCovered(p.outcome, f,
		p.bracketLo, p.bracketHi, idealSign, observedSign) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

// A clear rotating sweep certifies the ideal source boxes over every fraction.
// Replay only accepts the rounded poses when their exact held source corners
// remain separated after charging the producer's pose error bound.
func (p *sweepReplayProof) certifiedRotationalPosesAtFraction(f *big.Rat) (
	r3.Transform, r3.Transform, error) {
	if p.planar != nil {
		return p.certifiedPlanarPosesAtFraction(f)
	}
	if p.track != nil && p.track.rolling != nil {
		return p.track.rolling.ReplayPoses(f, p.request.PointResolution)
	}
	paths := p.rotation
	var pose [2]r3.Transform
	var box [2]contactbox.OrientedSourceBox
	var deviation [2]float64
	for i := range paths {
		path := paths[i]
		var err error
		pose[i], err = path.Path.RoundedPoseAt(f)
		if err != nil {
			return r3.Transform{}, r3.Transform{}, err
		}
		var ok bool
		box[i].OrientedBox, deviation[i], ok = path.RoundedBox(pose[i], f)
		box[i].Faces = path.startBox.Faces
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay pose has no finite error bound", ErrUnsupported)
		}
	}
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok || new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1])).Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose exceeds point resolution", ErrUnsupported)
	}
	if p.outcome == SweepImpactBracket && p.bracketLo != nil && f.Cmp(p.bracketLo) > 0 {
		deviation := new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1]))
		if !sweepmemo.BracketDepthWithin(f, deviation, resolution, p.bracketLo,
			p.bracketHi, p.bracketGap, p.bracketTravel) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating impact pose leaves the point resolution of contact", ErrUnsupported)
		}
		return pose[0], pose[1], nil
	}
	if p.outcome == SweepPersistentTouch {
		if !p.certifiedOrientedTouchAtFraction(f, box[0], box[1]) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose loses the certified face track", ErrUnsupported)
		}
		return pose[0], pose[1], nil
	}
	relation, gap, normSquared := contactbox.OrientedBoxRelation(box[0], box[1])
	if relation != ContactSeparated {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose is not separated", ErrUnsupported)
	}
	norm := proofbound.RatSqrtUp(normSquared.Rat())
	if !measurement.FiniteValues(norm) || norm <= 0 {
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
func (p *sweepReplayProof) certifiedOrientedTouchAtFraction(f *big.Rat, a, b contactbox.OrientedSourceBox) bool {
	track := p.track
	if track == nil || track.orientedA == nil || track.orientedB == nil ||
		f.Cmp(track.start) < 0 || f.Cmp(track.end) > 0 ||
		p.rotation[0].Path.Drift != nil || p.rotation[1].Path.Drift != nil {
		return false
	}
	report := &nodeContactReport{Request: p.request}
	report.Relation, _, _ = contactbox.OrientedBoxRelation(a, b)
	if report.Relation == ContactTouching {
		contactbox.PublishOrientedBoxPatch(report, a, b)
		if report.Manifold == nil {
			contactbox.PublishClippedHorizontalPatch(report, a, b)
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

func translatedReplayBox(box contactbox.SourceBox, from, at r3.Transform) (contactbox.SourceBox, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return contactbox.SourceBox{}, false
	}
	start, end := from.Translation(), at.Translation()
	before := [3]float64{start.X, start.Y, start.Z}
	after := [3]float64{end.X, end.Y, end.Z}
	for i := range 3 {
		if !measurement.FiniteValues(before[i], after[i]) {
			return contactbox.SourceBox{}, false
		}
		move := proofarith.DySubScalar(proofarith.MustDyOf(after[i]), proofarith.MustDyOf(before[i]))
		box.Lo[i], box.Hi[i] = proofarith.DyAdd(box.Lo[i], move), proofarith.DyAdd(box.Hi[i], move)
	}
	return box, true
}

func translatedReplaySphere(sphere contactround.SphereProof, from, at r3.Transform) (contactround.SphereProof, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return contactround.SphereProof{}, false
	}
	start, end := from.Translation(), at.Translation()
	before := [3]float64{start.X, start.Y, start.Z}
	after := [3]float64{end.X, end.Y, end.Z}
	for i := range 3 {
		if !measurement.FiniteValues(before[i], after[i]) {
			return contactround.SphereProof{}, false
		}
		move := proofarith.DySubScalar(proofarith.MustDyOf(after[i]), proofarith.MustDyOf(before[i]))
		sphere.Center[i] = proofarith.DyAdd(sphere.Center[i], move)
	}
	return sphere, true
}

func rotatingReplaySphere(sphere contactround.SphereProof, path affinePairPath,
	at r3.Transform) (contactround.SphereProof, bool) {
	if path.Drift == nil {
		return translatedReplaySphere(sphere, path.From, at)
	}
	if !at.IsValid() || at.IsReflection() || !path.From.IsValid() {
		return contactround.SphereProof{}, false
	}
	from := path.From.Translation()
	if proofarith.DyCmp(sphere.Center[0], proofarith.MustDyOf(from.X)) != 0 ||
		proofarith.DyCmp(sphere.Center[1], proofarith.MustDyOf(from.Y)) != 0 ||
		proofarith.DyCmp(sphere.Center[2], proofarith.MustDyOf(from.Z)) != 0 {
		return contactround.SphereProof{}, false
	}
	sphere.Center = proofarith.DyVec(at.Translation())
	return sphere, true
}

// The sphere producer proves one face corridor and an affine support gap.
// Replay checks the rounded placements against both claims at the requested
// fraction, including fractions that are not dyadic.
func (p *sweepReplayProof) certifiedSpherePosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	spherePath, boxPath := p.pb, p.pa
	spherePose, boxPose := poseB, poseA
	box := p.boxA
	if p.sphereFirst {
		spherePath, boxPath = p.pa, p.pb
		spherePose, boxPose = poseA, poseB
		box = p.boxB
	}
	sphere, okSphere := rotatingReplaySphere(*p.sphere, spherePath, spherePose)
	observedBox, okBox := translatedReplayBox(box, boxPath.From, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose is not an affine translation", ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay point resolution is invalid", ErrUnsupported)
	}
	deviation := boxPoseDeviation(box, observedBox, boxPath.Delta, f)
	for i := range 3 {
		ideal := new(big.Rat).Add(p.sphere.Center[i].Rat(), new(big.Rat).Mul(spherePath.Delta[i].Rat(), f))
		difference := new(big.Rat).Sub(sphere.Center[i].Rat(), ideal)
		deviation.Add(deviation, difference.Abs(difference))
		if i == p.sphereAxis {
			continue
		}
		if proofarith.DyCmp(proofarith.DySubScalar(sphere.Center[i], sphere.Radius), observedBox.Lo[i]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(sphere.Center[i], sphere.Radius), observedBox.Hi[i]) >= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere leaves the certified face corridor", ErrUnsupported)
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	var faceDistance, oppositeDistance proofarith.Dyadic
	if p.sphereSide == 1 {
		faceDistance = proofarith.DySubScalar(sphere.Center[p.sphereAxis], observedBox.Hi[p.sphereAxis])
		oppositeDistance = proofarith.DySubScalar(sphere.Center[p.sphereAxis], observedBox.Lo[p.sphereAxis])
	} else {
		faceDistance = proofarith.DySubScalar(observedBox.Lo[p.sphereAxis], sphere.Center[p.sphereAxis])
		oppositeDistance = proofarith.DySubScalar(observedBox.Hi[p.sphereAxis], sphere.Center[p.sphereAxis])
	}
	if faceDistance.Sign() <= 0 || proofarith.DyCmp(oppositeDistance, sphere.Radius) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified face", ErrUnsupported)
	}
	observedGap := proofarith.DySubScalar(faceDistance, sphere.Radius).Rat()
	idealGap := new(big.Rat).Add(p.sphereGap.Rat(), new(big.Rat).Mul(p.sphereSlope.Rat(), f))
	difference := new(big.Rat).Sub(observedGap, idealGap)
	if difference.Abs(difference).Cmp(resolution) > 0 || !sweepmemo.SphereRelationCovered(p.outcome,
		f, p.bracketLo, p.bracketHi, idealGap, observedGap, resolution) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
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
// most the transfer charge ‖R_r − R_i‖_F·δ (motionbound.SweepTransferCharge),
// so the lower gap must also exceed that for each body. A translating path
// charges nothing, and a rotating one a few ulps of δ.
func (p *sweepReplayProof) certifiedPlanarPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	var poses [2]r3.Transform
	deviation, displacement := new(big.Rat), new(big.Rat)
	for i, path := range p.rotation {
		pose, err := path.Path.RoundedPoseAt(f)
		if err != nil {
			return r3.Transform{}, r3.Transform{}, err
		}
		bound, moved, ok := path.ReplayDeviation(pose, f)
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar replay pose has no finite error bound", ErrUnsupported)
		}
		poses[i] = pose
		deviation.Add(deviation, bound)
		displacement.Add(displacement, moved)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose exceeds point resolution", ErrUnsupported)
	}
	if p.track != nil {
		if _, ok := p.track.planar.ReplayHeights(f); !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose leaves the certified band", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if f.Sign() == 0 && p.planar.Departure != nil {
		if deviation.Sign() != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar departure start is not the exact touch", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if p.bracketLo != nil && f.Cmp(p.bracketLo) > 0 {
		if !sweepmemo.BracketDepthWithin(f, new(big.Rat).Add(deviation, displacement), resolution,
			p.bracketLo, p.bracketHi, p.bracketGap, p.bracketTravel) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar impact pose leaves the point resolution of contact", ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	lower := p.planar.LowerGap(f)
	if lower == nil || lower.Cmp(new(big.Rat).Add(deviation, displacement)) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay gap does not exceed pose error", ErrUnsupported)
	}
	return poses[0], poses[1], nil
}
