package sweepproof

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	vocab "github.com/lestrrat-3d/decad/report"

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

// RotationalPath is one body's prepared rotating path with its start box, the
// part of a rotating sweep a replay needs.
type RotationalPath struct {
	planarsweep.BodyPath
	StartBox contactbox.OrientedSourceBox
}

// Proof keeps the source geometry used by the continuous sweep. A
// replayed float pose is checked against it, rather than inferred from the
// sweep's endpoint samples.
type Proof struct {
	PA, PB                     sweeppath.AffinePath
	BoxA, BoxB                 contactbox.SourceBox
	Sphere                     *contactround.SphereProof
	SpherePair                 *[2]contactround.SphereProof
	FacetedClear               *contactfaceted.BoundedExtent
	FacetedFirst               bool
	Cylinder                   *contactround.CylinderProof
	CylinderFirst              bool
	ClearAxis, ClearSign       int
	ClearGap                   *big.Rat
	CylinderGap, CylinderSlope proofarith.Dyadic
	CylinderSide               int
	OrientedSphere             *contactround.SphereProof
	OrientedSphereBox          *contactbox.OrientedSourceBox
	SphereFirst                bool
	SphereAxis, SphereSide     int
	SphereGap, SphereSlope     proofarith.Dyadic
	Rotation                   *[2]RotationalPath
	Planar                     *planarsweep.Replay // a general planar sweep (internal/sweeprun/planar.go)
	Track                      *Track
	Request                    vocab.ContactRequest
	Outcome                    vocab.SweepOutcome
	BracketLo                  *big.Rat
	BracketHi                  *big.Rat
	BracketGap                 *big.Rat // a rotating bracket's proven lower gap at its left edge
	BracketTravel              *big.Rat // both bodies' travel bound per unit fraction, rotating brackets
	GrazingAt                  *big.Rat
	poses                      ringmemo.Memo[string, sweepmemo.ReplayPoses] // certifiedPosesAtFraction's answers; changes no outcome
}

// Snapshot records the sweep's outcome.
func (p *Proof) Snapshot(outcome vocab.SweepOutcome) {
	p.Outcome = outcome
}

// SetBracket records the exact ends of the impact bracket.
func (p *Proof) SetBracket(left, right *big.Rat) {
	p.BracketLo = new(big.Rat).Set(left)
	p.BracketHi = new(big.Rat).Set(right)
}

// SetRotatingBracketGap records what sweepmemo.BracketDepthWithin needs from a
// rotating source-box impact: the left sample's certified lower gap and both
// bodies' travel bound per unit fraction. A left sample without a positive
// gap leaves the bracket replayable only through its left edge.
func SetRotatingBracketGap[B, F, E, V any](p *Proof, samples []vocab.SweepSampleOf[B, F, E, V], travel *big.Rat) {
	for i := range samples {
		sample := &samples[i]
		exact := sample.ExactFraction(seal.Key{})
		if exact == nil || exact.Cmp(p.BracketLo) != 0 {
			continue
		}
		if gap := SampleLowerGap(sample.Ideal.Relation, sample.Ideal.Gap); gap != nil {
			p.BracketGap, p.BracketTravel = gap, travel
		}
		return
	}
}

// SampleLowerGap is a separated sample's proven positive lower gap, or nil.
func SampleLowerGap(relation vocab.ContactRelation, gapReading *vocab.Measurement) *big.Rat {
	if relation != vocab.ContactSeparated || gapReading == nil {
		return nil
	}
	value, okValue := sweeppath.ExactBaseValue(gapReading.Value)
	bound, okBound := sweeppath.ExactBaseValue(gapReading.Bound)
	if !okValue || !okBound {
		return nil
	}
	lower := new(big.Rat).Sub(value, bound)
	if lower.Sign() <= 0 {
		return nil
	}
	return lower
}

// Present reports whether the proof exists.
func (p *Proof) Present() bool { return p != nil }

// HasAffineProof reports whether this sweep can certify rounded poses
// along an affine source-box, source-sphere, faceted clear, extruded or
// revolved source-cylinder, or oriented face-track path.
func (p *Proof) HasAffineProof() bool {
	if p.Rotation == nil {
		switch p.Outcome {
		case vocab.SweepClear, vocab.SweepDepartedClear, vocab.SweepPersistentTouch, vocab.SweepGrazingTouch,
			vocab.SweepImpactBracket, vocab.SweepContactTransitionBracket:
			return true
		default:
			return false
		}
	}
	return p.Outcome == vocab.SweepPersistentTouch && p.Track != nil &&
		p.Rotation[0].Path.Drift == nil && p.Rotation[1].Path.Drift == nil
}

// PosesAt evaluates the recorded paths at elapsed time and checks
// their rounded placements against the exact source proof.
func (p *Proof) PosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if elapsed.Kind() != units.Time || !measurement.FiniteValues(elapsed.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	t, ok := sweeppath.ExactBaseValue(elapsed)
	duration := p.PA.Duration
	if p.Rotation != nil {
		duration = p.Rotation[0].Path.Duration
	}
	if !ok || t.Sign() < 0 || t.Cmp(duration) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the sweep", decaderr.ErrDegenerate)
	}
	return p.posesAtFraction(new(big.Rat).Quo(t, duration))
}

// PosesAtInterval maps an exact held time from [start, end] onto the
// certified spatial path.
func (p *Proof) PosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error) {
	if time.Kind() != units.Time || start.Kind() != units.Time ||
		end.Kind() != units.Time || !measurement.FiniteValues(time.Base(), start.Base(), end.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no replay proof", decaderr.ErrUnsupported)
	}
	timeValue, timeOK := sweeppath.ExactBaseValue(time)
	startValue, startOK := sweeppath.ExactBaseValue(start)
	endValue, endOK := sweeppath.ExactBaseValue(end)
	if !timeOK || !startOK || !endOK {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is not finite", decaderr.ErrDegenerate)
	}
	span := new(big.Rat).Sub(endValue, startValue)
	if span.Sign() <= 0 || timeValue.Cmp(startValue) < 0 || timeValue.Cmp(endValue) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the interval", decaderr.ErrDegenerate)
	}
	f := new(big.Rat).Quo(new(big.Rat).Sub(timeValue, startValue), span)
	return p.posesAtFraction(f)
}

// certifiedPosesAtFraction answers from the replay's memo when it holds f,
// and replays f otherwise.
func (p *Proof) posesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	key := sweepmemo.ReplayFractionKey(f)
	if value, ok := p.poses.Load(key); ok {
		return value.A, value.B, value.Err
	}
	a, b, err := p.replayPosesAtFraction(f)
	p.poses.Store(key, sweepmemo.ReplayPoses{A: a, B: b, Err: err}, sweepmemo.ReplayPoseCap)
	return a, b, err
}

func (p *Proof) replayPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	var trackEnd *big.Rat
	if p.Track != nil && (p.Track.Planar != nil || p.Track.Rolling != nil) {
		trackEnd = p.Track.To
	}
	if !sweepmemo.FractionCovered(p.Outcome, f, trackEnd, p.BracketLo,
		p.BracketHi, p.BracketGap, p.Rotation != nil) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the certified sweep prefix", decaderr.ErrUnsupported)
	}
	if p.Rotation != nil {
		return p.certifiedRotationalPosesAtFraction(f)
	}
	poseAt := func(path sweeppath.AffinePath) (r3.Transform, error) { return path.PoseAt(f) }
	if p.Sphere != nil || p.SpherePair != nil {
		poseAt = func(path sweeppath.AffinePath) (r3.Transform, error) {
			return path.SpherePoseAt(f)
		}
	}
	poseA, err := poseAt(p.PA)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	poseB, err := poseAt(p.PB)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	if p.Sphere != nil {
		return p.certifiedSpherePosesAtFraction(f, poseA, poseB)
	}
	if p.Cylinder != nil {
		return p.certifiedCylinderPosesAtFraction(f, poseA, poseB)
	}
	if p.SpherePair != nil {
		return p.certifiedSpherePairPosesAtFraction(f, poseA, poseB)
	}
	if p.OrientedSphere != nil {
		return p.certifiedOrientedSpherePosesAtFraction(f, poseA, poseB)
	}
	if p.FacetedClear != nil {
		return p.certifiedBoundedFacetedPosesAtFraction(f, poseA, poseB)
	}
	actualA, ok := TranslatedReplayBox(p.BoxA, p.PA.From, poseA)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose A is not an affine translation", decaderr.ErrUnsupported)
	}
	actualB, ok := TranslatedReplayBox(p.BoxB, p.PB.From, poseB)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose B is not an affine translation", decaderr.ErrUnsupported)
	}
	deviation := BoxPoseDeviation(p.BoxA, actualA, p.PA.Delta, f)
	deviation.Add(deviation, BoxPoseDeviation(p.BoxB, actualB, p.PB.Delta, f))
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	contact := &contactReport{Request: p.Request}
	contactbox.ClassifySourceBoxes(contact, actualA, actualB)
	if !sweepmemo.RelationCovered(p.Outcome, f, p.BracketLo,
		resolution, contact.Relation, actualA.AxisBox(), actualB.AxisBox()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose changes the certified relation", decaderr.ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (p *Proof) certifiedBoundedFacetedPosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	floor := p.BoxA
	if p.FacetedFirst {
		floor = p.BoxB
	}
	observedFloor, observedExtent, deviation, ok := BoundedFacetedReplayBoxes(
		floor, *p.FacetedClear, p.PA, p.PB, poseA, poseB, f, p.FacetedFirst)
	resolution, valid := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok || !valid || deviation.Cmp(resolution) > 0 ||
		!contactfaceted.InsideFloor(observedExtent, observedFloor) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: bounded faceted replay leaves its clear corridor", decaderr.ErrUnsupported)
	}
	if _, proved := contactfaceted.FloorGap(observedExtent, observedFloor); !proved {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded bounded faceted replay loses its clear gap", decaderr.ErrUnsupported)
	}
	return poseA, poseB, nil
}

func (p *Proof) certifiedCylinderPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	actualA, okA := TranslatedReplayBox(p.BoxA, p.PA.From, poseA)
	actualB, okB := TranslatedReplayBox(p.BoxB, p.PB.From, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay pose is not affine", decaderr.ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay has no resolution", decaderr.ErrUnsupported)
	}
	deviation := BoxPoseDeviation(p.BoxA, actualA, p.PA.Delta, f)
	deviation.Add(deviation, BoxPoseDeviation(p.BoxB, actualB, p.PB.Delta, f))
	actualCylinder, actualBox := actualB, actualA
	if p.CylinderFirst {
		actualCylinder, actualBox = actualA, actualB
	}
	if deviation.Cmp(resolution) > 0 ||
		!pairbox.CylinderInsideFace(actualCylinder.AxisBox(), actualBox.AxisBox(), p.ClearAxis) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its separated outer boxes", decaderr.ErrUnsupported)
	}
	if p.Outcome == vocab.SweepClear {
		if p.ClearGap == nil || p.ClearGap.Cmp(resolution) <= 0 ||
			!OuterBoxGapExceeds(actualA, actualB, p.ClearAxis, p.ClearSign, deviation) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its clear gap", decaderr.ErrUnsupported)
		}
		return poseA, poseB, nil
	}
	axis := p.ClearAxis
	var actualGap *big.Rat
	if p.CylinderSide == 1 {
		actualGap = new(big.Rat).Sub(actualCylinder.Lo[axis].Rat(), actualBox.Hi[axis].Rat())
	} else {
		actualGap = new(big.Rat).Sub(actualBox.Lo[axis].Rat(), actualCylinder.Hi[axis].Rat())
	}
	idealGap := new(big.Rat).Add(p.CylinderGap.Rat(),
		new(big.Rat).Mul(p.CylinderSlope.Rat(), f))
	difference := new(big.Rat).Sub(actualGap, idealGap)
	if new(big.Rat).Abs(difference).Cmp(deviation) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay exceeds affine path", decaderr.ErrUnsupported)
	}
	switch p.Outcome {
	case vocab.SweepDepartedClear:
		if f.Sign() == 0 {
			if idealGap.Sign() != 0 || actualGap.Sign() != 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder departure does not start at touch", decaderr.ErrUnsupported)
			}
		} else if idealGap.Sign() <= 0 || actualGap.Cmp(deviation) <= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder departure loses its gap", decaderr.ErrUnsupported)
		}
	case vocab.SweepPersistentTouch:
		// The rounded gap lies within the deviation, itself within
		// PointResolution, of the track's identically zero ideal gap.
		if p.Track == nil || p.Track.Cylinder == nil || idealGap.Sign() != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay loses its disk track", decaderr.ErrUnsupported)
		}
	case vocab.SweepImpactBracket:
		if p.BracketLo == nil || p.BracketHi == nil {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact has no bracket", decaderr.ErrUnsupported)
		}
		if f.Cmp(p.BracketLo) < 0 {
			if idealGap.Sign() <= 0 || actualGap.Cmp(deviation) <= 0 {
				return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact prefix loses its gap", decaderr.ErrUnsupported)
			}
		} else if new(big.Rat).Abs(idealGap).Cmp(resolution) > 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder impact bracket exceeds point resolution", decaderr.ErrUnsupported)
		}
	default:
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: cylinder replay outcome is unsupported", decaderr.ErrUnsupported)
	}
	return poseA, poseB, nil
}

// The sphere-pair producer proves the ideal center-distance quadratic over
// its full certified interval. Replay checks the two rounded source centers
// against that same exact path and retains the producer's relation gate.
func (p *Proof) certifiedSpherePairPosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	start := p.SpherePair
	actualA, okA := RotatingReplaySphere(start[0], p.PA, poseA)
	actualB, okB := RotatingReplaySphere(start[1], p.PB, poseB)
	if !okA || !okB {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay pose is not affine", decaderr.ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sphere-pair replay resolution is invalid", decaderr.ErrUnsupported)
	}
	motion := spherepath.PairMotion{
		CenterA: start[0].Center, CenterB: start[1].Center,
		RadiusA: start[0].Radius, RadiusB: start[1].Radius,
		DeltaA: p.PA.Delta, DeltaB: p.PB.Delta,
	}
	if detail := spherepath.PairReplay(motion, actualA.Center, actualB.Center,
		p.Outcome, f, resolution, p.BracketLo, p.BracketHi,
		p.GrazingAt, p.Track != nil); detail != "" {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: %s", decaderr.ErrUnsupported, detail)
	}
	if p.Outcome == vocab.SweepPersistentTouch {
		contact := &contactReport{Request: p.Request}
		contactround.ClassifySpherePair(contact, actualA, actualB)
		if contact.Manifold == nil || len(contact.Manifold.Points) != 1 ||
			contact.Manifold.Points[0].FeatureA != p.Track.Sources[0] ||
			contact.Manifold.Points[0].FeatureB != p.Track.Sources[1] {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere-pair point is unproved", decaderr.ErrUnsupported)
		}
	}
	return poseA, poseB, nil
}

// The producer's face corridor makes one support distance affine. Replay
// compares the rounded cached source sets with that ideal support at any
// fraction, without asking the document to classify a new pair.
func (p *Proof) certifiedOrientedSpherePosesAtFraction(f *big.Rat,
	poseA, poseB r3.Transform) (r3.Transform, r3.Transform, error) {
	spherePath, boxPath := p.PB, p.PA
	spherePose, boxPose := poseB, poseA
	if p.SphereFirst {
		spherePath, boxPath = p.PA, p.PB
		spherePose, boxPose = poseA, poseB
	}
	sphere, okSphere := TranslatedReplaySphere(*p.OrientedSphere, spherePath.From, spherePose)
	box, okBox := spherepath.TranslateObservedBox(p.OrientedSphereBox.OrientedBox, boxPath.From, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere replay is not affine", decaderr.ErrUnsupported)
	}
	deviation := spherepath.OrientedSpherePoseDeviation(p.OrientedSphereBox.OrientedBox, box,
		boxPath.Delta, p.OrientedSphere.Center, sphere.Center, spherePath.Delta, f)
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	axis, side, outward, _, observed2, faceOK := pairbox.OrientedSphereFace(
		sphere.Center, sphere.Radius, box)
	if !faceOK || axis != p.SphereAxis || side != p.SphereSide || observed2 == nil {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere leaves the face corridor", decaderr.ErrUnsupported)
	}
	startOutward := pairbox.OrientedDual(p.OrientedSphereBox.OrientedBox, p.SphereAxis)
	if p.SphereSide == 0 {
		for k := range 3 {
			startOutward[k] = proofarith.DyNeg(startOutward[k])
		}
	}
	if !proofarith.DvEqual(outward, startOutward) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotated sphere face normal changed", decaderr.ErrUnsupported)
	}
	ideal := new(big.Rat).Add(p.SphereGap.Rat(), new(big.Rat).Mul(p.SphereSlope.Rat(), f))
	ideal2 := new(big.Rat).Mul(ideal, ideal)
	radius2 := proofarith.DyMul(p.OrientedSphere.Radius, p.OrientedSphere.Radius).Rat()
	threshold := new(big.Rat).Mul(radius2, proofarith.DvDot(outward, outward).Rat())
	idealSign := ideal2.Cmp(threshold)
	observedSign := observed2.Cmp(radius2)
	if ideal.Sign() <= 0 || !sweepmemo.OrientedSphereRelationCovered(p.Outcome, f,
		p.BracketLo, p.BracketHi, idealSign, observedSign) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotated sphere changes the certified relation", decaderr.ErrUnsupported)
	}
	return poseA, poseB, nil
}

// A clear rotating sweep certifies the ideal source boxes over every fraction.
// Replay only accepts the rounded poses when their exact held source corners
// remain separated after charging the producer's pose error bound.
func (p *Proof) certifiedRotationalPosesAtFraction(f *big.Rat) (
	r3.Transform, r3.Transform, error) {
	if p.Planar != nil {
		return p.certifiedPlanarPosesAtFraction(f)
	}
	if p.Track != nil && p.Track.Rolling != nil {
		return p.Track.Rolling.ReplayPoses(f, p.Request.PointResolution)
	}
	paths := p.Rotation
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
		box[i].Faces = path.StartBox.Faces
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay pose has no finite error bound", decaderr.ErrUnsupported)
		}
	}
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok || new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1])).Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	if p.Outcome == vocab.SweepImpactBracket && p.BracketLo != nil && f.Cmp(p.BracketLo) > 0 {
		deviation := new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1]))
		if !sweepmemo.BracketDepthWithin(f, deviation, resolution, p.BracketLo,
			p.BracketHi, p.BracketGap, p.BracketTravel) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating impact pose leaves the point resolution of contact", decaderr.ErrUnsupported)
		}
		return pose[0], pose[1], nil
	}
	if p.Outcome == vocab.SweepPersistentTouch {
		if !p.certifiedOrientedTouchAtFraction(f, box[0], box[1]) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose loses the certified face track", decaderr.ErrUnsupported)
		}
		return pose[0], pose[1], nil
	}
	relation, gap, normSquared := contactbox.OrientedBoxRelation(box[0], box[1])
	if relation != vocab.ContactSeparated {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay pose is not separated", decaderr.ErrUnsupported)
	}
	norm := proofbound.RatSqrtUp(normSquared.Rat())
	if !measurement.FiniteValues(norm) || norm <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rotating replay gap has no finite bound", decaderr.ErrUnsupported)
	}
	lower := new(big.Rat).Quo(gap.Rat(), proofarith.FloatRat(norm))
	if lower.Cmp(new(big.Rat).Add(proofarith.FloatRat(deviation[0]), proofarith.FloatRat(deviation[1]))) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rotating replay gap does not exceed pose error", decaderr.ErrUnsupported)
	}
	return pose[0], pose[1], nil
}

// The producer proves the ideal face track for every fraction. Replay checks
// the rounded source boxes against that same face pair and its point bounds.
func (p *Proof) certifiedOrientedTouchAtFraction(f *big.Rat, a, b contactbox.OrientedSourceBox) bool {
	track := p.Track
	if track == nil || track.OrientedA == nil || track.OrientedB == nil ||
		f.Cmp(track.From) < 0 || f.Cmp(track.To) > 0 ||
		p.Rotation[0].Path.Drift != nil || p.Rotation[1].Path.Drift != nil {
		return false
	}
	report := &contactReport{Request: p.Request}
	report.Relation, _, _ = contactbox.OrientedBoxRelation(a, b)
	if report.Relation == vocab.ContactTouching {
		contactbox.PublishOrientedBoxPatch(report, a, b)
		if report.Manifold == nil {
			contactbox.PublishClippedHorizontalPatch(report, a, b)
		}
	}
	if report.Relation != vocab.ContactTouching || report.Manifold == nil ||
		len(report.Manifold.Points) != track.PointCount {
		return false
	}
	for _, point := range report.Manifold.Points {
		if point.FeatureA != track.Sources[0] || point.FeatureB != track.Sources[1] ||
			point.Normal.Value != track.NormalRead.Value ||
			point.OnA.Bound.Base() > track.Request.PointResolution.Base() ||
			point.OnB.Bound.Base() > track.Request.PointResolution.Base() ||
			point.NormalAngle.Base() > track.Request.NormalResolution.Base() {
			return false
		}
	}
	return true
}

func TranslatedReplayBox(box contactbox.SourceBox, from, at r3.Transform) (contactbox.SourceBox, bool) {
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

func TranslatedReplaySphere(sphere contactround.SphereProof, from, at r3.Transform) (contactround.SphereProof, bool) {
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

func RotatingReplaySphere(sphere contactround.SphereProof, path sweeppath.AffinePath,
	at r3.Transform) (contactround.SphereProof, bool) {
	if path.Drift == nil {
		return TranslatedReplaySphere(sphere, path.From, at)
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
func (p *Proof) certifiedSpherePosesAtFraction(f *big.Rat, poseA, poseB r3.Transform) (
	r3.Transform, r3.Transform, error) {
	spherePath, boxPath := p.PB, p.PA
	spherePose, boxPose := poseB, poseA
	box := p.BoxA
	if p.SphereFirst {
		spherePath, boxPath = p.PA, p.PB
		spherePose, boxPose = poseA, poseB
		box = p.BoxB
	}
	sphere, okSphere := RotatingReplaySphere(*p.Sphere, spherePath, spherePose)
	observedBox, okBox := TranslatedReplayBox(box, boxPath.From, boxPose)
	if !okSphere || !okBox {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose is not an affine translation", decaderr.ErrUnsupported)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay point resolution is invalid", decaderr.ErrUnsupported)
	}
	deviation := BoxPoseDeviation(box, observedBox, boxPath.Delta, f)
	for i := range 3 {
		ideal := new(big.Rat).Add(p.Sphere.Center[i].Rat(), new(big.Rat).Mul(spherePath.Delta[i].Rat(), f))
		difference := new(big.Rat).Sub(sphere.Center[i].Rat(), ideal)
		deviation.Add(deviation, difference.Abs(difference))
		if i == p.SphereAxis {
			continue
		}
		if proofarith.DyCmp(proofarith.DySubScalar(sphere.Center[i], sphere.Radius), observedBox.Lo[i]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(sphere.Center[i], sphere.Radius), observedBox.Hi[i]) >= 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere leaves the certified face corridor", decaderr.ErrUnsupported)
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	var faceDistance, oppositeDistance proofarith.Dyadic
	if p.SphereSide == 1 {
		faceDistance = proofarith.DySubScalar(sphere.Center[p.SphereAxis], observedBox.Hi[p.SphereAxis])
		oppositeDistance = proofarith.DySubScalar(sphere.Center[p.SphereAxis], observedBox.Lo[p.SphereAxis])
	} else {
		faceDistance = proofarith.DySubScalar(observedBox.Lo[p.SphereAxis], sphere.Center[p.SphereAxis])
		oppositeDistance = proofarith.DySubScalar(observedBox.Hi[p.SphereAxis], sphere.Center[p.SphereAxis])
	}
	if faceDistance.Sign() <= 0 || proofarith.DyCmp(oppositeDistance, sphere.Radius) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified face", decaderr.ErrUnsupported)
	}
	observedGap := proofarith.DySubScalar(faceDistance, sphere.Radius).Rat()
	idealGap := new(big.Rat).Add(p.SphereGap.Rat(), new(big.Rat).Mul(p.SphereSlope.Rat(), f))
	difference := new(big.Rat).Sub(observedGap, idealGap)
	if difference.Abs(difference).Cmp(resolution) > 0 || !sweepmemo.SphereRelationCovered(p.Outcome,
		f, p.BracketLo, p.BracketHi, idealGap, observedGap, resolution) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded sphere changes the certified relation", decaderr.ErrUnsupported)
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
func (p *Proof) certifiedPlanarPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	var poses [2]r3.Transform
	deviation, displacement := new(big.Rat), new(big.Rat)
	for i, path := range p.Rotation {
		pose, err := path.Path.RoundedPoseAt(f)
		if err != nil {
			return r3.Transform{}, r3.Transform{}, err
		}
		bound, moved, ok := path.ReplayDeviation(pose, f)
		if !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar replay pose has no finite error bound", decaderr.ErrUnsupported)
		}
		poses[i] = pose
		deviation.Add(deviation, bound)
		displacement.Add(displacement, moved)
	}
	resolution, ok := sweeppath.ExactBaseValue(p.Request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose exceeds point resolution", decaderr.ErrUnsupported)
	}
	if p.Track != nil {
		if _, ok := p.Track.Planar.ReplayHeights(f); !ok {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay pose leaves the certified band", decaderr.ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if f.Sign() == 0 && p.Planar.Departure != nil {
		if deviation.Sign() != 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: planar departure start is not the exact touch", decaderr.ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	if p.BracketLo != nil && f.Cmp(p.BracketLo) > 0 {
		if !sweepmemo.BracketDepthWithin(f, new(big.Rat).Add(deviation, displacement), resolution,
			p.BracketLo, p.BracketHi, p.BracketGap, p.BracketTravel) {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar impact pose leaves the point resolution of contact", decaderr.ErrUnsupported)
		}
		return poses[0], poses[1], nil
	}
	lower := p.Planar.LowerGap(f)
	if lower == nil || lower.Cmp(new(big.Rat).Add(deviation, displacement)) <= 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded planar replay gap does not exceed pose error", decaderr.ErrUnsupported)
	}
	return poses[0], poses[1], nil
}

var _ vocab.SweepReplay = (*Proof)(nil)
