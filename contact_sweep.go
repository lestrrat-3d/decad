package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"sort"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var errSweepPoseBudget = errors.New("decad: sweep pose budget exhausted")

// PairPath names one body's motion during a two-body sweep. Affine source-box,
// source-sphere, and certified faceted-floor paths, co-translating oblique
// source boxes, rotating source-box or centered-sphere rigid drifts, and any
// path of two exact planar solids can receive continuous certificates; other
// valid paths report an undecided sweep.
type PairPath interface{ pairPath() }

// PoseSegment joins two placements relative to the body's current placement.
type PoseSegment struct {
	From, To r3.Transform
	Duration units.Value
}

func (PoseSegment) pairPath() {}

// QuantityVec carries three components of one physical kind.
type QuantityVec struct{ X, Y, Z units.Value }

// RigidDriftSegment moves a center linearly while its orientation rotates.
type RigidDriftSegment struct {
	From            r3.Transform
	Center          r3.Vec
	LinearVelocity  QuantityVec
	AngularVelocity QuantityVec
	Duration        units.Value
}

func (RigidDriftSegment) pairPath() {}

// SweepStartPolicy selects what to prove when the bodies initially touch.
type SweepStartPolicy int

const (
	StopAtInitialContact SweepStartPolicy = iota
	ContinueSeparatingTouch
	ContinueCertifiedTouch
)

// SweepRequest bounds the time search and the contact geometry resolution.
type SweepRequest struct {
	ContactRequest
	TimeResolution     units.Value
	MaxPoseEvaluations uint64
	StartPolicy        SweepStartPolicy
}

// SweepOutcome states the certified relation over the requested path.
type SweepOutcome int

const (
	SweepClear SweepOutcome = iota + 1
	SweepDepartedClear
	SweepPersistentTouch
	SweepContactTransitionBracket
	SweepImpactBracket
	SweepInitiallyTouching
	SweepInitiallyOverlapping
	SweepUndecided
	SweepGrazingTouch
)

// SweepCause explains why a continuous claim was not proved.
type SweepCause int

const (
	SweepNoCause SweepCause = iota
	SweepPoseRelation
	SweepMissingBound
	SweepTimeFloor
	SweepFractionFloor
	SweepPoseBudget
	SweepContactUnsupported
	SweepDepartureUnproved
	SweepContactTrackUnproved
	SweepEventUnrepresentable
)

// SweepInstant identifies a dyadic fraction of the requested duration.
type SweepInstant struct {
	Fraction units.Value
	Elapsed  Measurement
}

// SweepInterval identifies an interval of the requested duration.
type SweepInterval struct{ From, To SweepInstant }

// SweepDeparture certifies positive separation after an initial touch.
type SweepDeparture struct {
	Until      SweepInstant
	GapAtUntil Measurement
}

// SweepContactTrack owns the exact source geometry and affine motion of a
// certified touching prefix. Its source face pointers are the original faces.
type SweepContactTrack struct {
	a, b          sourceBoxContactProof
	deltaA        [3]proofarith.Dyadic
	deltaB        [3]proofarith.Dyadic
	sphere        *sourceSphereContactProof
	spherePair    *[2]sourceSphereContactProof
	sphereFirst   bool
	start, end    *big.Rat
	duration      *big.Rat
	request       ContactRequest
	features      [2]ContactFeature
	normal        VecMeasurement
	pointCount    int
	orientedA     *orientedSourceBox
	orientedB     *orientedSourceBox
	orientedDelta [3]proofarith.Dyadic
}

func (t *SweepContactTrack) Start() SweepInstant { return sweepInstant(t.start, t.duration) }

func (t *SweepContactTrack) End() SweepInstant { return sweepInstant(t.end, t.duration) }

func (t *SweepContactTrack) Features() (ContactFeature, ContactFeature) {
	return t.features[0], t.features[1]
}

func (t *SweepContactTrack) Normal() VecMeasurement { return t.normal }

// ManifoldAt returns a fresh reduction of the complete exact touching set at
// a fraction in this track's certified interval.
func (t *SweepContactTrack) ManifoldAt(fraction units.Value) (*ContactManifold, error) {
	if fraction.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: contact-track fraction must be dimensionless", ErrUnitKind)
	}
	f, ok := exactBaseValue(fraction)
	if !ok || !finiteMeasurementValues(fraction.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact-track fraction", ErrNotFinite)
	}
	if f.Cmp(t.start) < 0 || f.Cmp(t.end) > 0 {
		return nil, fmt.Errorf("%w: fraction is outside contact track", ErrDegenerate)
	}
	if t.orientedA != nil && t.orientedB != nil {
		a, okA := translatedOrientedBox(*t.orientedA, t.orientedDelta, f)
		b, okB := translatedOrientedBox(*t.orientedB, t.orientedDelta, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: oriented contact-track fraction cannot be represented", ErrUnsupported)
		}
		report := &ContactReport{Request: t.request}
		report.Relation, _, _ = orientedBoxRelation(a, b)
		if report.Relation == ContactTouching {
			publishOrientedBoxPatch(report, a, b)
			if report.Manifold == nil {
				publishClippedHorizontalPatch(report, a, b)
			}
		}
		if report.Relation != ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != t.pointCount {
			return nil, fmt.Errorf("%w: oriented contact track lost its face patch", ErrUnsupported)
		}
		return report.Manifold, nil
	}
	if t.sphere != nil {
		sphereDelta, boxDelta := t.deltaB, t.deltaA
		box := t.a
		if t.sphereFirst {
			sphereDelta, boxDelta, box = t.deltaA, t.deltaB, t.b
		}
		sphere, okSphere := translatedSphere(*t.sphere, sphereDelta, f)
		box, okBox := translatedContactBox(box, boxDelta, f)
		if !okSphere || !okBox {
			return nil, fmt.Errorf("%w: contact-track fraction cannot be represented", ErrUnsupported)
		}
		report := &ContactReport{Request: t.request}
		classifySourceSphereBox(report, sphere, box, t.sphereFirst)
		if report.Relation != ContactTouching || report.Manifold == nil {
			return nil, fmt.Errorf("%w: contact track has no bounded sphere point", ErrUnsupported)
		}
		return report.Manifold, nil
	}
	if t.spherePair != nil {
		a, okA := translatedSphere(t.spherePair[0], t.deltaA, f)
		b, okB := translatedSphere(t.spherePair[1], t.deltaB, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: sphere-pair contact-track fraction cannot be represented", ErrUnsupported)
		}
		report := &ContactReport{Request: t.request}
		classifySourceSpherePair(report, a, b)
		if report.Relation != ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != 1 ||
			report.Manifold.Points[0].FeatureA != t.features[0] ||
			report.Manifold.Points[0].FeatureB != t.features[1] {
			return nil, fmt.Errorf("%w: sphere-pair contact track lost its point", ErrUnsupported)
		}
		return report.Manifold, nil
	}
	a, b, ok := translatedSourceBoxes(t.a, t.b, t.deltaA, t.deltaB, f)
	if !ok {
		return nil, fmt.Errorf("%w: fraction is not representable as a source-box translation", ErrUnsupported)
	}
	report := &ContactReport{Request: t.request}
	classifySourceBoxes(report, a, b)
	if report.Relation != ContactTouching || report.Manifold == nil {
		return nil, fmt.Errorf("%w: contact track has no bounded face patch", ErrUnsupported)
	}
	return report.Manifold, nil
}

// SweepEvent reports the ideal path relation at one sampled instant.
type SweepEvent struct {
	At       SweepInstant
	Relation ContactRelation
	Gap      *Measurement
	Overlap  *Measurement
	Manifold *ContactManifold
	Reason   ContactReason
}

// SweepSample keeps the query pose and the transferred pair finding.
type SweepSample struct {
	At            SweepInstant
	PoseA, PoseB  r3.Transform
	FloatContact  *ContactReport
	Ideal         SweepEvent
	exactFraction *big.Rat
}

// SweepReport states the first certified event or the earliest unresolved span.
type SweepReport struct {
	A, B            *Body
	PathA, PathB    PairPath
	Request         SweepRequest
	Outcome         SweepOutcome
	Bracket         *SweepInterval
	Unresolved      *SweepInterval
	InitialEvent    *SweepEvent
	Event           *SweepEvent
	Departure       *SweepDeparture
	ContactTrack    *SweepContactTrack
	Cause           SweepCause
	Samples         []SweepSample
	BoxExcluded     bool
	PoseEvaluations uint64
	replay          *sweepReplayProof
	bracketRight    *big.Rat
}

// BracketEndsAtDuration reports whether the sweep producer proved its bracket
// right endpoint is the exact final fraction of the requested duration.
func (r *SweepReport) BracketEndsAtDuration() bool {
	return r != nil && r.Bracket != nil && r.bracketRight != nil &&
		r.bracketRight.Cmp(big.NewRat(1, 1)) == 0
}

type affinePairPath struct {
	from, to  r3.Transform
	delta     [3]proofarith.Dyadic // displacement over the full path, in millimetres
	duration  *big.Rat             // exact seconds represented by the input value
	drift     *RigidDriftSegment
	screw     *r3.Screw
	read      *r3.Screw // the read screw of a rotating PoseSegment, admitted or not
	supported bool
}

func exactBaseValue(v units.Value) (*big.Rat, bool) {
	m, f := proofarith.FloatRat(v.Mag()), proofarith.FloatRat(v.Unit().Factor())
	if m == nil || f == nil {
		return nil, false
	}
	return new(big.Rat).Mul(m, f), true
}

func validatePairPath(path PairPath) (affinePairPath, error) {
	var out affinePairPath
	switch p := path.(type) {
	case PoseSegment:
		out.from, out.to = p.From, p.To
		if err := (Between{From: p.From, To: p.To}).validate(); err != nil {
			return out, err
		}
		out.duration, _ = exactBaseValue(p.Duration)
		if err := sweepDuration(p.Duration, out.duration); err != nil {
			return out, err
		}
		if p.From.Basis() != p.To.Basis() {
			inverse, err := p.From.Inverse()
			if err != nil {
				return out, err
			}
			relative, err := inverse.Then(p.To)
			if err != nil {
				return out, err
			}
			screw, err := relative.Screw()
			if err != nil {
				return out, err
			}
			out.read = &screw
			if _, _, ok := signedAxis(screw.Axis); ok && screw.Angle.Base() > 0 &&
				finiteMeasurementValues(screw.Point.X, screw.Point.Y, screw.Point.Z, screw.Slide) {
				out.screw, out.supported = &screw, true
			}
			return out, nil
		}
		start, end := p.From.Translation(), p.To.Translation()
		out.delta = [3]proofarith.Dyadic{proofarith.DySubScalar(proofarith.MustDyOf(end.X), proofarith.MustDyOf(start.X)),
			proofarith.DySubScalar(proofarith.MustDyOf(end.Y), proofarith.MustDyOf(start.Y)), proofarith.DySubScalar(proofarith.MustDyOf(end.Z), proofarith.MustDyOf(start.Z))}
		out.supported = true
	case RigidDriftSegment:
		out.from = p.From
		if !p.From.IsValid() || !finiteVec(p.Center) {
			return out, fmt.Errorf("%w: invalid drift placement or center", ErrDegenerate)
		}
		out.duration, _ = exactBaseValue(p.Duration)
		if err := sweepDuration(p.Duration, out.duration); err != nil {
			return out, err
		}
		for _, v := range []units.Value{p.LinearVelocity.X, p.LinearVelocity.Y, p.LinearVelocity.Z} {
			if err := motionValueValid(v, units.Velocity, "drift linear velocity"); err != nil {
				return out, err
			}
		}
		angularKind := units.Angle.Div(units.Time)
		rotating := false
		for _, v := range []units.Value{p.AngularVelocity.X, p.AngularVelocity.Y, p.AngularVelocity.Z} {
			if v.Kind() != angularKind {
				return out, fmt.Errorf("%w: drift angular velocity must have Angle/Time kind", ErrUnitKind)
			}
			registered, ok := units.Lookup(v.Unit().Symbol())
			if !ok || registered != v.Unit() {
				return out, fmt.Errorf("%w: drift angular velocity requires a registered unit", ErrUnitKind)
			}
			if !finiteMeasurementValues(v.Base()) {
				return out, fmt.Errorf("%w: nonfinite drift angular velocity", ErrNotFinite)
			}
			if v.Base() != 0 {
				rotating = true
			}
		}
		if rotating {
			out.drift = &p
			out.supported = true
			return out, nil
		}
		for i, v := range []units.Value{p.LinearVelocity.X, p.LinearVelocity.Y, p.LinearVelocity.Z} {
			base, ok := exactBaseValue(v)
			if !ok {
				return out, fmt.Errorf("%w: nonfinite drift velocity", ErrNotFinite)
			}
			full := new(big.Rat).Mul(base, out.duration)
			d, ok := proofarith.DyOfRat(full)
			if !ok {
				return out, fmt.Errorf("%w: drift displacement is not dyadic", ErrUnsupported)
			}
			out.delta[i] = d
		}
		out.supported = true
	default:
		return out, fmt.Errorf("%w: unknown pair path", ErrDegenerate)
	}
	return out, nil
}

func sweepDuration(v units.Value, base *big.Rat) error {
	if err := motionValueValid(v, units.Time, "sweep duration"); err != nil {
		return err
	}
	if base == nil || base.Sign() <= 0 {
		return fmt.Errorf("%w: sweep duration must be positive", ErrDegenerate)
	}
	return nil
}

func (p affinePairPath) poseAt(f *big.Rat) (r3.Transform, error) {
	if f.Sign() == 0 {
		return p.from, nil
	}
	if f.Cmp(big.NewRat(1, 1)) == 0 && p.to.IsValid() {
		return p.to, nil
	}
	d := r3.NewVec(ratFloatNearest(new(big.Rat).Mul(p.delta[0].Rat(), f)),
		ratFloatNearest(new(big.Rat).Mul(p.delta[1].Rat(), f)),
		ratFloatNearest(new(big.Rat).Mul(p.delta[2].Rat(), f)))
	step, err := r3.Translation(d)
	if err != nil {
		return r3.Transform{}, err
	}
	return p.from.Then(step)
}

func ratFloatNearest(v *big.Rat) float64 { f, _ := v.Float64(); return f }

func sweepInstant(f, duration *big.Rat) SweepInstant {
	t := new(big.Rat).Mul(f, duration)
	value := ratFloatNearest(t)
	bound := proofarith.RationalFloatError(t, value)
	return SweepInstant{
		Fraction: units.Scalar(ratFloatNearest(f)),
		Elapsed:  Measurement{Value: units.Seconds(value), Bound: units.Seconds(bound), Exactness: exactnessFromBound(bound)},
	}
}

func exactnessFromBound(bound float64) Exactness {
	if bound == 0 {
		return Exact
	}
	return Approximate
}

// SweepPair certifies the first encounter of two live solids under one shared
// duration. Continuous proofs cover affine source-box paths, exact faceted
// floor contact and bounded faceted floor clearance, co-translating oblique
// source boxes, a source sphere in
// an axis or orthogonal rotated box face corridor, an affine pair of source
// spheres, source-cylinder face paths, rotating source-box and centered-sphere rigid drifts,
// and admitted rotating PoseSegments. Two exact planar solids — prisms over
// whole LineSeg sections and zero-bound faceted Booleans — that no narrower
// path admits receive a clear path or a first-impact bracket under any rotating
// or affine path, with no replay proof.
// Unsupported paths return SweepUndecided.
// Both body pointers, both paths, and ctx must be non-nil.
func (d *Document) SweepPair(ctx context.Context, a, b *Body, pathA, pathB PairPath,
	req SweepRequest) (*SweepReport, error) {
	if d == nil || ctx == nil {
		return nil, fmt.Errorf("%w: nil document or context", ErrDegenerate)
	}
	if err := d.requireLive(a); err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if a == b {
		return nil, fmt.Errorf("%w: a pair must name distinct bodies", ErrDegenerate)
	}
	pa, err := validatePairPath(pathA)
	if err != nil {
		return nil, err
	}
	pb, err := validatePairPath(pathB)
	if err != nil {
		return nil, err
	}
	if pa.duration.Cmp(pb.duration) != 0 {
		return nil, fmt.Errorf("%w: pair paths have different durations", ErrDegenerate)
	}
	resolution, ok := exactBaseValue(req.TimeResolution)
	if err := sweepDuration(req.TimeResolution, resolution); err != nil {
		return nil, err
	}
	if !ok || resolution.Cmp(pa.duration) > 0 || req.MaxPoseEvaluations < 2 ||
		req.StartPolicy < StopAtInitialContact || req.StartPolicy > ContinueCertifiedTouch {
		return nil, fmt.Errorf("%w: invalid sweep request", ErrDegenerate)
	}
	if req.PointResolution.Kind() != units.Length || req.NormalResolution.Kind() != units.Angle {
		return nil, fmt.Errorf("%w: contact resolutions must be Length and Angle", ErrUnitKind)
	}
	if !finiteMeasurementValues(req.PointResolution.Base(), req.NormalResolution.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact resolution", ErrNotFinite)
	}
	if req.PointResolution.Base() <= 0 || req.NormalResolution.Base() <= 0 {
		return nil, fmt.Errorf("%w: contact resolutions must be positive", ErrDegenerate)
	}
	report := &SweepReport{A: a, B: b, PathA: pathA, PathB: pathB, Request: req}
	if !pa.supported || !pb.supported {
		report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, nil
	}
	if pa.drift != nil || pb.drift != nil || pa.screw != nil || pb.screw != nil {
		if pa.screw == nil && pb.screw == nil {
			if result, admitted, err := d.sweepRotatingSpherePair(ctx, a, b, pa, pb, req, resolution, report); admitted {
				return result, err
			}
			if result, admitted, err := d.sweepRotatingSphereBox(ctx, a, b, pa, pb, req, resolution, report); admitted {
				return result, err
			}
		}
		return d.sweepRotatingPair(ctx, a, b, pa, pb, req, resolution, report)
	}
	boxA, okA := sourceBoxAtPose(a, pa.from)
	boxB, okB := sourceBoxAtPose(b, pb.from)
	if !okA || !okB {
		if okA {
			if _, faceted := b.payload.(facetedPayload); faceted {
				return d.sweepFacetedFloor(ctx, a, b, pa, pb, req, report, boxA, false)
			}
		}
		if okB {
			if _, faceted := a.payload.(facetedPayload); faceted {
				return d.sweepFacetedFloor(ctx, a, b, pa, pb, req, report, boxB, true)
			}
		}
		if !okA && !okB {
			sphereA, sphereOKA := sourceSphereAtPose(a, pa.from)
			sphereB, sphereOKB := sourceSphereAtPose(b, pb.from)
			if sphereOKA && sphereOKB {
				pair := [2]sourceSphereContactProof{sphereA, sphereB}
				report.replay = &sweepReplayProof{pa: pa, pb: pb, spherePair: &pair,
					request: req.ContactRequest}
				return (&sourceSpherePairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: req, report: report, sphereA: sphereA, sphereB: sphereB}).execute(ctx, resolution)
			}
		}
		if okA && !okB {
			if sphere, ok := sourceSphereAtPose(b, pb.from); ok {
				return (&sourceSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: req, report: report, sphere: sphere, box: boxA}).execute(ctx, resolution)
			}
			if cylinder, ok := sourceCylinderAtPose(b, pb.from); ok {
				return d.sourceCylinderFaceSweep(ctx, a, b, pa, pb, req, report, cylinder, boxA, false)
			}
		}
		if okB && !okA {
			if sphere, ok := sourceSphereAtPose(a, pa.from); ok {
				return (&sourceSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: req, report: report, sphere: sphere, box: boxB, sphereFirst: true}).execute(ctx, resolution)
			}
			if cylinder, ok := sourceCylinderAtPose(a, pa.from); ok {
				return d.sourceCylinderFaceSweep(ctx, a, b, pa, pb, req, report, cylinder, boxB, true)
			}
		}
		if sphere, sphereOK := sourceSphereAtPose(a, pa.from); sphereOK {
			if box, boxOK := sourceOrientedBoxAtPose(b, pb.from); boxOK {
				return (&orientedSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: req, report: report, sphere: sphere, box: box, sphereFirst: true}).execute(ctx, resolution)
			}
		}
		if sphere, sphereOK := sourceSphereAtPose(b, pb.from); sphereOK {
			if box, boxOK := sourceOrientedBoxAtPose(a, pa.from); boxOK {
				return (&orientedSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: req, report: report, sphere: sphere, box: box}).execute(ctx, resolution)
			}
		}
		if _, orientedA := sourceOrientedBoxAtPose(a, pa.from); orientedA {
			if _, orientedB := sourceOrientedBoxAtPose(b, pb.from); orientedB {
				return d.sweepRotatingPair(ctx, a, b, pa, pb, req, resolution, report)
			}
		}
		if result, admitted, err := d.sweepPlanarPair(ctx, a, b, pa, pb, req, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, nil
	}
	report.replay = &sweepReplayProof{pa: pa, pb: pb, boxA: boxA, boxB: boxB,
		request: req.ContactRequest}
	run := pairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb, req: req, report: report,
		boxA: boxA, boxB: boxB}
	result, err := run.execute(ctx, resolution)
	if err != nil || result == nil {
		return result, err
	}
	result.replay.snapshot(result)
	return result, nil
}

type pairSweepRun struct {
	doc          *Document
	a, b         *Body
	pa, pb       affinePairPath
	req          SweepRequest
	report       *SweepReport
	boxA, boxB   sourceBoxContactProof
	facetedFloor bool // box for faceted body is its exact lower support patch
}

func (r *pairSweepRun) sample(ctx context.Context, f *big.Rat) (*SweepSample, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if r.report.PoseEvaluations >= r.req.MaxPoseEvaluations {
		return nil, errSweepPoseBudget
	}
	poseA, err := r.pa.poseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.pb.poseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.duration)
	event := r.idealContact(f, at)
	r.transferManifold(f, poseA, poseB, contact, &event)
	sample := SweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact, Ideal: event,
		exactFraction: new(big.Rat).Set(f)}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

// idealContact classifies the exact source boxes at the dyadic path fraction.
// Its manifold is an internal feature proof. transferManifold publishes either
// that bounded proof or a matching real ContactPair manifold.
func (r *pairSweepRun) idealContact(f *big.Rat, at SweepInstant) SweepEvent {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	a, b := r.boxA, r.boxB
	for i := range 3 {
		moveA, moveB := proofarith.DyMul(r.pa.delta[i], fraction), proofarith.DyMul(r.pb.delta[i], fraction)
		a.lo[i], a.hi[i] = proofarith.DyAdd(a.lo[i], moveA), proofarith.DyAdd(a.hi[i], moveA)
		b.lo[i], b.hi[i] = proofarith.DyAdd(b.lo[i], moveB), proofarith.DyAdd(b.hi[i], moveB)
	}
	report := &ContactReport{A: r.a, B: r.b, Request: r.req.ContactRequest}
	if r.facetedFloor {
		floor, faceted := a, b
		if _, ok := r.a.payload.(facetedPayload); ok {
			floor, faceted = b, a
		}
		if proofarith.DyCmp(faceted.lo[2], floor.hi[2]) < 0 {
			return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
		}
	}
	classifySourceBoxes(report, a, b)
	return SweepEvent{At: at, Relation: report.Relation, Gap: report.Gap,
		Overlap: report.Overlap, Manifold: report.Manifold, Reason: report.Reason}
}

// transferManifold checks that the float query and ideal boxes select the same
// source faces and normal. The float manifold's witnesses are then widened by
// the exact L1 pose discrepancy. Tangential clipped corners can depend on
// either body, so each point receives the sum of both pose discrepancies.
// When read-pose rounding changes the relation or makes that widening too
// coarse, the exact source-box proof supplies its own bounded ideal manifold.
func (r *pairSweepRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, event *SweepEvent) {
	ideal := event.Manifold
	event.Manifold = nil
	useIdeal := func() {
		if ideal != nil {
			event.Manifold = ideal
			event.Reason = ContactNoReason
		}
	}
	if ideal == nil || contact.Manifold == nil || event.Relation != contact.Relation ||
		len(ideal.Points) != len(contact.Manifold.Points) {
		if ideal != nil {
			useIdeal()
		}
		return
	}
	for i, idealPoint := range ideal.Points {
		actual := contact.Manifold.Points[i]
		if actual.FaceA != idealPoint.FaceA || actual.FaceB != idealPoint.FaceB ||
			actual.FeatureA != idealPoint.FeatureA || actual.FeatureB != idealPoint.FeatureB ||
			actual.Normal.Value != idealPoint.Normal.Value ||
			actual.Normal.Bound.Base() != 0 || actual.NormalAngle.Base() != 0 {
			useIdeal()
			return
		}
	}
	observedA, okA := sourceBoxAtPose(r.a, poseA)
	observedB, okB := sourceBoxAtPose(r.b, poseB)
	if !okA || !okB {
		useIdeal()
		return
	}
	deviation := boxPoseDeviation(r.boxA, observedA, r.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(r.boxB, observedB, r.pb.delta, f))
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok {
		useIdeal()
		return
	}
	points := append([]ContactPoint(nil), contact.Manifold.Points...)
	for i := range points {
		point := &points[i]
		for _, witness := range []*VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(proofarith.FloatRat(witness.Bound.Base()), deviation)
			if bound.Cmp(resolution) > 0 {
				useIdeal()
				return
			}
			publishedBound := ratFloatUp(bound)
			if publishedBound > r.req.PointResolution.Base() {
				useIdeal()
				return
			}
			witness.Bound = units.Millimeters(publishedBound)
			witness.Exactness = exactnessFromBound(witness.Bound.Base())
		}
		separationBound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
		point.Separation.Bound = units.Millimeters(ratFloatUp(separationBound))
		point.Separation.Exactness = exactnessFromBound(point.Separation.Bound.Base())
	}
	event.Manifold = &ContactManifold{Points: points}
	event.Reason = ContactNoReason
}

// boxPoseDeviation bounds the L1 distance between a float-pose source box and
// the ideal affine box. Each support endpoint is compared as an exact rational.
func boxPoseDeviation(start, observed sourceBoxContactProof, delta [3]proofarith.Dyadic, f *big.Rat) *big.Rat {
	total := new(big.Rat)
	for i := range 3 {
		move := new(big.Rat).Mul(delta[i].Rat(), f)
		idealLo := new(big.Rat).Add(start.lo[i].Rat(), move)
		idealHi := new(big.Rat).Add(start.hi[i].Rat(), move)
		lo := new(big.Rat).Sub(observed.lo[i].Rat(), idealLo)
		hi := new(big.Rat).Sub(observed.hi[i].Rat(), idealHi)
		lo.Abs(lo)
		hi.Abs(hi)
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		total.Add(total, lo)
	}
	return total
}

func translatedSourceBoxes(a, b sourceBoxContactProof, da, db [3]proofarith.Dyadic,
	f *big.Rat) (sourceBoxContactProof, sourceBoxContactProof, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return sourceBoxContactProof{}, sourceBoxContactProof{}, false
	}
	for i := range 3 {
		moveA, moveB := proofarith.DyMul(da[i], fraction), proofarith.DyMul(db[i], fraction)
		a.lo[i], a.hi[i] = proofarith.DyAdd(a.lo[i], moveA), proofarith.DyAdd(a.hi[i], moveA)
		b.lo[i], b.hi[i] = proofarith.DyAdd(b.lo[i], moveB), proofarith.DyAdd(b.hi[i], moveB)
	}
	return a, b, true
}

// fullSourceBoxTrack admits an entire affine face-contact span only when the
// same source faces and clipped-patch corner owners hold on its open interval.
// Endpoint ties at zero use their right-sided order.
func (r *pairSweepRun) fullSourceBoxTrack(first *SweepSample, end *big.Rat) *SweepContactTrack {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 4 {
		return nil
	}
	normal := first.Ideal.Manifold.Points[0].Normal
	axis, side, ok := signedAxis(normal.Value)
	if !ok || normal.Bound.Base() != 0 ||
		first.Ideal.Manifold.Points[0].NormalAngle.Base() != 0 ||
		proofarith.DyCmp(r.pa.delta[axis], r.pb.delta[axis]) != 0 {
		return nil
	}
	if side == 1 && proofarith.DyCmp(r.boxA.hi[axis], r.boxB.lo[axis]) != 0 ||
		side == 0 && proofarith.DyCmp(r.boxB.hi[axis], r.boxA.lo[axis]) != 0 {
		return nil
	}
	for i := range 3 {
		if i == axis {
			continue
		}
		// Both projected overlap inequalities are affine, so strict endpoint
		// tests prove positive overlap for every intervening time.
		for _, f := range []*big.Rat{new(big.Rat), end} {
			a, b, _ := translatedSourceBoxes(r.boxA, r.boxB, r.pa.delta, r.pb.delta, f)
			if proofarith.DyCmp(a.lo[i], b.hi[i]) >= 0 || proofarith.DyCmp(b.lo[i], a.hi[i]) >= 0 {
				return nil
			}
		}
		// A change of the lower or upper corner owner changes the patch
		// structure. The first full-span increment refuses those transitions.
		if affineEqualityRootWithin(r.boxA.lo[i], r.pa.delta[i], r.boxB.lo[i], r.pb.delta[i], end) ||
			affineEqualityRootWithin(r.boxA.hi[i], r.pa.delta[i], r.boxB.hi[i], r.pb.delta[i], end) {
			return nil
		}
	}
	if !sourceTrackPointsWithin(r.boxA, r.boxB, r.pa.delta, r.pb.delta, r.req.PointResolution.Base()) {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	return &SweepContactTrack{
		a: r.boxA, b: r.boxB, deltaA: r.pa.delta, deltaB: r.pb.delta,
		start: new(big.Rat), end: new(big.Rat).Set(end), duration: new(big.Rat).Set(r.pa.duration),
		request: r.req.ContactRequest, features: [2]ContactFeature{point.FeatureA, point.FeatureB},
		normal: normal,
	}
}

func affineEqualityRoot(a, da, b, db proofarith.Dyadic) *big.Rat {
	delta := proofarith.DySubScalar(da, db)
	if delta.IsZero() {
		return nil
	}
	return new(big.Rat).Quo(proofarith.DySubScalar(b, a).Rat(), delta.Rat())
}

func affineEqualityRootWithin(a, da, b, db proofarith.Dyadic, end *big.Rat) bool {
	root := affineEqualityRoot(a, da, b, db)
	return root != nil && root.Sign() > 0 && root.Cmp(end) <= 0
}

// sourceBoxTransitionRoot finds the first possible change of a projected
// clipped-patch owner or the first edge-contact limit. Roots at zero use the
// right-sided patch and do not restart the solver at its initial time.
func (r *pairSweepRun) sourceBoxTransitionRoot(first *SweepSample) *big.Rat {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 4 {
		return nil
	}
	axis, _, ok := signedAxis(first.Ideal.Manifold.Points[0].Normal.Value)
	if !ok {
		return nil
	}
	var earliest *big.Rat
	for i := range 3 {
		if i == axis {
			continue
		}
		for _, pair := range [][4]proofarith.Dyadic{
			{r.boxA.lo[i], r.pa.delta[i], r.boxB.lo[i], r.pb.delta[i]},
			{r.boxA.hi[i], r.pa.delta[i], r.boxB.hi[i], r.pb.delta[i]},
			{r.boxA.lo[i], r.pa.delta[i], r.boxB.hi[i], r.pb.delta[i]},
			{r.boxB.lo[i], r.pb.delta[i], r.boxA.hi[i], r.pa.delta[i]},
		} {
			root := affineEqualityRoot(pair[0], pair[1], pair[2], pair[3])
			if root == nil || root.Sign() <= 0 || root.Cmp(big.NewRat(1, 1)) > 0 {
				continue
			}
			if earliest == nil || root.Cmp(earliest) < 0 {
				earliest = root
			}
		}
	}
	return earliest
}

func sourceContactRootBracket(root, duration, resolution *big.Rat) (*big.Rat, *big.Rat, bool) {
	grid := big.NewInt(1)
	for range 61 {
		width := new(big.Rat).Quo(duration, new(big.Rat).SetInt(grid))
		if width.Cmp(resolution) <= 0 {
			scaled := new(big.Rat).Mul(root, new(big.Rat).SetInt(grid))
			leftIdx := new(big.Int).Quo(scaled.Num(), scaled.Denom())
			rightIdx := new(big.Int).Add(new(big.Int).Set(leftIdx), big.NewInt(1))
			if scaled.IsInt() {
				leftIdx.Sub(leftIdx, big.NewInt(1))
				rightIdx.Sub(rightIdx, big.NewInt(1))
			}
			left := new(big.Rat).SetFrac(leftIdx, grid)
			right := new(big.Rat).SetFrac(rightIdx, grid)
			if left.Sign() < 0 || right.Cmp(big.NewRat(1, 1)) > 0 ||
				proofarith.FloatRat(ratFloatNearest(left)).Cmp(left) != 0 ||
				proofarith.FloatRat(ratFloatNearest(right)).Cmp(right) != 0 {
				return nil, nil, false
			}
			return left, right, true
		}
		grid.Lsh(grid, 1)
	}
	return nil, nil, false
}

// Every contact coordinate lies within the start/end box endpoint envelope.
// One ULP at its maximum magnitude safely bounds conversion of any enclosed
// dyadic fraction to float; radius3D turns that into a point-ball radius.
func sourceTrackPointsWithin(a, b sourceBoxContactProof, da, db [3]proofarith.Dyadic, resolution float64) bool {
	maximum := new(big.Rat)
	for _, moving := range []struct {
		box   sourceBoxContactProof
		delta [3]proofarith.Dyadic
	}{{a, da}, {b, db}} {
		for i := range 3 {
			for _, endpoint := range []proofarith.Dyadic{moving.box.lo[i], moving.box.hi[i]} {
				for _, value := range []proofarith.Dyadic{endpoint, proofarith.DyAdd(endpoint, moving.delta[i])} {
					abs := new(big.Rat).Abs(value.Rat())
					if abs.Cmp(maximum) > 0 {
						maximum = abs
					}
				}
			}
		}
	}
	maxFloat := ratFloatUp(maximum)
	if !finiteMeasurementValues(maxFloat) {
		return false
	}
	ulp := math.Nextafter(maxFloat, math.Inf(1)) - maxFloat
	bound := radius3D(ulp)
	return finiteMeasurementValues(bound) && bound <= resolution
}

func (r *pairSweepRun) execute(ctx context.Context, resolution *big.Rat) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx, zero)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, zero, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case ContactOverlapping:
		r.report.InitialEvent = &first.Ideal
		r.report.Outcome, r.report.Event = SweepInitiallyOverlapping, &first.Ideal
		return r.report, nil
	case ContactTouching:
		r.report.InitialEvent = &first.Ideal
		if r.req.StartPolicy == StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if !r.provesDeparture() {
			if r.req.StartPolicy == ContinueCertifiedTouch {
				if track := r.fullSourceBoxTrack(first, one); track != nil {
					last, err := r.sample(ctx, one)
					if errors.Is(err, errSweepPoseBudget) {
						return r.undecided(zero, one, SweepPoseBudget), nil
					}
					if err != nil {
						return nil, err
					}
					if last.Ideal.Relation == ContactTouching && last.Ideal.Manifold != nil {
						r.report.Outcome, r.report.ContactTrack = SweepPersistentTouch, track
						return r.report, nil
					}
				}
				if root := r.sourceBoxTransitionRoot(first); root != nil {
					leftF, rightF, ok := sourceContactRootBracket(root, r.pa.duration, resolution)
					if !ok || leftF.Sign() == 0 {
						return r.undecided(zero, root, SweepFractionFloor), nil
					}
					if track := r.fullSourceBoxTrack(first, leftF); track != nil {
						left, err := r.sample(ctx, leftF)
						if errors.Is(err, errSweepPoseBudget) {
							return r.undecided(zero, leftF, SweepPoseBudget), nil
						}
						if err != nil {
							return nil, err
						}
						right, err := r.sample(ctx, rightF)
						if errors.Is(err, errSweepPoseBudget) {
							return r.undecided(leftF, rightF, SweepPoseBudget), nil
						}
						if err != nil {
							return nil, err
						}
						if left.Ideal.Relation == ContactTouching &&
							(right.Ideal.Relation == ContactTouching || right.Ideal.Relation == ContactSeparated) {
							r.report.Outcome, r.report.ContactTrack = SweepContactTransitionBracket, track
							r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
							r.report.bracketRight = new(big.Rat).Set(rightF)
							r.report.replay.setBracket(leftF, rightF)
							r.report.Event = &right.Ideal
							r.sortSamples()
							return r.report, nil
						}
					}
				}
			}
			cause := SweepDepartureUnproved
			if r.req.StartPolicy == ContinueCertifiedTouch {
				cause = SweepContactTrackUnproved
			}
			return r.undecided(zero, one, cause), nil
		}
		last, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		if last.Ideal.Relation != ContactSeparated || last.Ideal.Gap == nil {
			return r.undecided(zero, one, SweepPoseRelation), nil
		}
		r.report.Outcome = SweepDepartedClear
		r.report.Departure = &SweepDeparture{Until: last.At, GapAtUntil: *last.Ideal.Gap}
		return r.report, nil
	case ContactSeparated:
		// Exact source boxes below independently establish the path relation.
	default:
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
	entry, exit, intersects := r.contactSpan()
	if !intersects || entry.Cmp(one) > 0 || exit.Sign() < 0 || entry.Cmp(exit) > 0 {
		_, err := r.sample(ctx, one)
		if errors.Is(err, errSweepPoseBudget) {
			return r.undecided(zero, one, SweepPoseBudget), nil
		}
		if err != nil {
			return nil, err
		}
		r.report.Outcome = SweepClear
		return r.report, nil
	}
	if entry.Sign() <= 0 {
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
	depth := 0
	step := new(big.Rat).Set(paDurationFraction(resolution, r.pa.duration))
	step.Quo(step, big.NewRat(2, 1))
	for step.Cmp(one) < 0 && depth < 60 {
		depth++
		step.Mul(step, big.NewRat(2, 1))
	}
	if depth == 60 && step.Cmp(one) < 0 {
		return r.undecided(zero, one, SweepFractionFloor), nil
	}
	grid := new(big.Int).Lsh(big.NewInt(1), uint(depth))
	scaled := new(big.Rat).Mul(entry, new(big.Rat).SetInt(grid))
	floorIdx := new(big.Int).Quo(scaled.Num(), scaled.Denom())
	leftIdx := new(big.Int).Set(floorIdx)
	rightIdx := new(big.Int).Add(floorIdx, big.NewInt(2))
	exactEntry := new(big.Rat).SetInt(floorIdx).Cmp(scaled) == 0
	if exactEntry {
		leftIdx.Sub(leftIdx, big.NewInt(1))
		rightIdx.Sub(rightIdx, big.NewInt(1))
	}
	if r.facetedFloor {
		if !exactEntry {
			return r.undecided(zero, one, SweepContactUnsupported), nil
		}
		rightIdx.Set(floorIdx)
	}
	leftF := new(big.Rat).SetFrac(leftIdx, grid)
	rightF := new(big.Rat).SetFrac(rightIdx, grid)
	if rightF.Cmp(exit) > 0 || rightF.Cmp(one) > 0 {
		// The first grid point after entry is still a valid contact sample.
		// In particular, an impact at the path endpoint has no later sample.
		if exactEntry {
			rightIdx.Set(floorIdx)
		} else {
			rightIdx.Add(floorIdx, big.NewInt(1))
		}
		rightF.SetFrac(rightIdx, grid)
	}
	if rightF.Cmp(exit) > 0 || rightF.Cmp(one) > 0 {
		return r.undecided(leftF, rightF, SweepTimeFloor), nil
	}
	left, err := r.sample(ctx, leftF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(zero, leftF, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	right, err := r.sample(ctx, rightF)
	if errors.Is(err, errSweepPoseBudget) {
		return r.undecided(leftF, rightF, SweepPoseBudget), nil
	}
	if err != nil {
		return nil, err
	}
	if left.Ideal.Relation != ContactSeparated ||
		(right.Ideal.Relation != ContactTouching && right.Ideal.Relation != ContactOverlapping) {
		return r.undecided(leftF, rightF, SweepPoseRelation), nil
	}
	r.report.Outcome, r.report.Event = SweepImpactBracket, &right.Ideal
	r.report.Bracket = &SweepInterval{From: left.At, To: right.At}
	r.report.bracketRight = new(big.Rat).Set(rightF)
	r.report.replay.setBracket(leftF, rightF)
	r.sortSamples()
	return r.report, nil
}

func paDurationFraction(resolution, duration *big.Rat) *big.Rat {
	return new(big.Rat).Quo(resolution, duration)
}

func (r *pairSweepRun) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

func (r *pairSweepRun) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.duration), To: sweepInstant(to, r.pa.duration)}
	r.sortSamples()
	return r.report
}

func (r *pairSweepRun) provesDeparture() bool {
	for axis := range 3 {
		if proofarith.DyCmp(r.boxA.hi[axis], r.boxB.lo[axis]) == 0 {
			if proofarith.DySubScalar(r.pb.delta[axis], r.pa.delta[axis]).Sign() > 0 {
				return true
			}
		}
		if proofarith.DyCmp(r.boxB.hi[axis], r.boxA.lo[axis]) == 0 {
			if proofarith.DySubScalar(r.pa.delta[axis], r.pb.delta[axis]).Sign() > 0 {
				return true
			}
		}
	}
	return false
}

// contactSpan intersects six exact linear support inequalities. It returns
// the first and last fractions where the closed source boxes can meet.
func (r *pairSweepRun) contactSpan() (*big.Rat, *big.Rat, bool) {
	entry, exit := new(big.Rat), big.NewRat(1, 1)
	for axis := range 3 {
		constraints := [2][2]proofarith.Dyadic{
			{proofarith.DySubScalar(r.boxA.hi[axis], r.boxB.lo[axis]), proofarith.DySubScalar(r.pa.delta[axis], r.pb.delta[axis])},
			{proofarith.DySubScalar(r.boxB.hi[axis], r.boxA.lo[axis]), proofarith.DySubScalar(r.pb.delta[axis], r.pa.delta[axis])},
		}
		for _, c := range constraints {
			start, slope := c[0], c[1]
			if slope.IsZero() {
				if start.Sign() < 0 {
					return entry, exit, false
				}
				continue
			}
			root := new(big.Rat).Quo(proofarith.DyNeg(start).Rat(), slope.Rat())
			if slope.Sign() > 0 && root.Cmp(entry) > 0 {
				entry = root
			}
			if slope.Sign() < 0 && root.Cmp(exit) < 0 {
				exit = root
			}
		}
	}
	return entry, exit, entry.Cmp(exit) <= 0
}
