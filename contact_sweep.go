package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
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

func encodedPairPath(path PairPath) sweeppath.PairPath {
	switch p := path.(type) {
	case PoseSegment:
		return sweeppath.PoseSegment(p)
	case RigidDriftSegment:
		return sweeppath.RigidDriftSegment{
			From: p.From, Center: p.Center,
			LinearVelocity:  sweeppath.QuantityVec(p.LinearVelocity),
			AngularVelocity: sweeppath.QuantityVec(p.AngularVelocity),
			Duration:        p.Duration,
		}
	default:
		return nil
	}
}

// SweepStartPolicy selects what to prove when the bodies initially touch.
type SweepStartPolicy int

const (
	StopAtInitialContact SweepStartPolicy = iota
	ContinueSeparatingTouch
	ContinueCertifiedTouch
)

// SweepRequest bounds the time search and the contact geometry resolution.
//
// RestSpeed is a nonnegative Velocity; the zero Value rests no vertex. A band
// track over a support set (docs/multibody-dynamics-design.md §10.5) holds a
// lifted vertex that closes on the support plane no faster than RestSpeed, or
// rises, on both sides of the plane, as it holds a contact vertex (§10.8), so
// the track does not end where that vertex's lower height bound reaches the
// plane. A departure rests nothing.
type SweepRequest struct {
	ContactRequest
	TimeResolution     units.Value
	MaxPoseEvaluations uint64
	StartPolicy        SweepStartPolicy
	RestSpeed          units.Value
}

// validateRestSpeed checks SweepRequest.RestSpeed as validateSupportBand
// checks the band: the zero Value, or a finite, nonnegative Velocity.
func validateRestSpeed(v units.Value) error {
	if v == (units.Value{}) {
		return nil
	}
	if v.Kind() != units.Velocity {
		return fmt.Errorf("%w: rest speed must be a Velocity", ErrUnitKind)
	}
	if !measurement.FiniteValues(v.Base()) {
		return fmt.Errorf("%w: rest speed is non-finite", ErrNotFinite)
	}
	if v.Base() < 0 {
		return fmt.Errorf("%w: rest speed must be nonnegative", ErrDegenerate)
	}
	return nil
}

// The public start policy holds the same constants, in the same order, as its
// reportvocab twin; a constant that drifts fails to compile here.
var (
	_ = [1]struct{}{}[int(StopAtInitialContact)-int(reportvocab.StopAtInitialContact)]
	_ = [1]struct{}{}[int(ContinueSeparatingTouch)-int(reportvocab.ContinueSeparatingTouch)]
	_ = [1]struct{}{}[int(ContinueCertifiedTouch)-int(reportvocab.ContinueCertifiedTouch)]
)

// sweepRequestToInternal gives a public request its reportvocab form, field
// for field.
func sweepRequestToInternal(req SweepRequest) reportvocab.SweepRequest {
	return reportvocab.SweepRequest{
		ContactRequest:     contactRequestToInternal(req.ContactRequest),
		TimeResolution:     req.TimeResolution,
		MaxPoseEvaluations: req.MaxPoseEvaluations,
		StartPolicy:        reportvocab.SweepStartPolicy(req.StartPolicy),
		RestSpeed:          req.RestSpeed,
	}
}

// SweepOutcome states the certified relation over the requested path.
type SweepOutcome int

const (
	SweepClear                    SweepOutcome = SweepOutcome(reportvocab.SweepClear)
	SweepDepartedClear            SweepOutcome = SweepOutcome(reportvocab.SweepDepartedClear)
	SweepPersistentTouch          SweepOutcome = SweepOutcome(reportvocab.SweepPersistentTouch)
	SweepContactTransitionBracket SweepOutcome = SweepOutcome(reportvocab.SweepContactTransitionBracket)
	SweepImpactBracket            SweepOutcome = SweepOutcome(reportvocab.SweepImpactBracket)
	SweepInitiallyTouching        SweepOutcome = SweepOutcome(reportvocab.SweepInitiallyTouching)
	SweepInitiallyOverlapping     SweepOutcome = SweepOutcome(reportvocab.SweepInitiallyOverlapping)
	SweepUndecided                SweepOutcome = SweepOutcome(reportvocab.SweepUndecided)
	SweepGrazingTouch             SweepOutcome = SweepOutcome(reportvocab.SweepGrazingTouch)
	// SweepPersistentBand: the pair stays within a certified band of one
	// normal through its contact track. At every instant of the track the
	// signed separation along Normal() of every point the track publishes
	// lies in [−Depth, Depth], the interiors overlap by at most Depth along
	// that normal, every vertex of the touching body the track does not
	// publish stays strictly in front of the support plane, and the source
	// features are stable. Depth
	// is the track's Band(). The track may end before the duration.
	SweepPersistentBand SweepOutcome = SweepOutcome(reportvocab.SweepPersistentBand)
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
	cylinder      *sourceCylinderContactProof // disk-on-face track (contact_cylinder_sweep.go)
	cylinderFirst bool
	start, end    *big.Rat
	duration      *big.Rat
	request       reportvocab.ContactRequest
	features      [2]reportvocab.ContactFeature
	normal        VecMeasurement
	pointCount    int
	orientedA     *orientedSourceBox
	orientedB     *orientedSourceBox
	orientedDelta [3]proofarith.Dyadic
	planar        *planarTrackProof  // general planar touch or band (contact_sweep_planar.go)
	rolling       *rollingTrackProof // rolling cylinder touch or band (contact_sweep_rolling.go)
}

func (t *SweepContactTrack) Start() SweepInstant { return sweepInstant(t.start, t.duration) }

func (t *SweepContactTrack) End() SweepInstant { return sweepInstant(t.end, t.duration) }

func (t *SweepContactTrack) Features() (ContactFeature, ContactFeature) {
	return contactFeatureFromInternal(t.features[0]), contactFeatureFromInternal(t.features[1])
}

func (t *SweepContactTrack) Normal() VecMeasurement { return t.normal }

// Band returns the depth of a band track (docs/multibody-dynamics-design.md
// §10.3), a Length Measurement whose Value plus Bound bounds every contact
// point's separation in both directions and the interiors' overlap along
// Normal(). It is nil for an exact touch track.
func (t *SweepContactTrack) Band() *Measurement {
	var band *Measurement
	switch {
	case t.planar != nil:
		band = t.planar.band
	case t.rolling != nil:
		band = t.rolling.band
	}
	if band == nil {
		return nil
	}
	out := *band
	return &out
}

// BandAt returns the depth of a band track over its prefix through fraction
// (docs/multibody-dynamics-design.md §10.3): a Length Measurement whose Value
// plus Bound bounds every contact point's separation in both directions and
// the interiors' overlap along Normal() at every instant from the track's
// start through fraction. At the track's end it is at most Band(); it grows
// with fraction, so a caller can end a slice where the band reaches its own
// residual. It is nil for an exact touch track.
func (t *SweepContactTrack) BandAt(fraction units.Value) (*Measurement, error) {
	if fraction.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: contact-track fraction must be dimensionless", ErrUnitKind)
	}
	f, ok := sweeppath.ExactBaseValue(fraction)
	if !ok || !measurement.FiniteValues(fraction.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact-track fraction", ErrNotFinite)
	}
	if f.Cmp(t.start) < 0 || f.Cmp(t.end) > 0 {
		return nil, fmt.Errorf("%w: fraction is outside contact track", ErrDegenerate)
	}
	var depth *big.Rat
	switch {
	case t.planar != nil && t.planar.band != nil:
		if depth, ok = t.planar.depthThrough(f); !ok {
			return nil, fmt.Errorf("%w: band depth has no finite bound", ErrUnsupported)
		}
	case t.rolling != nil && t.rolling.band != nil:
		depth = t.rolling.depthThrough(f)
	default:
		return nil, nil //nolint:nilnil // an exact touch track has no band, as Band() reports
	}
	value := sweeppath.RatFloatNearest(depth)
	bound := proofarith.RationalFloatError(depth, value)
	if !measurement.FiniteValues(value, bound) {
		return nil, fmt.Errorf("%w: band depth has no finite bound", ErrUnsupported)
	}
	return &Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessFromBound(bound)}, nil
}

// ManifoldAt returns a fresh reduction of the complete exact touching set at
// a fraction in this track's certified interval.
func (t *SweepContactTrack) ManifoldAt(fraction units.Value) (*ContactManifold, error) {
	if fraction.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: contact-track fraction must be dimensionless", ErrUnitKind)
	}
	f, ok := sweeppath.ExactBaseValue(fraction)
	if !ok || !measurement.FiniteValues(fraction.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact-track fraction", ErrNotFinite)
	}
	if f.Cmp(t.start) < 0 || f.Cmp(t.end) > 0 {
		return nil, fmt.Errorf("%w: fraction is outside contact track", ErrDegenerate)
	}
	if t.planar != nil {
		return t.planar.planarManifoldAt(f, t.request)
	}
	if t.rolling != nil {
		return t.rolling.manifoldAt(f, t.request)
	}
	if t.cylinder != nil {
		return t.cylinderTrackManifold(f)
	}
	if t.orientedA != nil && t.orientedB != nil {
		// Translation keeps both boxes' source face identities.
		a, b := *t.orientedA, *t.orientedB
		var okA, okB bool
		a.OrientedBox, okA = pairbox.TranslateOriented(a.OrientedBox, t.orientedDelta, f)
		b.OrientedBox, okB = pairbox.TranslateOriented(b.OrientedBox, t.orientedDelta, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: oriented contact-track fraction cannot be represented", ErrUnsupported)
		}
		report := &reportvocab.ContactReport{Request: t.request}
		report.Relation, _, _ = orientedBoxRelation(a, b)
		if report.Relation == reportvocab.ContactTouching {
			publishOrientedBoxPatch(report, a, b)
			if report.Manifold == nil {
				publishClippedHorizontalPatch(report, a, b)
			}
		}
		if report.Relation != reportvocab.ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != t.pointCount {
			return nil, fmt.Errorf("%w: oriented contact track lost its face patch", ErrUnsupported)
		}
		return contactManifoldFromInternal(report.Manifold), nil
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
		report := &reportvocab.ContactReport{Request: t.request}
		classifySourceSphereBox(report, sphere, box, t.sphereFirst)
		if report.Relation != reportvocab.ContactTouching || report.Manifold == nil {
			return nil, fmt.Errorf("%w: contact track has no bounded sphere point", ErrUnsupported)
		}
		return contactManifoldFromInternal(report.Manifold), nil
	}
	if t.spherePair != nil {
		a, okA := translatedSphere(t.spherePair[0], t.deltaA, f)
		b, okB := translatedSphere(t.spherePair[1], t.deltaB, f)
		if !okA || !okB {
			return nil, fmt.Errorf("%w: sphere-pair contact-track fraction cannot be represented", ErrUnsupported)
		}
		report := &reportvocab.ContactReport{Request: t.request}
		classifySourceSpherePair(report, a, b)
		if report.Relation != reportvocab.ContactTouching || report.Manifold == nil ||
			len(report.Manifold.Points) != 1 ||
			report.Manifold.Points[0].FeatureA != t.features[0] ||
			report.Manifold.Points[0].FeatureB != t.features[1] {
			return nil, fmt.Errorf("%w: sphere-pair contact track lost its point", ErrUnsupported)
		}
		return contactManifoldFromInternal(report.Manifold), nil
	}
	a, b, ok := translatedSourceBoxes(t.a, t.b, t.deltaA, t.deltaB, f)
	if !ok {
		return nil, fmt.Errorf("%w: fraction is not representable as a source-box translation", ErrUnsupported)
	}
	report := &reportvocab.ContactReport{Request: t.request}
	classifySourceBoxes(report, a, b)
	if report.Relation != reportvocab.ContactTouching || report.Manifold == nil {
		return nil, fmt.Errorf("%w: contact track has no bounded face patch", ErrUnsupported)
	}
	return contactManifoldFromInternal(report.Manifold), nil
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

// sweepEventOf publishes a contact producer's finished report as the ideal
// event at at, converting it to its public form.
func sweepEventOf(at SweepInstant, contact *reportvocab.ContactReport) SweepEvent {
	return SweepEvent{At: at, Relation: ContactRelation(contact.Relation),
		Gap: measurementPtrFromInternal(contact.Gap), Overlap: measurementPtrFromInternal(contact.Overlap),
		Manifold: contactManifoldFromInternal(contact.Manifold), Reason: ContactReason(contact.Reason)}
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

type affinePairPath = sweeppath.AffinePath

func sweepInstant(f, duration *big.Rat) SweepInstant {
	t := new(big.Rat).Mul(f, duration)
	value := sweeppath.RatFloatNearest(t)
	bound := proofarith.RationalFloatError(t, value)
	return SweepInstant{
		Fraction: units.Scalar(sweeppath.RatFloatNearest(f)),
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
// or affine path, and from an initial touch a departure or a persistent touch
// or band track (docs/multibody-dynamics-design.md §10.2, §10.3). A
// positive-bound faceted Boolean or all-planar cap-loop chamfer takes the
// same run through its held mesh under §10.4: a first impact brackets onto a
// ContactBand sample, and a band start never departs but may carry a band
// track widened by twice the displacement. Under a positive SupportBand the
// band track runs over §10.5's support set and ends before any of its
// vertices reaches the plane, and an exact pair that starts inside the band
// may depart. A full source cylinder whose
// ruling touches a face of an exact planar body that only translates may,
// under a spinning drift, carry §10.4's rolling touch or band track.
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
	pa, err := sweeppath.Validate(encodedPairPath(pathA))
	if err != nil {
		return nil, err
	}
	pb, err := sweeppath.Validate(encodedPairPath(pathB))
	if err != nil {
		return nil, err
	}
	if pa.Duration.Cmp(pb.Duration) != 0 {
		return nil, fmt.Errorf("%w: pair paths have different durations", ErrDegenerate)
	}
	resolution, ok := sweeppath.ExactBaseValue(req.TimeResolution)
	if err := sweeppath.DurationValid(req.TimeResolution, resolution); err != nil {
		return nil, err
	}
	if !ok || resolution.Cmp(pa.Duration) > 0 || req.MaxPoseEvaluations < 2 ||
		req.StartPolicy < StopAtInitialContact || req.StartPolicy > ContinueCertifiedTouch {
		return nil, fmt.Errorf("%w: invalid sweep request", ErrDegenerate)
	}
	if req.PointResolution.Kind() != units.Length || req.NormalResolution.Kind() != units.Angle {
		return nil, fmt.Errorf("%w: contact resolutions must be Length and Angle", ErrUnitKind)
	}
	if !measurement.FiniteValues(req.PointResolution.Base(), req.NormalResolution.Base()) {
		return nil, fmt.Errorf("%w: nonfinite contact resolution", ErrNotFinite)
	}
	if req.PointResolution.Base() <= 0 || req.NormalResolution.Base() <= 0 {
		return nil, fmt.Errorf("%w: contact resolutions must be positive", ErrDegenerate)
	}
	if err := validateSupportBand(req.SupportBand); err != nil {
		return nil, err
	}
	if err := validateHeldChord(req.HeldChord); err != nil {
		return nil, err
	}
	if err := validateRestSpeed(req.RestSpeed); err != nil {
		return nil, err
	}
	report := &SweepReport{A: a, B: b, PathA: pathA, PathB: pathB, Request: req}
	request := sweepRequestToInternal(req)
	if !pa.Supported || !pb.Supported {
		report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
			To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	if pa.Drift != nil || pb.Drift != nil || pa.Screw != nil || pb.Screw != nil {
		if pa.Screw == nil && pb.Screw == nil {
			if result, admitted, err := d.sweepRotatingSpherePair(ctx, a, b, pa, pb, request, resolution, report); admitted {
				return result, err
			}
			if result, admitted, err := d.sweepRotatingSphereBox(ctx, a, b, pa, pb, request, resolution, report); admitted {
				return result, err
			}
		}
		return d.sweepRotatingPair(ctx, a, b, pa, pb, request, resolution, report)
	}
	boxA, okA := sourceBoxAtPose(a, pa.From)
	boxB, okB := sourceBoxAtPose(b, pb.From)
	if !okA || !okB {
		if okA {
			if _, faceted := b.n.Payload.(facetedPayload); faceted {
				return d.sweepFacetedFloor(ctx, a, b, pa, pb, request, report, boxA, false)
			}
		}
		if okB {
			if _, faceted := a.n.Payload.(facetedPayload); faceted {
				return d.sweepFacetedFloor(ctx, a, b, pa, pb, request, report, boxB, true)
			}
		}
		if !okA && !okB {
			sphereA, sphereOKA := sourceSphereAtPose(a, pa.From)
			sphereB, sphereOKB := sourceSphereAtPose(b, pb.From)
			if sphereOKA && sphereOKB {
				pair := [2]sourceSphereContactProof{sphereA, sphereB}
				report.replay = &sweepReplayProof{pa: pa, pb: pb, spherePair: &pair,
					request: request.ContactRequest}
				return (&sourceSpherePairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphereA: sphereA, sphereB: sphereB}).execute(ctx, resolution)
			}
		}
		if okA && !okB {
			if sphere, ok := sourceSphereAtPose(b, pb.From); ok {
				return (&sourceSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: boxA}).execute(ctx, resolution)
			}
			if cylinder, ok := sourceCylinderAtPose(b, pb.From); ok {
				return d.sourceCylinderFaceSweep(ctx, a, b, pa, pb, request, report, cylinder, boxA, false)
			}
		}
		if okB && !okA {
			if sphere, ok := sourceSphereAtPose(a, pa.From); ok {
				return (&sourceSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: boxB, sphereFirst: true}).execute(ctx, resolution)
			}
			if cylinder, ok := sourceCylinderAtPose(a, pa.From); ok {
				return d.sourceCylinderFaceSweep(ctx, a, b, pa, pb, request, report, cylinder, boxB, true)
			}
		}
		if sphere, sphereOK := sourceSphereAtPose(a, pa.From); sphereOK {
			if box, boxOK := sourceOrientedBoxAtPose(b, pb.From); boxOK {
				return (&orientedSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: box, sphereFirst: true}).execute(ctx, resolution)
			}
		}
		if sphere, sphereOK := sourceSphereAtPose(b, pb.From); sphereOK {
			if box, boxOK := sourceOrientedBoxAtPose(a, pa.From); boxOK {
				return (&orientedSphereSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: box}).execute(ctx, resolution)
			}
		}
		if _, orientedA := sourceOrientedBoxAtPose(a, pa.From); orientedA {
			if _, orientedB := sourceOrientedBoxAtPose(b, pb.From); orientedB {
				return d.sweepRotatingPair(ctx, a, b, pa, pb, request, resolution, report)
			}
		}
		if result, admitted, err := d.sweepPlanarPair(ctx, a, b, pa, pb, request, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
			To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	report.replay = &sweepReplayProof{pa: pa, pb: pb, boxA: boxA, boxB: boxB,
		request: request.ContactRequest}
	run := pairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb, req: request, report: report,
		boxA: boxA, boxB: boxB}
	result, err := run.execute(ctx, resolution)
	if err != nil || result == nil {
		return result, err
	}
	result.replay.snapshot(result)
	return result, nil
}
