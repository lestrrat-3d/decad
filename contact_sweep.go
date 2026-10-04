package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var errSweepPoseBudget = errors.New("decad: sweep pose budget exhausted")

// PairPath names one body's motion during a two-body sweep. The first
// implementation certifies affine translations; other valid paths report an
// undecided sweep until their continuous bound is available.
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
// The current sweep slice certifies only zero angular velocity.
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

// SweepContactTrack will hold a continuous contact proof in later slices.
type SweepContactTrack struct{}

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
	At           SweepInstant
	PoseA, PoseB r3.Transform
	FloatContact *ContactReport
	Ideal        SweepEvent
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
}

type affinePairPath struct {
	from, to  r3.Transform
	delta     [3]dyadic // displacement over the full path, in millimetres
	duration  *big.Rat  // exact seconds represented by the input value
	supported bool
}

func exactBaseValue(v units.Value) (*big.Rat, bool) {
	m, f := floatRat(v.Mag()), floatRat(v.Unit().Factor())
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
			return out, nil
		}
		start, end := p.From.Translation(), p.To.Translation()
		out.delta = [3]dyadic{dySubScalar(mustDyOf(end.X), mustDyOf(start.X)),
			dySubScalar(mustDyOf(end.Y), mustDyOf(start.Y)), dySubScalar(mustDyOf(end.Z), mustDyOf(start.Z))}
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
			return out, nil
		}
		for i, v := range []units.Value{p.LinearVelocity.X, p.LinearVelocity.Y, p.LinearVelocity.Z} {
			base, ok := exactBaseValue(v)
			if !ok {
				return out, fmt.Errorf("%w: nonfinite drift velocity", ErrNotFinite)
			}
			full := new(big.Rat).Mul(base, out.duration)
			d, ok := dyOfRat(full)
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
	d := r3.NewVec(ratFloatNearest(new(big.Rat).Mul(p.delta[0].rat(), f)),
		ratFloatNearest(new(big.Rat).Mul(p.delta[1].rat(), f)),
		ratFloatNearest(new(big.Rat).Mul(p.delta[2].rat(), f)))
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
	bound := rationalFloatError(t, value)
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
// duration. Current continuous proofs cover source-certified boxes undergoing
// pure affine translation. Unsupported paths return SweepUndecided.
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
	boxA, okA := sourceBoxAtPose(a, pa.from)
	boxB, okB := sourceBoxAtPose(b, pb.from)
	if !okA || !okB {
		report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, nil
	}
	run := pairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb, req: req, report: report,
		boxA: boxA, boxB: boxB}
	return run.execute(ctx, resolution)
}

type pairSweepRun struct {
	doc        *Document
	a, b       *Body
	pa, pb     affinePairPath
	req        SweepRequest
	report     *SweepReport
	boxA, boxB sourceBoxContactProof
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
	sample := SweepSample{At: at, PoseA: poseA, PoseB: poseB, FloatContact: contact, Ideal: event}
	r.report.Samples = append(r.report.Samples, sample)
	r.report.PoseEvaluations++
	return &sample, nil
}

// idealContact classifies the exact source boxes at the dyadic path fraction.
// Its manifold is an internal feature proof; only transferManifold can publish
// the manifold returned by the real ContactPair query.
func (r *pairSweepRun) idealContact(f *big.Rat, at SweepInstant) SweepEvent {
	fraction, ok := dyOfRat(f)
	if !ok {
		return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
	}
	a, b := r.boxA, r.boxB
	for i := range 3 {
		moveA, moveB := dyMul(r.pa.delta[i], fraction), dyMul(r.pb.delta[i], fraction)
		a.lo[i], a.hi[i] = dyAdd(a.lo[i], moveA), dyAdd(a.hi[i], moveA)
		b.lo[i], b.hi[i] = dyAdd(b.lo[i], moveB), dyAdd(b.hi[i], moveB)
	}
	report := &ContactReport{A: r.a, B: r.b, Request: r.req.ContactRequest}
	classifySourceBoxes(report, a, b)
	return SweepEvent{At: at, Relation: report.Relation, Gap: report.Gap,
		Overlap: report.Overlap, Manifold: report.Manifold, Reason: report.Reason}
}

// transferManifold checks that the float query and ideal boxes select the same
// source faces and normal. The float manifold's witnesses are then widened by
// the exact L1 pose discrepancy. Tangential clipped corners can depend on
// either body, so each point receives the sum of both pose discrepancies.
func (r *pairSweepRun) transferManifold(f *big.Rat, poseA, poseB r3.Transform,
	contact *ContactReport, event *SweepEvent) {
	ideal := event.Manifold
	event.Manifold = nil
	if ideal == nil || contact.Manifold == nil || event.Relation != contact.Relation ||
		len(ideal.Points) != len(contact.Manifold.Points) {
		if ideal != nil {
			event.Reason = ContactNoNormalProof
		}
		return
	}
	for i, idealPoint := range ideal.Points {
		actual := contact.Manifold.Points[i]
		if actual.FaceA != idealPoint.FaceA || actual.FaceB != idealPoint.FaceB ||
			actual.FeatureA != idealPoint.FeatureA || actual.FeatureB != idealPoint.FeatureB ||
			actual.Normal.Value != idealPoint.Normal.Value ||
			actual.Normal.Bound.Base() != 0 || actual.NormalAngle.Base() != 0 {
			event.Reason = ContactNoNormalProof
			return
		}
	}
	observedA, okA := sourceBoxAtPose(r.a, poseA)
	observedB, okB := sourceBoxAtPose(r.b, poseB)
	if !okA || !okB {
		event.Reason = ContactPayloadUnsupported
		return
	}
	deviation := boxPoseDeviation(r.boxA, observedA, r.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(r.boxB, observedB, r.pb.delta, f))
	resolution, ok := exactBaseValue(r.req.PointResolution)
	if !ok {
		event.Reason = ContactPointTooCoarse
		return
	}
	points := append([]ContactPoint(nil), contact.Manifold.Points...)
	for i := range points {
		point := &points[i]
		for _, witness := range []*VecMeasurement{&point.OnA, &point.OnB} {
			bound := new(big.Rat).Add(floatRat(witness.Bound.Base()), deviation)
			if bound.Cmp(resolution) > 0 {
				event.Reason = ContactPointTooCoarse
				return
			}
			publishedBound := ratFloatUp(bound)
			if publishedBound > r.req.PointResolution.Base() {
				event.Reason = ContactPointTooCoarse
				return
			}
			witness.Bound = units.Millimeters(publishedBound)
			witness.Exactness = exactnessFromBound(witness.Bound.Base())
		}
		separationBound := new(big.Rat).Add(floatRat(point.Separation.Bound.Base()), deviation)
		point.Separation.Bound = units.Millimeters(ratFloatUp(separationBound))
		point.Separation.Exactness = exactnessFromBound(point.Separation.Bound.Base())
	}
	event.Manifold = &ContactManifold{Points: points}
	event.Reason = ContactNoReason
}

// boxPoseDeviation bounds the L1 distance between a float-pose source box and
// the ideal affine box. Each support endpoint is compared as an exact rational.
func boxPoseDeviation(start, observed sourceBoxContactProof, delta [3]dyadic, f *big.Rat) *big.Rat {
	total := new(big.Rat)
	for i := range 3 {
		move := new(big.Rat).Mul(delta[i].rat(), f)
		idealLo := new(big.Rat).Add(start.lo[i].rat(), move)
		idealHi := new(big.Rat).Add(start.hi[i].rat(), move)
		lo := new(big.Rat).Sub(observed.lo[i].rat(), idealLo)
		hi := new(big.Rat).Sub(observed.hi[i].rat(), idealHi)
		lo.Abs(lo)
		hi.Abs(hi)
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		total.Add(total, lo)
	}
	return total
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
	if new(big.Rat).SetInt(floorIdx).Cmp(scaled) == 0 {
		leftIdx.Sub(leftIdx, big.NewInt(1))
		rightIdx.Sub(rightIdx, big.NewInt(1))
	}
	leftF := new(big.Rat).SetFrac(leftIdx, grid)
	rightF := new(big.Rat).SetFrac(rightIdx, grid)
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
		if dyCmp(r.boxA.hi[axis], r.boxB.lo[axis]) == 0 {
			if dySubScalar(r.pb.delta[axis], r.pa.delta[axis]).sign() > 0 {
				return true
			}
		}
		if dyCmp(r.boxB.hi[axis], r.boxA.lo[axis]) == 0 {
			if dySubScalar(r.pa.delta[axis], r.pb.delta[axis]).sign() > 0 {
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
		constraints := [2][2]dyadic{
			{dySubScalar(r.boxA.hi[axis], r.boxB.lo[axis]), dySubScalar(r.pa.delta[axis], r.pb.delta[axis])},
			{dySubScalar(r.boxB.hi[axis], r.boxA.lo[axis]), dySubScalar(r.pb.delta[axis], r.pa.delta[axis])},
		}
		for _, c := range constraints {
			start, slope := c[0], c[1]
			if slope.isZero() {
				if start.sign() < 0 {
					return entry, exit, false
				}
				continue
			}
			root := new(big.Rat).Quo(dyNeg(start).rat(), slope.rat())
			if slope.sign() > 0 && root.Cmp(entry) > 0 {
				entry = root
			}
			if slope.sign() < 0 && root.Cmp(exit) < 0 {
				exit = root
			}
		}
	}
	return entry, exit, entry.Cmp(exit) <= 0
}
