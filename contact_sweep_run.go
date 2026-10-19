package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type pairSweepRun struct {
	doc          *Document
	a, b         *Body
	pa, pb       affinePairPath
	req          reportvocab.SweepRequest
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
	poseA, err := r.pa.PoseAt(f)
	if err != nil {
		return nil, err
	}
	poseB, err := r.pb.PoseAt(f)
	if err != nil {
		return nil, err
	}
	contact, err := r.doc.ContactPair(ctx, r.a, r.b, poseA, poseB, contactRequestFromInternal(r.req.ContactRequest))
	if err != nil {
		return nil, err
	}
	at := sweepInstant(f, r.pa.Duration)
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
		moveA, moveB := proofarith.DyMul(r.pa.Delta[i], fraction), proofarith.DyMul(r.pb.Delta[i], fraction)
		a.lo[i], a.hi[i] = proofarith.DyAdd(a.lo[i], moveA), proofarith.DyAdd(a.hi[i], moveA)
		b.lo[i], b.hi[i] = proofarith.DyAdd(b.lo[i], moveB), proofarith.DyAdd(b.hi[i], moveB)
	}
	report := &reportvocab.ContactReport{A: bodyNode(r.a), B: bodyNode(r.b),
		Request: r.req.ContactRequest}
	if r.facetedFloor {
		floor, faceted := a, b
		if _, ok := r.a.n.Payload.(facetedPayload); ok {
			floor, faceted = b, a
		}
		if proofarith.DyCmp(faceted.lo[2], floor.hi[2]) < 0 {
			return SweepEvent{At: at, Relation: ContactUndecided, Reason: ContactPayloadUnsupported}
		}
	}
	classifySourceBoxes(report, a, b)
	return sweepEventOf(at, report)
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
	deviation := boxPoseDeviation(r.boxA, observedA, r.pa.Delta, f)
	deviation.Add(deviation, boxPoseDeviation(r.boxB, observedB, r.pb.Delta, f))
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
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
			publishedBound := proofbound.RatFloatUp(bound)
			if publishedBound > r.req.PointResolution.Base() {
				useIdeal()
				return
			}
			witness.Bound = units.Millimeters(publishedBound)
			witness.Exactness = exactnessFromBound(witness.Bound.Base())
		}
		separationBound := new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), deviation)
		point.Separation.Bound = units.Millimeters(proofbound.RatFloatUp(separationBound))
		point.Separation.Exactness = exactnessFromBound(point.Separation.Bound.Base())
	}
	event.Manifold = &ContactManifold{Points: points}
	event.Reason = ContactNoReason
}

// boxPoseDeviation bounds the L1 distance between a float-pose source box and
// the ideal affine box. Each support endpoint is compared as an exact rational.
func boxPoseDeviation(start, observed sourceBoxContactProof, delta [3]proofarith.Dyadic, f *big.Rat) *big.Rat {
	return box.PoseDeviation(start.axisBox(), observed.axisBox(), delta, f)
}

func translatedSourceBoxes(a, b sourceBoxContactProof, da, db [3]proofarith.Dyadic,
	f *big.Rat) (sourceBoxContactProof, sourceBoxContactProof, bool) {
	translatedA, translatedB, ok := box.TranslatePair(a.axisBox(), b.axisBox(), da, db, f)
	if !ok {
		return sourceBoxContactProof{}, sourceBoxContactProof{}, false
	}
	a.lo, a.hi = translatedA.Lo, translatedA.Hi
	b.lo, b.hi = translatedB.Lo, translatedB.Hi
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
	axis, side, ok := clearance.SignedAxis(normal.Value)
	if !ok || normal.Bound.Base() != 0 ||
		first.Ideal.Manifold.Points[0].NormalAngle.Base() != 0 ||
		proofarith.DyCmp(r.pa.Delta[axis], r.pb.Delta[axis]) != 0 {
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
			a, b, _ := translatedSourceBoxes(r.boxA, r.boxB, r.pa.Delta, r.pb.Delta, f)
			if proofarith.DyCmp(a.lo[i], b.hi[i]) >= 0 || proofarith.DyCmp(b.lo[i], a.hi[i]) >= 0 {
				return nil
			}
		}
		// A change of the lower or upper corner owner changes the patch
		// structure. The first full-span increment refuses those transitions.
		if box.AffineEqualityRootWithin(r.boxA.lo[i], r.pa.Delta[i], r.boxB.lo[i], r.pb.Delta[i], end) ||
			box.AffineEqualityRootWithin(r.boxA.hi[i], r.pa.Delta[i], r.boxB.hi[i], r.pb.Delta[i], end) {
			return nil
		}
	}
	if !box.TrackPointsWithin(r.boxA.axisBox(), r.boxB.axisBox(), r.pa.Delta, r.pb.Delta,
		r.req.PointResolution.Base()) {
		return nil
	}
	point := first.Ideal.Manifold.Points[0]
	return &SweepContactTrack{
		a: r.boxA, b: r.boxB, deltaA: r.pa.Delta, deltaB: r.pb.Delta,
		start: new(big.Rat), end: new(big.Rat).Set(end), duration: new(big.Rat).Set(r.pa.Duration),
		request:  r.req.ContactRequest,
		features: [2]reportvocab.ContactFeature{contactFeatureToInternal(point.FeatureA), contactFeatureToInternal(point.FeatureB)},
		normal:   normal,
	}
}

// sourceBoxTransitionRoot finds the first possible change of a projected
// clipped-patch owner or the first edge-contact limit. Roots at zero use the
// right-sided patch and do not restart the solver at its initial time.
func (r *pairSweepRun) sourceBoxTransitionRoot(first *SweepSample) *big.Rat {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) != 4 {
		return nil
	}
	axis, _, ok := clearance.SignedAxis(first.Ideal.Manifold.Points[0].Normal.Value)
	if !ok {
		return nil
	}
	return box.TransitionRoot(r.boxA.axisBox(), r.boxB.axisBox(), r.pa.Delta, r.pb.Delta, axis)
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
		if r.req.StartPolicy == reportvocab.StopAtInitialContact {
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		}
		if !box.ProvesDeparture(r.boxA.axisBox(), r.boxB.axisBox(), r.pa.Delta, r.pb.Delta) {
			if r.req.StartPolicy == reportvocab.ContinueCertifiedTouch {
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
					leftF, rightF, ok := box.ContactRootBracket(root, r.pa.Duration, resolution)
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
			if r.req.StartPolicy == reportvocab.ContinueCertifiedTouch {
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
	entry, exit, intersects := box.ContactSpan(r.boxA.axisBox(), r.boxB.axisBox(), r.pa.Delta, r.pb.Delta)
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
	step := new(big.Rat).Set(paDurationFraction(resolution, r.pa.Duration))
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
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.pa.Duration), To: sweepInstant(to, r.pa.Duration)}
	r.sortSamples()
	return r.report
}
