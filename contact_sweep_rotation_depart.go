package decad

import (
	"context"
	"errors"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/sweepdeparture"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/units"
)

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
	contact, err := r.doc.ContactPair(ctx, r.a.body, r.b.body, sample.PoseA, sample.PoseB, contactRequestFromInternal(req))
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
	if r.req.StartPolicy == reportvocab.ContinueCertifiedTouch {
		return SweepContactTrackUnproved
	}
	return SweepDepartureUnproved
}

// coMovingOrientedTouch admits a touch of two co-translating source boxes
// whose manifold is a box face patch. A manifold the exact planar path
// completed (an edge or vertex on a face, or a support set) is left to the
// planar continuation, since the oriented track rereads face patches only.
func (r *rotationalPairSweep) coMovingOrientedTouch(first *SweepSample) bool {
	if r.a.path.Drift != nil || r.b.path.Drift != nil || first.Ideal.Manifold == nil {
		return false
	}
	for _, point := range first.Ideal.Manifold.Points {
		if point.FeatureA.Face == nil || point.FeatureB.Face == nil {
			return false
		}
	}
	for axis := range 3 {
		if proofarith.DyCmp(r.a.path.Delta[axis], r.b.path.Delta[axis]) != 0 {
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
		for corner := range box.Corner {
			box.Corner[corner][axis] = proofarith.DyAdd(box.Corner[corner][axis], step)
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
	if r.a.path.Drift == nil || r.b.path.Drift == nil || first.Ideal.Manifold == nil ||
		len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	point := first.Ideal.Manifold.Points[0]
	axis, side, ok := clearance.SignedAxis(point.Normal.Value)
	if !ok || point.Normal.Bound.Base() != 0 || point.NormalAngle.Base() != 0 {
		return nil, false
	}
	boxA, okA := sourceBoxAtPose(r.a.body, r.a.path.From)
	boxB, okB := sourceBoxAtPose(r.b.body, r.b.path.From)
	if !okA || !okB ||
		(side == 1 && proofarith.DyCmp(boxA.hi[axis], boxB.lo[axis]) != 0) ||
		(side == 0 && proofarith.DyCmp(boxB.hi[axis], boxA.lo[axis]) != 0) {
		return nil, false
	}
	return sweepdeparture.CommonSpin([2]sweepdeparture.Path{r.a.departurePath(), r.b.departurePath()}, axis, side)
}

// obliqueAffineDepartureFraction proves an open-at-zero gap between the
// complete source boxes. Their common support plane is exact even though its
// published unit normal is rounded. Translation makes the signed plane gap
// affine, so a positive slope keeps the entire pair clear after time zero.
func (r *rotationalPairSweep) obliqueAffineDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil {
		return nil, false
	}
	return sweepdeparture.ObliqueAffine([2]sweepdeparture.Path{r.a.departurePath(), r.b.departurePath()},
		len(first.Ideal.Manifold.Points))
}

// A spin around the contact normal leaves both bodies' Z supports unchanged.
// Positive relative Z travel therefore opens a whole-body gap from exact touch.
func (r *rotationalPairSweep) horizontalSpinDepartureFraction(first *SweepSample) (*big.Rat, bool) {
	if first.Ideal.Manifold == nil || len(first.Ideal.Manifold.Points) == 0 {
		return nil, false
	}
	paths := [2]rotationalSweepPath{r.a, r.b}
	return sweepdeparture.HorizontalSpin(
		[2]sweepdeparture.Path{paths[0].departurePath(), paths[1].departurePath()},
		first.Ideal.Manifold.Points[0].Normal.Value,
		func(i int) (pairbox.OrientedBox, bool) {
			box, ok := sourceOrientedBoxAtPose(paths[i].body, paths[i].path.From)
			return box.OrientedBox, ok
		},
	)
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
	normals := make([]sweepdeparture.ContactNormal, len(first.Ideal.Manifold.Points))
	for i, point := range first.Ideal.Manifold.Points {
		normals[i] = sweepdeparture.ContactNormal{
			Value: point.Normal.Value,
			Exact: point.Normal.Bound.Base() == 0 && point.NormalAngle.Base() == 0,
		}
	}
	return sweepdeparture.TangentAxisSpin([2]sweepdeparture.Path{r.a.departurePath(), r.b.departurePath()}, normals)
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
	boxA, boxB := r.a.startBox.OrientedBox, r.b.startBox.OrientedBox
	var faceA, faceB pairbox.OrientedFace
	if !pairbox.OrientedAxisFace(&boxA, axis, sideA, &faceA) ||
		!pairbox.OrientedAxisFace(&boxB, axis, sideB, &faceB) ||
		proofarith.DyCmp(faceA.Origin[axis], faceB.Origin[axis]) != 0 {
		return nil, false
	}
	return sweepdeparture.AxisFace([2]sweepdeparture.Path{r.a.departurePath(), r.b.departurePath()}, axis, side)
}

func (r *rotationalPairSweep) refine(ctx context.Context, left, right *SweepSample,
	depth int) (bool, error) {
	lf, rf := proofarith.FloatRat(left.At.Fraction.Base()), proofarith.FloatRat(right.At.Fraction.Base())
	if left.Ideal.Relation == ContactSeparated && right.Ideal.Relation == ContactSeparated &&
		r.intervalClear(left, right, lf, rf) {
		return false, nil
	}
	width := new(big.Rat).Mul(new(big.Rat).Sub(rf, lf), r.a.path.Duration)
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
	if sweeppath.RatFloatNearest(middle) == left.At.Fraction.Base() ||
		sweeppath.RatFloatNearest(middle) == right.At.Fraction.Base() {
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
		if sweeppath.RatFloatNearest(middle) == left.At.Fraction.Base() ||
			sweeppath.RatFloatNearest(middle) == right.At.Fraction.Base() {
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
// PointResolution (sweepmemo.BracketDepthWithin), with g the left
// sample's proven lower gap and T both travel bounds per unit fraction. The
// deviation is the one replay charges, read at the same rounded poses. A left
// sample without a positive gap, or a right edge whose deviation has no finite
// bound, has nothing narrowing can settle and reports true.
func (r *rotationalPairSweep) bracketRightReplays(left, right *SweepSample) bool {
	gap := sampleLowerGap(left)
	resolution, ok := sweeppath.ExactBaseValue(r.req.PointResolution)
	if gap == nil || !ok {
		return true
	}
	lo, hi := proofarith.FloatRat(left.At.Fraction.Base()), proofarith.FloatRat(right.At.Fraction.Base())
	charge := new(big.Rat)
	for _, path := range [2]rotationalSweepPath{r.a, r.b} {
		pose, err := path.path.RoundedPoseAt(hi)
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
	return sweepmemo.BracketDepthWithin(hi, charge, resolution, lo, hi, gap,
		new(big.Rat).Add(r.a.fullTravel, r.b.fullTravel))
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
		value, okValue := sweeppath.ExactBaseValue(sample.Ideal.Gap.Value)
		bound, okBound := sweeppath.ExactBaseValue(sample.Ideal.Gap.Bound)
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
	if r.a.path.Drift != nil || r.b.path.Drift != nil {
		return false
	}
	a0, okA := translatedOrientedBox(r.a.startBox, r.a.path.Delta, from)
	b0, okB := translatedOrientedBox(r.b.startBox, r.b.path.Delta, from)
	a1, okC := translatedOrientedBox(r.a.startBox, r.a.path.Delta, to)
	b1, okD := translatedOrientedBox(r.b.startBox, r.b.path.Delta, to)
	if !okA || !okB || !okC || !okD {
		return false
	}
	pa0, pb0, pa1, pb1 := a0.OrientedBox, b0.OrientedBox, a1.OrientedBox, b1.OrientedBox
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := proofarith.DvCross(r.a.startBox.Edge[i], r.a.startBox.Edge[j])
		if proofarith.DvIsZero(normal) {
			continue
		}
		alo0, ahi0 := pairbox.OrientedProjection(pa0, normal)
		blo0, bhi0 := pairbox.OrientedProjection(pb0, normal)
		alo1, ahi1 := pairbox.OrientedProjection(pa1, normal)
		blo1, bhi1 := pairbox.OrientedProjection(pb1, normal)
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
	if a.Len() == 0 || b.Len() == 0 {
		return nil
	}
	displacement := proofarith.DyAdd(r.a.delta, r.b.delta).Rat()
	var widest *big.Rat
	for axis := range 3 {
		aLow, aHigh := a.Hull(axis)
		bLow, bHigh := b.Hull(axis)
		for _, gap := range []*big.Rat{new(big.Rat).Sub(bLow, aHigh), new(big.Rat).Sub(aLow, bHigh)} {
			gap.Sub(gap, displacement)
			if gap.Sign() > 0 && (widest == nil || gap.Cmp(widest) > 0) {
				widest = gap
			}
		}
	}
	return widest
}

// cornerSpan encloses each source point's ideal path over the fraction span
// [from, to]. An affine path moves a point by delta·f. A rotating one takes
// the point at the span's mid time, rotationMid·(p − c) + c + v·t_mid, grown
// on each axis by its largest speed over the span, |v + ω × rotationSpan·(p −
// c)| per axis, times the half span: the per-point arithmetic runs in the
// common-denominator form, so every endpoint is the exact rational the
// interval expression denotes.
func (p rotationalSweepPath) cornerSpan(from, to *big.Rat) cornerSpans {
	if !p.memo.Open() {
		return p.cornerSpanAfresh(from, to)
	}
	key := sweepmemo.SpanKeyOf(p.startPoints, from, to)
	if spans, ok := p.memo.LoadSpans(key); ok {
		return spans
	}
	spans := p.cornerSpanAfresh(from, to)
	p.memo.StoreSpans(key, spans)
	return spans
}

// cornerSpanAfresh is cornerSpan computed without the memo.
func (p rotationalSweepPath) cornerSpanAfresh(from, to *big.Rat) cornerSpans {
	spans := motionbound.SweepCornerSpan(p.startPoints, p.path.Delta, p.frame, p.velocity,
		p.omegaLow, p.omegaHigh, p.path.Duration, from, to, p.path.Drift != nil)
	return cornerSpans{Den: spans.Den, Lo: spans.Lo, Hi: spans.Hi}
}

func (r *rotationalPairSweep) sortSamples() {
	sort.Slice(r.report.Samples, func(i, j int) bool {
		return r.report.Samples[i].At.Fraction.Base() < r.report.Samples[j].At.Fraction.Base()
	})
}

func (r *rotationalPairSweep) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, r.a.path.Duration),
		To: sweepInstant(to, r.a.path.Duration)}
	r.sortSamples()
	return r.report
}
