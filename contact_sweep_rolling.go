package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the rolling band track of docs/multibody-dynamics-design.md
// §10.4 ("Rolling"): a full source cylinder M lying on
// its side on an exact planar body S, M drifting at constant ω and S only
// translating, keeps a §10.3 track whose contact set is the two ends of the
// contact ruling.
//
// A cylinder is no vertex hull, so the proof reads the centers c± of its two
// end disks, material points on its axis, staged exactly through the start
// pose's float basis B (placedCylinder). B is orthonormal only to rounding, so
// the start body's section is the disk's image under B; gram bounds that, and
// α = n̂·Bâ is the staged axis column's normal component. Over an end disk
// the least height along the unit normal n̂ is n̂·c − r·|P·Bᵀ(u)n̂|, B(u) =
// R(u)·B and P removing the identity axis; over the solid it is the lesser of
// its two disks', since height is affine along the axis. With S's plane at
// offset d and H(u) = n̂·c(u) − d − r, each rim's least height is g(u) = H(u) +
// r·(1 − |P·Bᵀ(u)n̂|). With ã = Bâ and β = |ω×ã|:
//
//	H'(0) = n̂·(v_M − v_S + ω×(c − c_M))           exact
//	|H''| <= |ω|·|ω×(c − c_M)|                    c'' = R(u)·(ω×(ω×(c − c_M))); S only translates
//	|r − r·|P·Bᵀ(u)n̂|| <= r·(gram + α(u)²)        sectionDrift
//	|α(u)| <= |α| + β·u                           |R(u)ã − ã| <= β·u
//
// so each end satisfies |g(u)| <= c₀ + (|H'(0)| + 2·r·β·|α|)·u + (K + r·β²)·u²
// on [0, h], c₀ = |H(0)| + r·(gram + α²) and K half the curvature bound, and
// the band depth is that bound at h for the larger coefficients of the two
// ends. The cylinder's lowest point is one of the two rims', so the whole body
// stays above −depth. Each rim's lowest point lies within
// r·(3·gram + (3/2)·|α(u)|) of c − r·n̂ while |α(u)| <= 1/4 (rimDrift); the
// foot check and ManifoldAt charge that drift. A signed-axis start with α = 0
// has gram = c₀ = 0, and its drift r·√(2·(1 − s)) <= (3/2)·r·β·u, s the cosine
// of the axis tilt, holds with no gate.
//
// S's plane may be face-local (§10.6), a tray's floor with walls in front of
// it: the band search then runs the column test at every grid fraction, so no
// material of S in front of the plane comes near the cylinder inside the
// track.

// rollingTrackProof is the private certificate of a rolling band or exact
// rolling touch track. It owns copies of both prepared paths. M's source
// points are the eight corners of its identity disk-by-interval box, whose
// hull holds the whole cylinder, so their deviation bounds every cylinder
// point's.
type rollingTrackProof struct {
	paths        [2]rotationalSweepPath
	m, s         int
	ends         [2]proofarith.DyV3 // M's end-disk centers at the identity pose, in published order
	radius       *big.Rat
	normal       proofarith.DyV3 // S's exact unit outward normal, a signed axis
	origin       int             // an S vertex on the plane
	featureM     ContactFeature
	featureS     ContactFeature
	direction    VecMeasurement // the published A-to-B normal
	angle        units.Value
	coefficients rollingCoefficients
	depth        *big.Rat // mm
	depthUp      float64
	band         *Measurement
}

// rollingPairSweep continues a source cylinder's ruling touch, or its placed
// band (classifyPlacedRuling), on an exact planar body. paths are in sweep
// order.
type rollingPairSweep struct {
	doc        *Document
	paths      [2]rotationalSweepPath
	m, s       int
	cylinder   placedCylinder // M staged through its start pose
	req        SweepRequest
	report     *SweepReport
	resolution *big.Rat
}

// sweepRollingPair runs the rolling track when one body is a full source
// cylinder, at a start pose with a positive determinant, whose path rotates
// and the other an exact planar body with no
// held displacement whose path only translates. It reports false, leaving
// report untouched, when the pair is not one.
//
// The two sides are named locals, not a local array indexed by m: Go 1.26's
// compiler kept pointers it made into such an array live while the array
// itself was neither live nor a stack object, so the collector freed a
// path's duration that nothing else held.
func (d *Document) sweepRollingPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	m, bodyM, bodyS, pathM, pathS := 0, a, b, pa, pb
	start, ok := placedCylinderAt(a, pa.from)
	if !ok {
		m, bodyM, bodyS, pathM, pathS = 1, b, a, pb, pa
		if start, ok = placedCylinderAt(b, pb.from); !ok {
			return nil, false, nil
		}
	}
	if pathM.drift == nil || pathM.screw != nil || pathS.drift != nil || pathS.screw != nil {
		return nil, false, nil
	}
	solid, delta, ok, err := planarSolidAtPose(ctx, proofbound.NewWorkBudget(ctx), bodyS, r3.Identity(), heldChordOf(req.ContactRequest))
	if err != nil {
		return nil, true, err
	}
	if !ok || delta.Sign() != 0 {
		return nil, false, nil
	}
	preparedM, okM := prepareRollingSweepPath(bodyM, pathM, &start)
	preparedS, okS := preparePlanarSweepPath(bodyS, pathS, &solid, delta)
	if !okM || !okS {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, true, nil
	}
	run := &rollingPairSweep{doc: d, m: m, s: 1 - m, cylinder: start, req: req, report: report,
		resolution: resolution}
	run.paths[run.m], run.paths[run.s] = preparedM, preparedS
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, true, err
	}
	if result.ContactTrack != nil {
		result.replay = &sweepReplayProof{rotation: &[2]rotationalSweepPath{run.paths[0], run.paths[1]},
			track: result.ContactTrack, request: req.ContactRequest, outcome: result.Outcome}
	}
	return result, true, nil
}

// prepareRollingSweepPath pairs the cylinder's ideal motion with the eight
// corners of its identity box as source points and its two end-disk centers
// at the path's From as start points.
func prepareRollingSweepPath(body *Body, path affinePairPath,
	source *placedCylinder) (rotationalSweepPath, bool) {
	prepared, ok := prepareSweepMotion(body, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	box := source.box
	for i := range 8 {
		var corner proofarith.DyV3
		for k := range 3 {
			corner[k] = box.lo[k]
			if i&(1<<k) != 0 {
				corner[k] = box.hi[k]
			}
		}
		prepared.sourcePoints = append(prepared.sourcePoints, corner)
	}
	prepared.startPoints = append(prepared.startPoints, source.centers[:]...)
	return prepared, true
}

// rollingEnds are the centers of a source cylinder's two end disks.
func rollingEnds(c sourceCylinderContactProof) [2]proofarith.DyV3 {
	half := proofarith.MustDyOf(.5)
	var center proofarith.DyV3
	for k := range 3 {
		center[k] = proofarith.DyMul(proofarith.DyAdd(c.box.lo[k], c.box.hi[k]), half)
	}
	ends := [2]proofarith.DyV3{center, center}
	ends[0][c.axis], ends[1][c.axis] = c.box.lo[c.axis], c.box.hi[c.axis]
	return ends
}

func (r *rollingPairSweep) execute(ctx context.Context) (*SweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx)
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case ContactOverlapping:
		r.report.Outcome, r.report.InitialEvent, r.report.Event =
			SweepInitiallyOverlapping, &first.Ideal, &first.Ideal
		return r.report, nil
	case ContactTouching, ContactBand:
		// A ContactBand start is a placed cylinder whose pose basis rounds
		// its section (classifyPlacedRuling); the track charges that band.
		r.report.InitialEvent = &first.Ideal
		switch r.req.StartPolicy {
		case StopAtInitialContact:
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		case ContinueCertifiedTouch:
			track, ok, err := r.band(ctx, first)
			if err != nil {
				return nil, err
			}
			if !ok {
				return r.undecided(zero, one, SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
			if track.rolling.band == nil {
				r.report.Outcome = SweepPersistentTouch
			}
			return r.report, nil
		default:
			return r.undecided(zero, one, SweepDepartureUnproved), nil
		}
	case ContactSeparated:
		// No clear search covers a rotating cylinder.
		return r.undecided(zero, one, SweepMissingBound), nil
	default:
		return r.undecided(zero, zero, SweepPoseRelation), nil
	}
}

// sample reads the start relation through ContactPair at both From poses. At
// the start both rounded poses are the ideal ones, which pointDeviation
// confirms before the relation transfers whole.
func (r *rollingPairSweep) sample(ctx context.Context) (*SweepSample, error) {
	zero := new(big.Rat)
	var poses [2]r3.Transform
	exact := true
	for i := range r.paths {
		pose, err := r.paths[i].poseAt(zero)
		if err != nil {
			return nil, err
		}
		_, bound, ok, _ := r.paths[i].pointDeviation(pose, zero, noSweepPoll)
		exact = exact && ok && bound == 0
		poses[i] = pose
	}
	contact, err := r.doc.ContactPair(ctx, r.paths[0].body, r.paths[1].body, poses[0], poses[1], r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweepInstant(zero, r.paths[0].path.duration)
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	if exact {
		event.Relation, event.Gap, event.Overlap = contact.Relation, contact.Gap, contact.Overlap
		event.Manifold = contact.Manifold
	}
	sample := &SweepSample{At: at, PoseA: poses[0], PoseB: poses[1], FloatContact: contact,
		Ideal: event, exactFraction: zero}
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

func (r *rollingPairSweep) undecided(from, to *big.Rat, cause SweepCause) *SweepReport {
	duration := r.paths[0].path.duration
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, duration), To: sweepInstant(to, duration)}
	return r.report
}

// support finds the plane of S the cylinder's ruling rests on
// (rulingSupport): ContactPair reads the same plane at the start poses.
func (r *rollingPairSweep) support(poll func() error) (rulingPlane, bool, error) {
	return rulingSupport(&r.cylinder, &r.paths[r.s], poll)
}

// dyUnitAxis reads an exact vector with exactly one nonzero component as its
// axis and the matching signed unit vector.
func dyUnitAxis(n proofarith.DyV3) (int, proofarith.DyV3, bool) {
	axis := -1
	for k := range 3 {
		if n[k].Sign() == 0 {
			continue
		}
		if axis >= 0 {
			return 0, proofarith.DyV3{}, false
		}
		axis = k
	}
	if axis < 0 {
		return 0, proofarith.DyV3{}, false
	}
	var unit proofarith.DyV3
	unit[axis] = proofarith.DyInt(int64(n[axis].Sign()))
	return axis, unit, true
}

// rollingCoefficients are the depth bound's terms, constant + rate·t +
// quadratic·t², and the rim drift's, lateralBase + lateral·t. With ã the
// staged axis column, α = n̂·ã, β = |ω×ã| and both ends' H±:
//
//	constant  = max|H±(0)| + r·gram + r·α²   (sectionDrift)
//	rate      = max|H±'(0)| + 2·r·β·|α|
//	quadratic = max K + r·β²
//
// since |α(u)| <= |α| + β·u. The drift is r·(3·gram + (3/2)·|α(u)|)
// (rimDrift), which needs gram <= 1/16 and |α(u)| <= 1/4. An exact start, a
// signed-axis pose with α = 0, has a zero constant and lateralBase, and its
// drift r·√(2·(1 − s)) <= (3/2)·r·β·u holds with no gate.
type rollingCoefficients struct {
	constant, rate, quadratic *big.Rat
	lateralBase, lateral      *big.Rat
	alpha, beta               *big.Rat // |α| and an upper bound on β
	gated                     bool     // |α| + β·t <= 1/4 must hold through t
}

func (r *rollingPairSweep) coefficients(support rulingPlane) (rollingCoefficients, bool) {
	motionM, okM := planarMotionOf(&r.paths[r.m])
	motionS, okS := planarMotionOf(&r.paths[r.s])
	if !okM || !okS || motionS.rotating || proofarith.DyCmp(r.cylinder.gram, rulingGramLimit) > 0 {
		return rollingCoefficients{}, false
	}
	radius := r.cylinder.radius.Rat()
	normal := ratOfDyV3(support.normal)
	relative := ratSub3(motionM.velocity, motionS.velocity)
	out := rollingCoefficients{rate: new(big.Rat), quadratic: new(big.Rat), constant: new(big.Rat),
		alpha: new(big.Rat).Abs(support.alpha.Rat())}
	for i, c := range r.paths[r.m].startPoints {
		lever := ratCross3(motionM.omega, ratSub3(ratOfDyV3(c), motionM.center))
		rate := ratDot3(normal, ratAdd3(relative, lever))
		if rate.Abs(rate).Cmp(out.rate) > 0 {
			out.rate = rate
		}
		curvature, ok := ratSqrtUpRat(proofbound.RatMul(motionM.omegaSq, ratDot3(lever, lever)))
		if !ok {
			return rollingCoefficients{}, false
		}
		if k := proofbound.RatMul(curvature, big.NewRat(1, 2)); k.Cmp(out.quadratic) > 0 {
			out.quadratic = k
		}
		if h := new(big.Rat).Abs(support.heights[i].Rat()); h.Cmp(out.constant) > 0 {
			out.constant = h
		}
	}
	// β² = |ω|²·|ã|² − (ω·ã)², exact.
	axis := ratOfDyV3(r.cylinder.columns[r.cylinder.axis])
	along := ratDot3(motionM.omega, axis)
	tiltSq := new(big.Rat).Sub(proofbound.RatMul(motionM.omegaSq, ratDot3(axis, axis)), proofbound.RatMul(along, along))
	tilt, ok := ratSqrtUpRat(tiltSq)
	if !ok {
		return rollingCoefficients{}, false
	}
	out.beta = tilt
	out.constant = proofbound.RatAdd(out.constant, r.cylinder.sectionDrift(support.alpha).Rat())
	out.rate = proofbound.RatAdd(out.rate, proofbound.RatMul(big.NewRat(2, 1), radius, tilt, out.alpha))
	out.quadratic = proofbound.RatAdd(out.quadratic, proofbound.RatMul(radius, tiltSq))
	out.lateral = proofbound.RatMul(big.NewRat(3, 2), radius, tilt)
	out.lateralBase = new(big.Rat)
	if r.cylinder.gram.Sign() != 0 || support.alpha.Sign() != 0 {
		out.gated = true
		out.lateralBase = r.cylinder.rimDrift(support.alpha.Rat())
	}
	return out, true
}

// at is the band depth and the rim drift at t seconds.
func (c rollingCoefficients) at(t *big.Rat) (*big.Rat, *big.Rat) {
	return proofbound.RatAdd(c.constant, proofbound.RatMul(c.rate, t), proofbound.RatMul(c.quadratic, t, t)),
		proofbound.RatAdd(c.lateralBase, proofbound.RatMul(c.lateral, t))
}

// driftAdmitted reports whether the rim drift's gate holds through t.
func (c rollingCoefficients) driftAdmitted(t *big.Rat) bool {
	return !c.gated || proofbound.RatAdd(c.alpha, proofbound.RatMul(c.beta, t)).Cmp(big.NewRat(1, 4)) <= 0
}

// band proves the rolling track. Each end's foot must stay inside S's face:
// the end centers' ideal path boxes over [0, f], less S's translation and
// grown by the depth and the rim drift, hold every rim point near the plane,
// and their union holds the ruling between the ends. Projected along the
// normal's axis, the union must meet no bounding edge of the face and have a
// corner inside one of its triangles. On a face-local plane the column test
// (column) must hold through f as well.
func (r *rollingPairSweep) band(ctx context.Context, first *SweepSample) (*SweepContactTrack, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	support, ok, err := r.support(budget.Step)
	if err != nil || !ok {
		return nil, false, err
	}
	coefficients, ok := r.coefficients(support)
	if !ok {
		return nil, false, nil
	}
	duration := r.paths[r.m].path.duration
	box, ok := r.columnBox()
	if !ok {
		return nil, false, nil
	}
	holds := func(f *big.Rat) (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		t := new(big.Rat).Mul(f, duration)
		if !coefficients.driftAdmitted(t) {
			return false, nil
		}
		depth, lateral := coefficients.at(t)
		if !r.footInside(support, f, proofbound.RatAdd(depth, lateral)) {
			return false, nil
		}
		return r.column(support, box, f, budget.Step)
	}
	end, ok, err := sweepGridHorizon(r.resolution, duration, holds)
	if err != nil || !ok {
		return nil, false, err
	}
	depth, _ := coefficients.at(new(big.Rat).Mul(end, duration))
	return r.track(first, support, coefficients, end, depth)
}

// rollingColumnBox holds what the column test reads of the cylinder: its
// eight staged corners at the start, whose hull holds it, and reach, an upper
// bound on r·√(1 + gram), the farthest any cylinder point lies from the point
// of its axis segment in the same section: that offset is B·w with |w| <= r,
// and |B·w|² <= (1 + gram)·|w|².
type rollingColumnBox struct {
	corners []proofarith.DyV3
	reach   *big.Rat
}

func (r *rollingPairSweep) columnBox() (rollingColumnBox, bool) {
	corners := r.cylinder.stagedCorners()
	stretch, ok := ratSqrtUpRat(proofbound.RatAdd(big.NewRat(1, 1), r.cylinder.gram.Rat()))
	if !ok {
		return rollingColumnBox{}, false
	}
	return rollingColumnBox{corners: corners[:], reach: proofbound.RatMul(r.cylinder.radius.Rat(), stretch)}, true
}

// column is §10.6's column test through fraction f on a face-local plane.
// Two boxes hold the cylinder at every instant of [0, f]: the coordinate box
// of its staged corners' ideal paths, and the hull of its end centers' ideal
// path boxes grown by box.reach, since every cylinder point lies within reach
// of a point between the end centers. Their intersection, less S's own
// translation over the span, holds the cylinder in S's start frame, and every
// S triangle with a vertex strictly in front of the plane must project apart
// from it. The corners' interval enclosure loosens past a quarter turn while
// the end centers of a cylinder turning about its own axis barely move, so
// the second box carries a whole turn. Both grow with f, so the test is
// monotone. A plane with all of S behind it needs no test.
func (r *rollingPairSweep) column(support rulingPlane, box rollingColumnBox, f *big.Rat,
	poll func() error) (bool, error) {
	if !support.local {
		return true, nil
	}
	zero := new(big.Rat)
	path := r.paths[r.m]
	centers := path.cornerSpan(zero, f)
	path.startPoints = box.corners
	corners := path.cornerSpan(zero, f)
	S := &r.paths[r.s]
	var lo, hi [3]*big.Rat
	for axis := range 3 {
		low, high := corners.hull(axis)
		axisLow, axisHigh := centers.hull(axis)
		low = ratMax(low, new(big.Rat).Sub(axisLow, box.reach))
		high = ratMin(high, new(big.Rat).Add(axisHigh, box.reach))
		shift := new(big.Rat).Mul(S.path.delta[axis].Rat(), f)
		lo[axis] = new(big.Rat).Sub(low, ratMax(shift, zero))
		hi[axis] = new(big.Rat).Sub(high, ratMin(shift, zero))
	}
	solid := pair.PlanarSolid{Verts: S.startPoints, Tris: S.solid.Tris}
	_, apart, err := pair.PlanarColumnClear(&solid, support.normal, S.startPoints[support.origin], lo, hi, poll)
	return apart, err
}

func (r *rollingPairSweep) footInside(support rulingPlane, f, growth *big.Rat) bool {
	S := &r.paths[r.s]
	spans := r.paths[r.m].cornerSpan(new(big.Rat), f)
	var lo, hi [2]*big.Rat
	for slot, axis := range [2]int{(support.axis + 1) % 3, (support.axis + 2) % 3} {
		low, high := spans.hull(axis)
		shift := new(big.Rat).Mul(S.path.delta[axis].Rat(), f)
		shiftLo, shiftHi := ratMin(shift, new(big.Rat)), ratMax(shift, new(big.Rat))
		lo[slot] = proofbound.RatAdd(low, new(big.Rat).Neg(shiftHi), new(big.Rat).Neg(growth))
		hi[slot] = proofbound.RatAdd(high, new(big.Rat).Neg(shiftLo), growth)
	}
	return support.face.holdsBox(lo, hi)
}

// track builds the public track. The start manifold ContactPair published
// must name the same two faces, and the track's own start manifold must
// publish.
func (r *rollingPairSweep) track(first *SweepSample, support rulingPlane,
	coefficients rollingCoefficients, end, depth *big.Rat) (*SweepContactTrack, bool, error) {
	S, M := &r.paths[r.s], &r.paths[r.m]
	faces := S.body.Faces()
	if support.face.id < 0 || support.face.id >= len(faces) {
		return nil, false, nil
	}
	featureS := ContactFeature{Face: faces[support.face.id]}
	featureM := ContactFeature{Face: r.cylinder.wall}
	if first.Ideal.Manifold != nil {
		for _, point := range first.Ideal.Manifold.Points {
			onM, onS := point.FeatureB, point.FeatureA
			if r.m == 0 {
				onM, onS = point.FeatureA, point.FeatureB
			}
			if onM.Face != featureM.Face || onS.Face != featureS.Face {
				return nil, false, nil
			}
		}
	}
	direction := support.normal
	if r.m == 0 {
		direction = proofarith.DvSub(proofarith.DyV3{}, direction)
	}
	normal, angle, ok := planarNormal(direction)
	if !ok || angle.Base() > r.req.NormalResolution.Base() {
		return nil, false, nil
	}
	// The ends publish in the exact coordinate order of their start rims, as
	// ContactPair publishes the ruling; a rigid motion keeps that pairing.
	ends := r.cylinder.source
	starts := [2]proofarith.DyV3{M.startPoints[0], M.startPoints[1]}
	if ordered := orderedRulingEnds(starts); !sameDyV3(ordered[0], starts[0]) {
		ends = [2]proofarith.DyV3{ends[1], ends[0]}
	}
	proof := &rollingTrackProof{paths: r.paths, m: r.m, s: r.s, ends: ends, radius: r.cylinder.radius.Rat(),
		normal: support.normal, origin: support.origin, featureM: featureM, featureS: featureS,
		direction: normal, angle: angle, coefficients: coefficients, depth: depth, depthUp: proofbound.RatFloatUp(depth)}
	if !finiteMeasurementValues(proof.depthUp) {
		return nil, false, nil
	}
	if depth.Sign() > 0 || end.Cmp(big.NewRat(1, 1)) < 0 {
		value := ratFloatNearest(depth)
		bound := proofarith.RationalFloatError(depth, value)
		proof.band = &Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessFromBound(bound)}
	}
	track := &SweepContactTrack{start: new(big.Rat), end: new(big.Rat).Set(end), duration: M.path.duration,
		request: r.req.ContactRequest, normal: normal, rolling: proof, pointCount: len(ends)}
	track.features[r.m], track.features[r.s] = featureM, featureS
	// A track whose start manifold is refused is not published; the refusal
	// is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	return track, true, nil
}

// depthThrough is the band depth over [0, f]: every coefficient's term is
// nondecreasing in time, so their value at f bounds every earlier instant.
func (p *rollingTrackProof) depthThrough(f *big.Rat) *big.Rat {
	depth, _ := p.coefficients.at(new(big.Rat).Mul(f, p.paths[p.m].path.duration))
	return depth
}

// rounded stages both bodies through their float poses at f and returns each
// pose with its deviation and S's staged vertices. S must keep its start
// basis, so its rounded plane keeps its normal.
func (p *rollingTrackProof) rounded(f *big.Rat) ([2]r3.Transform, [2]*big.Rat, []proofarith.DyV3, bool) {
	var poses [2]r3.Transform
	var eta [2]*big.Rat
	var vertsS []proofarith.DyV3
	for i := range p.paths {
		pose, err := p.paths[i].poseAt(f)
		if err != nil {
			return poses, eta, nil, false
		}
		if i == p.s && pose.Basis() != p.paths[i].path.from.Basis() {
			return poses, eta, nil, false
		}
		points, bound, ok, _ := p.paths[i].pointDeviation(pose, f, noSweepPoll)
		if !ok {
			return poses, eta, nil, false
		}
		if i == p.s {
			vertsS = points
		}
		poses[i], eta[i] = pose, proofarith.FloatRat(bound)
	}
	return poses, eta, vertsS, true
}

// manifoldAt publishes the two ruling ends at an exact fraction. Each end
// center is staged through M's rounded pose and lowered by r along the
// normal; the true rim point lies within M's pose deviation plus the lateral
// drift of it, and its foot on S's rounded plane within S's deviation more.
// The separation is the band's two-sided bound.
func (p *rollingTrackProof) manifoldAt(f *big.Rat, req ContactRequest) (*ContactManifold, error) {
	poses, eta, vertsS, ok := p.rounded(f)
	if !ok {
		return nil, fmt.Errorf("%w: rolling contact track pose has no finite bound", ErrUnsupported)
	}
	resolution, okResolution := exactBaseValue(req.PointResolution)
	if !okResolution {
		return nil, fmt.Errorf("%w: rolling contact track resolution is invalid", ErrUnsupported)
	}
	n := ratOfDyV3(p.normal)
	q := ratOfDyV3(vertsS[p.origin])
	_, lateral := p.coefficients.at(new(big.Rat).Mul(f, p.paths[p.m].path.duration))
	separation := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(p.depthUp),
		Exactness: exactnessFromBound(p.depthUp)}
	points := make([]ContactPoint, 0, len(p.ends))
	for _, end := range p.ends {
		center := ratOfDyV3(exactContactTransform(poses[p.m], end))
		var rim, foot [3]*big.Rat
		for k := range 3 {
			rim[k] = new(big.Rat).Sub(center[k], new(big.Rat).Mul(p.radius, n[k]))
		}
		height := ratDot3(n, ratSub3(ratVec(rim), q))
		for k := range 3 {
			foot[k] = new(big.Rat).Sub(rim[k], new(big.Rat).Mul(height, n[k]))
		}
		onM, okM := planarTrackPoint(rim, proofbound.RatAdd(eta[p.m], lateral), resolution)
		onS, okS := planarTrackPoint(foot, proofbound.RatAdd(eta[p.m], eta[p.s], lateral), resolution)
		if !okM || !okS {
			return nil, fmt.Errorf("%w: rolling contact track point exceeds point resolution", ErrUnsupported)
		}
		point := ContactPoint{Normal: p.direction, NormalAngle: p.angle, Separation: separation}
		if p.m == 0 {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = p.featureM, p.featureS
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = p.featureS, p.featureM
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	return &ContactManifold{Points: points}, nil
}

// certifiedRollingPosesAtFraction replays a rolling track without rerunning
// the pair relation. Both rounded poses must fit PointResolution of their
// ideal poses, and each rounded end center's height less r above S's rounded
// plane must stay within the depth widened by both deviations: the ideal one
// lies within the depth, and the deviations bound the move to the rounded
// one. It can only refuse; the producer's certificate covers the ideal path.
func (r *SweepReport) certifiedRollingPosesAtFraction(f *big.Rat) (r3.Transform, r3.Transform, error) {
	p := r.replay.track.rolling
	poses, eta, vertsS, ok := p.rounded(f)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rolling replay pose has no finite error bound", ErrUnsupported)
	}
	deviation := new(big.Rat).Add(eta[0], eta[1])
	resolution, ok := exactBaseValue(r.replay.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rolling replay pose exceeds point resolution", ErrUnsupported)
	}
	limit := new(big.Rat).Add(p.depth, deviation)
	q := vertsS[p.origin]
	for _, end := range p.ends {
		height := proofarith.DvDot(p.normal, proofarith.DvSub(exactContactTransform(poses[p.m], end), q)).Rat()
		height.Sub(height, p.radius)
		if height.Abs(height).Cmp(limit) > 0 {
			return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded rolling replay pose leaves the certified band", ErrUnsupported)
		}
	}
	return poses[0], poses[1], nil
}
