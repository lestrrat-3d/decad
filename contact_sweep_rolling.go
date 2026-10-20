package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file runs the rolling band track of docs/multibody-dynamics-design.md
// §10.4 ("Rolling"): it picks the cylinder and planar sides, reads the start
// relation through ContactPair, and publishes the track that
// internal/planarsweep proves (planarsweep.ProveRollingBand) and certifies
// (planarsweep.RollingTrack). It stays in the root because it calls
// Document.ContactPair and builds the public SweepReport and
// SweepContactTrack.

// rollingPairSweep continues a source cylinder's ruling touch, or its placed
// band (classifyPlacedRuling), on an exact planar body. paths are in sweep
// order.
type rollingPairSweep struct {
	doc        *Document
	paths      [2]rotationalSweepPath
	m, s       int
	cylinder   placedCylinder // M staged through its start pose
	req        reportvocab.SweepRequest
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
	pa, pb affinePairPath, req reportvocab.SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	m, bodyM, bodyS, pathM, pathS := 0, a, b, pa, pb
	start, ok := placedCylinderAt(a, pa.From)
	if !ok {
		m, bodyM, bodyS, pathM, pathS = 1, b, a, pb, pa
		if start, ok = placedCylinderAt(b, pb.From); !ok {
			return nil, false, nil
		}
	}
	if pathM.Drift == nil || pathM.Screw != nil || pathS.Drift != nil || pathS.Screw != nil {
		return nil, false, nil
	}
	solid, delta, ok, err := planarpose.SolidAtPose(ctx, proofbound.NewWorkBudget(ctx), verifyAllMesher{}, bodyNode(bodyS), r3.Identity(), req.HeldChord.Base())
	if err != nil {
		return nil, true, err
	}
	if !ok || delta.Sign() != 0 {
		return nil, false, nil
	}
	preparedM, okM := planarsweep.PrepareRollingPath(&bodyM.n, pathM, &start.Cylinder)
	preparedS, okS := planarsweep.PreparePlanarPath(&bodyS.n, pathS, &solid, delta)
	if !okM || !okS {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.Duration),
			To: sweepInstant(big.NewRat(1, 1), pa.Duration)}
		return report, true, nil
	}
	planarsweep.AttachMemos(&preparedM, &preparedS)
	defer planarsweep.CloseMemos(&preparedM, &preparedS)
	run := &rollingPairSweep{doc: d, m: m, s: 1 - m, cylinder: start, req: req, report: report,
		resolution: resolution}
	run.paths[run.m] = rotationalSweepPath{BodyPath: preparedM, body: bodyM}
	run.paths[run.s] = rotationalSweepPath{BodyPath: preparedS, body: bodyS}
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

// rollingEnds are the centers of a source cylinder's two end disks.
func rollingEnds(c contactround.CylinderProof) [2]proofarith.DyV3 {
	half := proofarith.MustDyOf(.5)
	var center proofarith.DyV3
	for k := range 3 {
		center[k] = proofarith.DyMul(proofarith.DyAdd(c.Box.Lo[k], c.Box.Hi[k]), half)
	}
	ends := [2]proofarith.DyV3{center, center}
	ends[0][c.Axis], ends[1][c.Axis] = c.Box.Lo[c.Axis], c.Box.Hi[c.Axis]
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
		case reportvocab.StopAtInitialContact:
			r.report.Outcome, r.report.Event = SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		case reportvocab.ContinueCertifiedTouch:
			track, ok, err := r.track(ctx, first)
			if err != nil {
				return nil, err
			}
			if !ok {
				return r.undecided(zero, one, SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack, r.report.Outcome = track, SweepPersistentBand
			if track.rolling.Band == nil {
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
// the start both rounded poses are the ideal ones, which PointDeviation
// confirms before the relation transfers whole.
func (r *rollingPairSweep) sample(ctx context.Context) (*SweepSample, error) {
	zero := new(big.Rat)
	var poses [2]r3.Transform
	exact := true
	for i := range r.paths {
		pose, err := r.paths[i].Path.RoundedPoseAt(zero)
		if err != nil {
			return nil, err
		}
		_, bound, ok, _ := r.paths[i].PointDeviation(pose, zero, planarsweep.NoPoll)
		exact = exact && ok && bound == 0
		poses[i] = pose
	}
	contact, err := r.doc.ContactPair(ctx, r.paths[0].body, r.paths[1].body, poses[0], poses[1],
		contactRequestFromInternal(r.req.ContactRequest))
	if err != nil {
		return nil, err
	}
	at := sweepInstant(zero, r.paths[0].Path.Duration)
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
	duration := r.paths[0].Path.Duration
	r.report.Outcome, r.report.Cause = SweepUndecided, cause
	r.report.Unresolved = &SweepInterval{From: sweepInstant(from, duration), To: sweepInstant(to, duration)}
	return r.report
}

// track proves the rolling band (planarsweep.ProveRollingBand) and builds
// the public track. The start manifold ContactPair published must name the
// same two faces, and the track's own start manifold must publish.
func (r *rollingPairSweep) track(ctx context.Context, first *SweepSample) (*SweepContactTrack, bool, error) {
	paths := [2]planarsweep.BodyPath{r.paths[0].BodyPath, r.paths[1].BodyPath}
	band, ok, err := planarsweep.ProveRollingBand(paths, r.m, r.cylinder.Cylinder, r.resolution,
		proofbound.NewWorkBudget(ctx).Step)
	if err != nil || !ok {
		return nil, false, err
	}
	faces := r.paths[r.s].body.Faces()
	if band.Support.Face.ID < 0 || band.Support.Face.ID >= len(faces) {
		return nil, false, nil
	}
	featureS := ContactFeature{Face: faces[band.Support.Face.ID]}
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
	proof, ok := band.Track(r.cylinder.source, contactFeatureToInternal(featureM), contactFeatureToInternal(featureS),
		r.req.NormalResolution)
	if !ok {
		return nil, false, nil
	}
	track := &SweepContactTrack{start: new(big.Rat), end: new(big.Rat).Set(band.End),
		duration: r.paths[r.m].Path.Duration, request: r.req.ContactRequest,
		normal: vecMeasurementFromInternal(proof.Direction), rolling: proof, pointCount: len(r.cylinder.source)}
	track.features[r.m], track.features[r.s] = proof.FeatureM, proof.FeatureS
	// A track whose start manifold is refused is not published; the refusal
	// is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	return track, true, nil
}
