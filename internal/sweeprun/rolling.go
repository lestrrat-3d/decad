package sweeprun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactruling"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file runs the rolling band track of agent-docs/multibody-dynamics-design.md
// §10.4 ("Rolling"): it picks the cylinder and planar sides, reads the start
// relation through ContactPair, and publishes the track that
// internal/planarsweep proves (planarsweep.ProveRollingBand) and certifies
// (planarsweep.RollingTrack). It stays in the root because it calls
// Document.ContactPair and builds the public sweepReport and
// SweepContactTrack.

// rollingPairSweep continues a source cylinder's ruling touch, or its placed
// band (contactruling.ClassifyPlacedRuling), on an exact planar body. paths are in sweep
// order.
type rollingPairSweep struct {
	hooks      Hooks
	paths      [2]rotationalSweepPath
	m, s       int
	cylinder   contactruling.PlacedCylinder // M staged through its start pose
	req        vocab.SweepRequest
	report     *sweepReport
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
func sweepRollingPair(ctx context.Context, h Hooks, a, b *topo.Body,
	pa, pb affinePairPath, req vocab.SweepRequest, resolution *big.Rat,
	report *sweepReport) (*sweepReport, bool, error) {
	m, bodyM, bodyS, pathM, pathS := 0, a, b, pa, pb
	start, ok := contactruling.PlacedCylinderAt(a, pa.From)
	if !ok {
		m, bodyM, bodyS, pathM, pathS = 1, b, a, pb, pa
		if start, ok = contactruling.PlacedCylinderAt(b, pb.From); !ok {
			return nil, false, nil
		}
	}
	if pathM.Drift == nil || pathM.Screw != nil || pathS.Drift != nil || pathS.Screw != nil {
		return nil, false, nil
	}
	solid, delta, ok, err := planarpose.SolidAtPose(ctx, proofbound.NewWorkBudget(ctx), h.Mesher, bodyS, r3.Identity(), req.HeldChord.Base())
	if err != nil {
		return nil, true, err
	}
	if !ok || delta.Sign() != 0 {
		return nil, false, nil
	}
	preparedM, okM := planarsweep.PrepareRollingPath(bodyM, pathM, &start.Cylinder)
	preparedS, okS := planarsweep.PreparePlanarPath(bodyS, pathS, &solid, delta)
	if !okM || !okS {
		report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepMissingBound
		report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), pa.Duration),
			To: sweeppath.Instant(big.NewRat(1, 1), pa.Duration)}
		return report, true, nil
	}
	planarsweep.AttachMemos(&preparedM, &preparedS)
	defer planarsweep.CloseMemos(&preparedM, &preparedS)
	run := &rollingPairSweep{hooks: h, m: m, s: 1 - m, cylinder: start, req: req, report: report,
		resolution: resolution}
	run.paths[run.m] = rotationalSweepPath{BodyPath: preparedM, body: bodyM}
	run.paths[run.s] = rotationalSweepPath{BodyPath: preparedS, body: bodyS}
	result, err := run.execute(ctx)
	if err != nil || result == nil {
		return result, true, err
	}
	if result.ContactTrack != nil {
		result.SetReplay(seal.Key{}, &sweepproof.Proof{Rotation: &[2]sweepproof.RotationalPath{run.paths[0].RotationalPath, run.paths[1].RotationalPath},
			Track: trackOf(result.ContactTrack), Request: req.ContactRequest, Outcome: result.Outcome})
	}
	return result, true, nil
}

func (r *rollingPairSweep) execute(ctx context.Context) (*sweepReport, error) {
	zero, one := new(big.Rat), big.NewRat(1, 1)
	first, err := r.sample(ctx)
	if err != nil {
		return nil, err
	}
	switch first.Ideal.Relation {
	case vocab.ContactOverlapping:
		r.report.Outcome, r.report.InitialEvent, r.report.Event =
			vocab.SweepInitiallyOverlapping, &first.Ideal, &first.Ideal
		return r.report, nil
	case vocab.ContactTouching, vocab.ContactBand:
		// A ContactBand start is a placed cylinder whose pose basis rounds
		// its section (contactruling.ClassifyPlacedRuling); the track charges that band.
		r.report.InitialEvent = &first.Ideal
		switch r.req.StartPolicy {
		case vocab.StopAtInitialContact:
			r.report.Outcome, r.report.Event = vocab.SweepInitiallyTouching, &first.Ideal
			return r.report, nil
		case vocab.ContinueCertifiedTouch:
			track, ok, err := r.track(ctx, first)
			if err != nil {
				return nil, err
			}
			if !ok {
				return r.undecided(zero, one, vocab.SweepContactTrackUnproved), nil
			}
			r.report.ContactTrack, r.report.Outcome = nodeTrack(track), vocab.SweepPersistentBand
			if track.Rolling.Band == nil {
				r.report.Outcome = vocab.SweepPersistentTouch
			}
			return r.report, nil
		default:
			return r.undecided(zero, one, vocab.SweepDepartureUnproved), nil
		}
	case vocab.ContactSeparated:
		// No clear search covers a rotating cylinder.
		return r.undecided(zero, one, vocab.SweepMissingBound), nil
	default:
		return r.undecided(zero, zero, vocab.SweepPoseRelation), nil
	}
}

// sample reads the start relation through ContactPair at both From poses. At
// the start both rounded poses are the ideal ones, which PointDeviation
// confirms before the relation transfers whole.
func (r *rollingPairSweep) sample(ctx context.Context) (*sweepSample, error) {
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
	contact, err := r.hooks.Contact(ctx, poses[0], poses[1],
		r.req.ContactRequest)
	if err != nil {
		return nil, err
	}
	at := sweeppath.Instant(zero, r.paths[0].Path.Duration)
	event := sweepEvent{At: at, Relation: vocab.ContactUndecided, Reason: contact.Reason}
	if exact {
		event.Relation, event.Gap, event.Overlap = contact.Relation, contact.Gap, contact.Overlap
		event.Manifold = contact.Manifold
	}
	sample := sampleWithPtr(sweepSample{At: at, PoseA: poses[0], PoseB: poses[1], FloatContact: contact,
		Ideal: event}, zero)
	r.report.Samples = append(r.report.Samples, *sample)
	r.report.PoseEvaluations++
	return sample, nil
}

func (r *rollingPairSweep) undecided(from, to *big.Rat, cause vocab.SweepCause) *sweepReport {
	duration := r.paths[0].Path.Duration
	r.report.Outcome, r.report.Cause = vocab.SweepUndecided, cause
	r.report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(from, duration), To: sweeppath.Instant(to, duration)}
	return r.report
}

// track proves the rolling band (planarsweep.ProveRollingBand) and builds
// the public track. The start manifold ContactPair published must name the
// same two faces, and the track's own start manifold must publish.
func (r *rollingPairSweep) track(ctx context.Context, first *sweepSample) (*sweepproof.Track, bool, error) {
	paths := [2]planarsweep.BodyPath{r.paths[0].BodyPath, r.paths[1].BodyPath}
	band, ok, err := planarsweep.ProveRollingBand(paths, r.m, r.cylinder.Cylinder, r.resolution,
		proofbound.NewWorkBudget(ctx).Step)
	if err != nil || !ok {
		return nil, false, err
	}
	faces := topobuild.Faces(r.paths[r.s].body)
	if band.Support.Face.ID < 0 || band.Support.Face.ID >= len(faces) {
		return nil, false, nil
	}
	featureS := contactFeature{Face: faces[band.Support.Face.ID]}
	featureM := contactFeature{Face: r.cylinder.Wall}
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
	proof, ok := band.Track(r.cylinder.Source, featureM, featureS,
		r.req.NormalResolution)
	if !ok {
		return nil, false, nil
	}
	track := &sweepproof.Track{From: new(big.Rat), To: new(big.Rat).Set(band.End),
		Duration: r.paths[r.m].Path.Duration, Request: r.req.ContactRequest,
		NormalRead: proof.Direction, Rolling: proof, PointCount: len(r.cylinder.Source)}
	track.Sources[r.m], track.Sources[r.s] = proof.FeatureM, proof.FeatureS
	// A track whose start manifold is refused is not published; the refusal
	// is the answer.
	if _, refused := track.ManifoldAt(units.Scalar(0)); refused != nil {
		return nil, false, nil //nolint:nilerr // a refused manifold withholds the track, it is no failure
	}
	return track, true, nil
}
