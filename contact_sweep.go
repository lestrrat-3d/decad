package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sweeprun"
	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/units"
)

type (
	// PairPath names one body's motion during a two-body sweep.
	PairPath = report.PairPath
	// PoseSegment joins two placements relative to the body's current placement.
	PoseSegment = report.PoseSegment
	// QuantityVec carries three components of one physical kind.
	QuantityVec = report.QuantityVec
	// RigidDriftSegment moves a center linearly while its orientation rotates.
	RigidDriftSegment = report.RigidDriftSegment
)

// SweepStartPolicy selects what to prove when the bodies initially touch.
type SweepStartPolicy = report.SweepStartPolicy

const (
	StopAtInitialContact    = report.StopAtInitialContact
	ContinueSeparatingTouch = report.ContinueSeparatingTouch
	ContinueCertifiedTouch  = report.ContinueCertifiedTouch
)

const (
	SweepClear                    = report.SweepClear
	SweepDepartedClear            = report.SweepDepartedClear
	SweepPersistentTouch          = report.SweepPersistentTouch
	SweepContactTransitionBracket = report.SweepContactTransitionBracket
	SweepImpactBracket            = report.SweepImpactBracket
	SweepInitiallyTouching        = report.SweepInitiallyTouching
	SweepInitiallyOverlapping     = report.SweepInitiallyOverlapping
	SweepUndecided                = report.SweepUndecided
	SweepGrazingTouch             = report.SweepGrazingTouch
	SweepPersistentBand           = report.SweepPersistentBand
)

const (
	SweepNoCause              = report.SweepNoCause
	SweepPoseRelation         = report.SweepPoseRelation
	SweepMissingBound         = report.SweepMissingBound
	SweepTimeFloor            = report.SweepTimeFloor
	SweepFractionFloor        = report.SweepFractionFloor
	SweepPoseBudget           = report.SweepPoseBudget
	SweepContactUnsupported   = report.SweepContactUnsupported
	SweepDepartureUnproved    = report.SweepDepartureUnproved
	SweepContactTrackUnproved = report.SweepContactTrackUnproved
	SweepEventUnrepresentable = report.SweepEventUnrepresentable
)

// SweepRequest bounds the time search and the contact geometry resolution.
//
// RestSpeed is a nonnegative Velocity; the zero Value rests no vertex. A band
// track over a support set (agent-docs/multibody-dynamics-design.md §10.5) holds a
// lifted vertex that closes on the support plane no faster than RestSpeed, or
// rises, on both sides of the plane, as it holds a contact vertex (§10.8), so
// the track does not end where that vertex's lower height bound reaches the
// plane. A departure rests nothing.
type SweepRequest = report.SweepRequest

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

// SweepOutcome states the certified relation over the requested path.
type SweepOutcome = report.SweepOutcome

// SweepCause explains why a continuous claim was not proved.
type SweepCause = report.SweepCause

type (
	// SweepInstant identifies a dyadic fraction of the requested duration.
	SweepInstant = report.SweepInstant
	// SweepInterval identifies an interval of the requested duration.
	SweepInterval = report.SweepInterval
	// SweepDeparture certifies positive separation after an initial touch.
	SweepDeparture = report.SweepDeparture
	// SweepEvent reports the ideal path relation at one sampled instant.
	SweepEvent = report.SweepEventOf[*Face, *Edge, *Vertex]
	// SweepSample keeps the query pose and the transferred pair finding.
	SweepSample = report.SweepSampleOf[*Body, *Face, *Edge, *Vertex]
	// SweepContactTrack owns the exact source geometry and affine motion of a
	// certified touching prefix. Its source face pointers are the original faces.
	SweepContactTrack = report.SweepContactTrackOf[*Face, *Edge, *Vertex]
	// SweepReport states the first certified event or the earliest unresolved span.
	SweepReport = report.SweepReportOf[*Body, *Face, *Edge, *Vertex, *sweepproof.Proof]
)

type affinePairPath = sweeppath.AffinePath

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
// or band track (agent-docs/multibody-dynamics-design.md §10.2, §10.3). A
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
	pa, err := sweeppath.Validate(pathA)
	if err != nil {
		return nil, err
	}
	pb, err := sweeppath.Validate(pathB)
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
	hooks := sweeprun.Hooks{
		Contact: func(ctx context.Context, poseA, poseB r3.Transform, request ContactRequest) (*nodeContactReport, error) {
			return d.contactNodes(ctx, a, b, poseA, poseB, request)
		},
		Mesher: verifyAllMesher{},
	}
	found := sweeprun.NewReport(bodyNode(a), bodyNode(b), pathA, pathB, req)
	result, err := sweeprun.Pair(ctx, hooks, bodyNode(a), bodyNode(b), pa, pb, req, resolution, found)
	return report.LiftSweepReport(seal.Key{}, result, &handleLift), err
}
