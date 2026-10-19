package planarsweep

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/placedruling"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepdeparture"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// BodyPath is one body's prepared sweep path: its ideal motion and travel
// bound, and the exact source points whose images bound the float-to-ideal
// pose deviation. A source box carries its eight corners. An exact planar
// body carries every vertex of its snapshot. A rolling cylinder carries its
// identity box's corners as source points and its end-disk centers as start
// points.
type BodyPath struct {
	sweeppath.Motion
	StartPoints  []proofarith.DyV3   // exact source points under the path's From
	SourcePoints []proofarith.DyV3   // exact source points at the identity query pose
	Solid        *planar.PlanarSolid // the identity-pose planar snapshot, planar paths only
	HeldDelta    proofarith.Dyadic   // the snapshot's held displacement δ (multibody §10.4), planar paths only
	Memo         *sweepmemo.PathMemo // the run's memo (AttachMemos); nil outside a run
}

// PrepareBodyPath reads node's radius for sweeppath's motion plan. The
// caller supplies the source and start points.
func PrepareBodyPath(node *topo.Body, path sweeppath.AffinePath) (BodyPath, bool) {
	motion, ok := sweeppath.PrepareMotion(path, func(from r3.Transform, center r3.Vec,
		axis motionbound.RatVec) (*big.Rat, bool) {
		return sweeppath.RotationalSweepRadius(node, from, center, axis)
	})
	if !ok {
		return BodyPath{}, false
	}
	return BodyPath{Motion: motion}, true
}

// PreparePlanarPath pairs node's ideal motion with its identity-pose planar
// snapshot and its held displacement δ (multibody §10.4, zero for an exact
// body). The snapshot's vertices are the source points: ContactPair stages
// the same vertices through the query pose, and the held body is their hull's
// subset, so they bound its deviation (PointDeviation) and its coordinate
// span (CornerSpan). The true body lies within δ of that hull.
func PreparePlanarPath(node *topo.Body, path sweeppath.AffinePath, solid *planar.PlanarSolid,
	delta proofarith.Dyadic) (BodyPath, bool) {
	prepared, ok := PrepareBodyPath(node, path)
	if !ok {
		return BodyPath{}, false
	}
	prepared.Solid, prepared.HeldDelta = solid, delta
	prepared.SourcePoints = solid.Verts
	prepared.StartPoints = make([]proofarith.DyV3, len(solid.Verts))
	for i, v := range solid.Verts {
		prepared.StartPoints[i] = proofarith.DvTransform(path.From, v)
	}
	return prepared, true
}

// PrepareRollingPath pairs node's ideal motion with the eight corners of the
// cylinder's identity box as source points and its two end-disk centers at
// the path's From as start points.
func PrepareRollingPath(node *topo.Body, path sweeppath.AffinePath, c *placedruling.Cylinder) (BodyPath, bool) {
	prepared, ok := PrepareBodyPath(node, path)
	if !ok {
		return BodyPath{}, false
	}
	for i := range 8 {
		var corner proofarith.DyV3
		for k := range 3 {
			corner[k] = c.BoxLo[k]
			if i&(1<<k) != 0 {
				corner[k] = c.BoxHi[k]
			}
		}
		prepared.SourcePoints = append(prepared.SourcePoints, corner)
	}
	prepared.StartPoints = append(prepared.StartPoints, c.Centers[:]...)
	return prepared, true
}

// AttachMemos gives each of a run's paths its own memo. CloseMemos closes
// them. Each sweep entry point attaches before its run and closes, deferred,
// before it returns.
func AttachMemos(paths ...*BodyPath) {
	for _, path := range paths {
		path.Memo = sweepmemo.NewPathMemo()
	}
}

// CloseMemos closes each path's memo. A copy of a path made during the run
// shares the memo, so after this it computes afresh and holds no memory.
func CloseMemos(paths ...*BodyPath) {
	for _, path := range paths {
		if path.Memo != nil {
			path.Memo.Close()
		}
	}
}

// NoPoll is the poll of a reading that charges no work budget.
func NoPoll() error { return nil }

// Departure is the departure proofs' view of the path whose start box is box.
func (p BodyPath) Departure(box pairbox.OrientedBox) sweepdeparture.Path {
	return sweepdeparture.Path{
		Box: box, Delta: p.Path.Delta,
		Axis: p.Frame.Axis, Center: p.Frame.Center, Velocity: p.Velocity,
		Duration: p.Path.Duration, OmegaUpper: p.OmegaHigh,
		Drift: p.Path.Drift != nil, Screw: p.Path.Screw != nil,
	}
}

// SupportPath is the support-plane search's view of a planar path.
func (p *BodyPath) SupportPath() SupportPath {
	return SupportPath{
		Points: p.StartPoints, Solid: p.Solid,
		MovingOwner: p.Path.Drift != nil || p.Path.Screw != nil,
		Motion: MotionInput{
			Points: p.StartPoints, Delta: p.Path.Delta, Duration: p.Path.Duration,
			Velocity: p.Velocity, Frame: p.Frame,
			Drift: p.Path.Drift != nil, Screw: p.Path.Screw != nil,
		},
	}
}

// IdealAt is the ideal pose at fraction f. A rotating path's rotation
// encloses the turn ω·t over both rate bounds, and its shift is the pivot's
// image R·(t(From) − c) + c + v·t. ok is false only when the From basis does
// not read as points, which motionbound.ExactTransform never produces.
func (p BodyPath) IdealAt(f *big.Rat) (sweepmemo.IdealPose, bool) {
	if !p.Memo.Open() {
		return p.idealAtAfresh(f)
	}
	key := sweepmemo.RatKey(f)
	if ideal, ok := p.Memo.LoadIdeal(key); ok {
		return ideal, true
	}
	ideal, ok := p.idealAtAfresh(f)
	if ok {
		p.Memo.StoreIdeal(key, ideal)
	}
	return ideal, ok
}

// idealAtAfresh is IdealAt computed without the memo.
func (p BodyPath) idealAtAfresh(f *big.Rat) (sweepmemo.IdealPose, bool) {
	rot, shift, ok := motionbound.SweepIdealPoseAt(p.FromRot, p.FromT, p.Path.Delta,
		p.Frame, p.Velocity, p.OmegaLow, p.OmegaHigh, p.Path.Duration, f, p.Path.Drift != nil)
	return sweepmemo.IdealPose{Rot: rot, Shift: shift}, ok
}

// RoundedBox compares the exact staged source corners used by ContactPair
// with their ideal path positions and returns the rounded pose's box and the
// corners' deviation bound. The source points must be a box's eight corners.
// ok is false when the bound is not finite or an edge is zero.
func (p BodyPath) RoundedBox(pose r3.Transform, f *big.Rat) (pairbox.OrientedBox, float64, bool) {
	points, bound, ok, _ := p.PointDeviation(pose, f, NoPoll)
	if !ok {
		return pairbox.OrientedBox{}, 0, false
	}
	var box pairbox.OrientedBox
	copy(box.Corner[:], points)
	box.Edge = [3]proofarith.DyV3{proofarith.DvSub(box.Corner[1], box.Corner[0]),
		proofarith.DvSub(box.Corner[2], box.Corner[0]), proofarith.DvSub(box.Corner[4], box.Corner[0])}
	for _, edge := range box.Edge {
		if proofarith.DvIsZero(edge) {
			return pairbox.OrientedBox{}, 0, false
		}
	}
	return box, bound, true
}

// ReplayDeviation is what replay charges one rounded pose at f: the staged
// points' distance bound from the ideal path (PointDeviation), and for a
// positive-displacement planar body the multibody §10.4 transfer charge
// (motionbound.SweepTransferCharge). A source-box path holds δ zero and
// charges nothing.
func (p BodyPath) ReplayDeviation(pose r3.Transform, f *big.Rat) (*big.Rat, *big.Rat, bool) {
	ideal, ok := p.IdealAt(f)
	if !ok {
		return nil, nil, false
	}
	var bound float64
	if p.Memo.Open() {
		_, bound, ok, _ = p.PointDeviation(pose, f, NoPoll)
	} else {
		_, bound, ok, _ = p.pointDeviationFrom(pose, ideal, NoPoll)
	}
	if !ok {
		return nil, nil, false
	}
	charge, ok := motionbound.SweepTransferCharge(pose, ideal.Rot, p.HeldDelta)
	if !ok {
		return nil, nil, false
	}
	return proofarith.FloatRat(bound), charge, true
}

// PointDeviation maps the exact source points through the read query pose,
// staged exactly as ContactPair stages them, and bounds their distance from
// their ideal path positions (contact-sweep §3). The rounded and ideal bodies
// are two affine images of one source, so their difference is affine and the
// largest point distance bounds every point of the points' hull. This charges
// placement and pose-composition rounding without trusting a rounded composed
// transform. poll is charged once per point.
//
// Each point's squared distance bound is Σ max(|o − hi|, |o − lo|)² over the
// axes, o the staged coordinate and [lo, hi] the ideal one,
// rot·p + shift, evaluated exactly in the common-denominator form (see
// motionbound.ScaledIvMat): the source and staged points share one q, and
// every point's sum shares one denominator, so points compare by numerator.
//
// A memo hit charges poll once per point, as the reading it replaces did.
func (p BodyPath) PointDeviation(pose r3.Transform, f *big.Rat,
	poll func() error) ([]proofarith.DyV3, float64, bool, error) {
	if !pose.IsValid() {
		return nil, 0, false, nil
	}
	var key sweepmemo.DeviationKey
	if p.Memo.Open() {
		key = sweepmemo.DeviationKeyOf(p.SourcePoints, pose, f)
		if entry, ok := p.Memo.LoadDeviation(key); ok {
			for range p.SourcePoints {
				if err := poll(); err != nil {
					return nil, 0, false, err
				}
			}
			return entry.Points, entry.Bound, entry.OK, nil
		}
	}
	ideal, ok := p.IdealAt(f)
	if !ok {
		return nil, 0, false, nil
	}
	points, bound, ok, err := p.pointDeviationFrom(pose, ideal, poll)
	if err == nil && p.Memo.Open() {
		p.Memo.StoreDeviation(key, sweepmemo.Deviation{Points: points, Bound: bound, OK: ok})
	}
	return points, bound, ok, err
}

// pointDeviationFrom is PointDeviation against an already enclosed ideal pose.
func (p BodyPath) pointDeviationFrom(pose r3.Transform, ideal sweepmemo.IdealPose,
	poll func() error) ([]proofarith.DyV3, float64, bool, error) {
	if !pose.IsValid() {
		return nil, 0, false, nil
	}
	actual, squared, err := motionbound.SweepPointDeviation(p.SourcePoints, pose, ideal.Rot, ideal.Shift, poll)
	if err != nil {
		return nil, 0, false, err
	}
	bound := proofbound.RatSqrtUp(squared)
	return actual, bound, measurement.FiniteValues(bound), nil
}

// CornerSpan encloses each start point's ideal path over the fraction span
// [from, to]. An affine path moves a point by delta·f. A rotating one takes
// the point at the span's mid time, rotationMid·(p − c) + c + v·t_mid, grown
// on each axis by its largest speed over the span, |v + ω × rotationSpan·(p −
// c)| per axis, times the half span: the per-point arithmetic runs in the
// common-denominator form, so every endpoint is the exact rational the
// interval expression denotes.
func (p BodyPath) CornerSpan(from, to *big.Rat) sweepmemo.CornerSpans {
	if !p.Memo.Open() {
		return p.cornerSpanAfresh(from, to)
	}
	key := sweepmemo.SpanKeyOf(p.StartPoints, from, to)
	if spans, ok := p.Memo.LoadSpans(key); ok {
		return spans
	}
	spans := p.cornerSpanAfresh(from, to)
	p.Memo.StoreSpans(key, spans)
	return spans
}

// cornerSpanAfresh is CornerSpan computed without the memo.
func (p BodyPath) cornerSpanAfresh(from, to *big.Rat) sweepmemo.CornerSpans {
	spans := motionbound.SweepCornerSpan(p.StartPoints, p.Path.Delta, p.Frame, p.Velocity,
		p.OmegaLow, p.OmegaHigh, p.Path.Duration, from, to, p.Path.Drift != nil)
	return sweepmemo.CornerSpans{Den: spans.Den, Lo: spans.Lo, Hi: spans.Hi}
}
