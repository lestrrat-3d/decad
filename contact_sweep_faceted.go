package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is the general rotating sweep of docs/multibody-dynamics-design.md
// §10.1: rotationalPairSweep's search, travel bound and pose deviation run
// over the exact vertex set of two §9 planar bodies instead of eight source-box
// corners. Each sample's relation and gap come from ContactPair's exact planar
// relation at the rounded pose, and the float-to-ideal deviation is the largest
// outward vertex distance (pointDeviation). Affine paths take the same run with
// a zero angular term.

// sweepPlanarPair runs the general sweep when both bodies are admitted §9
// planar solids. It reports false, leaving report untouched, when either is
// not. A path whose ideal motion or travel bound cannot be formed is
// SweepUndecided with SweepMissingBound.
func (d *Document) sweepPlanarPair(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, resolution *big.Rat,
	report *SweepReport) (*SweepReport, bool, error) {
	budget := newWorkBudget(ctx)
	solidA, okA, err := planarSolidAtPose(ctx, budget, a, r3.Identity())
	if err != nil {
		return nil, true, err
	}
	solidB, okB, err := planarSolidAtPose(ctx, budget, b, r3.Identity())
	if err != nil {
		return nil, true, err
	}
	if !okA || !okB {
		return nil, false, nil
	}
	aPath, okA := preparePlanarSweepPath(a, pa, &solidA)
	bPath, okB := preparePlanarSweepPath(b, pb, &solidB)
	if !okA || !okB {
		report.Outcome, report.Cause = SweepUndecided, SweepMissingBound
		report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), pa.duration),
			To: sweepInstant(big.NewRat(1, 1), pa.duration)}
		return report, true, nil
	}
	run := rotationalPairSweep{doc: d, a: aPath, b: bPath, req: req,
		resolution: resolution, report: report, planar: true}
	result, err := run.execute(ctx)
	return result, true, err
}

// preparePlanarSweepPath pairs a body's ideal motion with its identity-pose
// planar snapshot. The snapshot's vertices are the source points: ContactPair
// stages the same vertices through the query pose, and the body is their hull's
// subset, so they bound its deviation (pointDeviation) and its coordinate span
// (cornerSpan).
func preparePlanarSweepPath(body *Body, path affinePairPath, solid *pair.PlanarSolid) (rotationalSweepPath, bool) {
	prepared, ok := prepareSweepMotion(body, path)
	if !ok {
		return rotationalSweepPath{}, false
	}
	prepared.solid = solid
	prepared.sourcePoints = solid.Verts
	prepared.startPoints = make([]proofarith.DyV3, len(solid.Verts))
	for i, v := range solid.Verts {
		prepared.startPoints[i] = exactContactTransform(path.from, v)
	}
	return prepared, true
}

// planarIdealEvent transfers the float-pose planar relation to the ideal path
// (contact-sweep §3). With zero deviation on both bodies the rounded pose is
// the ideal pose and the report transfers whole. Otherwise a gap transfers
// with both deviations charged; an overlap transfers only through a vertex of
// one body that lies inside the other farther than both deviations from its
// boundary (pair.PlanarDeepVertex); a touch cannot survive a nonzero
// deviation and stays undecided.
func (r *rotationalPairSweep) planarIdealEvent(ctx context.Context, f *big.Rat, at SweepInstant,
	poseA, poseB r3.Transform, contact *ContactReport) (SweepEvent, error) {
	event := SweepEvent{At: at, Relation: ContactUndecided, Reason: contact.Reason}
	budget := newWorkBudget(ctx)
	vertsA, etaA, okA, err := r.a.pointDeviation(poseA, f, budget.step)
	if err != nil {
		return SweepEvent{}, err
	}
	vertsB, etaB, okB, err := r.b.pointDeviation(poseB, f, budget.step)
	if err != nil {
		return SweepEvent{}, err
	}
	if !okA || !okB {
		return event, nil
	}
	if etaA == 0 && etaB == 0 &&
		(contact.Relation == ContactTouching || contact.Relation == ContactOverlapping) {
		event.Relation, event.Gap, event.Overlap = contact.Relation, contact.Gap, contact.Overlap
		event.Manifold = contact.Manifold
		return event, nil
	}
	switch contact.Relation {
	case ContactSeparated:
		if gap, ok := separatedIdealGap(contact, etaA, etaB); ok {
			event.Relation, event.Gap, event.Reason = ContactSeparated, gap, ContactNoReason
		}
	case ContactOverlapping:
		if !positiveAffine(poseA) || !positiveAffine(poseB) {
			return event, nil
		}
		margin := proofarith.DyAdd(proofarith.MustDyOf(etaA), proofarith.MustDyOf(etaB))
		a := pair.PlanarSolid{Verts: vertsA, Tris: r.a.solid.Tris}
		b := pair.PlanarSolid{Verts: vertsB, Tris: r.b.solid.Tris}
		deep, err := pair.PlanarDeepVertex(&a, &b, margin, budget.step)
		if err != nil {
			return SweepEvent{}, err
		}
		if deep {
			event.Relation = ContactOverlapping
		}
	}
	return event, nil
}
