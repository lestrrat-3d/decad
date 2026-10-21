package planarband

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/planarfeaturemap"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// bandPair is one admitted pair at least one of whose held meshes
// carries a positive displacement, with both displacements at the query
// poses (agent-docs/multibody-dynamics-design.md §10.4).
type bandPair struct {
	a, b           *planar.PlanarSolid
	deltaA, deltaB proofarith.Dyadic
	certificates   Convexity
}

// classify turns the exact held relation into §10.4's published one. With δ
// the summed displacement, every true boundary point lies within δ of the
// held pair's: a held gap within the lifted band b = max(SupportBand, δ) over
// which the zero-δ body hosts a lifted set is a band carrying that set
// (liftedBand); a held gap whose lower end exceeds δ is a true gap with δ
// charged; a held touch, or a held gap or penetration depth at most δ, puts
// the true signed separation in [−2δ, 2δ], ContactBand; a held vertex deeper
// than δ inside the other held body is a true overlap (planar.PlanarDeepVertex)
// carrying the held penetration patch charged with δ (overlapManifold).
// Anything between is undecided.
func (p *bandPair) classify(ctx context.Context, budget *proofbound.WorkBudget, report *contactReport,
	result planar.PlanarResult) error {
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	switch result.Relation {
	case pair.Separated:
		value, bound := proofarith.FloatRat(result.Gap.ValueMM), proofarith.FloatRat(result.Gap.BoundMM)
		upper := new(big.Rat).Add(value, bound)
		if banded, err := p.liftedBand(budget, report, result, upper); err != nil || banded {
			return err
		}
		charged := new(big.Rat).Add(bound, delta)
		if published := proofbound.RatFloatUp(charged); measurement.FiniteValues(published) &&
			value.Cmp(proofarith.FloatRat(published)) > 0 {
			report.Relation, report.Reason = vocab.ContactSeparated, vocab.ContactNoReason
			report.Gap = &measurement.Measurement{Value: units.Millimeters(result.Gap.ValueMM),
				Bound: units.Millimeters(published), Exactness: measurement.ExactnessOf(published)}
			return nil
		}
		if upper.Cmp(delta) <= 0 {
			return p.publishBand(ctx, budget, report, nil)
		}
		report.Reason = vocab.ContactNoGapProof
	case pair.Touching:
		convexA, convexB, err := convexities(ctx, budget, report, p.certificates)
		if err != nil {
			return err
		}
		if !convexA && !convexB {
			if err := p.publishBand(ctx, budget, report, nil); err != nil {
				return err
			}
			report.Reason = vocab.ContactNonConvex
			return nil
		}
		manifold, err := planar.PlanarTouchManifold(p.a, p.b, result.Contacts, convexA, convexB, budget.Step)
		if err != nil {
			return err
		}
		if err := p.publishBand(ctx, budget, report, manifold.Points); err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = pair.ContactReasonOf(manifold.Reason)
		}
	case pair.Overlapping:
		deep, err := planar.PlanarDeepVertex(p.a, p.b, proofarith.DyAdd(p.deltaA, p.deltaB), budget.Step)
		if err != nil {
			return err
		}
		if deep {
			report.Relation, report.Reason = vocab.ContactOverlapping, vocab.ContactNoNormalProof
			return p.overlapManifold(ctx, budget, report, result)
		}
		report.Reason = vocab.ContactNoGapProof
		convexA, convexB, err := convexities(ctx, budget, report, p.certificates)
		if err != nil || !convexA || !convexB {
			return err
		}
		points, err := planar.PlanarPenetrationManifold(p.a, p.b, budget.Step)
		if err != nil || points == nil {
			return err
		}
		for _, point := range points {
			// The held depth is minus the separation's lower end.
			depth := new(big.Rat).Sub(proofarith.FloatRat(point.Separation.BoundMM),
				proofarith.FloatRat(point.Separation.ValueMM))
			if depth.Cmp(delta) > 0 {
				return nil
			}
		}
		return p.publishBand(ctx, budget, report, points)
	}
	return nil
}

// liftedBand publishes a held separated pair whose gap's upper end is within
// the lifted band b = max(SupportBand, δ) as ContactBand when some face of a
// zero-δ body hosts a nonempty lifted set within b (§10.4, §10.5). A support
// plane hosted by a displaced body publishes no lifted set: its held face
// only approximates the true face's direction. The band reads δ from below,
// so a resting displaced pair, placed with its held gap at most δ, keeps its
// lifted set however small SupportBand is. The gap becomes [0 ± (g + δ)], g
// the held gap's upper float, and the manifold is the lifted sets charged
// with δ (chargedManifold). It reports whether it published.
func (p *bandPair) liftedBand(budget *proofbound.WorkBudget, report *contactReport, result planar.PlanarResult,
	upper *big.Rat) (bool, error) {
	band := p.liftedWidth(report.Request)
	hostA, hostB := p.deltaA.Sign() == 0, p.deltaB.Sign() == 0
	if upper.Cmp(band.Rat()) > 0 || !hostA && !hostB {
		return false, nil
	}
	lifted, err := planar.PlanarSupportSets(p.a, p.b, result,
		planar.SupportPlanes(p.a, p.b, hostA, hostB), band, false, budget.Step)
	if err != nil || len(lifted) == 0 {
		return false, err
	}
	g := proofbound.RatFloatUp(upper)
	charged := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(g), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !measurement.FiniteValues(g, charged) {
		return false, nil
	}
	report.Relation, report.Reason = vocab.ContactBand, vocab.ContactNoReason
	report.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(charged),
		Exactness: measurement.ExactnessOf(charged)}
	manifold, reason, err := p.chargedManifold(budget, report, lifted, nil)
	if err != nil {
		return false, err
	}
	report.Manifold, report.Reason = manifold, reason
	return true, nil
}

// liftedWidth is the lifted band b = max(SupportBand, δ) of a displaced pair.
func (p *bandPair) liftedWidth(req vocab.ContactRequest) proofarith.Dyadic {
	band, delta := SupportBand(req), proofarith.DyAdd(p.deltaA, p.deltaB)
	if proofarith.DyCmp(band, delta) < 0 {
		return delta
	}
	return band
}

// overlapManifold publishes the held penetration patch of a pair proven to
// overlap (§10.4): §9.3's convex-convex patch or §9.6's face-local one, each
// body read as M grown by its own δ, then the support plane's lifted set
// within the lifted band, every point charged with δ (chargedManifold). A
// patch the kernel withholds keeps the reason it names, or the one report
// carries.
func (p *bandPair) overlapManifold(ctx context.Context, budget *proofbound.WorkBudget, report *contactReport,
	result planar.PlanarResult) error {
	convexA, convexB, err := convexities(ctx, budget, report, p.certificates)
	if err != nil {
		return err
	}
	if !convexA && !convexB {
		report.Reason = vocab.ContactNonConvex
		return nil
	}
	points, planes, reason, err := planar.OverlapPatch(p.a, p.b, result, convexA, convexB,
		p.deltaA, p.deltaB, budget.Step)
	if err != nil {
		return err
	}
	if points == nil {
		if reason != pair.NoReason {
			report.Reason = pair.ContactReasonOf(reason)
		}
		return nil
	}
	manifold, contactReason, err := p.chargedManifold(budget, report, points, nil)
	if err != nil || manifold == nil {
		report.Reason = contactReason
		return err
	}
	lifted, err := planar.PlanarSupportSets(p.a, p.b, result, planes,
		p.liftedWidth(report.Request), true, budget.Step)
	if err != nil {
		return err
	}
	if lifted != nil {
		extra, contactReason, err := p.chargedManifold(budget, report, lifted, nil)
		if err != nil || extra == nil {
			report.Reason = contactReason
			return err
		}
		manifold.Points = append(manifold.Points, extra.Points...)
	}
	report.Manifold, report.Reason = manifold, vocab.ContactNoReason
	return nil
}

// publishBand publishes ContactBand with Gap [−2δ, 2δ] and, when points is
// not nil, the held manifold charged with the band, every Separation the
// band itself (chargedManifold).
func (p *bandPair) publishBand(ctx context.Context, budget *proofbound.WorkBudget, report *contactReport,
	points []planar.PatchPoint) error {
	band := proofbound.RatFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !measurement.FiniteValues(band) {
		report.Reason = vocab.ContactNoGapProof
		return nil
	}
	report.Relation, report.Reason = vocab.ContactBand, vocab.ContactNoNormalProof
	gap := measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(band), Exactness: measurement.ExactnessOf(band)}
	report.Gap = &gap
	if points == nil {
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	manifold, reason, err := p.chargedManifold(budget, report, points, &gap)
	if err != nil {
		return err
	}
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// chargedManifold charges a held manifold with the displacements, as §9.4 and
// §10.4 state for a positive-displacement body: the points are the held
// pair's, each witness ball grows by its own body's δ, since a held vertex
// or foot is a point of the held mesh and Bound does not place it on the true
// surface, and each Separation is band when band is not nil and otherwise the
// held one widened by the summed δ. The normal must be the exact face normal
// of a body with no displacement, read at a face of that body that holds the
// point: a held face of a displaced body only approximates the true face's
// direction, so a point whose normal no exact face supplies withholds the
// manifold with ContactNoNormalProof.
func (p *bandPair) chargedManifold(budget *proofbound.WorkBudget, report *contactReport, points []planar.PatchPoint,
	band *measurement.Measurement) (*contactManifold, vocab.ContactReason, error) {
	exact := -1
	switch {
	case p.deltaA.Sign() == 0:
		exact = 0
	case p.deltaB.Sign() == 0:
		exact = 1
	}
	if exact < 0 {
		return nil, vocab.ContactNoNormalProof, nil
	}
	solids := [2]*planar.PlanarSolid{p.a, p.b}
	for _, point := range points {
		if err := budget.Step(); err != nil {
			return nil, vocab.ContactNoReason, err
		}
		feature := point.A
		if exact == 1 {
			feature = point.B
		}
		normal, ok := planar.FaceNormal(solids[exact], feature)
		if !ok || !proofarith.DvIsZero(proofarith.DvCross(normal, point.Normal)) {
			return nil, vocab.ContactNoNormalProof, nil
		}
	}
	features, err := planarfeaturemap.NewPlanarFeatureMap(report.A, report.B, p.a, p.b)
	if err != nil {
		return nil, vocab.ContactNoReason, err
	}
	manifold, reason := patchManifold(report.Request, points, features)
	if manifold == nil {
		return nil, reason, nil
	}
	resolution := proofarith.FloatRat(report.Request.PointResolution.Base())
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	for i := range manifold.Points {
		point := &manifold.Points[i]
		for side, position := range []*measurement.VecMeasurement{&point.OnA, &point.OnB} {
			own := p.deltaA
			if side == 1 {
				own = p.deltaB
			}
			published := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), own.Rat()))
			if !measurement.FiniteValues(published) || proofarith.FloatRat(published).Cmp(resolution) > 0 {
				return nil, vocab.ContactPointTooCoarse, nil
			}
			position.Bound, position.Exactness = units.Millimeters(published), measurement.ExactnessOf(published)
		}
		if band != nil {
			point.Separation = *band
			continue
		}
		widened := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), delta))
		if !measurement.FiniteValues(widened) {
			return nil, vocab.ContactNoGapProof, nil
		}
		point.Separation.Bound, point.Separation.Exactness = units.Millimeters(widened), measurement.ExactnessOf(widened)
	}
	return manifold, vocab.ContactNoReason, nil
}
