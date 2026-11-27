package booleaneval

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// matchingLoftPrismSection proves only equality of the loft's end profile and
// the prism's start profile. It does not classify a Boolean contact.
func matchingLoftPrismSection(ctx context.Context, loft, prism *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if ctx == nil || budget == nil || loft == nil || prism == nil {
		return false, nil
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	lp, ok := loft.Payload.(loftrecord.Payload)
	if !ok || !lp.MatchesDirectSections(loft) || lp.SourceIdentity == nil || lp.SourceIdentity.Origin != loft.Origin.Producer {
		return false, nil
	}
	pp, ok := prism.Payload.(prismrecord.Payload)
	if !ok || !prism.HasStrictPrismBuildSeal() || pp.Xform != r3.Identity() || pp.Frame != lp.Frame1 ||
		pp.Z0 != 0 || pp.Z0Delta != 0 || !(pp.Z1 > 0) || math.IsInf(pp.Z1, 0) ||
		!(pp.Z1Delta >= 0 && pp.Z1Delta < pp.Z1) || pp.SectionDelta != 0 ||
		pp.SurfaceResult || len(pp.BlendSegs) != 0 || len(pp.Profile.Holes) != 0 || len(lp.Profile1.Holes) != 0 {
		return false, nil
	}
	if level := pp.DenotedZ0(); level != nil && level.Sign() != 0 {
		return false, nil
	}
	sealed, err := prism.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform, pp.Z0, pp.Z1, pp.Z0Delta, pp.Z1Delta,
		pp.SectionDelta, pp.Z0Denoted, pp.Z1Denoted, []sectionrecord.AnalyticProfileView{pp.Profile.AnalyticSealView()}, budget)
	if err != nil || !sealed {
		return false, err
	}
	a, b := lp.Profile1.Outer.Segments, pp.Profile.Outer.Segments
	if len(a) == 0 || len(a) != len(b) {
		return false, nil
	}
	for i, seg := range a {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		var start, end float64
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			start, end = s.TStart, s.TEnd
		case sectionrecord.ArcSeg:
			start, end = s.TStart, s.TEnd
		default:
			return false, nil
		}
		if !(start == 0 && end == 1 || start == 1 && end == 0) || !sectionrecord.IdenticalAnalyticSegment(seg, b[i]) {
			return false, nil
		}
	}
	return true, nil
}

// separatedLoftPrismSections additionally proves the loft and prism occupy
// opposite sides of their shared section plane.
func separatedLoftPrismSections(ctx context.Context, loft, prism *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	matched, err := matchingLoftPrismSection(ctx, loft, prism, budget)
	if err != nil || !matched {
		return false, err
	}
	lp, ok := loft.Payload.(loftrecord.Payload)
	if !ok || !lp.Frame0.IsValid() || !lp.Frame1.IsValid() || lp.Frame0.U() != lp.Frame1.U() ||
		lp.Frame0.V() != lp.Frame1.V() || !proofbound.FiniteVec(lp.Frame0.Origin()) ||
		!proofbound.FiniteVec(lp.Frame1.Origin()) || !proofbound.FiniteVec(lp.Frame1.U()) ||
		!proofbound.FiniteVec(lp.Frame1.V()) {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	normal := proof.DvCross(proof.DyVec(lp.Frame1.U()), proof.DyVec(lp.Frame1.V()))
	delta := proof.DvSub(proof.DyVec(lp.Frame1.Origin()), proof.DyVec(lp.Frame0.Origin()))
	cross := proof.DvCross(normal, delta)
	return cross[0].Sign() == 0 && cross[1].Sign() == 0 && cross[2].Sign() == 0 &&
		proof.DvDot(normal, delta).Sign() > 0 && proofbound.FiniteVec(lp.Frame1.N()) &&
		proof.DvDot(normal, proof.DyVec(lp.Frame1.N())).Sign() > 0, nil
}

// SeparatedLoftPrismSections proves the matching-section arrangement used by
// the private Boolean-history continuation.
func SeparatedLoftPrismSections(ctx context.Context, loft, prism *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	return separatedLoftPrismSections(ctx, loft, prism, budget)
}
