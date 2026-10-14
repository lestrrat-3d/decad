package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// tryPrismHoledIntersect admits one cleanly nested, one-hole prism under a
// hole-free prism. Intersect is symmetric, so put the hole-free operand first.
// The shared Intersect admission used by the overlap-area reading stays
// hole-free; this body-only arm uses the same gates and work cap explicitly.
func tryPrismHoledIntersect(ctx context.Context, a, b *Body) (prismPayload, bool, error) {
	ah, aok := a.n.Payload.(prismPayload)
	bh, bok := b.n.Payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, false, nil
	}
	if len(ah.Profile.Holes) == 1 && len(bh.Profile.Holes) == 0 {
		a, b = b, a
	} else if len(ah.Profile.Holes) != 0 || len(bh.Profile.Holes) != 1 {
		return prismPayload{}, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, meshbool.OpIntersect, a, b)
	if err != nil || !ok {
		return prismPayload{}, false, err
	}
	for _, profile := range []profileRecord{pa.Profile, pb.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return prismPayload{}, false, err
		}
		if trimmed {
			return prismPayload{}, false, nil
		}
	}
	if !prismplacement.IntersectZIntervalOverlaps(pa.PlacementOperand(), pb.PlacementOperand()) {
		return prismPayload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return prismPayload{}, false, err
	}
	return resolveAndBuildPrismIntersect(ctx, budget, pa, pb, reexpress)
}

// This file is docs/prism-boolean-design.md §4.2's "clean" sub-case for
// Cut and Intersect: the structural whole-loop match against prismcells.BuildScene's
// own tag map (prism_boolean.go), reusing that file's scene construction, G1-G4
// admission and work-budget/cancellation machinery. There is no assembly and no
// §6 build-time audit on this path — the candidate is one s.Profiles() result
// taken verbatim, so §5's authentication claim 1 (every individual segment is
// authentic) already covers claim 2 (the assembly is correct) entirely. When
// this search comes back unresolved, resolveAndBuildPrismCut/Intersect fall
// through to internal/prismcells/payload_resolve.go's edge-orientation classifier before
// giving up — a pair whose operands' boundaries genuinely cross reaches that
// classifier, not straight to the mesh path; only what NEITHER path resolves
// falls through unresolved (§4.4).
//
// A structurally-matched loop's every edge is Whole (§4.2's own match
// condition), so the matched profile carries no cut charge (§7's δ_cut) —
// but a consumed source segment whose OWN recorded range already narrows its
// natural domain still entered prismcells.BuildScene's private scene at a walked
// endpoint the boolean computed, and that charge (§7's δ_walk,
// prismcells.WalkChargeOf) composes into both ops' own sectionDelta
// here, exactly as it does on Union's own merge path. A scene with a split
// boundary never matches (some entity is not whole), so it reaches the
// crossing sub-case, which charges its crossings (A6).

// resolveAndBuildPrismCut runs Cut's clean-nesting structural match (§4.2)
// first and, once it finds a unique candidate, authenticates it and builds
// §7's exactness. There is no §6 audit on this path (§6 is titled "Union's
// merge only" no longer describes this file alone — see prism_boolean.go's
// own header; §5 states the clean-nesting claim needs no second proof — the
// candidate is one s.Profiles() result taken verbatim, so §5's authentication
// claim 1 already covers claim 2 entirely). When the clean-nesting search
// comes back unresolved, this tries internal/prismcells/payload_resolve.go's
// edge-orientation classifier before giving up.
func resolveAndBuildPrismCut(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, sceneDelta, resolved, err := prismcells.ResolveCut(ctx, budget, target.Profile, tool.Profile, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		// §4.2's crossing sub-case (prismcells.ResolveAndBuildCutCrossing): the tool's
		// boundary genuinely crosses the target's, rather than leaving both
		// operands' loops untouched.
		result, ok, err := prismcells.ResolveAndBuildCutCrossing(ctx, budget, target.Payload, tool.Payload, reexpress)
		return prismPayload{Payload: result}, ok, err
	}

	// Point of no return (§3.4): every further problem is a genuine refusal
	// (RB8 below). The matched profile is authenticated verbatim through the
	// existing public seam — no new authentication code (§5).
	profile, err := prismcells.RecordProfileContext(ctx, s, match)
	if err != nil {
		return prismPayload{}, false, err
	}
	result := prismPayload{Payload: prismrecord.Payload{
		Profile: profile,
		// The record carries the TARGET's own frame/xform (operand A's — the
		// private scene's own reference plane, §4.1). The scene's returned
		// PlaneRecord describes that private scene's own world-XY plane, not
		// the body's; using it here would silently swap the result onto the
		// wrong plane, so it is discarded.
		Frame: target.Frame,
		Xform: target.Xform,
		// §3.2's Cut row: the result's z-interval is the target's own,
		// unchanged — the tool removes material across the target's full
		// height, never narrows it.
		Z0:      target.Z0,
		Z1:      target.Z1,
		Z0Delta: target.Z0Delta,
		Z1Delta: target.Z1Delta,
		// §7/Task 4.4: every matched edge is whole, so the cut charge is
		// zero — there is no fresh cut parameter to charge at all. What
		// remains is the same formula PR1's Union path uses for its own
		// re-expression and prior-displacement terms (now including each
		// operand's own walk charge), with the (already zero) cut term
		// omitted rather than added back in.
		SectionDelta: max(
			proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta),
		),
	}}
	return result, true, nil
}

// resolveAndBuildPrismIntersect runs Intersect's clean-nesting structural
// match (§4.2) in both directions first and, once it finds the unique nested
// operand, authenticates its own region verbatim and builds §7's exactness.
// There is no §6 audit on this path, for the same reason as Cut's. When
// neither direction matches, this tries internal/prismcells/payload_resolve.go's
// edge-orientation classifier before giving up.
func resolveAndBuildPrismIntersect(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, sceneDelta, nestedIsB, resolved, err := prismcells.ResolveIntersect(ctx, budget, pa.Profile, pb.Profile, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		// §4.2's crossing sub-case (prismcells.ResolveAndBuildIntersectCrossing): a
		// general-position overlap, rather than one operand's boundary
		// leaving the other's fully untouched.
		result, ok, err := prismcells.ResolveAndBuildIntersectCrossing(ctx, budget, pa.Payload, pb.Payload, reexpress)
		return prismPayload{Payload: result}, ok, err
	}

	profile, err := prismcells.RecordProfileContext(ctx, s, match)
	if err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's exact shift (prismplacement.ZShift) is applied
	// to B's own recorded interval. Each result endpoint is A's own recorded
	// float, or B's shifted endpoint rounded once with that rounding charged
	// (prismplacement.IntersectLowerEnd and IntersectUpperEnd); a tie takes
	// the larger displacement, since both operands' coordinates denote it.
	pbZ0, pbZ1 := prismplacement.AdmittedShiftedInterval(pa.PlacementOperand(), pb.PlacementOperand())
	z0, z0Delta := prismplacement.IntersectLowerEnd(pa.Z0, pa.Z0Delta, pbZ0, pb.Z0Delta)
	z1, z1Delta := prismplacement.IntersectUpperEnd(pa.Z1, pa.Z1Delta, pbZ1, pb.Z1Delta)

	// §7/Task 4.4: the record traces to the NESTED operand alone, so only
	// that operand's own displacement term (now including its own walk
	// charge) reaches the result — never the max of both, which would be
	// conservative where the code already knows which operand's coordinates
	// it took.
	sectionDelta := proofbound.AbsSumUpper(pa.SectionDelta, sceneDelta.A)
	if nestedIsB {
		sectionDelta = proofbound.AbsSumUpper(pb.SectionDelta, sceneDelta.B, reexpress.Delta)
	}

	result := prismPayload{Payload: prismrecord.Payload{
		Profile: profile,
		// The private scene's reference plane is always operand A's (§4.1),
		// regardless of which operand turned out to be nested — B's own
		// coordinates, when they are the ones that survive, already live in
		// A's frame through the scene's re-expression.
		Frame:        pa.Frame,
		Xform:        pa.Xform,
		Z0:           z0,
		Z1:           z1,
		Z0Delta:      z0Delta,
		Z1Delta:      z1Delta,
		SectionDelta: sectionDelta,
	}}
	return result, true, nil
}
