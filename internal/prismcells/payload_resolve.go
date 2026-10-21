package prismcells

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is agent-docs/prism-boolean-design.md §4.2's crossing sub-case for Cut
// and Intersect: edge-orientation propagation over the same private scene
// BuildScene constructs. ResolveAndBuildCut/Intersect call this path when
// payload_nesting.go's clean-nesting structural match is unresolved.
// Every cell sketch's arrangement returns is classified against
// each operand by comparing a boundary edge's own Reversed flag against the
// entity's authoredReversed bookkeeping BuildScene records at creation
// time (Origin) — a flag comparison, never a geometric test — and
// a cell with no direct edge from an operand takes that operand's
// classification from a neighboring cell reached by crossing an edge that
// does NOT belong to that operand (crossing a non-operand edge cannot move
// across that operand's own boundary, so membership is unchanged). A cell
// whose membership this propagation cannot reach for either operand — its
// whole such-connected component carries no direct edge from that operand —
// is unresolved, and per TryAnalytic's own contract that makes the WHOLE
// attempt unresolved, never a partial result.
//
// This increment does not identify a coincident carrier shared under only
// one operand's entity (§4.2's own further extension, needed when sketch
// welds two numerically-matching carrier entities from A and B into one and
// names a returned edge under only one of them): a cell reachable only
// through such a carrier is unresolved here and falls back to the mesh path,
// silently, exactly like any other cell this propagation cannot reach. It is
// also scoped to hole-free operands on both sides — G6 already requires this
// for Intersect, and for Cut it additionally requires the TARGET hole-free
// even though G6 alone would allow a holed target through the clean-nesting
// path; a holed target reaching this classifier instead falls back to the
// mesh path, unchanged from before this increment.
//
// Once classified, membership selects a per-op cell subset — Cut keeps "A
// and not B", Intersect keeps "A and B" — and Merge
// assembles the selected cells' surviving boundary edges
// into one candidate ProfileRecord, exactly the mechanism Union's own
// select-all path already uses over its own (unfiltered) selection.
// AuditMergeSection then re-proves the assembly, the same §6 audit
// Union's own merge runs.

// ResolveAndBuildUnion runs Union's hole-free select-all/merge/chain
// resolution (§4.2) and, once it commits, §6's build-time audit and §7's
// exactness. Split out of TryAnalytic only so each op's own resolve+build
// sequence reads as one unit.
func ResolveAndBuildUnion(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismrecord.Payload, reexpress *Reexpression) (prismrecord.Payload, bool, error) {
	merged, sceneDelta, cutDelta, resolved, err := ResolveUnion(ctx, budget, pa.Profile, pb.Profile,
		pa.SectionDelta, pb.SectionDelta, reexpress)
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if !resolved {
		return prismrecord.Payload{}, false, nil // §4.4: this topology is unresolved
	}

	// Point of no return (§3.4): every further problem is a genuine refusal,
	// unless the cuts carry an amplified displacement (AmplifiedFallback).
	if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, AuditMergeSection(budget, pa, merged)); fallBack || err != nil {
		return prismrecord.Payload{}, false, err
	}
	result := prismrecord.Payload{
		Profile: merged,
		Frame:   pa.Frame,
		Z0:      pa.Z0,
		Z1:      pa.Z1,
		Z0Delta: max(pa.Z0Delta, pb.Z0Delta),
		Z1Delta: max(pa.Z1Delta, pb.Z1Delta),
		Xform:   pa.Xform,
		// §7: operand A's coordinates are unchanged except for its own
		// consumed segments' walk charge (sceneDelta.A), while B's existing
		// displacement passes through its rigid re-expression, accumulates
		// the rounding that re-expression commits, and its own walk charge
		// (sceneDelta.B) besides. On top of both, every surviving CUT
		// fragment names its endpoints by a freshly rounded parameter this
		// union did not have before it ran, so cutDelta stands even where the
		// two operands carried nothing at all, and every cut the inputs'
		// displacement can move carries A6's crossing charge
		// (sceneDelta.Crossing). A chained union must not discard any of
		// the five.
		SectionDelta: sceneDelta.Merged(pa.SectionDelta, pb.SectionDelta, reexpress.Delta, cutDelta),
	}
	return result, true, nil
}

// AuditMergeSection runs the modify §5 audit (the root fillet_audit.go,
// §6 of this design) on a merged section, reused verbatim with an empty
// blend map — offset2d.AuditOffsetSectionBudget's own precedent for "no cutback data, still run
// the shared audit". orig supplies the S8 sign reference: operand A's own
// outer loop, whose CCW orientation and non-degenerate area the merged Outer
// loop must match — the merge is correct only when it keeps that same
// convention. S9 (nesting) is a no-op here: G6 keeps every operand hole-free
// for Union, and the crossing sub-case's own scoping
// (ResolveAndBuildCutCrossing, ResolveAndBuildIntersectCrossing) does the same, so the merged result is always
// hole-free. This audit runs on every assembled (chained-and-merged) section —
// Union's own select-all path and the crossing sub-case for Cut/Intersect
// alike; see this file's header comment for why Cut/Intersect's clean-nesting
// match needs none.
func AuditMergeSection(budget *proofbound.WorkBudget, pa prismrecord.Payload, merged momentinput.Profile) error {
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, merged)
	if err != nil {
		return err
	}
	blendAt := make([]map[int]*offset2d.Blend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*offset2d.Blend{}
	}
	orig := momentinput.Profile{Outer: pa.Profile.Outer}
	return offset2d.AuditRewriteBudget(budget, orig, merged, loops, blendAt)
}

// ResolveAndBuildCutCrossing runs §4.2's crossing sub-case for
// Cut(target, tool) once payload_nesting.go's clean-nesting search
// comes back unresolved. resolved=false (err always nil in that case) is
// silent fallback per this file's own header; a non-nil error is a genuine
// refusal past §3.4's point of no return.
func ResolveAndBuildCutCrossing(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismrecord.Payload, reexpress *Reexpression) (prismrecord.Payload, bool, error) {
	if len(target.Profile.Holes) != 0 || len(tool.Profile.Holes) != 0 {
		// This classifier is scoped to hole-free operands (this file's own
		// header); a holed target/tool falls back to the mesh path exactly
		// as it did before this increment.
		return prismrecord.Payload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := ResolveCrossing(ctx, budget, target.Profile, tool.Profile,
		target.SectionDelta, tool.SectionDelta, reexpress,
		func(a, b bool) bool { return a && !b }, "cut")
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if !resolved {
		return prismrecord.Payload{}, false, nil
	}

	// Point of no return (§3.4): every further problem is a genuine refusal,
	// unless the cuts carry an amplified displacement (AmplifiedFallback).
	if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, AuditMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismrecord.Payload{}, false, err
	}

	result := prismrecord.Payload{
		Profile: merged,
		Frame:   target.Frame,
		Xform:   target.Xform,
		// §3.2's Cut row: the result's z-interval is the target's own,
		// unchanged — the tool removes material across the target's full
		// height, never narrows it.
		Z0:      target.Z0,
		Z1:      target.Z1,
		Z0Delta: target.Z0Delta,
		Z1Delta: target.Z1Delta,
		// §7's formula with A6's crossing term, Union's own: this path
		// assembles a merged loop exactly like Union's, so it carries the
		// same displacement terms, cutDelta and the crossing charge included.
		SectionDelta: sceneDelta.Merged(target.SectionDelta, tool.SectionDelta, reexpress.Delta, cutDelta),
	}
	return result, true, nil
}

// ResolveAndBuildIntersectCrossing is the Intersect twin: keeps the
// cells that are material of BOTH operands. §3.2's Intersect z-interval and
// axial displacement selection reuse prismplacement.ZShift/IntersectLowerEnd/IntersectUpperEnd
// (payload_nesting.go) unchanged.
func ResolveAndBuildIntersectCrossing(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismrecord.Payload, reexpress *Reexpression) (prismrecord.Payload, bool, error) {
	if len(pa.Profile.Holes) != 0 || len(pb.Profile.Holes) != 0 {
		return prismrecord.Payload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := ResolveCrossing(ctx, budget, pa.Profile, pb.Profile,
		pa.SectionDelta, pb.SectionDelta, reexpress,
		func(a, b bool) bool { return a && b }, "intersect")
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if !resolved {
		return prismrecord.Payload{}, false, nil
	}

	if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, AuditMergeSection(budget, pa, merged)); fallBack || err != nil {
		return prismrecord.Payload{}, false, err
	}

	// §3.2's Intersect row, after G5's exact shift (prismplacement.ZShift) is applied
	// to B's own recorded interval, exactly as the clean-nesting path's own
	// Intersect builder does; prismplacement.IntersectLowerEnd/IntersectUpperEnd charge a shifted endpoint's
	// single rounding.
	pbZ0, pbZ1 := prismplacement.AdmittedShiftedInterval(pa.PlacementOperand(), pb.PlacementOperand())
	z0, z0Delta := prismplacement.IntersectLowerEnd(pa.Z0, pa.Z0Delta, pbZ0, pb.Z0Delta)
	z1, z1Delta := prismplacement.IntersectUpperEnd(pa.Z1, pa.Z1Delta, pbZ1, pb.Z1Delta)

	result := prismrecord.Payload{
		Profile:      merged,
		Frame:        pa.Frame,
		Xform:        pa.Xform,
		Z0:           z0,
		Z1:           z1,
		Z0Delta:      z0Delta,
		Z1Delta:      z1Delta,
		SectionDelta: sceneDelta.Merged(pa.SectionDelta, pb.SectionDelta, reexpress.Delta, cutDelta),
	}
	return result, true, nil
}
