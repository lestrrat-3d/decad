package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is docs/prism-boolean-design.md §4.2's crossing sub-case for Cut
// and Intersect: edge-orientation propagation over the SAME private scene
// prism_boolean.go's prismcells.BuildScene builds, dispatched by
// prism_boolean_nesting.go's resolveAndBuildPrismCut/Intersect once the
// clean-nesting structural match (prism_boolean_nesting.go) comes back
// unresolved. Every cell sketch's arrangement returns is classified against
// each operand by comparing a boundary edge's own Reversed flag against the
// entity's authoredReversed bookkeeping prismcells.BuildScene records at creation
// time (prismcells.Origin) — a flag comparison, never a geometric test — and
// a cell with no direct edge from an operand takes that operand's
// classification from a neighboring cell reached by crossing an edge that
// does NOT belong to that operand (crossing a non-operand edge cannot move
// across that operand's own boundary, so membership is unchanged). A cell
// whose membership this propagation cannot reach for either operand — its
// whole such-connected component carries no direct edge from that operand —
// is unresolved, and per tryPrismBoolean's own contract that makes the WHOLE
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
// and not B", Intersect keeps "A and B" — and prismcells.Merge
// assembles the selected cells' surviving boundary edges
// into one candidate ProfileRecord, exactly the mechanism Union's own
// select-all path already uses over its own (unfiltered) selection.
// auditPrismMergeSection then re-proves the assembly, the same §6 audit
// Union's own merge runs.

// resolveAndBuildPrismCutCrossing runs §4.2's crossing sub-case for
// Cut(target, tool) once prism_boolean_nesting.go's clean-nesting search
// comes back unresolved. resolved=false (err always nil in that case) is
// silent fallback per this file's own header; a non-nil error is a genuine
// refusal past §3.4's point of no return.
func resolveAndBuildPrismCutCrossing(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	if len(target.Profile.Holes) != 0 || len(tool.Profile.Holes) != 0 {
		// This classifier is scoped to hole-free operands (this file's own
		// header); a holed target/tool falls back to the mesh path exactly
		// as it did before this increment.
		return prismPayload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := prismcells.ResolveCrossing(ctx, budget, target.Profile, tool.Profile,
		target.SectionDelta, tool.SectionDelta, reexpress,
		func(a, b bool) bool { return a && !b }, "cut")
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil
	}

	// Point of no return (§3.4): every further problem is a genuine refusal,
	// unless the cuts carry an amplified displacement (prismcells.AmplifiedFallback).
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, auditPrismMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}

	result := prismPayload{Payload: prismrecord.Payload{
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
	}}
	return result, true, nil
}

// resolveAndBuildPrismIntersectCrossing is the Intersect twin: keeps the
// cells that are material of BOTH operands. §3.2's Intersect z-interval and
// axial displacement selection reuse prismplacement.ZShift/IntersectLowerEnd/IntersectUpperEnd
// (prism_boolean_nesting.go) unchanged.
func resolveAndBuildPrismIntersectCrossing(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	if len(pa.Profile.Holes) != 0 || len(pb.Profile.Holes) != 0 {
		return prismPayload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := prismcells.ResolveCrossing(ctx, budget, pa.Profile, pb.Profile,
		pa.SectionDelta, pb.SectionDelta, reexpress,
		func(a, b bool) bool { return a && b }, "intersect")
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil
	}

	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, auditPrismMergeSection(budget, pa, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's exact shift (prismplacement.ZShift) is applied
	// to B's own recorded interval, exactly as the clean-nesting path's own
	// Intersect builder does; prismplacement.IntersectLowerEnd/IntersectUpperEnd charge a shifted endpoint's
	// single rounding.
	pbZ0, pbZ1 := prismplacement.AdmittedShiftedInterval(prismPlacementOf(pa), prismPlacementOf(pb))
	z0, z0Delta := prismplacement.IntersectLowerEnd(pa.Z0, pa.Z0Delta, pbZ0, pb.Z0Delta)
	z1, z1Delta := prismplacement.IntersectUpperEnd(pa.Z1, pa.Z1Delta, pbZ1, pb.Z1Delta)

	result := prismPayload{Payload: prismrecord.Payload{
		Profile:      merged,
		Frame:        pa.Frame,
		Xform:        pa.Xform,
		Z0:           z0,
		Z1:           z1,
		Z0Delta:      z0Delta,
		Z1Delta:      z1Delta,
		SectionDelta: sceneDelta.Merged(pa.SectionDelta, pb.SectionDelta, reexpress.Delta, cutDelta),
	}}
	return result, true, nil
}
