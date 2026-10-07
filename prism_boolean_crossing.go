package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/prism-boolean-design.md §4.2's crossing sub-case for Cut
// and Intersect: edge-orientation propagation over the SAME private scene
// prism_boolean.go's buildPrismScene builds, dispatched by
// prism_boolean_nesting.go's resolveAndBuildPrismCut/Intersect once the
// clean-nesting structural match (prism_boolean_nesting.go) comes back
// unresolved. Every cell sketch's arrangement returns is classified against
// each operand by comparing a boundary edge's own Reversed flag against the
// entity's authoredReversed bookkeeping buildPrismScene records at creation
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
// and not B", Intersect keeps "A and B" — and mergePrismCells
// (prism_boolean.go) assembles the selected cells' surviving boundary edges
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
	if len(target.profile.Holes) != 0 || len(tool.profile.Holes) != 0 {
		// This classifier is scoped to hole-free operands (this file's own
		// header); a holed target/tool falls back to the mesh path exactly
		// as it did before this increment.
		return prismPayload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := resolvePrismCrossing(ctx, budget, target, tool, reexpress,
		func(a, b bool) bool { return a && !b }, "cut")
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil
	}

	// Point of no return (§3.4): every further problem is a genuine refusal,
	// unless the cuts carry an amplified displacement (prismAmplifiedFallback).
	if fallBack, err := prismAmplifiedFallback(sceneDelta.amplified, auditPrismMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}

	result := prismPayload{
		profile: merged,
		frame:   target.frame,
		xform:   target.xform,
		// §3.2's Cut row: the result's z-interval is the target's own,
		// unchanged — the tool removes material across the target's full
		// height, never narrows it.
		z0:      target.z0,
		z1:      target.z1,
		z0Delta: target.z0Delta,
		z1Delta: target.z1Delta,
		// §7's formula with A6's crossing term, Union's own: this path
		// assembles a merged loop exactly like Union's, so it carries the
		// same displacement terms, cutDelta and the crossing charge included.
		sectionDelta: sceneDelta.merged(target, tool, reexpress, cutDelta),
	}
	return result, true, nil
}

// resolveAndBuildPrismIntersectCrossing is the Intersect twin: keeps the
// cells that are material of BOTH operands. §3.2's Intersect z-interval and
// axial displacement selection reuse prismZShift/prismIntersectEnd
// (prism_boolean_nesting.go) unchanged.
func resolveAndBuildPrismIntersectCrossing(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 {
		return prismPayload{}, false, nil
	}

	merged, sceneDelta, cutDelta, resolved, err := resolvePrismCrossing(ctx, budget, pa, pb, reexpress,
		func(a, b bool) bool { return a && b }, "intersect")
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil
	}

	if fallBack, err := prismAmplifiedFallback(sceneDelta.amplified, auditPrismMergeSection(budget, pa, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's exact shift (prismZShift) is applied
	// to B's own recorded interval, exactly as the clean-nesting path's own
	// Intersect builder does; prismIntersectEnd charges a shifted endpoint's
	// single rounding.
	pbZ0, pbZ1 := prismShiftedIntervalAdmitted(pa, pb)
	z0, z0Delta := prismIntersectEnd(pa.z0, pa.z0Delta, pbZ0, pb.z0Delta, func(c int) bool { return c > 0 })
	z1, z1Delta := prismIntersectEnd(pa.z1, pa.z1Delta, pbZ1, pb.z1Delta, func(c int) bool { return c < 0 })

	result := prismPayload{
		profile:      merged,
		frame:        pa.frame,
		xform:        pa.xform,
		z0:           z0,
		z1:           z1,
		z0Delta:      z0Delta,
		z1Delta:      z1Delta,
		sectionDelta: sceneDelta.merged(pa, pb, reexpress, cutDelta),
	}
	return result, true, nil
}

// resolvePrismCrossing is the shared resolution shape behind both builders
// above: resolvePrismCrossingCells's selection (scene, classification,
// per-op keep, crossing charge), then mergePrismCells's assembly
// (prism_boolean.go). resolved=false (err always nil in that case) is silent
// fallback, which includes a near-tangent crossing A6's charge cannot bound
// and a merge failure on cuts that carry an amplified displacement. opName
// feeds mergePrismCells's own RB1 message.
func resolvePrismCrossing(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression, keep func(a, b bool) bool, opName string) (ProfileRecord, prismSceneDelta, float64, bool, error) {
	selected, sceneDelta, resolved, err := resolvePrismCrossingCells(ctx, budget, pa, pb, reexpress, keep)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !resolved {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
	}

	merged, cutDelta, mergedResolved, err := mergePrismCells(budget, selected, opName)
	if fallBack, err := prismAmplifiedFallback(sceneDelta.amplified, err); fallBack || err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !mergedResolved {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
	}
	return merged, sceneDelta, cutDelta, true, nil
}

// resolvePrismCrossingCells is §4.2's crossing sub-case selection alone, the
// first half of resolvePrismCrossing's own shape: build the private scene,
// classify every cell (prismcells.Classify), select the op's own subset
// (keep), and charge every crossing the operands' incoming displacement can
// move (A6, prismSceneDelta.chargeCrossings) — stopping short of mergePrismCells's
// assembly tail, which requires the selected cells to chain into one closed
// loop and so cannot answer a multi-region selection at all
// (docs/prism-boolean-design.md §4.4). resolvePrismCrossing above is this
// task's own caller; §4.5's overlap-area reading
// (prismOverlapVolume, prism_overlap.go) is the second, sharing this exact
// selection so it never re-derives §4.2's classification.
//
// resolved=false (err always nil in that case) is silent fallback (§4.4),
// including a crossing too close to tangent for A6's charge.
func resolvePrismCrossingCells(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression, keep func(a, b bool) bool) (selected []*sketch.Profile, sceneDelta prismSceneDelta, resolved bool, err error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	if len(profiles) == 0 {
		return nil, prismSceneDelta{}, false, nil // §4.4: the scene holds no bounded cell at all
	}
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	if err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	if !resolved {
		return nil, prismSceneDelta{}, false, nil
	}

	selected, err = prismcells.Select(budget, profiles, matterA, matterB, keep)
	if err != nil {
		return nil, prismSceneDelta{}, false, err
	}
	// docs/general-boolean-design.md §3 A6, Union's own charge: every cut the
	// operands' incoming displacement can move is charged, and a crossing too
	// close to tangent for that charge sends the pair to the mesh path.
	if ok, err := sceneDelta.chargeCrossings(budget, tags, profiles, pa, pb, reexpress); err != nil || !ok {
		return nil, prismSceneDelta{}, false, err
	}
	return selected, sceneDelta, true, nil
}
