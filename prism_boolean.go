package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is docs/prism-boolean-design.md's analytic reduction of
// Union/Cut/Intersect over co-directional straight prisms on one plane or on
// shared-axis offset planes, routed entirely through sketch (§4) rather than
// through the mesh boolean (evaluator-design §9). The entry gate (§3) is reject-only and never
// surfaces an error on a miss — the caller falls back to the unchanged mesh
// path exactly as it did before this file existed. Where either input
// carries a section displacement, a consumed source segment's own walk
// computed a coordinate (§7's δ_walk, prismcells.WalkChargeOf), or B's re-expression is
// nonidentity, every crossing the arrangement cuts can amplify that
// displacement by 1/sin θ, and docs/general-boolean-design.md §3 A6 charges
// it (prismSceneDelta.ChargeCrossings, prismcells.CrossingCharge). Such a
// pair never refuses: a crossing too close to tangent to bound, or any later
// analytic failure, sends it to the mesh path (prismcells.AmplifiedFallback). Every cut also charges the cut
// parameters' OWN rounding into the result's section displacement (§7,
// prismcells.CutDelta) — a merged section built from fragments is never
// exact, whatever the operands were. A
// consumed source segment whose own recorded range is narrower than its
// natural domain enters the private scene at a WALKED endpoint the boolean
// itself computed (prismcells.BuildScene's own doc comment) rather than at the
// record's own coordinate, and that computed endpoint is charged too — a
// trimmed circular carrier (ArcSeg/CircleSeg) moves by more than a
// coordinate displacement can state, so this file refuses that pair before
// building the scene at all rather than under-charge it
// (prismcells.ProfileHasTrimmedCircularSource). Only once §4.2's remaining
// resolution finds a unique candidate does a further problem become a
// genuine, typed refusal (§3.4, §9) rather than a reroute to the mesh path.
//
// Union's resolution (§4.2) is the hole-free select-all/merge/chain path: the
// candidate is assembled from every returned cell, and §6's audit re-proves
// the assembly the same way every modify op re-checks its own rewrite. It
// uses prismcells.Merge — the same chain/merge
// tail the crossing sub-case below reuses over its own, narrower cell
// selection. Cut/Intersect's clean-nesting structural match (§4.2's "clean"
// sub-case) is prism_boolean_nesting.go — see that file's own header for why
// it needs no assembly and no §6 audit. This file owns what every path
// shares: G1-G4 admission, the work-budget cap, the private scene
// construction (prismcells.BuildScene) and the operand coordinate re-expression
// (prismReexpression).
//
// Cut/Intersect's crossing sub-case — genuine boundary contact, not clean
// nesting — is internal/prismcells/payload_resolve.go: §4.2's edge-orientation
// propagation over hole-free operands, dispatched by
// prism_boolean_nesting.go's resolveAndBuildPrismCut/Intersect once the
// clean-nesting search comes back unresolved. It does not yet identify a
// coincident carrier shared under only one operand's entity (§4.2's own
// further extension) or admit a holed operand past what G6 already allows
// for Cut's target; either miss is silent fallback, never a refusal, so a
// pair needing them still reaches the mesh path exactly as before this
// increment. A pair whose operands' boundaries genuinely cross and whose
// membership this propagation cannot reach falls through to tryPrismBoolean
// returning ok=false, err=nil, exactly like any other unresolved topology
// (§4.4).

// tryPrismBoolean attempts the analytic reduction for op. ok=false (err
// always nil in that case) means "not admitted" per §3.1/§3.4: the caller
// MUST fall back to the unchanged mesh path with no error surfaced. A
// non-nil err means the bounded analytic resolution reached a genuine
// refusal (§3.4), and the caller MUST propagate it rather than reroute to
// the mesh path. A pair whose cuts carry an amplified displacement never
// refuses: it falls back (prismcells.AmplifiedFallback).
//
// G1-G4 (admitPrismPairBudget) and the work cap are shared, unchanged, by
// every op; G5 and G6 (§3.1) and the resolution path (§4.2) are op-specific,
// per §3.2's table.
func tryPrismBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op == meshbool.OpIntersect {
		if result, ok, err := tryPrismHoledIntersect(ctx, a, b); ok || err != nil {
			return result, ok, err
		}
		// admitPrismIntersectPair is Intersect's own preamble (G1-G4, the
		// trimmed-circular refusal, G6, G5, the arrangement cap, the
		// re-expression) factored out so §4.5's overlap-area reading
		// (prism_overlap.go) can share it unchanged; see that helper's own
		// doc comment.
		budget, pa, pb, reexpress, ok, err := admitPrismIntersectPair(ctx, a, b)
		if err != nil {
			return prismPayload{}, false, err
		}
		if !ok {
			return prismPayload{}, false, nil
		}
		return resolveAndBuildPrismIntersect(ctx, budget, pa, pb, reexpress)
	}

	budget := proofbound.NewWorkBudget(ctx) // §10: one counter for the whole attempt
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, op, a, b) // G1-G4
	if err != nil {
		return prismPayload{}, false, err
	}
	if !ok {
		return prismPayload{}, false, nil
	}

	// A trimmed circular carrier (ArcSeg/CircleSeg) enters the private scene
	// through two cos/sin-COMPUTED points, so its rebuilt carrier's radius and
	// sweep both move, not merely its endpoints — a displacement
	// proofbound.WalkEndpointAllow's plain coordinate charge does not state. Rather than
	// publish an under-charged bound for it, the pair is refused before the
	// scene is even built, the same silent-fallback shape the entry gate
	// above already uses (§3.4).
	trimmedCircular, err := prismcells.ProfileHasTrimmedCircularSource(budget, pa.Profile.Outer, pa.Profile.Holes)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !trimmedCircular {
		if trimmedCircular, err = prismcells.ProfileHasTrimmedCircularSource(budget, pb.Profile.Outer, pb.Profile.Holes); err != nil {
			return prismPayload{}, false, err
		}
	}
	if trimmedCircular {
		return prismPayload{}, false, nil
	}

	switch op {
	case meshbool.OpUnion:
		if len(pa.Profile.Holes) != 0 || len(pb.Profile.Holes) != 0 { // G6: both hole-free
			return prismPayload{}, false, nil
		}
		if !prismplacement.UnionZIntervalMatches(pa.PlacementOperand(), pb.PlacementOperand()) { // G5
			return prismPayload{}, false, nil
		}
	case meshbool.OpCut:
		if len(pb.Profile.Holes) != 0 { // G6: the TOOL must be hole-free; the target's own holes carry through
			return prismPayload{}, false, nil
		}
		if !prismplacement.CutZIntervalSpans(pa.PlacementOperand(), pb.PlacementOperand()) { // G5, §3.2's Cut row
			return prismPayload{}, false, nil
		}
	default:
		// No other op reaches this evaluator through performBoolean's dispatch
		// (meshbool.OpIntersect is handled above, before this shared preamble runs).
		return prismPayload{}, false, nil
	}

	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1); combine the sections into one profile instead of applying this op once per feature, or accept the mesh path by making the pair non-coplanar`,
			ErrUnsupported, op, segments, prismcells.MaxArrangementSegments)
	}

	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return prismPayload{}, false, err
	}

	switch op {
	case meshbool.OpUnion:
		result, ok, err := prismcells.ResolveAndBuildUnion(ctx, budget, pa, pb, reexpress)
		return result, ok, err
	default: // meshbool.OpCut
		return resolveAndBuildPrismCut(ctx, budget, pa, pb, reexpress)
	}
}

// admitPrismIntersectPair runs Intersect's own per-op preamble from
// tryPrismBoolean — G1-G4 (admitPrismPairBudget), the trimmed-circular
// refusal (prismcells.ProfileHasTrimmedCircularSource) on both operands, G6's
// hole-free arms, G5's Intersect z-interval overlap
// (prismplacement.IntersectZIntervalOverlaps), the arrangement work cap
// (prismcells.RegionsWithinWorkCap), and the operand re-expression
// (prismcells.NewReexpression) — factored out of tryPrismBoolean's meshbool.OpIntersect arm
// (docs/prism-boolean-design.md §4.5's "Entry" paragraph) so a second caller
// can share it unchanged rather than duplicate it: tryPrismBoolean's own
// meshbool.OpIntersect case above, and §4.5's overlap-area reading
// (prismOverlapVolume, prism_overlap.go). This task adds no capability and
// changes no behaviour — every gate below runs in the exact order and shape
// it always has.
//
// ok=false (err always nil in that case) means "not admitted" (§3.1/§3.4):
// every miss above is silent fallback, exactly as tryPrismBoolean's own
// contract states. A non-nil err is RB7 (§9, the arrangement work cap — the
// one preamble refusal that is a genuine error, not a silent miss) or ctx
// cancellation.
func admitPrismIntersectPair(ctx context.Context, a, b *Body) (budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression, ok bool, err error) {
	budget = proofbound.NewWorkBudget(ctx) // §10: one counter for the whole attempt
	if err := budget.Err(); err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	pa, pb, ok, err = admitPrismPairBudget(budget, meshbool.OpIntersect, a, b) // G1-G4
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !ok {
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	trimmedCircular, err := prismcells.ProfileHasTrimmedCircularSource(budget, pa.Profile.Outer, pa.Profile.Holes)
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !trimmedCircular {
		if trimmedCircular, err = prismcells.ProfileHasTrimmedCircularSource(budget, pb.Profile.Outer, pb.Profile.Holes); err != nil {
			return nil, prismPayload{}, prismPayload{}, nil, false, err
		}
	}
	if trimmedCircular {
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	if len(pa.Profile.Holes) != 0 || len(pb.Profile.Holes) != 0 { // G6: both hole-free (§4.4's multi-lump row is why)
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}
	if !prismplacement.IntersectZIntervalOverlaps(pa.PlacementOperand(), pb.PlacementOperand()) { // G5
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !withinCap {
		return nil, prismPayload{}, prismPayload{}, nil, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1); combine the sections into one profile instead of applying this op once per feature, or accept the mesh path by making the pair non-coplanar`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismcells.MaxArrangementSegments)
	}

	reexpress, err = prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	return budget, pa, pb, reexpress, true, nil
}

// admitPrismPair checks G1 (both operands a prismPayload), G3 (the composed
// world normals are bit-identical and the two planes share one sweep axis,
// exactly) and G4 (every segment of both records is line/circle/arc) — §3.1.
// A reflected placement is no gate (docs/general-boolean-design.md §3 A4):
// G3 reads each world normal through ApplyDir, which maps a reflection's
// sweep direction like any other, and prismcells.BuildScene re-winds operand B's
// record when the composed relative map is a reflection
// (prismcells.Reexpression.Rewind). Two operands reflected by one placement share
// it in G3's shared-axis arm and need no re-expression.
//
// G3 has a coplanar arm and two shared-axis arms. SharedAxisOf needs equal
// stored transform bases and frame U/V, and proves the composed-origin offset
// lies exactly along the transformed frame normal. The coplanar arm needs the
// float dot product of the world origin difference with the world normal to
// be the literal zero. Every
// comparison is ordinary Go == on the stored r3.Vec/float64 values (the
// design's "bit-identical" / "the literal zero") or exact dyadic arithmetic:
// Go's == already treats -0.0 and 0.0 as equal, so no separate zero-sign
// handling is needed.
// A miss on any row returns ok=false, never an error (§3.1: "passing them is
// not admission" for what follows, but MISSING one is never a refusal). The
// G4 scan shares the operation budget because a large rejected profile must
// remain cancelable too.
func admitPrismPairBudget(budget *proofbound.WorkBudget, op meshbool.OperationKind, a, b *Body) (pa, pb prismPayload, ok bool, err error) {
	pa, aok := a.n.Payload.(prismPayload) // G1
	pb, bok := b.n.Payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, prismPayload{}, false, nil
	}
	aAnalytic, err := prismcells.ProfileAnalytic(budget, pa.Profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, false, err
	}
	bAnalytic, err := prismcells.ProfileAnalytic(budget, pb.Profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, false, err
	}
	if !aAnalytic || !bAnalytic { // G4
		return prismPayload{}, prismPayload{}, false, nil
	}
	worldNormalA := pa.Xform.ApplyDir(pa.Frame.N())
	worldNormalB := pb.Xform.ApplyDir(pb.Frame.N())
	if worldNormalA != worldNormalB { // G3: co-directional, bit-identical
		return prismPayload{}, prismPayload{}, false, nil
	}
	shared := prismplacement.SharedAxisOf(pa.PlacementOperand(), pb.PlacementOperand())
	if shared.OK && pa.Xform != pb.Xform && !prismcells.PlacedAxisProfileAdmitted(op, pa, pb) {
		return prismPayload{}, prismPayload{}, false, nil
	}
	if !shared.OK { // G3
		worldOriginA := pa.Xform.Apply(pa.Frame.Origin())
		worldOriginB := pb.Xform.Apply(pb.Frame.Origin())
		if worldOriginB.Sub(worldOriginA).Dot(worldNormalA) != 0.0 { // G3's coplanar arm, unchanged
			return prismPayload{}, prismPayload{}, false, nil
		}
	}
	return pa, pb, true, nil
}

// prismSceneDelta names the internal scene charge carried through boolean paths.
type prismSceneDelta = prismcells.SceneDelta

// prismReexpression names the internal scene mapper used by boolean paths.
type prismReexpression = prismcells.Reexpression
