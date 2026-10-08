package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/prism-boolean-design.md's analytic reduction of
// Union/Cut/Intersect over co-directional straight prisms on one plane or on
// shared-axis offset planes, routed entirely through sketch (§4) rather than
// through the mesh boolean (evaluator-design §9). The entry gate (§3) is reject-only and never
// surfaces an error on a miss — the caller falls back to the unchanged mesh
// path exactly as it did before this file existed. Where either input
// carries a section displacement, a consumed source segment's own walk
// computed a coordinate (§7's δ_walk, walkChargeOf), or B's re-expression is
// nonidentity, every crossing the arrangement cuts can amplify that
// displacement by 1/sin θ, and docs/general-boolean-design.md §3 A6 charges
// it (prismSceneDelta.chargeCrossings, prismcells.CrossingCharge). Such a
// pair never refuses: a crossing too close to tangent to bound, or any later
// analytic failure, sends it to the mesh path (prismAmplifiedFallback). Every cut also charges the cut
// parameters' OWN rounding into the result's section displacement (§7,
// prismcells.CutDelta) — a merged section built from fragments is never
// exact, whatever the operands were. A
// consumed source segment whose own recorded range is narrower than its
// natural domain enters the private scene at a WALKED endpoint the boolean
// itself computed (buildPrismScene's own doc comment) rather than at the
// record's own coordinate, and that computed endpoint is charged too — a
// trimmed circular carrier (ArcSeg/CircleSeg) moves by more than a
// coordinate displacement can state, so this file refuses that pair before
// building the scene at all rather than under-charge it
// (prismProfileHasTrimmedCircularSource). Only once §4.2's remaining
// resolution finds a unique candidate does a further problem become a
// genuine, typed refusal (§3.4, §9) rather than a reroute to the mesh path.
//
// Union's resolution (§4.2) is the hole-free select-all/merge/chain path: the
// candidate is assembled from every returned cell, and §6's audit re-proves
// the assembly the same way every modify op re-checks its own rewrite. It
// uses prismcells.Merge through mergePrismCells — the same chain/merge
// tail the crossing sub-case below reuses over its own, narrower cell
// selection. Cut/Intersect's clean-nesting structural match (§4.2's "clean"
// sub-case) is prism_boolean_nesting.go — see that file's own header for why
// it needs no assembly and no §6 audit. This file owns what every path
// shares: G1-G4 admission, the work-budget cap, the private scene
// construction (buildPrismScene) and the operand coordinate re-expression
// (prismReexpression).
//
// Cut/Intersect's crossing sub-case — genuine boundary contact, not clean
// nesting — is prism_boolean_crossing.go: §4.2's edge-orientation
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
// refuses: it falls back (prismAmplifiedFallback).
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
	pa, pb, ok, err := admitPrismPairBudget(budget, a, b) // G1-G4
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
	trimmedCircular, err := prismProfileHasTrimmedCircularSource(budget, pa.profile)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !trimmedCircular {
		if trimmedCircular, err = prismProfileHasTrimmedCircularSource(budget, pb.profile); err != nil {
			return prismPayload{}, false, err
		}
	}
	if trimmedCircular {
		return prismPayload{}, false, nil
	}

	switch op {
	case meshbool.OpUnion:
		if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 { // G6: both hole-free
			return prismPayload{}, false, nil
		}
		if !prismUnionZIntervalMatches(pa, pb) { // G5, §3.2's Union row
			return prismPayload{}, false, nil
		}
	case meshbool.OpCut:
		if len(pb.profile.Holes) != 0 { // G6: the TOOL must be hole-free; the target's own holes carry through
			return prismPayload{}, false, nil
		}
		if !prismCutZIntervalSpans(pa, pb) { // G5, §3.2's Cut row: the tool spans the target
			return prismPayload{}, false, nil
		}
	default:
		// No other op reaches this evaluator through performBoolean's dispatch
		// (meshbool.OpIntersect is handled above, before this shared preamble runs).
		return prismPayload{}, false, nil
	}

	segments, withinCap, err := prismSceneWithinWorkCap(budget, pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1); combine the sections into one profile instead of applying this op once per feature, or accept the mesh path by making the pair non-coplanar`,
			ErrUnsupported, op, segments, prismMaxArrangementSegments)
	}

	reexpress, err := newPrismReexpression(pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}

	switch op {
	case meshbool.OpUnion:
		return resolveAndBuildPrismUnion(ctx, budget, pa, pb, reexpress)
	default: // meshbool.OpCut
		return resolveAndBuildPrismCut(ctx, budget, pa, pb, reexpress)
	}
}

// admitPrismIntersectPair runs Intersect's own per-op preamble from
// tryPrismBoolean — G1-G4 (admitPrismPairBudget), the trimmed-circular
// refusal (prismProfileHasTrimmedCircularSource) on both operands, G6's
// hole-free arms, G5's Intersect z-interval overlap
// (prismIntersectZIntervalOverlaps), the arrangement work cap
// (prismSceneWithinWorkCap), and the operand re-expression
// (newPrismReexpression) — factored out of tryPrismBoolean's meshbool.OpIntersect arm
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
	pa, pb, ok, err = admitPrismPairBudget(budget, a, b) // G1-G4
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !ok {
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	trimmedCircular, err := prismProfileHasTrimmedCircularSource(budget, pa.profile)
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !trimmedCircular {
		if trimmedCircular, err = prismProfileHasTrimmedCircularSource(budget, pb.profile); err != nil {
			return nil, prismPayload{}, prismPayload{}, nil, false, err
		}
	}
	if trimmedCircular {
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 { // G6: both hole-free (§4.4's multi-lump row is why)
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}
	if !prismIntersectZIntervalOverlaps(pa, pb) { // G5, §3.2's Intersect row
		return nil, prismPayload{}, prismPayload{}, nil, false, nil
	}

	segments, withinCap, err := prismSceneWithinWorkCap(budget, pa, pb)
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	if !withinCap {
		return nil, prismPayload{}, prismPayload{}, nil, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1); combine the sections into one profile instead of applying this op once per feature, or accept the mesh path by making the pair non-coplanar`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismMaxArrangementSegments)
	}

	reexpress, err = newPrismReexpression(pa, pb)
	if err != nil {
		return nil, prismPayload{}, prismPayload{}, nil, false, err
	}
	return budget, pa, pb, reexpress, true, nil
}

// resolveAndBuildPrismUnion runs Union's hole-free select-all/merge/chain
// resolution (§4.2) and, once it commits, §6's build-time audit and §7's
// exactness. Split out of tryPrismBoolean only so each op's own resolve+build
// sequence reads as one unit.
func resolveAndBuildPrismUnion(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	merged, sceneDelta, cutDelta, resolved, err := resolvePrismUnion(ctx, budget, pa, pb, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil // §4.4: this topology is unresolved
	}

	// Point of no return (§3.4): every further problem is a genuine refusal,
	// unless the cuts carry an amplified displacement (prismAmplifiedFallback).
	if fallBack, err := prismAmplifiedFallback(sceneDelta.amplified, auditPrismMergeSection(budget, pa, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}
	result := prismPayload{
		profile: merged,
		frame:   pa.frame,
		z0:      pa.z0,
		z1:      pa.z1,
		z0Delta: max(pa.z0Delta, pb.z0Delta),
		z1Delta: max(pa.z1Delta, pb.z1Delta),
		xform:   pa.xform,
		// §7: operand A's coordinates are unchanged except for its own
		// consumed segments' walk charge (sceneDelta.a), while B's existing
		// displacement passes through its rigid re-expression, accumulates
		// the rounding that re-expression commits, and its own walk charge
		// (sceneDelta.b) besides. On top of both, every surviving CUT
		// fragment names its endpoints by a freshly rounded parameter this
		// union did not have before it ran, so cutDelta stands even where the
		// two operands carried nothing at all, and every cut the inputs'
		// displacement can move carries A6's crossing charge
		// (sceneDelta.crossing). A chained union must not discard any of
		// the five.
		sectionDelta: sceneDelta.merged(pa, pb, reexpress, cutDelta),
	}
	return result, true, nil
}

// admitPrismPair checks G1 (both operands a prismPayload), G3 (the composed
// world normals are bit-identical and the two planes share one sweep axis,
// exactly) and G4 (every segment of both records is line/circle/arc) — §3.1.
// A reflected placement is no gate (docs/general-boolean-design.md §3 A4):
// G3 reads each world normal through ApplyDir, which maps a reflection's
// sweep direction like any other, and buildPrismScene re-winds operand B's
// record when the composed relative map is a reflection
// (prismReexpression.rewound). Two operands reflected by one placement share
// it in G3's shared-axis arm and need no re-expression.
//
// G3 has two arms. The shared-axis
// arm (prismSharedAxisOf) needs one placement, bit-identical U/V and a
// frame-origin difference whose cross product with N is exactly zero over the
// stored floats; the coplanar arm needs the float dot product of the world
// origin difference with the world normal to be the literal zero. Every
// comparison is ordinary Go == on the stored r3.Vec/float64 values (the
// design's "bit-identical" / "the literal zero") or exact dyadic arithmetic:
// Go's == already treats -0.0 and 0.0 as equal, so no separate zero-sign
// handling is needed.
// A miss on any row returns ok=false, never an error (§3.1: "passing them is
// not admission" for what follows, but MISSING one is never a refusal). The
// G4 scan shares the operation budget because a large rejected profile must
// remain cancelable too.
func admitPrismPairBudget(budget *proofbound.WorkBudget, a, b *Body) (pa, pb prismPayload, ok bool, err error) {
	pa, aok := a.payload.(prismPayload) // G1
	pb, bok := b.payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, prismPayload{}, false, nil
	}
	aAnalytic, err := prismProfileIsAnalytic(budget, pa.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, false, err
	}
	bAnalytic, err := prismProfileIsAnalytic(budget, pb.profile)
	if err != nil {
		return prismPayload{}, prismPayload{}, false, err
	}
	if !aAnalytic || !bAnalytic { // G4
		return prismPayload{}, prismPayload{}, false, nil
	}
	worldNormalA := pa.xform.ApplyDir(pa.frame.N())
	worldNormalB := pb.xform.ApplyDir(pb.frame.N())
	if worldNormalA != worldNormalB { // G3: co-directional, bit-identical
		return prismPayload{}, prismPayload{}, false, nil
	}
	if !prismSharedAxisOf(pa, pb).ok { // G3's shared-axis arm (exact, §3.1)
		worldOriginA := pa.xform.Apply(pa.frame.Origin())
		worldOriginB := pb.xform.Apply(pb.frame.Origin())
		if worldOriginB.Sub(worldOriginA).Dot(worldNormalA) != 0.0 { // G3's coplanar arm, unchanged
			return prismPayload{}, prismPayload{}, false, nil
		}
	}
	return pa, pb, true, nil
}

func admitPrismPair(a, b *Body) (pa, pb prismPayload, ok bool) {
	pa, pb, ok, _ = admitPrismPairBudget(proofbound.NewWorkBudget(context.Background()), a, b)
	return pa, pb, ok
}

// prismProfileIsAnalytic reports G4: every segment of every loop is a
// LineSeg, CircleSeg or ArcSeg. A single free-form segment blinds sketch's
// whole-scene TExact gate (§3.1's own reasoning), so the class excludes the
// kind entirely rather than admitting "the free-form parts don't touch."
func prismProfileIsAnalytic(budget *proofbound.WorkBudget, p ProfileRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return false, err
			}
			switch seg.(type) {
			case LineSeg, CircleSeg, ArcSeg:
			default:
				return false, nil
			}
		}
	}
	return true, nil
}

// prismMaxArrangementSegments bounds the private sketch arrangement before
// s.Profiles starts. The pinned sketch arranger densifies each line to one tiny
// segment and each admitted circle or arc to no more than 256, then compares
// every tiny-segment pair in one O(n^2) pass. sketch.Sketch.Profiles takes no
// context, so that pass is the longest stretch a cancelled caller must wait
// through, and this cap is what bounds it (§10). The pass costs about
// 8.3e-5 ms per segment squared, so this value bounds one arrangement at
// roughly 1.4 seconds, and it admits a rectangular plate carrying fourteen
// circular holes against one more circular tool. It bounds latency alone:
// peak memory at twice this many segments is under 16 MB.
const prismMaxArrangementSegments = 4096

func prismSceneWithinWorkCap(budget *proofbound.WorkBudget, pa, pb prismPayload) (int, bool, error) {
	return prismRegionsWithinWorkCap(budget, pa.profile, pb.profile)
}

// prismRegionsWithinWorkCap is prismSceneWithinWorkCap over every region a
// private scene will hold, a prism group's lumps included.
func prismRegionsWithinWorkCap(budget *proofbound.WorkBudget, profiles ...ProfileRecord) (int, bool, error) {
	segments := 0
	for _, profile := range profiles {
		for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
			for _, seg := range loop.Segments {
				if err := budget.Step(); err != nil {
					return segments, false, err
				}
				switch seg.(type) {
				case LineSeg:
					segments++
				case CircleSeg, ArcSeg:
					segments += 256
				default:
					return segments, false, nil // G4 already excluded this case.
				}
				if segments > prismMaxArrangementSegments {
					return segments, false, nil
				}
			}
		}
	}
	return segments, true, nil
}

// prismSharedAxis is G3's shared-axis arm (docs/prism-boolean-design.md §3.1):
// ok when the two operands share one placement and bit-identical U/V and B's
// frame origin sits on A's normal axis EXACTLY, with shift the exact rational
// s for which originB − originA == s·N over the stored floats. A pair drawn on
// one frame is the arm's d = 0 case.
type prismSharedAxis struct {
	ok    bool
	shift *big.Rat
}

func prismPlacementOf(p prismPayload) prismplacement.Operand {
	return prismplacement.Operand{Frame: p.frame, Xform: p.xform, Z0: p.z0, Z1: p.z1}
}

// prismSharedAxisOf decides G3's shared-axis arm over the stored floats taken
// exactly. B's denoted prism {X(oB + uU + vV + zN)} is then
// {X(oA + uU + vV + (z+s)N)} term for term, with no orthonormality assumption
// on the stored frame, so B's Point2 fields are A-frame coordinates verbatim
// and only the sweep interval moves, by s. It is reject-only: anything but an
// exactly zero cross product of the origin difference with N refuses the arm.
// No float arithmetic is performed — dvSub, dvCross, dyadic.rat and
// big.Rat.Quo are exact.
func prismSharedAxisOf(pa, pb prismPayload) prismSharedAxis {
	axis := prismplacement.SharedAxisOf(prismPlacementOf(pa), prismPlacementOf(pb))
	return prismSharedAxis{ok: axis.OK, shift: axis.Shift}
}

// prismZShift is G5's shift s as an exact rational (§3.1): the shared-axis
// arm's d_i/N_i, or the literal zero in G3's coplanar arm, whose float dot
// product G3 required to be exactly 0.0. No float operation is performed.
// Every op's G5 check, and Intersect's result interval, read this SAME shift.
func prismZShift(pa, pb prismPayload) *big.Rat {
	return prismplacement.ZShift(prismPlacementOf(pa), prismPlacementOf(pb))
}

// prismShiftedInterval is operand B's [z0, z1] re-expressed onto operand A's
// axis exactly: floatRat(z) + s per end. ok is false when a level does not
// lift (non-finite), which every G5 check treats as a miss.
func prismShiftedInterval(pa, pb prismPayload) (*big.Rat, *big.Rat, bool) {
	return prismplacement.ShiftedInterval(prismPlacementOf(pa), prismPlacementOf(pb))
}

// prismShiftedIntervalAdmitted is prismShiftedInterval for a pair G5 already
// admitted (so the lift cannot fail); it panics naming this gate otherwise,
// the mustDyOf contract.
func prismShiftedIntervalAdmitted(pa, pb prismPayload) (*big.Rat, *big.Rat) {
	z0, z1, ok := prismShiftedInterval(pa, pb)
	if !ok {
		panic("decad: G5 admitted a prism pair whose sweep interval does not lift exactly")
	}
	return z0, z1
}

// prismUnionZIntervalMatches is G5 for Union (§3.2): operand B's [z0, z1] is
// re-expressed onto operand A's normal axis by prismShiftedInterval, and Union
// requires the two intervals to match exactly, compared as rationals.
func prismUnionZIntervalMatches(pa, pb prismPayload) bool {
	return prismplacement.UnionZIntervalMatches(prismPlacementOf(pa), prismPlacementOf(pb))
}

// resolvePrismUnion is §4.2's hole-free select-all/merge/chain path. It
// returns the merged section, buildPrismScene's own per-operand §7 walk
// charge (sceneDelta), and §7's cut displacement over the merge — the largest
// allowance any surviving fragment's own cut parameters owe, zero when every
// survivor is a whole edge.
//
// sceneDelta also carries the crossing charge (prismSceneDelta.crossing) for
// every cut the arrangement made.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4), or a cut carries an amplified displacement the crossing
// charge cannot bound or the merge then fails on: the caller falls back to
// the mesh path with no error. A non-nil error — including ctx cancellation
// surfacing through budget — is genuine and must propagate.
func resolvePrismUnion(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (ProfileRecord, prismSceneDelta, float64, bool, error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.Err(); err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	profiles, err := prismCellProfiles(ctx, budget, s)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.Err(); err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.Step(); err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if len(profiles) == 0 {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil // §4.4: the scene holds no bounded cell at all
	}
	// docs/general-boolean-design.md §3 A6: every crossing the arrangement
	// cut is charged the input displacement it amplifies; a crossing too
	// close to tangent for that charge sends the pair to the mesh path.
	if ok, err := sceneDelta.chargeCrossings(budget, tags, profiles, pa, pb, reexpress); err != nil || !ok {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}

	// Union, hole-free operands (G6): select every returned cell, which is
	// the union once no cell is material of neither operand. Two hole-free
	// operands can still enclose such a cell between them, so a scene with
	// one is unresolved (§4.2, §4.4). mergePrismCells is the shared
	// merge/chain tail the crossing sub-case (prism_boolean_crossing.go)
	// reuses over its OWN, narrower selected cell set.
	voidFree, err := prismcells.CellsHaveNoVoid(budget, tags, profiles)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !voidFree {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
	}
	if ok, err := sceneDelta.sharedSpansBounded(budget, profiles); err != nil || !ok {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	merged, cutDelta, resolved, err := mergePrismCells(budget, profiles, "union")
	if fallBack, err := prismAmplifiedFallback(sceneDelta.amplified, err); fallBack || err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !resolved {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil // §4.4: not a shape this increment covers
	}
	return merged, sceneDelta, cutDelta, true, nil
}

// mergePrismCells preserves the root caller's profile result around the cell merge.
func mergePrismCells(budget *proofbound.WorkBudget, selected []*sketch.Profile, opName string) (ProfileRecord, float64, bool, error) {
	loop, cutDelta, resolved, err := prismcells.Merge(budget, selected, opName)
	if err != nil || !resolved {
		return ProfileRecord{}, 0, resolved, err
	}
	return ProfileRecord{Outer: loop}, cutDelta, true, nil
}

// prismProfilesHaveSplitBoundary keeps the existing root test seam.
func prismProfilesHaveSplitBoundary(budget *proofbound.WorkBudget, profiles []*sketch.Profile) (bool, error) {
	return prismcells.HasSplitBoundary(budget, profiles)
}

// prismCellProfiles is prismProfilesContext for a path that classifies and
// merges the scene's cells: it restates each cell's line runs at the
// vertices other cells report on the same line (prismcells.SplitRuns), so a
// span two operands share names one edge key in every cell walking it. A
// circular run it cannot restate returns no profile at all, which every
// caller reads as a scene with no cell to resolve: the mesh path.
func prismCellProfiles(ctx context.Context, budget *proofbound.WorkBudget, s *sketch.Sketch) ([]*sketch.Profile, error) {
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, err
	}
	split, resolved, err := prismcells.SplitRuns(budget, profiles)
	if err != nil || !resolved {
		return nil, err
	}
	return split, nil
}

// prismProfilesContext makes sketch's synchronous arrangement observable to a
// caller's context. Cancellation waits for the bounded arrangement worker to
// finish, so no worker survives the operation. Its discarded result cannot
// reach the document or its operands.
func prismProfilesContext(ctx context.Context, profiles func() []*sketch.Profile) ([]*sketch.Profile, error) {
	return prismcells.ProfilesContext(ctx, profiles)
}

// auditPrismMergeSection runs the modify §5 audit (fillet_audit.go,
// §6 of this design) on a merged section, reused verbatim with an empty
// blend map — shell_offset.go's own precedent for "no cutback data, still run
// the shared audit". orig supplies the S8 sign reference: operand A's own
// outer loop, whose CCW orientation and non-degenerate area the merged Outer
// loop must match — the merge is correct only when it keeps that same
// convention. S9 (nesting) is a no-op here: G6 keeps every operand hole-free
// for Union, and the crossing sub-case's own scoping
// (prism_boolean_crossing.go) does the same, so the merged result is always
// hole-free. This audit runs on every assembled (chained-and-merged) section —
// Union's own select-all path and the crossing sub-case for Cut/Intersect
// alike; see this file's header comment for why Cut/Intersect's clean-nesting
// match needs none.
func auditPrismMergeSection(budget *proofbound.WorkBudget, pa prismPayload, merged ProfileRecord) error {
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: merged})
	if err != nil {
		return err
	}
	blendAt := make([]map[int]*cornerBlend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*cornerBlend{}
	}
	orig := ProfileRecord{Outer: pa.profile.Outer}
	return auditRewriteBudget(budget, orig, merged, loops, blendAt)
}

// prismProfileHasTrimmedCircularSource preserves the root admission seam.
func prismProfileHasTrimmedCircularSource(budget *proofbound.WorkBudget, p ProfileRecord) (bool, error) {
	return prismcells.ProfileHasTrimmedCircularSource(budget, p.Outer, p.Holes)
}

// walkChargeOf preserves the root scene builder and its test seam.
func walkChargeOf(seg CurveSegment, w survey2d.SegmentWalk) (float64, error) {
	return prismcells.WalkChargeOf(seg, w)
}

// prismSceneDelta is buildPrismScene's own per-operand δ_walk accumulator
// (§7): the largest walkChargeOf allowance among the segments buildPrismScene
// actually consumed from operand A and from operand B, tracked separately
// because B's own charge composes BEFORE the re-expression's rounding
// (prismReexpression.delta), matching every other §7 term's own ordering.
//
// crossing is docs/general-boolean-design.md §3 A6's crossing charge
// (prismcells.CrossingCharge): the largest distance an input displacement
// can move a crossing the arrangement cut. It is zero when neither operand
// brings a displacement, and on every path that cuts nothing.
//
// amplified records that the arrangement cut something while an operand
// brought a displacement: the case prism-boolean §3.4 charges. Such a pair
// never refuses on the analytic path; every problem past the charge sends it
// to the mesh path instead (prismAmplifiedFallback).
//
// shared is the scene's coincident spans (docs/general-boolean-design.md
// §3 A3); sharedWidth is how far the result's record of a span can sit from
// either operand's true wall there: the gap between the two recorded
// entities plus both incoming displacements, zero for two exact walls drawn
// on one carrier. sharedDisplaced records that an incoming displacement is
// part of it.
type prismSceneDelta struct {
	a, b            float64
	crossing        float64
	amplified       bool
	shared          prismcells.CoincidentReading
	sharedWidth     float64
	sharedDisplaced bool
}

// incoming is §7's two incoming displacements: operand A's own section
// displacement with its walk charge, and operand B's with its walk charge
// and the re-expression's rounding.
func (d prismSceneDelta) incoming(pa, pb prismPayload, reexpress *prismReexpression) (float64, float64) {
	return proofbound.AbsSumUpper(pa.sectionDelta, d.a), proofbound.AbsSumUpper(pb.sectionDelta, d.b, reexpress.delta)
}

// chargeCrossings sets d.amplified, d.shared, d.sharedWidth and d.crossing
// from the arrangement's returned cells. d.crossing is the larger of two
// terms: A6's crossing charge (prismcells.CrossingCharge) for every cut an
// operand's displacement can move, and, where two operands' coincident lines
// share a span sketch resolved (prismcells.CoincidentEdges), sharedWidth:
// the result records the span on one operand's line, and the other
// operand's true wall sits within that width of it. That covers a span the
// result keeps as boundary; sharedSpansBounded refuses the rest. ok=false
// (err always nil then) means a crossing or a span has no proven charge, and
// the caller falls back to the mesh path. A non-nil error is cancellation.
func (d *prismSceneDelta) chargeCrossings(budget *proofbound.WorkBudget, tags map[sketch.Entity]prismcells.Origin, profiles []*sketch.Profile, pa, pb prismPayload, reexpress *prismReexpression) (bool, error) {
	inA, inB := d.incoming(pa, pb, reexpress)
	coincident, ok, err := prismcells.CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return false, err
	}
	d.shared = coincident
	if len(coincident.Spans) > 0 {
		d.sharedWidth = proofbound.AbsSumUpper(coincident.Gap(), inA, inB)
		d.sharedDisplaced = inA > 0 || inB > 0
		d.crossing = max(d.crossing, d.sharedWidth)
	}
	if inA == 0 && inB == 0 {
		return true, nil
	}
	split, err := prismProfilesHaveSplitBoundary(budget, profiles)
	if err != nil {
		return false, err
	}
	if !split {
		return true, nil
	}
	d.amplified = true
	crossing, ok, err := prismcells.CrossingCharge(budget, tags, profiles, inA, inB)
	if err != nil || !ok {
		return false, err
	}
	d.crossing = max(d.crossing, crossing)
	return true, nil
}

// sharedSpansBounded reports whether the section displacement covers every
// coincident span of a selection.
//
// sketch's identity gate decides that the two recorded entities of a span
// are one carrier (prism-boolean §4.1: that decision is sketch's to make),
// so the section the two records denote has one wall there, with no sliver
// between two walls. A span the result keeps as boundary (exactly one
// selected cell beside it) is recorded on the named entity, which
// sharedWidth, in d.crossing, covers for either operand's wall.
//
// An incoming displacement is different: it says the operands' TRUE walls
// can sit up to sharedWidth apart, leaving a sliver the result does not
// record. On a boundary span that moves the boundary by at most sharedWidth,
// which is charged. A span inside the result or outside it would leave the
// sliver away from every recorded edge, where no displacement covers it, so
// a displaced pair with such a span takes the mesh path.
func (d prismSceneDelta) sharedSpansBounded(budget *proofbound.WorkBudget, selected []*sketch.Profile) (bool, error) {
	if !d.sharedDisplaced {
		return true, nil
	}
	return d.shared.OnBoundary(budget, selected)
}

// prismAmplifiedFallback is the routing rule for a pair whose cuts carry an
// amplified input displacement (prismSceneDelta.amplified): an error from
// the analytic resolution past the charge does not refuse the boolean, it
// sends the pair to the mesh path (fallBack=true, err=nil). Cancellation
// always propagates, and a pair that is not amplified keeps its error.
func prismAmplifiedFallback(amplified bool, err error) (bool, error) {
	if err == nil {
		return false, nil
	}
	if !amplified || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return false, err
	}
	return true, nil
}

// merged is the section displacement of a section assembled from cut
// fragments (§7, with A6's amplified crossing term):
// up(max(δ_A + δ_walkA, δ_B + δ_walkB + δ_reexpress, crossing) + δ_cut).
// An uncut stretch of either operand's boundary moves by that operand's own
// incoming term, a cut endpoint by the crossing charge, and every fragment
// endpoint additionally by its own cut parameter's rounding.
func (d prismSceneDelta) merged(pa, pb prismPayload, reexpress *prismReexpression, cutDelta float64) float64 {
	inA, inB := d.incoming(pa, pb, reexpress)
	return proofbound.AbsSumUpper(max(inA, inB, d.crossing), cutDelta)
}

// buildPrismScene is §4.1's scene construction: one private sketch.Sketch
// holding both operands' recorded entities — Outer AND every Holes[i] loop of
// each, so a Cut target's own carried-through holes and a nested Intersect
// operand's own boundary both reach the arrangement (Union's own admitted
// pairs are hole-free by G6, so this is a no-op widening for that path).
// Operand A's segments are created verbatim (A's frame is the reference);
// operand B's are re-expressed into A's frame first (prismReexpression.point) —
// the one new rounding this design introduces. When that map is a
// reflection, B's record is mapped and re-wound first
// (prismReexpression.rewound) and its entities are built from that record. Entities are deduplicated
// WITHIN each operand (the same dedup key discipline internal/momentinput/reconstruct.go's
// momentRecordScene already uses for one record) but NEVER across operands: a
// coincident carrier is handed to sketch as two separate, numerically
// matching entities, and sketch's own coincident-carrier resolution decides
// whether they merge.
//
// The second return value is §4.1's tag map: which operand and which loop
// each created entity traces to (record.go's own "outer loops CCW, holes CW"
// convention is unaffected — this is provenance, not orientation). Union's
// resolution ignores it; Cut/Intersect's clean-nesting match
// (prism_boolean_nesting.go, §4.2) reads it to prove the nesting relation
// structurally.
//
// Every entity is built from the segment's own WALKED geometry (walkOf),
// never from a Partial segment's recorded Center/Start/End as if it named a
// whole curve. A Partial segment's Start/End/Center are its SOURCE entity's
// own defining data, verbatim (record.go), which is exactly what a whole
// curve's fields hold too — nothing in a CurveSegment's own shape marks a
// fragment as partial. Recreating that source curve WHOLE is correct for an
// operand recorded straight from a live sketch profile (a genuine curve that
// may cross the OTHER operand beyond this operand's own walked range, which
// is exactly why momentRecordScene does the same for a single record's own
// self-consistency check). It stops being correct once an operand is itself
// a prior prism-boolean result: a Partial segment surviving THAT merge
// traces to one of the ORIGINAL pre-merge operands' own walls, and the
// portion beyond its walked range is now genuinely INTERIOR material, not a
// boundary of anything — recreating it whole resurrects a wall this
// operand's own solid does not have, which can silently misclassify a
// region in whatever new arrangement it enters. Every segment's OWN walked
// portion, and nothing more, is what this operand's boundary IS, whole or
// partial alike — so that is what gets built.
//
// A segment whose recorded range narrows its own natural domain enters the
// scene at that WALKED endpoint — a coordinate this evaluator computed, not
// one the record states verbatim — and walkChargeOf's allowance for it is
// accumulated into the returned prismSceneDelta, the largest such charge over
// each operand's own consumed segments (§7's δ_walk).
func buildPrismScene(budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (*sketch.Sketch, map[sketch.Entity]prismcells.Origin, prismSceneDelta, error) {
	return buildPrismSceneRegions(budget, []ProfileRecord{pa.profile}, []ProfileRecord{pb.profile}, reexpress)
}

// buildPrismSceneRegions adapts root profile records and the composed
// placement to prismcells' private scene builder.
func buildPrismSceneRegions(budget *proofbound.WorkBudget, regionsA, regionsB []ProfileRecord, reexpress *prismReexpression) (*sketch.Sketch, map[sketch.Entity]prismcells.Origin, prismSceneDelta, error) {
	s, tags, charge, err := prismcells.BuildSceneRegions(budget, sceneProfiles(regionsA), sceneProfiles(regionsB), reexpress)
	return s, tags, prismSceneDelta{a: charge.A, b: charge.B}, err
}

func sceneProfiles(regions []ProfileRecord) []prismcells.SceneProfile {
	out := make([]prismcells.SceneProfile, len(regions))
	for i, region := range regions {
		out[i] = prismcells.SceneProfile{Outer: region.Outer, Holes: region.Holes}
	}
	return out
}

func (re *prismReexpression) Reflection() bool { return re.reflection }

func (re *prismReexpression) MapPoint(p Point2) Point2 { return re.point(p) }

func (re *prismReexpression) Rewind(budget *proofbound.WorkBudget, region prismcells.SceneProfile) (prismcells.SceneProfile, float64, error) {
	rewound, charge, err := re.rewound(budget, ProfileRecord{Outer: region.Outer, Holes: region.Holes})
	return prismcells.SceneProfile{Outer: rewound.Outer, Holes: rewound.Holes}, charge, err
}

// prismReexpression is §4.1's coordinate re-expression of operand B into
// operand A's frame, and §7's proven displacement bound on what that
// re-expression rounds. It is stateful on purpose: point accumulates the largest
// allowance any coordinate it re-expressed owes, which is the section
// displacement the built payload carries.
//
// The map is COMPOSED first and applied once — B's frame to world, B's
// placement, A's placement inverted, A's frame inverted, all folded into one
// rigid transform — rather than walked point by point through world space. The
// two routes are the same algebra and differ only in where they round: the
// composed one rounds at the RELATIVE offset between the two operands, the
// walked one at each operand's own world magnitude, so a pair sitting far from
// the origin loses the whole difference between those magnitudes for nothing.
// The composed route is not a proof of anything, though — it narrows the
// rounding, it does not remove it — which is why the displacement below is
// carried regardless.
type prismReexpression struct {
	relative r3.Transform
	// identity records §7's one decidable zero case: G3's shared-axis arm
	// holds (prismSharedAxisOf), so B's denoted prism is A's frame swept over a
	// shifted interval, B's Point2 fields are A-frame coordinates verbatim, and
	// nothing is computed at all. Two profiles drawn on one sketch plane under
	// one placement are the arm's d = 0 case; a datum and its
	// CreateOffsetPlane are the d = s·N case.
	identity bool
	// reflection records that the composed relative map is improper
	// (Transform.IsReflection, read once in newPrismReexpression): exactly one
	// of the two accumulated placements is a reflection. The mapped record
	// then winds the wrong way, and buildPrismScene builds B from rewound's
	// record instead (docs/general-boolean-design.md §3 A4).
	reflection bool
	transAbs   float64
	delta      float64
}

// newPrismReexpression composes the map once. A rigid map's inverse is exact —
// the transpose, r3.Transform's own contract — and a Frame is orthonormal, so
// every step here is a dot product, never a solve.
func newPrismReexpression(pa, pb prismPayload) (*prismReexpression, error) {
	re, err := prismplacement.Compose(prismPlacementOf(pa), prismPlacementOf(pb))
	if err != nil {
		return nil, err
	}
	return &prismReexpression{
		relative:   re.Map,
		identity:   re.Identity,
		reflection: re.Reflection,
		transAbs:   re.TransAbs,
	}, nil
}

// rewound is docs/general-boolean-design.md §3 A4's re-wound record: operand
// B's profile mapped into A's frame through point, with every loop walked the
// other way. A reflection turns a counter-clockwise outer loop clockwise and
// a clockwise hole counter-clockwise; reversing the walk restores record.go's
// "outer loops CCW, holes CW" convention, so the material is on the walk's
// left again, which is what prismcells.Classify's flag comparison reads.
//
// Each loop keeps its index (Outer stays Outer, Holes[i] stays Holes[i]) and
// its segments are taken in reverse order. Per kind, with m the map:
//
//   - a whole LineSeg{S, E} becomes LineSeg{m(E), m(S)} over the same range.
//     The range order still names the walk's sense, and the swapped fields
//     reverse it;
//   - a whole ArcSeg{C, S, E} becomes ArcSeg{m(C), m(E), m(S)} over the same
//     range. The reflection turns the arc clockwise from m(S) to m(E), which
//     is the counter-clockwise arc from m(E) to m(S), and the reversed walk
//     along it keeps the original range order;
//   - a whole CircleSeg keeps its Radius, CCW and range, with its centre
//     mapped. The reflection reverses the circle's winding and the reversed
//     walk reverses it back, so the walk's winding, and therefore its CCW
//     flag, is unchanged;
//   - a LineSeg recorded over a narrowed range enters as a whole LineSeg
//     between its mapped walked endpoints (walkOf), reversed. Re-ranging it
//     to 1 − t is not an exact float operation for a general t, so the walked
//     endpoints stand in, and walkChargeOf's allowance for them (§7's δ_walk)
//     is returned for the caller to fold into operand B's walk charge.
//
// A trimmed ArcSeg or CircleSeg, or any other kind, is ErrUnsupported: every
// caller refuses those pairs before the scene is built
// (prismProfileHasTrimmedCircularSource, G4), so this is a defensive check.
// point charges each mapped coordinate's rounding into re.delta exactly as
// the unreflected path does. A reflection adds no rounding term of its own.
func (re *prismReexpression) rewound(budget *proofbound.WorkBudget, profile ProfileRecord) (ProfileRecord, float64, error) {
	mapPoint := func(p Point2) (Point2, error) { return re.point(p), nil }
	outer, charge, err := rewindLoop(budget, profile.Outer, mapPoint)
	if err != nil {
		return ProfileRecord{}, 0, err
	}
	result := ProfileRecord{Outer: outer}
	for _, hole := range profile.Holes {
		rewoundHole, holeCharge, err := rewindLoop(budget, hole, mapPoint)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		charge = math.Max(charge, holeCharge)
		result.Holes = append(result.Holes, rewoundHole)
	}
	return result, charge, nil
}

// rewindLoop is rewound's per-loop rule over any point map: the loop's
// segments in reverse order, each mapped by mapPoint and walked the other
// way, beside the largest walk charge a narrowed line's walked endpoints
// owe. It is the whole re-winding for a reflected map, and the mirror join
// (mirror_join.go) runs it with its own exact reflection, so the two
// constructions share one statement of the rule. mapPoint owns its own
// rounding charge.
func rewindLoop(budget *proofbound.WorkBudget, loop LoopRecord, mapPoint func(Point2) (Point2, error)) (LoopRecord, float64, error) {
	return prismcells.RewindLoop(budget, loop, mapPoint)
}

// point re-expresses one of operand B's plane-local points into operand A's
// frame, dropping the resulting local z — which G3's coplanar arm certified is
// zero; the shared-axis arm never reaches this map — and charges the rounding
// it commits.
//
// The charge is proofbound.RigidRoundAllow's existing shape, at the INPUT coordinate and
// the composed map's own translation, and it covers the composition as well as
// the application: each r3.Transform.Then rounds a unit-magnitude basis entry
// and a translation-magnitude component a handful of ulps, so three
// compositions and one application together stay under ~48·u·|input| +
// ~40·u·|translation|, where proofbound.RigidRoundAllow's 16 ulps at 2·|input| +
// |translation|, read as a 3D radius, allow ~110·u·|input| + ~55·u·|translation|.
func (re *prismReexpression) point(p Point2) Point2 {
	return prismplacement.Point(prismplacement.Relative{
		Map: re.relative, Identity: re.identity, TransAbs: re.transAbs,
	}, &re.delta, p)
}
