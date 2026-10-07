package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// path exactly as it did before this file existed. A sketch-split boundary
// also reroutes before resolution accepts a candidate whenever either input
// carries a section displacement, a consumed source segment's own walk
// computed a coordinate (§7's δ_walk, walkChargeOf), or B's re-expression is
// nonidentity: any of the three can amplify at the cut. A split boundary the
// reroute admits still charges the cut parameters' OWN rounding into the
// result's section displacement (§7, prismcells.CutDelta) — a merged section
// built from fragments is never exact, whatever the operands were. A
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
// MUST fall back to the unchanged mesh path with no error surfaced. That
// includes a split arranged boundary when either input carries a section
// displacement, either input carries a walk charge, or B's re-expression is
// nonidentity — any one of the three alone. A non-nil err means the
// bounded analytic resolution reached a genuine refusal (§3.4) — the caller
// MUST propagate it rather than reroute to the mesh path.
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

	// Point of no return (§3.4): every further problem is a genuine refusal.
	if err := auditPrismMergeSection(budget, pa, merged); err != nil {
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
		// two operands carried nothing at all. A chained union must not
		// discard any of the four.
		sectionDelta: proofbound.AbsSumUpper(
			max(
				proofbound.AbsSumUpper(pa.sectionDelta, sceneDelta.a),
				proofbound.AbsSumUpper(pb.sectionDelta, sceneDelta.b, reexpress.delta),
			),
			cutDelta,
		),
	}
	return result, true, nil
}

// admitPrismPair checks G1 (both operands a prismPayload), G2 (neither
// placement a reflection), G3 (the composed world normals are bit-identical
// and the two planes share one sweep axis, exactly) and G4 (every segment of
// both records is line/circle/arc) — §3.1. G3 has two arms. The shared-axis
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
	if pa.reflected() || pb.reflected() { // G2
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
	segments := 0
	for _, profile := range []ProfileRecord{pa.profile, pb.profile} {
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

// prismSharedAxisOf decides G3's shared-axis arm over the stored floats taken
// exactly. B's denoted prism {X(oB + uU + vV + zN)} is then
// {X(oA + uU + vV + (z+s)N)} term for term, with no orthonormality assumption
// on the stored frame, so B's Point2 fields are A-frame coordinates verbatim
// and only the sweep interval moves, by s. It is reject-only: anything but an
// exactly zero cross product of the origin difference with N refuses the arm.
// No float arithmetic is performed — dvSub, dvCross, dyadic.rat and
// big.Rat.Quo are exact.
func prismSharedAxisOf(pa, pb prismPayload) prismSharedAxis {
	if pa.xform != pb.xform || pa.frame.U() != pb.frame.U() || pa.frame.V() != pb.frame.V() {
		return prismSharedAxis{}
	}
	oa, ob, n := pa.frame.Origin(), pb.frame.Origin(), pa.frame.N()
	if !proofbound.FiniteVec(oa) || !proofbound.FiniteVec(ob) || !proofbound.FiniteVec(n) {
		return prismSharedAxis{}
	}
	d := proofarith.DvSub(proofarith.DyVec(ob), proofarith.DyVec(oa))
	nd := proofarith.DyVec(n)
	if !proofarith.DvIsZero(proofarith.DvCross(d, nd)) {
		return prismSharedAxis{}
	}
	// d = s·N exactly, so any component with N_i != 0 gives s; the largest
	// |N_i| is chosen, and a zero there (no valid frame has a zero normal)
	// refuses the arm so the quotient stays total.
	comps := [3]float64{n.X, n.Y, n.Z}
	i := 0
	for j := 1; j < len(comps); j++ {
		if math.Abs(comps[j]) > math.Abs(comps[i]) {
			i = j
		}
	}
	if comps[i] == 0 {
		return prismSharedAxis{}
	}
	return prismSharedAxis{ok: true, shift: new(big.Rat).Quo(d[i].Rat(), nd[i].Rat())}
}

// prismZShift is G5's shift s as an exact rational (§3.1): the shared-axis
// arm's d_i/N_i, or the literal zero in G3's coplanar arm, whose float dot
// product G3 required to be exactly 0.0. No float operation is performed.
// Every op's G5 check, and Intersect's result interval, read this SAME shift.
func prismZShift(pa, pb prismPayload) *big.Rat {
	if sa := prismSharedAxisOf(pa, pb); sa.ok {
		return sa.shift
	}
	return new(big.Rat)
}

// prismShiftedInterval is operand B's [z0, z1] re-expressed onto operand A's
// axis exactly: floatRat(z) + s per end. ok is false when a level does not
// lift (non-finite), which every G5 check treats as a miss.
func prismShiftedInterval(pa, pb prismPayload) (*big.Rat, *big.Rat, bool) {
	b0, b1 := proofarith.FloatRat(pb.z0), proofarith.FloatRat(pb.z1)
	if b0 == nil || b1 == nil {
		return nil, nil, false
	}
	shift := prismZShift(pa, pb)
	return b0.Add(b0, shift), b1.Add(b1, shift), true
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
	a0, a1 := proofarith.FloatRat(pa.z0), proofarith.FloatRat(pa.z1)
	z0, z1, ok := prismShiftedInterval(pa, pb)
	if a0 == nil || a1 == nil || !ok {
		return false
	}
	return a0.Cmp(z0) == 0 && a1.Cmp(z1) == 0
}

// resolvePrismUnion is §4.2's hole-free select-all/merge/chain path. It
// returns the merged section, buildPrismScene's own per-operand §7 walk
// charge (sceneDelta), and §7's cut displacement over the merge — the largest
// allowance any surviving fragment's own cut parameters owe, zero when every
// survivor is a whole edge.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4), or a split boundary can amplify either input's section
// displacement, either operand's own walk charge, or B's nonidentity
// re-expression (§3.4): the caller falls back to the mesh path with no
// error. A non-nil error — including ctx cancellation surfacing through
// budget — is always genuine and must propagate.
func resolvePrismUnion(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (ProfileRecord, prismSceneDelta, float64, bool, error) {
	s, _, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.Err(); err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
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
	if pa.sectionDelta != 0 || pb.sectionDelta != 0 || !reexpress.identity || sceneDelta.a != 0 || sceneDelta.b != 0 {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return ProfileRecord{}, prismSceneDelta{}, 0, false, err
		}
		if split {
			// A coordinate error in re-expressed B, either source section's
			// existing displacement, or either operand's own walk charge can
			// move an intersection by delta/sin(theta). This increment has no
			// certified lower crossing-angle bound, so it cannot record the
			// fragment with an honest displacement bound.
			return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
		}
	}

	// Union, hole-free operands (G6): select every returned cell — by
	// construction there is no bounded cell that is material of neither
	// operand (§4.2). mergePrismCells is the shared merge/chain tail the
	// crossing sub-case (prism_boolean_crossing.go) reuses over its OWN,
	// narrower selected cell set.
	merged, cutDelta, resolved, err := mergePrismCells(budget, profiles, "union")
	if err != nil {
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

// prismProfilesContext makes sketch's synchronous arrangement observable to a
// caller's context. Cancellation waits for the bounded arrangement worker to
// finish, so no worker survives the operation. Its discarded result cannot
// reach the document or its operands.
func prismProfilesContext(ctx context.Context, profiles func() []*sketch.Profile) ([]*sketch.Profile, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	done := make(chan []*sketch.Profile)
	go func() {
		done <- profiles()
	}()
	select {
	case result := <-done:
		return result, nil
	case <-ctx.Done():
		<-done
		return nil, ctx.Err()
	}
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

// prismProfileHasTrimmedCircularSource reports whether p carries an ArcSeg or
// CircleSeg whose recorded range is narrower than its own natural domain
// (task fu143, §3.4's obligations). Such a segment enters buildPrismScene's
// private scene through entities built from TWO cos/sin-computed points, so
// its rebuilt carrier's radius and sweep both move, not merely its
// endpoints — the mechanism walkChargeOf's plain coordinate charge does not
// state. This evaluator refuses the pair rather than under-charge it.
//
// "Whole" is decided the same way walkChargeOf decides it, through the same
// wholeSegmentRange test: exact float equality against 0 and 1, never a
// tolerance. A CircleSeg is read from its OWN recorded range, never from its
// walk's closed-ness: circularWalk (extrude.go) calls a walk closed whenever
// its swept angle lands within a fixed tolerance of a full turn, so a range
// short of 1 by an ulp walks as closed while still being a trimmed segment
// this refusal owes an answer for — and a decad-side tolerance that can
// ACCEPT is the admission gate CLAUDE.md's reject-only rule forbids.
func prismProfileHasTrimmedCircularSource(budget *proofbound.WorkBudget, p ProfileRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return false, err
			}
			switch s := seg.(type) {
			case ArcSeg:
				if !wholeSegmentRange(s.TStart, s.TEnd) {
					return true, nil
				}
			case CircleSeg:
				if !wholeSegmentRange(s.TStart, s.TEnd) {
					return true, nil
				}
			}
		}
	}
	return false, nil
}

// wholeSegmentRange reports whether a recorded parameter range names the two
// natural bounds of the segment's own domain, so that the segment's walk
// restates the entity's own defining data rather than a coordinate this
// evaluator computed. It is the single owner of that test for this file:
// prismProfileHasTrimmedCircularSource's refusal and walkChargeOf's zero
// charge are the same question asked twice, and they must not drift apart.
//
// The comparison is exact float equality against 0 and 1, never a tolerance
// in either direction — a range one ulp short of a bound is a trimmed
// segment, and admitting it on nearness would be a decad-side check that
// ACCEPTS, which CLAUDE.md's reject-only rule forbids. Which bound sits in
// which field does not matter: a Reversed whole edge records TStart=1,
// TEnd=0 (recordEdge's own "TStart and TEnd swapped" comment) and both
// values still name a natural bound, while validateSegmentRange (record.go)
// already rejects an empty TStart == TEnd range.
func wholeSegmentRange(tStart, tEnd float64) bool {
	return (tStart == 0 || tStart == 1) && (tEnd == 0 || tEnd == 1)
}

// walkChargeOf is §7's δ_walk mechanism (task fu143): the charge ONE consumed
// source segment owes for entering buildPrismScene's private scene at its own
// WALKED endpoint(s) rather than at the coordinates its record states
// verbatim.
//
// A WHOLE segment's walk restates the entity's own defining data exactly —
// lerp2 (moments.go) and pinArcWalkEnds (extrude.go) both special-case the
// natural bounds t=0/t=1 to return the record's own Point2 verbatim
// regardless of which field holds which (a Reversed whole edge records
// TStart=1, TEnd=0 — recordEdge's own "TStart and TEnd swapped" comment —
// and both still name a natural bound), and a CircleSeg recorded over those
// bounds walks the recorded centre and radius directly (circularWalk never
// touches Center/Radius) — so a whole segment charges nothing. A narrowed range
// evaluates the carrier at a COMPUTED parameter instead (lerp2's else arm, or
// circularWalk's cos/sin at the walk's own angle), which proofbound.WalkEndpointAllow
// charges.
//
// proofbound.WalkEndpointAllow is charged at the SOURCE operands the walk's own
// arithmetic touches, never at the endpoint it produced, and the envelope each
// kind passes is the kind's own: a line passes lineWalkOperandUpper, because
// lerp2's b−a cancels and leaves the walked endpoint no witness at all to the
// carrier magnitude that rounding happened at; a circular walk passes
// survey2d.SegmentWalk.coordUpper, whose |cu|+|cv|+r+r L1 form (circularWalk) already
// bounds the centre and radius its cos/sin arithmetic works on, so it IS the
// source envelope for that kind rather than an answer standing in for one.
//
// Only the line arm is reachable through tryPrismBoolean: a trimmed circular
// carrier is refused before the scene is built
// (prismProfileHasTrimmedCircularSource), because a coordinate envelope does
// not state the movement of its rebuilt radius and sweep, so every ArcSeg or
// CircleSeg that reaches this function through the boolean is WHOLE and
// charges zero. The narrowed circular arm stands anyway, so that a widening
// of that refusal meets a charge rather than a silent zero, and it is
// exercised directly by this file's own unit test rather than through a
// boolean.
//
// "Whole" is decided by wholeSegmentRange for every kind — exact float
// equality against 0 and 1, never a tolerance: a range short of a natural
// bound is a trimmed segment and must be charged, however near that bound it
// lies. A CircleSeg is decided from its recorded range too, not from its
// walk's tolerance-decided closed-ness, so that this charge and
// prismProfileHasTrimmedCircularSource's refusal answer one question the same
// way.
func walkChargeOf(seg CurveSegment, w survey2d.SegmentWalk) (float64, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return 0, err
	}
	switch s := seg.(type) {
	case LineSeg:
		if wholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
		return proofbound.WalkEndpointAllow(lineWalkOperandUpper(s, w)), nil
	case ArcSeg:
		if wholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
	case CircleSeg:
		if wholeSegmentRange(s.TStart, s.TEnd) {
			return 0, nil
		}
	default:
		return 0, fmt.Errorf(`%w: a %T segment has no walk charge this evaluator states`, ErrUnsupported, seg)
	}
	return proofbound.WalkEndpointAllow(w.CoordUpper), nil
}

// lineWalkOperandUpper is the envelope proofbound.WalkEndpointAllow requires for a
// trimmed LineSeg: an upper bound on every operand lerp2's general arm
// touches when it computes fl(a + fl(t·fl(b−a))) for that segment.
//
// Those operands are the carrier's own RECORDED Start and End coordinates —
// a Partial line fragment records its source sketch.Line's full Start/End
// with a narrowed range (recordEdge, seam.go), so the carrier can reach far
// past the fragment — together with the walked endpoint the outer sum
// produces. Folding the walk's own coordinate envelope in beside the recorded
// four costs nothing when the recorded range lies in [0, 1] (the lerp is then
// inside the carrier's own hull, which the recorded coordinates already
// bound) and keeps the envelope proven for any parameter at all, so this
// helper never leans on a range check it does not perform.
//
// A NaN coordinate propagates through math.Max, and an infinite one arrives as
// +Inf, so an absent envelope reaches proofbound.WalkEndpointAllow as the non-finite it
// is rather than as a small number.
func lineWalkOperandUpper(s LineSeg, w survey2d.SegmentWalk) float64 {
	upper := w.CoordUpper
	for _, c := range [...]float64{s.Start.U, s.Start.V, s.End.U, s.End.V} {
		upper = math.Max(upper, math.Abs(c))
	}
	return upper
}

// prismSceneDelta is buildPrismScene's own per-operand δ_walk accumulator
// (§7): the largest walkChargeOf allowance among the segments buildPrismScene
// actually consumed from operand A and from operand B, tracked separately
// because B's own charge composes BEFORE the re-expression's rounding
// (prismReexpression.delta), matching every other §7 term's own ordering.
type prismSceneDelta struct {
	a, b float64
}

// buildPrismScene is §4.1's scene construction: one private sketch.Sketch
// holding both operands' recorded entities — Outer AND every Holes[i] loop of
// each, so a Cut target's own carried-through holes and a nested Intersect
// operand's own boundary both reach the arrangement (Union's own admitted
// pairs are hole-free by G6, so this is a no-op widening for that path).
// Operand A's segments are created verbatim (A's frame is the reference);
// operand B's are re-expressed into A's frame first (reexpressPrismPoint) —
// the one new rounding this design introduces. Entities are deduplicated
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
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, nil, prismSceneDelta{}, fmt.Errorf(`decad: failed to build the private prism-boolean scene: %w`, err)
	}

	points := map[Point2]*sketch.Point{}
	point := func(p Point2) *sketch.Point {
		if existing, ok := points[p]; ok {
			return existing
		}
		created := s.CreatePoint(p.U, p.V)
		points[p] = created
		return created
	}

	tags := map[sketch.Entity]prismcells.Origin{}
	sceneDelta := prismSceneDelta{}

	addOperand := func(profile ProfileRecord, isB bool) error {
		type entityKey struct {
			kind    uint8 // 1 = line, 2 = whole circle, 3 = arc (incl. a partial circle)
			a, b, c Point2
			radius  float64
		}
		// Fresh per operand: §4.1 forbids deduplicating an entity across the
		// two operands, even where the same physical curve appears in both.
		entities := map[entityKey]struct{}{}
		reexpressPt := func(p Point2) Point2 {
			if !isB {
				return p
			}
			return reexpress.point(p)
		}
		loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
		for li, loop := range loops {
			hole := li - 1 // -1 names Outer; 0.. names Holes[hole]
			tag := func(ent sketch.Entity, authoredReversed bool) {
				tags[ent] = prismcells.Origin{IsB: isB, Hole: hole, AuthoredReversed: authoredReversed}
			}
			for _, seg := range loop.Segments {
				if err := budget.Step(); err != nil {
					return err
				}
				w, err := walkOf(seg, nil) // G4 admits only Line/Circle/Arc: no free-form work counter needed
				if err != nil {
					return err
				}
				charge, err := walkChargeOf(seg, w)
				if err != nil {
					return err
				}
				if isB {
					sceneDelta.b = math.Max(sceneDelta.b, charge)
				} else {
					sceneDelta.a = math.Max(sceneDelta.a, charge)
				}
				// authoredReversed is §4.2's crossing-sub-case bookkeeping
				// (prism_boolean_crossing.go): whether this operand's own
				// recorded walk of this entity runs backwards relative to the
				// entity's own natural parameterization, as sketch will later
				// report it through a returned BoundaryEdge.Reversed. A line's
				// creation order always matches the walk (never reversed); a
				// circle or arc's does whenever the walk's own angle runs from
				// high to low — the SAME th1<th0 test the arc branch below
				// already computes for its own lo/hi ordering, read once here
				// for any circular kind, whole or arc alike, since w.th0/w.th1
				// are populated either way.
				authoredReversed := w.IsCircular() && w.Th1 < w.Th0
				switch {
				case w.IsLine():
					start := reexpressPt(Point2{U: w.StartU, V: w.StartV})
					end := reexpressPt(Point2{U: w.EndU, V: w.EndV})
					key := entityKey{kind: 1, a: start, b: end}
					if _, ok := entities[key]; !ok {
						tag(s.CreateLine(point(start), point(end)), authoredReversed)
						entities[key] = struct{}{}
					}
				case w.IsCircular() && w.Closed:
					center := reexpressPt(Point2{U: w.CU, V: w.CV})
					key := entityKey{kind: 2, a: center, radius: w.Radius}
					if _, ok := entities[key]; !ok {
						tag(s.CreateCircle(point(center), w.Radius), authoredReversed)
						entities[key] = struct{}{}
					}
				case w.IsCircular():
					// sketch.CreateArc sweeps CCW from its second point to its
					// third; the walk's own OWN direction may run either way, so
					// the two candidate endpoints are passed in ascending-angle
					// order — the physical set of points between th0 and th1 is
					// the same set either way, since a walked arc never spans a
					// full turn.
					loU, loV, hiU, hiV := w.StartU, w.StartV, w.EndU, w.EndV
					if w.Th1 < w.Th0 {
						loU, loV, hiU, hiV = hiU, hiV, loU, loV
					}
					center := reexpressPt(Point2{U: w.CU, V: w.CV})
					lo := reexpressPt(Point2{U: loU, V: loV})
					hi := reexpressPt(Point2{U: hiU, V: hiV})
					key := entityKey{kind: 3, a: center, b: lo, c: hi}
					if _, ok := entities[key]; !ok {
						tag(s.CreateArc(point(center), point(lo), point(hi)), authoredReversed)
						entities[key] = struct{}{}
					}
				default:
					// G4 already excludes every other kind before this runs.
					return fmt.Errorf(`%w: a %T segment is not part of the admitted class`, ErrUnsupported, seg)
				}
			}
		}
		return nil
	}

	if err := addOperand(pa.profile, false); err != nil {
		return nil, nil, prismSceneDelta{}, err
	}
	if err := addOperand(pb.profile, true); err != nil {
		return nil, nil, prismSceneDelta{}, err
	}
	return s, tags, sceneDelta, nil
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
	transAbs float64
	delta    float64
}

// newPrismReexpression composes the map once. A rigid map's inverse is exact —
// the transpose, r3.Transform's own contract — and a Frame is orthonormal, so
// every step here is a dot product, never a solve.
func newPrismReexpression(pa, pb prismPayload) (*prismReexpression, error) {
	// Equal frames under one placement are the arm's d = 0 case. They are
	// tested on their own too, because prismSharedAxisOf refuses a frame
	// whose stored normal is zero, and two bit-identical frames are the
	// identity map whatever their normal holds.
	if (pa.frame == pb.frame && pa.xform == pb.xform) || prismSharedAxisOf(pa, pb).ok {
		return &prismReexpression{identity: true}, nil
	}
	fail := func(err error) (*prismReexpression, error) {
		return nil, fmt.Errorf(`decad: the operands' relative placement has no rigid composition: %w`, err)
	}
	m, err := r3.FromFrame(pb.frame)
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(pb.xform); err != nil {
		return fail(err)
	}
	invA, err := pa.xform.Inverse()
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(invA); err != nil {
		return fail(err)
	}
	toWorldA, err := r3.FromFrame(pa.frame)
	if err != nil {
		return fail(err)
	}
	invFrameA, err := toWorldA.Inverse()
	if err != nil {
		return fail(err)
	}
	if m, err = m.Then(invFrameA); err != nil {
		return fail(err)
	}
	return &prismReexpression{relative: m, transAbs: proofbound.VecMaxAbs(m.Translation())}, nil
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
	if re.identity {
		return p
	}
	out := re.relative.Apply(r3.NewVec(p.U, p.V, 0))
	re.delta = math.Max(re.delta, proofbound.RigidRoundAllow(max(math.Abs(p.U), math.Abs(p.V)), re.transAbs))
	return Point2{U: out.X, V: out.Y}
}
