package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is docs/prism-boolean-design.md's analytic reduction of
// Union/Cut/Intersect over two co-directional coplanar straight prisms,
// routed entirely through sketch (§4) rather than through the mesh boolean
// (evaluator-design §9). The entry gate (§3) is reject-only and never
// surfaces an error on a miss — the caller falls back to the unchanged mesh
// path exactly as it did before this file existed. A sketch-split boundary
// also reroutes before resolution accepts a candidate whenever either input
// carries a section displacement or B's re-expression is nonidentity: either
// uncertainty can amplify at the cut. A split boundary the reroute admits
// still charges the cut parameters' OWN rounding into the result's section
// displacement (§7, prismUnionCutDelta) — a merged section built from
// fragments is never exact, whatever the operands were. Only once §4.2's
// remaining resolution finds a unique candidate does a further problem become
// a genuine, typed refusal (§3.4, §9) rather than a reroute to the mesh path.
//
// Union's resolution (§4.2) is the hole-free select-all/merge/chain path: the
// candidate is assembled from every returned cell, and §6's audit re-proves
// the assembly the same way every modify op re-checks its own rewrite. It
// lives entirely in this file. Cut/Intersect's clean-nesting structural match
// (§4.2's "clean" sub-case) is prism_boolean_nesting.go — see that file's own
// header for why it needs no assembly and no §6 audit. This file owns what
// both paths share: G1-G4 admission, the work-budget cap, the private scene
// construction (buildPrismScene) and the operand coordinate re-expression
// (prismReexpression). The general per-cell edge-orientation classification
// for Cut/Intersect's crossing sub-case (PR3) is not implemented, so a pair
// whose operands' boundaries genuinely cross falls through to tryPrismBoolean
// returning ok=false, err=nil, exactly like any other unresolved topology
// (§4.4).

// tryPrismBoolean attempts the analytic reduction for op. ok=false (err
// always nil in that case) means "not admitted" per §3.1/§3.4: the caller
// MUST fall back to the unchanged mesh path with no error surfaced. That
// includes a split arranged boundary when either input carries a section
// displacement or B's re-expression is nonidentity. A non-nil err means the
// bounded analytic resolution reached a genuine refusal (§3.4) — the caller
// MUST propagate it rather than reroute to the mesh path.
//
// G1-G4 (admitPrismPairBudget) and the work cap are shared, unchanged, by
// every op; G5 and G6 (§3.1) and the resolution path (§4.2) are op-specific,
// per §3.2's table.
func tryPrismBoolean(ctx context.Context, op OpKind, a, b *Body) (prismPayload, bool, error) {
	budget := newWorkBudget(ctx) // §10: one counter for the whole attempt
	if err := budget.err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, a, b) // G1-G4
	if err != nil {
		return prismPayload{}, false, err
	}
	if !ok {
		return prismPayload{}, false, nil
	}

	switch op {
	case OpUnion:
		if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 { // G6: both hole-free
			return prismPayload{}, false, nil
		}
		if !prismUnionZIntervalMatches(pa, pb) { // G5, §3.2's Union row
			return prismPayload{}, false, nil
		}
	case OpCut:
		if len(pb.profile.Holes) != 0 { // G6: the TOOL must be hole-free; the target's own holes carry through
			return prismPayload{}, false, nil
		}
		if !prismCutZIntervalSpans(pa, pb) { // G5, §3.2's Cut row: the tool spans the target
			return prismPayload{}, false, nil
		}
	case OpIntersect:
		if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 { // G6: both hole-free (§4.4's multi-lump row is why)
			return prismPayload{}, false, nil
		}
		if !prismIntersectZIntervalOverlaps(pa, pb) { // G5, §3.2's Intersect row
			return prismPayload{}, false, nil
		}
	default:
		// No other op reaches this evaluator through performBoolean's dispatch.
		return prismPayload{}, false, nil
	}

	withinCap, err := prismSceneWithinWorkCap(budget, pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(`%w: the analytic %s scene exceeds this evaluator's arrangement work cap`, ErrUnsupported, opKindNames[op])
	}

	reexpress, err := newPrismReexpression(pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}

	switch op {
	case OpUnion:
		return resolveAndBuildPrismUnion(ctx, budget, pa, pb, reexpress)
	case OpCut:
		return resolveAndBuildPrismCut(ctx, budget, pa, pb, reexpress)
	default: // OpIntersect
		return resolveAndBuildPrismIntersect(ctx, budget, pa, pb, reexpress)
	}
}

// resolveAndBuildPrismUnion runs Union's hole-free select-all/merge/chain
// resolution (§4.2) and, once it commits, §6's build-time audit and §7's
// exactness. Split out of tryPrismBoolean only so each op's own resolve+build
// sequence reads as one unit.
func resolveAndBuildPrismUnion(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	merged, cutDelta, resolved, err := resolvePrismUnion(ctx, budget, pa, pb, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil // §4.4: this topology is unresolved
	}

	// Point of no return (§3.4): every further problem is a genuine refusal.
	if err := auditPrismUnionSection(budget, pa, merged); err != nil {
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
		// §7: operand A's coordinates are unchanged, while B's existing
		// displacement passes through its rigid re-expression and accumulates
		// the rounding that re-expression commits. On top of both, every
		// surviving CUT fragment names its endpoints by a freshly rounded
		// parameter this union did not have before it ran, so cutDelta stands
		// even where the two operands carried nothing at all. A chained union
		// must not discard any of the three.
		sectionDelta: absSumUpper(
			max(pa.sectionDelta, absSumUpper(pb.sectionDelta, reexpress.delta)),
			cutDelta,
		),
	}
	return result, true, nil
}

// admitPrismPair checks G1 (both operands a prismPayload), G2 (neither
// placement a reflection), G3 (the composed world planes are the same plane,
// exactly) and G4 (every segment of both records is line/circle/arc) — §3.1.
// G3's comparisons are ordinary Go == on the stored r3.Vec/float64 values
// (the design's "bit-identical" / "the literal zero"): Go's == already
// treats -0.0 and 0.0 as equal, so no separate zero-sign handling is needed.
// A miss on any row returns ok=false, never an error (§3.1: "passing them is
// not admission" for what follows, but MISSING one is never a refusal). The
// G4 scan shares the operation budget because a large rejected profile must
// remain cancelable too.
func admitPrismPairBudget(budget *workBudget, a, b *Body) (pa, pb prismPayload, ok bool, err error) {
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
	worldOriginA := pa.xform.Apply(pa.frame.Origin())
	worldOriginB := pb.xform.Apply(pb.frame.Origin())
	if worldOriginB.Sub(worldOriginA).Dot(worldNormalA) != 0.0 { // G3: coplanar
		return prismPayload{}, prismPayload{}, false, nil
	}
	return pa, pb, true, nil
}

func admitPrismPair(a, b *Body) (pa, pb prismPayload, ok bool) {
	pa, pb, ok, _ = admitPrismPairBudget(newWorkBudget(context.Background()), a, b)
	return pa, pb, ok
}

// prismProfileIsAnalytic reports G4: every segment of every loop is a
// LineSeg, CircleSeg or ArcSeg. A single free-form segment blinds sketch's
// whole-scene TExact gate (§3.1's own reasoning), so the class excludes the
// kind entirely rather than admitting "the free-form parts don't touch."
func prismProfileIsAnalytic(budget *workBudget, p ProfileRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			if err := budget.step(); err != nil {
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
// every tiny-segment pair. Capping this upper bound keeps a canceled caller from
// leaving an unbounded private arrangement behind.
const prismMaxArrangementSegments = 1024

func prismSceneWithinWorkCap(budget *workBudget, pa, pb prismPayload) (bool, error) {
	segments := 0
	for _, profile := range []ProfileRecord{pa.profile, pb.profile} {
		for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
			for _, seg := range loop.Segments {
				if err := budget.step(); err != nil {
					return false, err
				}
				switch seg.(type) {
				case LineSeg:
					segments++
				case CircleSeg, ArcSeg:
					segments += 256
				default:
					return false, nil // G4 already excluded this case.
				}
				if segments > prismMaxArrangementSegments {
					return false, nil
				}
			}
		}
	}
	return true, nil
}

// prismZShift is G5's re-expression of operand B's [z0, z1] onto operand A's
// normal axis (§3.1): an origin shift along the axis G3 already proved
// identical — ordinary float arithmetic, no rotation. Every op's G5 check,
// and Intersect's result interval, read this SAME shift.
//
// G3 already tests (worldOriginB - worldOriginA)·worldNormalA != 0.0 and
// refuses on anything but the literal zero, so this shift is 0.0, exactly,
// for every pair that reaches it — computed rather than assumed only because
// that is the design's own stated shape for G5 (§3.1's own row). §7's
// "Intersect's shifted interval endpoint" term is therefore zero for every
// pair this design admits, and no pair reaching this shift commits a new
// axial displacement by computing it.
func prismZShift(pa, pb prismPayload) float64 {
	worldNormalA := pa.xform.ApplyDir(pa.frame.N())
	worldOriginA := pa.xform.Apply(pa.frame.Origin())
	worldOriginB := pb.xform.Apply(pb.frame.Origin())
	return worldOriginB.Sub(worldOriginA).Dot(worldNormalA)
}

// prismUnionZIntervalMatches is G5 for Union (§3.2): operand B's [z0, z1] is
// re-expressed onto operand A's normal axis by prismZShift, and Union
// requires the two intervals to match exactly.
func prismUnionZIntervalMatches(pa, pb prismPayload) bool {
	shift := prismZShift(pa, pb)
	return pa.z0 == pb.z0+shift && pa.z1 == pb.z1+shift
}

// resolvePrismUnion is §4.2's hole-free select-all/merge/chain path. It returns
// the merged section and §7's cut displacement over it — the largest allowance
// any surviving fragment's own cut parameters owe, zero when every survivor is a
// whole edge.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4), or a split boundary can amplify either input's section
// displacement or B's nonidentity re-expression (§3.4): the caller falls
// back to the mesh path with no error. A non-nil error — including ctx
// cancellation surfacing through budget — is always genuine and must
// propagate.
func resolvePrismUnion(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (ProfileRecord, float64, bool, error) {
	s, _, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return ProfileRecord{}, 0, false, err
	}
	if err := budget.err(); err != nil {
		return ProfileRecord{}, 0, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return ProfileRecord{}, 0, false, err
	}
	if err := budget.err(); err != nil {
		return ProfileRecord{}, 0, false, err
	}
	if err := budget.step(); err != nil {
		return ProfileRecord{}, 0, false, err
	}
	if len(profiles) == 0 {
		return ProfileRecord{}, 0, false, nil // §4.4: the scene holds no bounded cell at all
	}
	if pa.sectionDelta != 0 || pb.sectionDelta != 0 || !reexpress.identity {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return ProfileRecord{}, 0, false, err
		}
		if split {
			// A coordinate error in re-expressed B, or either source section's
			// existing displacement, can move an intersection by
			// delta/sin(theta). This increment has no certified lower
			// crossing-angle bound, so it cannot record the fragment with an
			// honest displacement bound.
			return ProfileRecord{}, 0, false, nil
		}
	}

	// Union, hole-free operands (G6): select every returned cell — by
	// construction there is no bounded cell that is material of neither
	// operand (§4.2). Count every boundary edge (Outer and every Hole loop
	// of every selected cell) by (Entity, TStart, TEnd): a shared wall
	// between two adjacent cells is walked in opposite senses by each but
	// reports the SAME natural-direction TStart < TEnd range (Reversed, not
	// the order, states the walk direction — sketch-seam-design's own
	// contract), so this key alone matches it on both sides.
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	counts := map[edgeKey]int{}
	for _, p := range profiles {
		if err := budget.step(); err != nil {
			return ProfileRecord{}, 0, false, err
		}
		if !p.Valid {
			// RB1: a candidate region the merge depends on is invalid. Every
			// cell in this two-operand scene is selected for Union, so any
			// invalid cell is a genuine refusal, not an unresolved topology.
			return ProfileRecord{}, 0, false, fmt.Errorf(`%w: the union scene's arrangement reports an invalid region`, ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.step(); err != nil {
					return ProfileRecord{}, 0, false, err
				}
				counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]++
			}
		}
	}

	// Second pass, in the arrangement's own deterministic profile/edge
	// order: keep every edge counted exactly once (a wall against the
	// unbounded exterior, or a coincident-carrier wall reported under only
	// one operand's entity); drop every edge counted exactly twice (an
	// interior wall between two selected cells). A count outside {1, 2} is
	// topology this increment does not cover.
	var survivors []sketch.BoundaryEdge
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.step(); err != nil {
					return ProfileRecord{}, 0, false, err
				}
				n := counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]
				switch n {
				case 1:
					survivors = append(survivors, e)
				case 2:
					// dropped: an interior wall
				default:
					return ProfileRecord{}, 0, false, nil // §4.4: not a shape this increment covers
				}
			}
		}
	}
	if len(survivors) == 0 {
		return ProfileRecord{}, 0, false, nil
	}

	chain, resolved, err := chainPrismUnionSurvivors(budget, survivors)
	if err != nil {
		return ProfileRecord{}, 0, false, err
	}
	if !resolved {
		return ProfileRecord{}, 0, false, nil // §4.4: the survivors do not close into one simple loop
	}

	// Point of no return crossed within this function's own contract: every
	// further problem (a rejected TExact fragment, §9's RB8; a non-closing
	// merged loop, §9's RB9) is genuine.
	segs := make([]CurveSegment, len(chain))
	joins := make([]loopJoin, len(chain))
	cutDelta := 0.0
	for i, e := range chain {
		if err := budget.step(); err != nil {
			return ProfileRecord{}, 0, false, err
		}
		seg, err := recordEdge(e)
		if err != nil {
			return ProfileRecord{}, 0, false, err
		}
		segs[i] = seg
		join, err := edgeJoin(e, seg)
		if err != nil {
			return ProfileRecord{}, 0, false, err
		}
		joins[i] = join
		d, err := prismUnionCutDelta(e, seg)
		if err != nil {
			return ProfileRecord{}, 0, false, err
		}
		cutDelta = math.Max(cutDelta, d)
	}
	// §6's closure row (RB9): the seam's own junction falsifier, run on the
	// merged chain's recorded coordinates. recordLoop is not called here — it
	// takes no budget.step() charge and would drop cutDelta's per-edge
	// pairing — so this calls edgeJoin/falsifyLoopJoins directly, unchanged.
	if err := falsifyLoopJoins("merged union loop", joins); err != nil {
		return ProfileRecord{}, 0, false, err
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}, cutDelta, true, nil
}

// prismUnionCutDelta is §7's cut charge for one surviving boundary edge: how
// far the point the recorded range names can sit from the crossing it denotes.
//
// A WHOLE edge charges nothing. Its range is the entity's own full domain, so
// its endpoints are the entity's own recorded endpoints and this union computed
// no coordinate for it at all.
//
// A Partial fragment charges cutDisplacementAllow. Its carrier is the entity's
// unchanged defining data, but its two ends are named by TStart/TEnd — the
// parameters sketch's arrangement COMPUTED for this pair, which are freshly
// rounded whatever the two operands carried, and which the operands' own zero
// displacement therefore says nothing about. That is the whole of what §7's
// decidable zero case does not reach.
func prismUnionCutDelta(e sketch.BoundaryEdge, seg CurveSegment) (float64, error) {
	if !e.Partial {
		return 0, nil
	}
	speed, err := carrierSpeedUpper(seg)
	if err != nil {
		return 0, err
	}
	return cutDisplacementAllow(speed), nil
}

// carrierSpeedUpper is a proven upper bound on |dP/dt| over the WHOLE of a
// recorded line, circle or arc's own parameterisation — the factor that turns a
// parameter allowance into a coordinate one. G4 admits only these three kinds,
// so no other kind can reach it.
//
// Each bound is the parameterisation walkOf reads the segment through
// (extrude.go): a line runs Start → End over [0, 1], so its speed is the chord;
// a circle runs a full turn over [0, 1], so its speed is 2πR; an arc runs its
// own sweep, which is at most a full turn, so 2πR bounds it too.
func carrierSpeedUpper(seg CurveSegment) (float64, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return 0, err
	}
	switch seg := seg.(type) {
	case LineSeg:
		return point2SeparationUpper(seg.Start, seg.End), nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return 0, fmt.Errorf(`decad: a merged circle segment's radius is not a length: %w`, err)
		}
		return productUpper(twoPiUpper(), math.Abs(r)), nil
	case ArcSeg:
		return productUpper(twoPiUpper(), point2SeparationUpper(seg.Center, seg.Start)), nil
	default:
		return 0, fmt.Errorf(`%w: a %T segment has no carrier speed this evaluator states`, ErrUnsupported, seg)
	}
}

// point2SeparationUpper bounds |b − a| for two recorded plane points. The two
// differences are taken EXACTLY over rationals and summed, which bounds the
// Euclidean norm because the L1 norm never falls below it. A non-finite
// coordinate has no separation to state and answers +Inf.
func point2SeparationUpper(a, b Point2) float64 {
	au, av, bu, bv := floatRat(a.U), floatRat(a.V), floatRat(b.U), floatRat(b.V)
	if au == nil || av == nil || bu == nil || bv == nil {
		return math.Inf(1)
	}
	return ratL1Upper(new(big.Rat).Sub(bu, au), new(big.Rat).Sub(bv, av))
}

// prismProfilesHaveSplitBoundary reports whether sketch narrowed any
// arranged boundary edge. Such a cut falls back before recordEdge can publish
// a trim when either source carries a section displacement or B's re-expression
// is nonidentity, because that uncertainty may be amplified by the crossing
// angle (prism-boolean-design §3.4, §7).
func prismProfilesHaveSplitBoundary(budget *workBudget, profiles []*sketch.Profile) (bool, error) {
	for _, profile := range profiles {
		if err := budget.step(); err != nil {
			return false, err
		}
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			for _, edge := range loop {
				if err := budget.step(); err != nil {
					return false, err
				}
				if edge.Partial {
					return true, nil
				}
			}
		}
	}
	return false, nil
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

// chainPrismUnionSurvivors chains the surviving boundary edges into one
// closed directed walk (§4.2), the same directed-edge-loop-closure shape
// boolean_body.go's face-patch construction performs for the mesh boolean's
// own loops. resolved=false means the survivors do not close into exactly
// one simple loop using every one of them — "not resolved" (§4.4): a
// disjoint pair of footprints (two separate lumps) or a union enclosing an
// internal void both leave a dangling end or a smaller cycle here.
//
// Connectivity is read off sketch's OWN walked boundary sample — Polyline[0]
// is the walk's start, Polyline[len-1] its end (sketch-seam-design's
// contract) — matched by exact (u, v) equality: these are vertices the SAME
// arrangement pass already computed and shares between the edges that meet
// there, so no tolerance is introduced. This is bookkeeping on sketch's own
// answer, not a re-derived geometric fact (CLAUDE.md's carve-out for this
// design).
func chainPrismUnionSurvivors(budget *workBudget, survivors []sketch.BoundaryEdge) (chain []sketch.BoundaryEdge, resolved bool, err error) {
	byStart := make(map[Point2]int, len(survivors))
	for i, e := range survivors {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
	}

	used := make([]bool, len(survivors))
	chain = make([]sketch.BoundaryEdge, 0, len(survivors))
	cur := 0
	for range survivors {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		if used[cur] {
			return nil, false, nil // a smaller cycle closed before every edge was used
		}
		used[cur] = true
		e := survivors[cur]
		chain = append(chain, e)
		end := Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
		next, ok := byStart[end]
		if !ok {
			return nil, false, nil // a dead end: no survivor continues from here
		}
		cur = next
	}
	if cur != 0 {
		return nil, false, nil // the walk did not return to its own start
	}
	return chain, true, nil
}

// auditPrismUnionSection runs the modify §5 audit (fillet_audit.go,
// §6 of this design) on the merged section, reused verbatim with an empty
// blend map — shell_offset.go's own precedent for "no cutback data, still run
// the shared audit". orig supplies the S8 sign reference: operand A's own
// outer loop, whose CCW orientation and non-degenerate area the merged Outer
// loop must match — the merge is correct only when it keeps that same
// convention. S9 (nesting) is a no-op here: G6 keeps every operand, and so
// the merged result, hole-free. This audit is Union's own — see this file's
// header comment for why Cut/Intersect's clean-nesting match needs none.
func auditPrismUnionSection(budget *workBudget, pa prismPayload, merged ProfileRecord) error {
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

// buildPrismScene is §4.1's scene construction: one private sketch.Sketch
// holding both operands' recorded entities — Outer AND every Holes[i] loop of
// each, so a Cut target's own carried-through holes and a nested Intersect
// operand's own boundary both reach the arrangement (Union's own admitted
// pairs are hole-free by G6, so this is a no-op widening for that path).
// Operand A's segments are created verbatim (A's frame is the reference);
// operand B's are re-expressed into A's frame first (reexpressPrismPoint) —
// the one new rounding this design introduces. Entities are deduplicated
// WITHIN each operand (the same dedup key discipline moments_validate.go's
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
func buildPrismScene(budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (*sketch.Sketch, map[sketch.Entity]prismEntityOrigin, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, nil, fmt.Errorf(`decad: failed to build the private prism-boolean scene: %w`, err)
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

	tags := map[sketch.Entity]prismEntityOrigin{}

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
			tag := func(ent sketch.Entity) { tags[ent] = prismEntityOrigin{isB: isB, hole: hole} }
			for _, seg := range loop.Segments {
				if err := budget.step(); err != nil {
					return err
				}
				w, err := walkOf(seg, nil) // G4 admits only Line/Circle/Arc: no free-form work counter needed
				if err != nil {
					return err
				}
				switch {
				case w.isLine():
					start := reexpressPt(Point2{U: w.startU, V: w.startV})
					end := reexpressPt(Point2{U: w.endU, V: w.endV})
					key := entityKey{kind: 1, a: start, b: end}
					if _, ok := entities[key]; !ok {
						tag(s.CreateLine(point(start), point(end)))
						entities[key] = struct{}{}
					}
				case w.isCircular() && w.closed:
					center := reexpressPt(Point2{U: w.cU, V: w.cV})
					key := entityKey{kind: 2, a: center, radius: w.radius}
					if _, ok := entities[key]; !ok {
						tag(s.CreateCircle(point(center), w.radius))
						entities[key] = struct{}{}
					}
				case w.isCircular():
					// sketch.CreateArc sweeps CCW from its second point to its
					// third; the walk's own OWN direction may run either way, so
					// the two candidate endpoints are passed in ascending-angle
					// order — the physical set of points between th0 and th1 is
					// the same set either way, since a walked arc never spans a
					// full turn.
					loU, loV, hiU, hiV := w.startU, w.startV, w.endU, w.endV
					if w.th1 < w.th0 {
						loU, loV, hiU, hiV = hiU, hiV, loU, loV
					}
					center := reexpressPt(Point2{U: w.cU, V: w.cV})
					lo := reexpressPt(Point2{U: loU, V: loV})
					hi := reexpressPt(Point2{U: hiU, V: hiV})
					key := entityKey{kind: 3, a: center, b: lo, c: hi}
					if _, ok := entities[key]; !ok {
						tag(s.CreateArc(point(center), point(lo), point(hi)))
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
		return nil, nil, err
	}
	if err := addOperand(pb.profile, true); err != nil {
		return nil, nil, err
	}
	return s, tags, nil
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
	// identity records §7's one decidable zero case: B's composed map into A's
	// frame is the identity in the stored floats, so B's Point2 fields are
	// copied verbatim and nothing is computed at all. Two profiles drawn on one
	// sketch plane under one placement are that case.
	identity bool
	transAbs float64
	delta    float64
}

// newPrismReexpression composes the map once. A rigid map's inverse is exact —
// the transpose, r3.Transform's own contract — and a Frame is orthonormal, so
// every step here is a dot product, never a solve.
func newPrismReexpression(pa, pb prismPayload) (*prismReexpression, error) {
	if pa.frame == pb.frame && pa.xform == pb.xform {
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
	return &prismReexpression{relative: m, transAbs: vecMaxAbs(m.Translation())}, nil
}

// point re-expresses one of operand B's plane-local points into operand A's
// frame, dropping the resulting local z — which G3 already certified is the
// shared plane's own zero axis — and charges the rounding it commits.
//
// The charge is rigidRoundAllow's existing shape, at the INPUT coordinate and
// the composed map's own translation, and it covers the composition as well as
// the application: each r3.Transform.Then rounds a unit-magnitude basis entry
// and a translation-magnitude component a handful of ulps, so three
// compositions and one application together stay under ~48·u·|input| +
// ~40·u·|translation|, where rigidRoundAllow's 16 ulps at 2·|input| +
// |translation|, read as a 3D radius, allow ~110·u·|input| + ~55·u·|translation|.
func (re *prismReexpression) point(p Point2) Point2 {
	if re.identity {
		return p
	}
	out := re.relative.Apply(r3.NewVec(p.U, p.V, 0))
	re.delta = math.Max(re.delta, rigidRoundAllow(max(math.Abs(p.U), math.Abs(p.V)), re.transAbs))
	return Point2{U: out.X, V: out.Y}
}
