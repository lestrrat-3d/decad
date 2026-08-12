package decad

import (
	"context"

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
// time (prismEntityOrigin) — a flag comparison, never a geometric test — and
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
func resolveAndBuildPrismCutCrossing(ctx context.Context, budget *workBudget, target, tool prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
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

	// Point of no return (§3.4): every further problem is a genuine refusal.
	if err := auditPrismMergeSection(budget, target, merged); err != nil {
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
		// §7's formula, unchanged from Union's own: this path assembles a
		// merged loop exactly like Union's, so it carries the same four
		// displacement terms, cutDelta included.
		sectionDelta: absSumUpper(
			max(
				absSumUpper(target.sectionDelta, sceneDelta.a),
				absSumUpper(tool.sectionDelta, sceneDelta.b, reexpress.delta),
			),
			cutDelta,
		),
	}
	return result, true, nil
}

// resolveAndBuildPrismIntersectCrossing is the Intersect twin: keeps the
// cells that are material of BOTH operands. §3.2's Intersect z-interval and
// axial displacement selection reuse prismZShift/prismIntersectEnd
// (prism_boolean_nesting.go) unchanged.
func resolveAndBuildPrismIntersectCrossing(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
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

	if err := auditPrismMergeSection(budget, pa, merged); err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's shift — provably zero once G3 holds
	// (prismZShift's own doc comment) — is applied to B's own recorded
	// interval, exactly as the clean-nesting path's own Intersect builder
	// does.
	shift := prismZShift(pa, pb)
	pbZ0, pbZ1 := pb.z0+shift, pb.z1+shift
	z0, z0Delta := prismIntersectEnd(pa.z0, pa.z0Delta, pbZ0, pb.z0Delta, func(x, y float64) bool { return x > y })
	z1, z1Delta := prismIntersectEnd(pa.z1, pa.z1Delta, pbZ1, pb.z1Delta, func(x, y float64) bool { return x < y })

	result := prismPayload{
		profile: merged,
		frame:   pa.frame,
		xform:   pa.xform,
		z0:      z0,
		z1:      z1,
		z0Delta: z0Delta,
		z1Delta: z1Delta,
		sectionDelta: absSumUpper(
			max(
				absSumUpper(pa.sectionDelta, sceneDelta.a),
				absSumUpper(pb.sectionDelta, sceneDelta.b, reexpress.delta),
			),
			cutDelta,
		),
	}
	return result, true, nil
}

// resolvePrismCrossing is the shared resolution shape behind both builders
// above: build the private scene, apply §3.4's split-boundary reroute
// (identical to Union's and the clean-nesting path's own), classify every
// cell (classifyPrismCells), select the op's own subset (keep), and merge it
// (mergePrismCells, prism_boolean.go). resolved=false (err always nil in
// that case) is silent fallback throughout — every check here runs before
// §3.4's point of no return. opName feeds mergePrismCells's own RB1 message.
func resolvePrismCrossing(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression, keep func(a, b bool) bool, opName string) (ProfileRecord, prismSceneDelta, float64, bool, error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.err(); err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if err := budget.err(); err != nil {
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
			// Mirrors Union's and the clean-nesting path's own reroute
			// (§3.4): a coordinate error in re-expressed B, either source
			// section's existing displacement, or either operand's own walk
			// charge can move a crossing by delta/sin(theta), which this
			// increment has no certified lower bound for.
			return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
		}
	}

	matterA, matterB, resolved, err := classifyPrismCells(budget, tags, profiles)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !resolved {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
	}

	selected, err := selectPrismCells(budget, profiles, matterA, matterB, keep)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}

	merged, cutDelta, mergedResolved, err := mergePrismCells(budget, selected, opName)
	if err != nil {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, err
	}
	if !mergedResolved {
		return ProfileRecord{}, prismSceneDelta{}, 0, false, nil
	}
	return merged, sceneDelta, cutDelta, true, nil
}

// selectPrismCells filters profiles to the cells keep admits, given each
// cell's own classification — a pure data selection, no geometry read.
func selectPrismCells(budget *workBudget, profiles []*sketch.Profile, matterA, matterB []bool, keep func(a, b bool) bool) ([]*sketch.Profile, error) {
	var selected []*sketch.Profile
	for i, p := range profiles {
		if err := budget.step(); err != nil {
			return nil, err
		}
		if keep(matterA[i], matterB[i]) {
			selected = append(selected, p)
		}
	}
	return selected, nil
}

// prismCellMembership is classifyPrismCells's per-cell, per-operand verdict.
// known reports whether propagation reached one at all; val, meaningful only
// when known, reports which side.
type prismCellMembership struct {
	known bool
	val   bool
}

// prismCellLink is one arrangement edge shared by exactly two cells: a and b
// are their indices into the profiles slice classifyPrismCells received, and
// isB names which operand's entity the shared edge traces to.
type prismCellLink struct {
	a, b int
	isB  bool
}

// classifyPrismCells is §4.2's edge-orientation propagation: for every cell
// profiles holds, whether it sits on operand A's material side and on
// operand B's material side. resolved=false (err always nil in that case)
// means at least one cell's membership could not be reached for one of the
// two operands, or the arrangement holds a shape this classifier does not
// cover (an invalid cell, a cell carrying its own hole, an edge shared by
// more than two cells, or an entity this scene did not create) — the whole
// attempt is unresolved, per this file's own header.
func classifyPrismCells(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin, profiles []*sketch.Profile) (matterA, matterB []bool, resolved bool, err error) {
	n := len(profiles)
	memberA := make([]prismCellMembership, n)
	memberB := make([]prismCellMembership, n)

	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type occurrence struct {
		cell int
		isB  bool
	}
	occ := map[edgeKey][]occurrence{}

	for i, p := range profiles {
		if err := budget.step(); err != nil {
			return nil, nil, false, err
		}
		if !p.Valid || len(p.Holes) != 0 {
			// Not a shape this classifier covers: an invalid cell, or one
			// carrying its own hole (both operands are hole-free on this
			// path, so a holed cell here is defensive, not expected).
			return nil, nil, false, nil
		}
		for _, e := range p.Outer {
			if err := budget.step(); err != nil {
				return nil, nil, false, err
			}
			origin, ok := tags[e.Entity]
			if !ok {
				return nil, nil, false, nil // defensive: an entity this scene did not create
			}
			// The flag comparison itself (§4.2, this file's own header):
			// e.Reversed is this cell's own walk direction on the entity;
			// origin.authoredReversed is the operand's own authored walk
			// direction on it. A match puts this cell on that operand's
			// material side, a mismatch on its void side.
			match := e.Reversed == origin.authoredReversed
			member := &memberA[i]
			if origin.isB {
				member = &memberB[i]
			}
			if member.known && member.val != match {
				return nil, nil, false, nil // conflicting direct edges: not covered
			}
			*member = prismCellMembership{known: true, val: match}

			k := edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
			occ[k] = append(occ[k], occurrence{cell: i, isB: origin.isB})
		}
	}

	// Every edge shared by exactly two cells is a propagation link; more
	// than two occurrences of the same edgeKey is topology this classifier
	// does not cover. A single occurrence is a perimeter edge (against the
	// unbounded exterior) — already spent above for its direct
	// classification, nothing further to connect.
	var links []prismCellLink
	for _, os := range occ {
		if err := budget.step(); err != nil {
			return nil, nil, false, err
		}
		switch len(os) {
		case 1:
		case 2:
			links = append(links, prismCellLink{a: os[0].cell, b: os[1].cell, isB: os[0].isB})
		default:
			return nil, nil, false, nil // §4.4: not a shape this increment covers
		}
	}

	if err := propagatePrismMembership(budget, links, true, memberA); err != nil {
		return nil, nil, false, err
	}
	if err := propagatePrismMembership(budget, links, false, memberB); err != nil {
		return nil, nil, false, err
	}

	matterA = make([]bool, n)
	matterB = make([]bool, n)
	for i := range profiles {
		if !memberA[i].known || !memberB[i].known {
			return nil, nil, false, nil // §4.4: isolated from an operand's own boundary
		}
		matterA[i] = memberA[i].val
		matterB[i] = memberB[i].val
	}
	return matterA, matterB, true, nil
}

// propagatePrismMembership floods a known cell classification across cells
// connected by an edge belonging to the OTHER operand: crossing such an edge
// cannot move across THIS operand's own boundary, so the two cells it joins
// carry the SAME membership. connectorIsB selects which links qualify — true
// when propagating operand A's own membership (over B's edges), false when
// propagating B's (over A's edges). A multi-source BFS from every
// already-known cell reaches every cell propagation can settle; what remains
// unknown afterward is genuinely unresolved (§4.4), read by the caller.
func propagatePrismMembership(budget *workBudget, links []prismCellLink, connectorIsB bool, member []prismCellMembership) error {
	adj := make([][]int, len(member))
	for _, l := range links {
		if l.isB != connectorIsB {
			continue
		}
		adj[l.a] = append(adj[l.a], l.b)
		adj[l.b] = append(adj[l.b], l.a)
	}
	queue := make([]int, 0, len(member))
	visited := make([]bool, len(member))
	for i := range member {
		if member[i].known {
			queue = append(queue, i)
			visited[i] = true
		}
	}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		if err := budget.step(); err != nil {
			return err
		}
		for _, nb := range adj[cur] {
			if visited[nb] {
				continue
			}
			visited[nb] = true
			member[nb] = member[cur]
			queue = append(queue, nb)
		}
	}
	return nil
}
