package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/sketch"
)

// This file is docs/prism-boolean-design.md §4.2's "clean" sub-case for
// Cut and Intersect: the structural whole-loop match against buildPrismScene's
// own tag map (prism_boolean.go), reusing that file's scene construction, G1-G4
// admission and work-budget/cancellation machinery. There is no assembly and no
// §6 build-time audit on this path — the candidate is one s.Profiles() result
// taken verbatim, so §5's authentication claim 1 (every individual segment is
// authentic) already covers claim 2 (the assembly is correct) entirely. The
// general per-cell edge-orientation classification for the crossing sub-case
// (PR3) is not implemented; a pair whose operands' boundaries genuinely cross
// falls through unresolved (§4.4), exactly like any other topology this
// increment's resolution does not cover.

// resolveAndBuildPrismCut runs Cut's clean-nesting structural match (§4.2)
// and, once it finds a unique candidate, authenticates it and builds §7's
// exactness. There is no §6 audit on this path (§6 is titled "Union's merge
// only"; §5 states the clean-nesting claim needs no second proof — the
// candidate is one s.Profiles() result taken verbatim, so §5's authentication
// claim 1 already covers claim 2 entirely).
func resolveAndBuildPrismCut(ctx context.Context, budget *workBudget, target, tool prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil // §4.4: this topology is unresolved
	}

	// Point of no return (§3.4): every further problem is a genuine refusal
	// (RB8 below). The matched profile is authenticated verbatim through the
	// existing public seam — no new authentication code (§5).
	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return prismPayload{}, false, err
	}
	result := prismPayload{
		profile: profile,
		// The record carries the TARGET's own frame/xform (operand A's — the
		// private scene's own reference plane, §4.1). The scene's returned
		// PlaneRecord describes that private scene's own world-XY plane, not
		// the body's; using it here would silently swap the result onto the
		// wrong plane, so it is discarded.
		frame: target.frame,
		xform: target.xform,
		// §3.2's Cut row: the result's z-interval is the target's own,
		// unchanged — the tool removes material across the target's full
		// height, never narrows it.
		z0:      target.z0,
		z1:      target.z1,
		z0Delta: target.z0Delta,
		z1Delta: target.z1Delta,
		// §7/Task 4.4: every matched edge is whole, so the cut charge is
		// zero — there is no fresh cut parameter to charge at all. What
		// remains is the same formula PR1's Union path uses for its own
		// re-expression and prior-displacement terms, with the (already
		// zero) cut term omitted rather than added back in.
		sectionDelta: max(target.sectionDelta, absSumUpper(tool.sectionDelta, reexpress.delta)),
	}
	return result, true, nil
}

// resolveAndBuildPrismIntersect runs Intersect's clean-nesting structural
// match (§4.2) in both directions and, once it finds the unique nested
// operand, authenticates its own disk verbatim and builds §7's exactness.
// There is no §6 audit on this path, for the same reason as Cut's.
func resolveAndBuildPrismIntersect(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, nestedIsB, resolved, err := resolvePrismIntersect(ctx, budget, pa, pb, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil // §4.4: this topology is unresolved
	}

	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's shift — provably zero once G3 holds
	// (see prismZShift) — is applied to B's own recorded interval. Each
	// result endpoint is one operand's own recorded float, so it carries
	// that operand's own axial displacement; a tie takes the larger of the
	// two, since both operands' own coordinates then equally denote it.
	shift := prismZShift(pa, pb)
	pbZ0, pbZ1 := pb.z0+shift, pb.z1+shift
	z0, z0Delta := prismIntersectEnd(pa.z0, pa.z0Delta, pbZ0, pb.z0Delta, func(x, y float64) bool { return x > y })
	z1, z1Delta := prismIntersectEnd(pa.z1, pa.z1Delta, pbZ1, pb.z1Delta, func(x, y float64) bool { return x < y })

	// §7/Task 4.4: the record traces to the NESTED operand alone, so only
	// that operand's own displacement term reaches the result — never the
	// max of both, which would be conservative where the code already knows
	// which operand's coordinates it took.
	sectionDelta := pa.sectionDelta
	if nestedIsB {
		sectionDelta = absSumUpper(pb.sectionDelta, reexpress.delta)
	}

	result := prismPayload{
		profile: profile,
		// The private scene's reference plane is always operand A's (§4.1),
		// regardless of which operand turned out to be nested — B's own
		// coordinates, when they are the ones that survive, already live in
		// A's frame through the scene's re-expression.
		frame:        pa.frame,
		xform:        pa.xform,
		z0:           z0,
		z1:           z1,
		z0Delta:      z0Delta,
		z1Delta:      z1Delta,
		sectionDelta: sectionDelta,
	}
	return result, true, nil
}

// prismCutZIntervalSpans is G5 for Cut (§3.2): the tool's re-expressed
// [z0, z1] must span the target's — the tool removes material across the
// target's whole height, which is exactly what §3.2's result row assumes
// when it takes the target's own interval verbatim. A boundary case (the
// tool's cap exactly meeting the target's) is a valid span.
func prismCutZIntervalSpans(target, tool prismPayload) bool {
	shift := prismZShift(target, tool)
	return tool.z0+shift <= target.z0 && tool.z1+shift >= target.z1
}

// prismIntersectZIntervalOverlaps is G5 for Intersect (§3.2): the two
// re-expressed intervals must overlap.
func prismIntersectZIntervalOverlaps(pa, pb prismPayload) bool {
	shift := prismZShift(pa, pb)
	return pa.z0 < pb.z1+shift && pb.z0+shift < pa.z1
}

// prismIntersectEnd picks §3.2's Intersect result at one sweep end: whichever
// operand's own recorded value the max (z0) or min (z1) selects, carrying
// THAT operand's own axial displacement for the end — never the max of both,
// since only one operand's coordinate reaches the result. better(x, y)
// reports whether x is the value this end selects over y (> for z0's max,
// < for z1's min); a tie takes the larger displacement, since both operands'
// own coordinates then equally denote the result.
func prismIntersectEnd(aVal, aDelta, bVal, bDelta float64, better func(x, y float64) bool) (float64, float64) {
	switch {
	case aVal == bVal:
		return aVal, max(aDelta, bDelta)
	case better(aVal, bVal):
		return aVal, aDelta
	default:
		return bVal, bDelta
	}
}

// prismEntityOrigin is buildPrismScene's own tag map value (§4.1's "tagged, in
// a side map, with its origin"): which operand a created scene entity traces
// to, and which of that operand's loops it came from — -1 for Outer, else the
// index into ProfileRecord.Holes. Union's resolution never reads it — every
// operand it admits is hole-free (G6), so only "which operand" would ever
// vary. Cut/Intersect's clean-nesting match (§4.2) is what needs the loop half
// too: proving §4.2's nesting relation is a pure data comparison against this
// map, never a geometric test.
type prismEntityOrigin struct {
	isB  bool
	hole int
}

// prismLoopEntitySet is the tag map's per-loop view: the set of entities
// buildPrismScene created for one operand's one loop (Outer at hole = -1,
// else Holes[hole]), read for the structural match (§4.2) alone.
func prismLoopEntitySet(budget *workBudget, tags map[sketch.Entity]prismEntityOrigin, isB bool, hole int) (map[sketch.Entity]struct{}, error) {
	out := map[sketch.Entity]struct{}{}
	for e, origin := range tags {
		if err := budget.step(); err != nil {
			return nil, err
		}
		if origin.isB == isB && origin.hole == hole {
			out[e] = struct{}{}
		}
	}
	return out, nil
}

// prismLoopMatchesOrigin reports whether a candidate boundary loop
// structurally reproduces the wanted entity set (§4.2's "clean" sub-case):
// every edge is Whole (Partial == false — the arrangement cut nothing), the
// edge count equals the wanted set's size, and the edges' Entity values equal
// the wanted set. Comparing Entity by interface identity is the same
// discipline buildPrismScene's own dedup key already uses. Deliberately NOT a
// check on edge order or starting index: a simple loop's own walk is
// determined only up to rotation, and requiring an index would make the match
// fragile without proving anything more.
func prismLoopMatchesOrigin(budget *workBudget, edges []sketch.BoundaryEdge, want map[sketch.Entity]struct{}) (bool, error) {
	if len(edges) != len(want) {
		return false, nil
	}
	seen := make(map[sketch.Entity]struct{}, len(edges))
	for _, e := range edges {
		if err := budget.step(); err != nil {
			return false, err
		}
		if e.Partial {
			return false, nil
		}
		if _, ok := want[e.Entity]; !ok {
			return false, nil
		}
		if _, dup := seen[e.Entity]; dup {
			return false, nil
		}
		seen[e.Entity] = struct{}{}
	}
	return true, nil
}

// prismHolesMatchOrigin reports whether a candidate profile's Holes
// structurally reproduce EXACTLY the wanted hole entity sets, as an unordered
// set of sets: each candidate hole matches at most one wanted hole (by
// prismLoopMatchesOrigin), and every wanted hole is matched by exactly one
// candidate hole. The bipartite match is small (a handful of holes at most,
// bounded by the same arrangement cap as everything else here) and needs no
// index correspondence — sketch's own Holes order is not decad's to assume.
func prismHolesMatchOrigin(budget *workBudget, holes [][]sketch.BoundaryEdge, want []map[sketch.Entity]struct{}) (bool, error) {
	matched := make([]bool, len(want))
	for _, h := range holes {
		if err := budget.step(); err != nil {
			return false, err
		}
		found := -1
		for i, w := range want {
			if matched[i] {
				continue
			}
			ok, err := prismLoopMatchesOrigin(budget, h, w)
			if err != nil {
				return false, err
			}
			if ok {
				found = i
				break
			}
		}
		if found == -1 {
			return false, nil
		}
		matched[found] = true
	}
	return true, nil
}

// prismFindLoopMatch is §4.2's clean-nesting structural search: the unique
// s.Profiles() result whose Outer structurally reproduces wantOuter and whose
// Holes structurally reproduce EXACTLY the entity sets in wantHoles — a pure
// data comparison against decad's own tag map, never a geometric test.
// resolved=false (err always nil in that case) means no such unique profile
// exists: zero candidates or more than one (ambiguous) are both §4.4's
// "unresolved," not a refusal.
func prismFindLoopMatch(budget *workBudget, profiles []*sketch.Profile, wantOuter map[sketch.Entity]struct{}, wantHoles []map[sketch.Entity]struct{}) (*sketch.Profile, bool, error) {
	var found *sketch.Profile
	for _, p := range profiles {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		outerOK, err := prismLoopMatchesOrigin(budget, p.Outer, wantOuter)
		if err != nil {
			return nil, false, err
		}
		if !outerOK {
			continue
		}
		if len(p.Holes) != len(wantHoles) {
			continue
		}
		holesOK, err := prismHolesMatchOrigin(budget, p.Holes, wantHoles)
		if err != nil {
			return nil, false, err
		}
		if !holesOK {
			continue
		}
		if found != nil {
			return nil, false, nil // ambiguous: more than one candidate matches
		}
		found = p
	}
	if found == nil {
		return nil, false, nil
	}
	return found, true, nil
}

// resolvePrismCut is §4.2's clean-nesting match for Cut(target, tool): when
// the tool's boundary does not touch the target's anywhere, the arrangement
// leaves both operands' original loops completely unmodified, and decad
// finds the one s.Profiles() result whose Outer structurally reproduces the
// target's own Outer and whose Holes structurally reproduce the target's own
// Holes plus EXACTLY ONE further hole reproducing the tool's own Outer (G6
// keeps the tool hole-free, so that one hole is the tool's whole solid).
// That profile IS the result, verbatim — no assembly.
//
// This is the discriminator the disjoint-footprint trap needs: a structural
// match on the tool's own cell alone would not by itself prove nesting, since
// two disjoint hole-free footprints also arrange into two untouched cells,
// each reporting its own Outer whole. Requiring the target's OWN cell to
// carry the tool's Outer as one of ITS holes is what a disjoint pair can
// never produce — the arrangement of two disjoint footprints yields two
// separate cells, neither with a hole at all.
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved (§4.4) — the caller falls back to the mesh path with no error.
// A non-nil error is always genuine and must propagate. The returned
// *sketch.Sketch is the private scene the match was found in, needed to
// authenticate it through RecordProfile.
func resolvePrismCut(ctx context.Context, budget *workBudget, target, tool prismPayload, reexpress *prismReexpression) (*sketch.Sketch, *sketch.Profile, bool, error) {
	s, tags, err := buildPrismScene(budget, target, tool, reexpress)
	if err != nil {
		return nil, nil, false, err
	}
	if err := budget.err(); err != nil {
		return nil, nil, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, false, err
	}
	if err := budget.err(); err != nil {
		return nil, nil, false, err
	}
	if len(profiles) == 0 {
		return nil, nil, false, nil // §4.4: the scene holds no bounded cell at all
	}
	if target.sectionDelta != 0 || tool.sectionDelta != 0 || !reexpress.identity {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return nil, nil, false, err
		}
		if split {
			return nil, nil, false, nil // §3.4, mirroring Union's own reroute
		}
	}

	targetOuter, err := prismLoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, nil, false, err
	}
	wantHoles := make([]map[sketch.Entity]struct{}, 0, len(target.profile.Holes)+1)
	for i := range target.profile.Holes {
		hs, err := prismLoopEntitySet(budget, tags, false, i)
		if err != nil {
			return nil, nil, false, err
		}
		wantHoles = append(wantHoles, hs)
	}
	toolOuter, err := prismLoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, nil, false, err
	}
	wantHoles = append(wantHoles, toolOuter) // the tool's own solid, as one new hole

	match, resolved, err := prismFindLoopMatch(budget, profiles, targetOuter, wantHoles)
	if err != nil {
		return nil, nil, false, err
	}
	if !resolved {
		return nil, nil, false, nil
	}
	if !match.Valid {
		// RB1, matching the Union path's own behaviour: a candidate region
		// the result depends on reports an invalid arrangement. Cut's matched
		// profile is both its nesting proof and its result, so this one check
		// covers both claims.
		return nil, nil, false, prismInvalidRegionErr("cut")
	}
	return s, match, true, nil
}

// resolvePrismIntersect is §4.2's clean-nesting match for Intersect(a, b):
// run the search in both directions (X=a,Y=b and X=b,Y=a) since Intersect's
// relation is symmetric (§3.2) — a caller may pass the smaller body first.
// Each direction asks the same question resolvePrismCut does for its own
// nesting proof: is X's own cell reported with Y's Outer as one further hole?
// That is what closes the disjoint-footprint trap here too. Once a direction
// proves nesting, the RESULT is a separate s.Profiles() candidate: the
// profile whose Outer reproduces the NESTED operand's own Outer with no
// holes — that operand's own disk cell, untouched. If both directions match,
// or neither does, the topology is unresolved (§4.4).
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved. nestedIsB reports which operand the result traces to, for
// §7's displacement selection. The returned *sketch.Sketch is the private
// scene the match was found in.
func resolvePrismIntersect(ctx context.Context, budget *workBudget, pa, pb prismPayload, reexpress *prismReexpression) (s *sketch.Sketch, match *sketch.Profile, nestedIsB, resolved bool, err error) {
	s, tags, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return nil, nil, false, false, err
	}
	if err := budget.err(); err != nil {
		return nil, nil, false, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, false, false, err
	}
	if err := budget.err(); err != nil {
		return nil, nil, false, false, err
	}
	if len(profiles) == 0 {
		return nil, nil, false, false, nil
	}
	if pa.sectionDelta != 0 || pb.sectionDelta != 0 || !reexpress.identity {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return nil, nil, false, false, err
		}
		if split {
			return nil, nil, false, false, nil
		}
	}

	aOuter, err := prismLoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, nil, false, false, err
	}
	bOuter, err := prismLoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, nil, false, false, err
	}

	// G6 keeps both operands hole-free for Intersect, so neither direction's
	// nesting search wants any hole beyond the other operand's own Outer.
	//
	// The proof cell carries the whole weight of the nesting claim, so its own
	// validity is checked exactly like the result cell's below: a cell sketch
	// reports degenerate proves nothing about which operand encloses which,
	// and reading a nesting off it would bless an arrangement sketch has
	// already disowned. That is RB1's "a candidate region the result depends
	// on", and this path depends on two.
	proofBNested, bNested, err := prismFindLoopMatch(budget, profiles, aOuter, []map[sketch.Entity]struct{}{bOuter})
	if err != nil {
		return nil, nil, false, false, err
	}
	if bNested && !proofBNested.Valid {
		return nil, nil, false, false, prismInvalidRegionErr("intersect")
	}
	proofANested, aNested, err := prismFindLoopMatch(budget, profiles, bOuter, []map[sketch.Entity]struct{}{aOuter})
	if err != nil {
		return nil, nil, false, false, err
	}
	if aNested && !proofANested.Valid {
		return nil, nil, false, false, prismInvalidRegionErr("intersect")
	}
	if bNested == aNested {
		// Both directions match (should not occur for a genuine pair) or
		// neither does (a disjoint or crossing pair, or any other topology
		// this increment does not cover): unresolved, §4.4.
		return nil, nil, false, false, nil
	}

	// The nested operand's own disk: its Outer reproduced verbatim, no holes
	// (G6 already keeps it hole-free) — a SEPARATE s.Profiles() candidate
	// from the nesting proof above.
	wantOuter, nested := aOuter, false
	if bNested {
		wantOuter, nested = bOuter, true
	}
	result, resultResolved, err := prismFindLoopMatch(budget, profiles, wantOuter, nil)
	if err != nil {
		return nil, nil, false, false, err
	}
	if !resultResolved {
		return nil, nil, false, false, nil
	}
	if !result.Valid {
		// RB1, matching the Union/Cut paths' own behaviour.
		return nil, nil, false, false, prismInvalidRegionErr("intersect")
	}
	return s, result, nested, true, nil
}

// prismInvalidRegionErr is §9's RB1: a candidate region this op's result
// depends on reports Profile.Valid == false. It is a genuine refusal past
// §3.4's point of no return, never a reroute to the mesh path.
func prismInvalidRegionErr(op string) error {
	return fmt.Errorf(`%w: the %s scene's arrangement reports an invalid region`, ErrUnsupported, op)
}

// prismRecordProfileContext makes RecordProfile's own internal re-arrangement
// (authenticateProfile's fresh s.Profiles() call, seam.go) observable to a
// caller's context, the same way prismProfilesContext wraps the FIRST
// arrangement. The scene is already capped by prismMaxArrangementSegments, so
// this second pass over it stays bounded too. The returned PlaneRecord is not
// read here — the caller keeps operand A's own frame/xform (§4.1) — so only
// the ProfileRecord and error are surfaced.
func prismRecordProfileContext(ctx context.Context, s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, error) {
	if err := ctx.Err(); err != nil {
		return ProfileRecord{}, err
	}
	type prismRecordResult struct {
		profile ProfileRecord
		err     error
	}
	done := make(chan prismRecordResult)
	go func() {
		profile, _, err := RecordProfile(s, p)
		done <- prismRecordResult{profile: profile, err: err}
	}()
	select {
	case result := <-done:
		return result.profile, result.err
	case <-ctx.Done():
		<-done
		return ProfileRecord{}, ctx.Err()
	}
}
