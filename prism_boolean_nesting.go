package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// tryPrismHoledIntersect admits one cleanly nested, one-hole prism under a
// hole-free prism. Intersect is symmetric, so put the hole-free operand first.
// The shared Intersect admission used by the overlap-area reading stays
// hole-free; this body-only arm uses the same gates and work cap explicitly.
func tryPrismHoledIntersect(ctx context.Context, a, b *Body) (prismPayload, bool, error) {
	ah, aok := a.payload.(prismPayload)
	bh, bok := b.payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, false, nil
	}
	if len(ah.profile.Holes) == 1 && len(bh.profile.Holes) == 0 {
		a, b = b, a
	} else if len(ah.profile.Holes) != 0 || len(bh.profile.Holes) != 1 {
		return prismPayload{}, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, a, b)
	if err != nil || !ok {
		return prismPayload{}, false, err
	}
	for _, profile := range []ProfileRecord{pa.profile, pb.profile} {
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, profile)
		if err != nil {
			return prismPayload{}, false, err
		}
		if trimmed {
			return prismPayload{}, false, nil
		}
	}
	if !prismIntersectZIntervalOverlaps(pa, pb) {
		return prismPayload{}, false, nil
	}
	segments, withinCap, err := prismSceneWithinWorkCap(budget, pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}
	return resolveAndBuildPrismIntersect(ctx, budget, pa, pb, reexpress)
}

// This file is docs/prism-boolean-design.md §4.2's "clean" sub-case for
// Cut and Intersect: the structural whole-loop match against buildPrismScene's
// own tag map (prism_boolean.go), reusing that file's scene construction, G1-G4
// admission and work-budget/cancellation machinery. There is no assembly and no
// §6 build-time audit on this path — the candidate is one s.Profiles() result
// taken verbatim, so §5's authentication claim 1 (every individual segment is
// authentic) already covers claim 2 (the assembly is correct) entirely. When
// this search comes back unresolved, resolveAndBuildPrismCut/Intersect fall
// through to prism_boolean_crossing.go's edge-orientation classifier before
// giving up — a pair whose operands' boundaries genuinely cross reaches that
// classifier, not straight to the mesh path; only what NEITHER path resolves
// falls through unresolved (§4.4).
//
// A structurally-matched loop's every edge is Whole (§4.2's own match
// condition), so the matched profile carries no cut charge (§7's δ_cut) —
// but a consumed source segment whose OWN recorded range already narrows its
// natural domain still entered buildPrismScene's private scene at a walked
// endpoint the boolean computed, and that charge (§7's δ_walk,
// prism_boolean.go's walkChargeOf) composes into both ops' own sectionDelta
// here, and into the split-boundary reroute condition, exactly as it does on
// Union's own merge path.

// resolveAndBuildPrismCut runs Cut's clean-nesting structural match (§4.2)
// first and, once it finds a unique candidate, authenticates it and builds
// §7's exactness. There is no §6 audit on this path (§6 is titled "Union's
// merge only" no longer describes this file alone — see prism_boolean.go's
// own header; §5 states the clean-nesting claim needs no second proof — the
// candidate is one s.Profiles() result taken verbatim, so §5's authentication
// claim 1 already covers claim 2 entirely). When the clean-nesting search
// comes back unresolved, this tries prism_boolean_crossing.go's
// edge-orientation classifier before giving up.
func resolveAndBuildPrismCut(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, sceneDelta, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		// §4.2's crossing sub-case (prism_boolean_crossing.go): the tool's
		// boundary genuinely crosses the target's, rather than leaving both
		// operands' loops untouched.
		return resolveAndBuildPrismCutCrossing(ctx, budget, target, tool, reexpress)
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
		// re-expression and prior-displacement terms (now including each
		// operand's own walk charge), with the (already zero) cut term
		// omitted rather than added back in.
		sectionDelta: max(
			proofbound.AbsSumUpper(target.sectionDelta, sceneDelta.a),
			proofbound.AbsSumUpper(tool.sectionDelta, sceneDelta.b, reexpress.delta),
		),
	}
	return result, true, nil
}

// resolveAndBuildPrismIntersect runs Intersect's clean-nesting structural
// match (§4.2) in both directions first and, once it finds the unique nested
// operand, authenticates its own region verbatim and builds §7's exactness.
// There is no §6 audit on this path, for the same reason as Cut's. When
// neither direction matches, this tries prism_boolean_crossing.go's
// edge-orientation classifier before giving up.
func resolveAndBuildPrismIntersect(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (prismPayload, bool, error) {
	s, match, sceneDelta, nestedIsB, resolved, err := resolvePrismIntersect(ctx, budget, pa, pb, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		// §4.2's crossing sub-case (prism_boolean_crossing.go): a
		// general-position overlap, rather than one operand's boundary
		// leaving the other's fully untouched.
		return resolveAndBuildPrismIntersectCrossing(ctx, budget, pa, pb, reexpress)
	}

	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return prismPayload{}, false, err
	}

	// §3.2's Intersect row, after G5's exact shift (prismZShift) is applied
	// to B's own recorded interval. Each result endpoint is A's own recorded
	// float, or B's shifted endpoint rounded once with that rounding charged
	// (prismIntersectEnd); a tie takes the larger of the two displacements,
	// since both operands' own coordinates then equally denote it.
	pbZ0, pbZ1 := prismShiftedIntervalAdmitted(pa, pb)
	z0, z0Delta := prismIntersectEnd(pa.z0, pa.z0Delta, pbZ0, pb.z0Delta, func(c int) bool { return c > 0 })
	z1, z1Delta := prismIntersectEnd(pa.z1, pa.z1Delta, pbZ1, pb.z1Delta, func(c int) bool { return c < 0 })

	// §7/Task 4.4: the record traces to the NESTED operand alone, so only
	// that operand's own displacement term (now including its own walk
	// charge) reaches the result — never the max of both, which would be
	// conservative where the code already knows which operand's coordinates
	// it took.
	sectionDelta := proofbound.AbsSumUpper(pa.sectionDelta, sceneDelta.a)
	if nestedIsB {
		sectionDelta = proofbound.AbsSumUpper(pb.sectionDelta, sceneDelta.b, reexpress.delta)
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
	t0, t1 := proofarith.FloatRat(target.z0), proofarith.FloatRat(target.z1)
	z0, z1, ok := prismShiftedInterval(target, tool)
	if t0 == nil || t1 == nil || !ok {
		return false
	}
	return z0.Cmp(t0) <= 0 && z1.Cmp(t1) >= 0
}

// prismIntersectZIntervalOverlaps is G5 for Intersect (§3.2): the two
// re-expressed intervals must overlap, compared as exact rationals.
func prismIntersectZIntervalOverlaps(pa, pb prismPayload) bool {
	a0, a1 := proofarith.FloatRat(pa.z0), proofarith.FloatRat(pa.z1)
	z0, z1, ok := prismShiftedInterval(pa, pb)
	if a0 == nil || a1 == nil || !ok {
		return false
	}
	return a0.Cmp(z1) < 0 && z0.Cmp(a1) < 0
}

// prismIntersectEnd picks §3.2's Intersect result at one sweep end over exact
// rationals, carrying the chosen end's own axial displacement — never the max
// of both, since only one operand's coordinate reaches the result.
// pickA(cmp) reports whether A's own endpoint wins given
// cmp = floatRat(aVal).Cmp(bVal) (cmp > 0 for z0's max, cmp < 0 for z1's
// min). A tie takes A's float with the larger displacement, since both
// operands' own coordinates then equally denote the result. When B's SHIFTED
// endpoint wins it is rounded to the nearest float once and
// rationalFloatError charges that rounding into the end's axial displacement
// beside B's own (§7).
func prismIntersectEnd(aVal, aDelta float64, bVal *big.Rat, bDelta float64, pickA func(cmp int) bool) (float64, float64) {
	cmp := proofarith.FloatRat(aVal).Cmp(bVal) // aVal is a payload level: finite by construction, G5 lifted it already
	switch {
	case cmp == 0:
		return aVal, max(aDelta, bDelta)
	case pickA(cmp):
		return aVal, aDelta
	}
	held, _ := bVal.Float64()
	return held, proofbound.AbsSumUpper(bDelta, proofarith.RationalFloatError(bVal, held))
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
// authenticate it through RecordProfile, and sceneDelta is buildPrismScene's
// own per-operand §7 walk charge.
func resolvePrismCut(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismPayload, reexpress *prismReexpression) (*sketch.Sketch, *sketch.Profile, prismSceneDelta, bool, error) {
	s, match, _, delta, resolved, err := resolvePrismCutWithTags(ctx, budget, target, tool, reexpress)
	return s, match, delta, resolved, err
}

// resolvePrismCutWithTags also returns the scene's entity-origin map. A
// stacked result uses it to retain the target's already recorded whole loops
// while taking only the new tool hole from RecordProfile's authenticated cell.
func resolvePrismCutWithTags(ctx context.Context, budget *proofbound.WorkBudget, target, tool prismPayload, reexpress *prismReexpression) (*sketch.Sketch, *sketch.Profile, map[sketch.Entity]prismcells.Origin, prismSceneDelta, bool, error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, target, tool, reexpress)
	if err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	if len(profiles) == 0 {
		return nil, nil, nil, prismSceneDelta{}, false, nil // §4.4: the scene holds no bounded cell at all
	}
	if target.sectionDelta != 0 || tool.sectionDelta != 0 || !reexpress.identity || sceneDelta.a != 0 || sceneDelta.b != 0 {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return nil, nil, nil, prismSceneDelta{}, false, err
		}
		if split {
			return nil, nil, nil, prismSceneDelta{}, false, nil // §3.4, mirroring Union's own reroute
		}
	}

	targetOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	wantHoles := make([]map[sketch.Entity]struct{}, 0, len(target.profile.Holes)+1)
	for i := range target.profile.Holes {
		hs, err := prismcells.LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return nil, nil, nil, prismSceneDelta{}, false, err
		}
		wantHoles = append(wantHoles, hs)
	}
	toolOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	wantHoles = append(wantHoles, toolOuter) // the tool's own solid, as one new hole

	match, resolved, err := prismcells.FindLoopMatch(budget, profiles, targetOuter, wantHoles)
	if err != nil {
		return nil, nil, nil, prismSceneDelta{}, false, err
	}
	if !resolved {
		return nil, nil, nil, prismSceneDelta{}, false, nil
	}
	if !match.Valid {
		// RB1, matching the Union path's own behaviour: a candidate region
		// the result depends on reports an invalid arrangement. Cut's matched
		// profile is both its nesting proof and its result, so this one check
		// covers both claims.
		return nil, nil, nil, prismSceneDelta{}, false, prismInvalidRegionErr("cut")
	}
	return s, match, tags, sceneDelta, true, nil
}

// resolvePrismIntersect is §4.2's clean-nesting match for Intersect(a, b):
// run the search in both directions (X=a,Y=b and X=b,Y=a) since Intersect's
// relation is symmetric (§3.2) — a caller may pass the smaller body first.
// Each direction asks the same question resolvePrismCut does for its own
// nesting proof: is X's own cell reported with Y's Outer as one further hole?
// That is what closes the disjoint-footprint trap here too. Once a direction
// proves nesting, the RESULT is a separate s.Profiles() candidate: the
// profile whose Outer reproduces the NESTED operand's own Outer, with its
// admitted hole if present — that operand's own cell, untouched. If both
// directions match,
// or neither does, the topology is unresolved (§4.4).
//
// resolved=false (err always nil in that case) means the pair's topology is
// unresolved. nestedIsB reports which operand the result traces to, for
// §7's displacement selection. The returned *sketch.Sketch is the private
// scene the match was found in.
func resolvePrismIntersect(ctx context.Context, budget *proofbound.WorkBudget, pa, pb prismPayload, reexpress *prismReexpression) (s *sketch.Sketch, match *sketch.Profile, sceneDelta prismSceneDelta, nestedIsB, resolved bool, err error) {
	s, tags, sceneDelta, err := buildPrismScene(budget, pa, pb, reexpress)
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	if len(profiles) == 0 {
		return nil, nil, prismSceneDelta{}, false, false, nil
	}
	if pa.sectionDelta != 0 || pb.sectionDelta != 0 || !reexpress.identity || sceneDelta.a != 0 || sceneDelta.b != 0 {
		split, err := prismProfilesHaveSplitBoundary(budget, profiles)
		if err != nil {
			return nil, nil, prismSceneDelta{}, false, false, err
		}
		if split {
			return nil, nil, prismSceneDelta{}, false, false, nil
		}
	}

	aOuter, err := prismcells.LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	bOuter, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}

	// The one-hole arm keeps A hole-free and needs only B-inside-A. The
	// hole-free arm searches in both directions as before.
	//
	// The proof cell carries the whole weight of the nesting claim, so its own
	// validity is checked exactly like the result cell's below: a cell sketch
	// reports degenerate proves nothing about which operand encloses which,
	// and reading a nesting off it would bless an arrangement sketch has
	// already disowned. That is RB1's "a candidate region the result depends
	// on", and this path depends on two.
	proofBNested, bNested, err := prismcells.FindLoopMatch(budget, profiles, aOuter, []map[sketch.Entity]struct{}{bOuter})
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	if bNested && !proofBNested.Valid {
		return nil, nil, prismSceneDelta{}, false, false, prismInvalidRegionErr("intersect")
	}
	aNested := false
	if len(pb.profile.Holes) == 0 {
		proofANested, matched, err := prismcells.FindLoopMatch(budget, profiles, bOuter, []map[sketch.Entity]struct{}{aOuter})
		if err != nil {
			return nil, nil, prismSceneDelta{}, false, false, err
		}
		if matched && !proofANested.Valid {
			return nil, nil, prismSceneDelta{}, false, false, prismInvalidRegionErr("intersect")
		}
		aNested = matched
	}
	if bNested == aNested {
		// Both directions match (should not occur for a genuine pair) or
		// neither does (a disjoint or crossing pair, or any other topology
		// this increment does not cover): unresolved, §4.4.
		return nil, nil, prismSceneDelta{}, false, false, nil
	}

	// The nested operand's own region is a SEPARATE s.Profiles() candidate
	// from the nesting proof above. B may carry one hole in the new arm.
	wantOuter, nested := aOuter, false
	var wantHoles []map[sketch.Entity]struct{}
	if bNested {
		wantOuter, nested = bOuter, true
		for i := range pb.profile.Holes {
			hole, err := prismcells.LoopEntitySet(budget, tags, true, i)
			if err != nil {
				return nil, nil, prismSceneDelta{}, false, false, err
			}
			wantHoles = append(wantHoles, hole)
		}
	}
	result, resultResolved, err := prismcells.FindLoopMatch(budget, profiles, wantOuter, wantHoles)
	if err != nil {
		return nil, nil, prismSceneDelta{}, false, false, err
	}
	if !resultResolved {
		return nil, nil, prismSceneDelta{}, false, false, nil
	}
	if !result.Valid {
		// RB1, matching the Union/Cut paths' own behaviour.
		return nil, nil, prismSceneDelta{}, false, false, prismInvalidRegionErr("intersect")
	}
	return s, result, sceneDelta, nested, true, nil
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
	return prismProfileRecordContext(ctx, func() (ProfileRecord, error) {
		profile, _, err := RecordProfile(s, p)
		return profile, err
	})
}

// prismRecordArrangedProfileContext is for a profile returned by the private
// scene's first Profiles call. The caller owns that scene and selects from its
// returned slice, so no second authentication arrangement is needed.
func prismRecordArrangedProfileContext(ctx context.Context, p *sketch.Profile) (ProfileRecord, error) {
	return prismProfileRecordContext(ctx, func() (ProfileRecord, error) {
		return recordArrangedProfile(p)
	})
}

func prismProfileRecordContext(ctx context.Context, record func() (ProfileRecord, error)) (ProfileRecord, error) {
	if err := ctx.Err(); err != nil {
		return ProfileRecord{}, err
	}
	type prismRecordResult struct {
		profile ProfileRecord
		err     error
	}
	done := make(chan prismRecordResult)
	go func() {
		profile, err := record()
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
